// Copyright (c) 2023 Aaron Lipinski

#include <memory>
#include <utility>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav_to_anywhere/utils.hpp"

struct Params
{
  std::string topic_footprint;
  std::string footprint_loaded;
  std::string footprint_unloaded;
};

struct Config
{
  std::vector<geometry_msgs::msg::Point> footprint;
  std::vector<geometry_msgs::msg::Point> footprint_loaded;
  std::vector<geometry_msgs::msg::Point> footprint_unloaded;
  geometry_msgs::msg::Pose2D pos_active{};
  const float velocity = 1;    // m/s
  const float interval = 0.2;  // seconds
};

geometry_msgs::msg::PolygonStamped transformFootprint(
  const geometry_msgs::msg::Pose2D & pose,
  const std::vector<geometry_msgs::msg::Point> & footprint)
{
  geometry_msgs::msg::PolygonStamped oriented_footprint;
  nav2_costmap_2d::transformFootprint(pose.x, pose.y, pose.theta, footprint, oriented_footprint);
  return oriented_footprint;
}

std::vector<geometry_msgs::msg::Point> makeFootprintFromString(const std::string & fp_string)
{
  std::vector<geometry_msgs::msg::Point> fp;
  return nav2_costmap_2d::makeFootprintFromString(fp_string, fp) ?
         fp : nav2_costmap_2d::makeFootprintFromRadius(0.3);
}

int main(int argc, char * argv[])
{
  const auto footprint_default_loaded =
    "[ [0.55, 0.32], [-0.47, 0.32], [-0.47, -0.32], [0.55, -0.32] ]";
  const auto footprint_default_unloaded = "";  // will default to a circle

  rclcpp::init(argc, argv);

  const auto node = std::make_shared<rclcpp::Node>("nav_to_anywhere");

  const auto bt_actions = get_action_details(node);
  RCLCPP_INFO(node->get_logger(), "Action config:");
  for (const auto & detail : bt_actions) {
    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "  name: " << detail.name << ", regex: '" << detail.regex <<
        "', type: " << detail.type << ", duration: " << detail.duration
    );
  }

  const Params params {
    .topic_footprint =
      node->declare_parameter<std::string>("topic_footprint", "local_costmap/published_footprint"),
    .footprint_loaded =
      node->declare_parameter<std::string>("footprint_loaded", footprint_default_loaded),
    .footprint_unloaded =
      node->declare_parameter<std::string>("footprint_unloaded", footprint_default_unloaded),
  };

  Config config {};

  tf2_ros::TransformBroadcaster tf_broadcaster(*node);
  const auto local_footprint_pub = node->create_publisher<geometry_msgs::msg::PolygonStamped>(
    params.topic_footprint,
    rclcpp::SystemDefaultsQoS());

  config.footprint_loaded = makeFootprintFromString(params.footprint_loaded);
  config.footprint_unloaded = makeFootprintFromString(params.footprint_unloaded);

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  std::shared_ptr<GoalHandleNavigateToPose> current_goal_handle;
  auto nav_start_time = node->get_clock()->now();

  /* map -> base_footprint tf */
  const auto get_transform =
    [](const geometry_msgs::msg::Pose2D & pose, const rclcpp::Time & stamp) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id = "map";
      transform.child_frame_id = "base_footprint";
      transform.transform.translation.x = pose.x;
      transform.transform.translation.y = pose.y;
      transform.transform.rotation = nav_2d_utils::pose2DToPose(pose).orientation;
      transform.header.stamp = stamp;
      return transform;
    };


  /* increment mission progress */
  const auto update_current_pos = [&]() {
      const auto current_action = get_action(
        bt_actions,
        current_goal_handle->get_goal()->behavior_tree);

      if (current_action.type == ACTION_PICK || current_action.type == ACTION_DROP) {
        const auto elapsed_time = node->get_clock()->now() - nav_start_time;
        if (elapsed_time.seconds() > current_action.duration) {
          config.footprint = current_action.type == ACTION_PICK ?
            config.footprint_loaded :
            config.footprint_unloaded;
          return true;
        }
        return false;
      }

      if (current_action.type != ACTION_NAV) {
        RCLCPP_INFO(node->get_logger(), "Beep boop - doing robot stuff");
        return true;
      }

      const auto pos_target = nav_2d_utils::poseToPose2D(
        current_goal_handle->get_goal()->pose.pose);
      const auto dy = pos_target.y - config.pos_active.y;
      const auto dx = pos_target.x - config.pos_active.x;
      const auto theta = std::atan2(dy, dx);

      const auto vx = std::cos(theta) * config.velocity;
      const auto vy = std::sin(theta) * config.velocity;
      const auto idx = vx * config.interval;
      const auto idy = vy * config.interval;

      /* if we are within one step of our goal */
      if (dy * dy + dx * dx < idy * idy + idx * idx) {
        config.pos_active.x = pos_target.x;
        config.pos_active.y = pos_target.y;
        config.pos_active.theta = pos_target.theta;
        return true;
      } else {
        config.pos_active.x += idx;
        config.pos_active.y += idy;
        config.pos_active.theta = theta;
      }
      return false;
    };


  /* periodic timer for incrementing progress */
  const auto tick = node->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(config.interval * 1000)), [&]() {
      /* if we have an active mission */
      if (current_goal_handle) {
        const auto goal_reached = update_current_pos();

        auto feedback = std::make_unique<NavigateToPose::Feedback>();
        feedback->number_of_recoveries = 0;
        feedback->current_pose.pose = nav_2d_utils::pose2DToPose(config.pos_active);
        feedback->navigation_time = node->get_clock()->now() - nav_start_time;
        current_goal_handle->publish_feedback(std::move(feedback));

        if (goal_reached) {
          current_goal_handle->succeed(std::make_unique<NavigateToPose::Result>());
          current_goal_handle.reset();
        } else if (current_goal_handle->is_canceling()) {
          auto result = std::make_shared<NavigateToPose::Result>();
          current_goal_handle->canceled(result);
          current_goal_handle.reset();
        }
      }
      /* broadcast base_footprint in map frame */
      tf_broadcaster.sendTransform(get_transform(config.pos_active, node->get_clock()->now()));

      /* publish local footprint */
      local_footprint_pub->publish(transformFootprint(config.pos_active, config.footprint));
    });


  /* offer NavigateToPose service */
  const auto nav_to_pose_action_service = rclcpp_action::create_server<NavigateToPose>(
    node,
    "navigate_to_pose",

    /* handle_goal */
    [&](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToPose::Goal> goal) {
      const auto pose2d = nav_2d_utils::poseToPose2D(goal->pose.pose);
      const auto action = get_action(bt_actions, goal->behavior_tree);
      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Received goal request" <<
          "\n  bt: " << goal->behavior_tree <<
          "\n  map: " << goal->pose.header.frame_id <<
          "\n  pose: {x: " << pose2d.x << ", y: " << pose2d.y << ", theta: " << pose2d.theta << "}"
          "\n  action: " << action.name
      );
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },

    /* handle_cancel */
    [&](const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
      RCLCPP_INFO(node->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    },

    /* handle_accepted */
    [&](const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
      if (current_goal_handle) {
        current_goal_handle->abort(std::make_unique<NavigateToPose::Result>());
      } else {
        nav_start_time = node->get_clock()->now();
      }
      current_goal_handle = goal_handle;
    }
  );


  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  return 0;
}
