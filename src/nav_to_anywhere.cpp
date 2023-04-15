// Copyright (c) 2023 Aaron Lipinski

#include <memory>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<rclcpp::Node>("nav_to_anywhere");

  tf2_ros::TransformBroadcaster tf_broadcaster(*node);
  const auto local_footprint_pub = node->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "local_costmap/published_footprint", rclcpp::SystemDefaultsQoS());

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  geometry_msgs::msg::Pose2D pos_active;
  std::shared_ptr<GoalHandleNavigateToPose> current_goal_handle;
  auto nav_start_time = node->get_clock()->now();

  const auto interval = 0.2;  // seconds
  const auto velocity = 1;    // m/s

  const auto tick = node->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(interval * 1000)), [&]() {
      /* broadcast map -> base_footprint tf */
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id = "map";
      transform.child_frame_id = "base_footprint";
      transform.transform.translation.x = pos_active.x;
      transform.transform.translation.y = pos_active.y;
      transform.transform.rotation = nav_2d_utils::pose2DToPose(pos_active).orientation;
      transform.header.stamp = node->get_clock()->now();
      tf_broadcaster.sendTransform(transform);

      if (current_goal_handle) {
        const auto pos_target = nav_2d_utils::poseToPose2D(
          current_goal_handle->get_goal()->pose.pose);
        const auto dy = pos_target.y - pos_active.y;
        const auto dx = pos_target.x - pos_active.x;
        const auto theta = std::atan2(dy, dx);

        const auto vx = std::cos(theta) * velocity;
        const auto vy = std::sin(theta) * velocity;
        const auto idx = vx * interval;
        const auto idy = vy * interval;

        if (dy * dy + dx * dx < idy * idy + idx * idx) {
          pos_active.x = pos_target.x;
          pos_active.y = pos_target.y;
          pos_active.theta = pos_target.theta;

          current_goal_handle->succeed(std::make_unique<NavigateToPose::Result>());
          current_goal_handle.reset();
          return;
        } else {
          pos_active.x += idx;
          pos_active.y += idy;
          pos_active.theta = theta;
        }

        auto feedback = std::make_unique<NavigateToPose::Feedback>();
        feedback->number_of_recoveries = 0;
        feedback->current_pose.pose = nav_2d_utils::pose2DToPose(pos_active);
        feedback->navigation_time = node->get_clock()->now() - nav_start_time;
        current_goal_handle->publish_feedback(std::move(feedback));
      }
    });

  const auto nav_to_pose_action_service = rclcpp_action::create_server<NavigateToPose>(
    node,
    "navigate_to_pose",

    [&](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToPose::Goal> goal) {
      const auto pose2d = nav_2d_utils::poseToPose2D(goal->pose.pose);
      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Received goal request" <<
          "\n  bt: " << goal->behavior_tree <<
          "\n  map: " << goal->pose.header.frame_id <<
          "\n  pose: {x: " << pose2d.x << ", y: " << pose2d.y << ", theta: " << pose2d.theta << "}"
      );
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },

    [&](const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
      RCLCPP_INFO(node->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    },

    [&](const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
      nav_start_time = node->get_clock()->now();
      current_goal_handle = goal_handle;
    }
  );


  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  return 0;
}
