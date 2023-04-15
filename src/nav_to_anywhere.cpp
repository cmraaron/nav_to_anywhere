// Copyright (c) 2023 Aaron Lipinski

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_2d_utils/conversions.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<rclcpp::Node>("nav_to_anywhere");

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  geometry_msgs::msg::Pose2D pos_active;
  geometry_msgs::msg::Pose2D pos_target;

  const auto interval = 0.2;  // seconds
  const auto velocity = 1;    // m/s

  const auto tick = node->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(interval * 1000)), [&]() {
      const auto theta = std::atan2(pos_target.y - pos_active.y, pos_target.x - pos_active.x);
      const auto vx = std::cos(theta) * velocity;
      const auto vy = std::sin(theta) * velocity;
      pos_active.x = vx * interval;
      pos_active.y = vy * interval;
      pos_active.theta = theta;
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
      pos_target = nav_2d_utils::poseToPose2D(goal_handle->get_goal()->pose.pose);
    }
  );


  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  return 0;
}
