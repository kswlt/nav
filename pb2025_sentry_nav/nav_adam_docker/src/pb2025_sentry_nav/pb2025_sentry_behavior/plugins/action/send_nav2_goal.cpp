// Copyright 2025 Lihan Chen
// Licensed under the Apache License, Version 2.0

#include "pb2025_sentry_behavior/plugins/action/send_nav2_goal.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // 需要用来转换角度

namespace pb2025_sentry_behavior
{

SendNav2GoalAction::SendNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
}

bool SendNav2GoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
{
  // 1. 直接读取三个独立的数值 (BehaviorTree 原生支持 double，不会报错)
  double x, y, yaw;
  if (!getInput("x", x) || !getInput("y", y) || !getInput("yaw", yaw)) {
    RCLCPP_ERROR(logger(), "Missing goal coordinates (x, y, yaw)");
    return false;
  }

  // 2. 手动构建 Pose
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = now();
  
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.position.z = 0.0;

  // 3. 将 yaw 转为四元数
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal.pose.pose.orientation = tf2::toMsg(q);

  return true;
}

BT::NodeStatus SendNav2GoalAction::onResultReceived(const WrappedResult & wr)
{
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger(), "Navigation succeeded!");
      return BT::NodeStatus::SUCCESS;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger(), "Navigation aborted");
      return BT::NodeStatus::FAILURE;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(logger(), "Navigation canceled");
      return BT::NodeStatus::FAILURE;
    default:
      return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus SendNav2GoalAction::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  return BT::NodeStatus::RUNNING;
}

void SendNav2GoalAction::onHalt() { 
    RCLCPP_INFO(logger(), "SendNav2Goal halted"); 
}

BT::NodeStatus SendNav2GoalAction::onFailure(BT::ActionNodeErrorCode error)
{
  return BT::NodeStatus::FAILURE;
}

BT::PortsList SendNav2GoalAction::providedPorts()
{
  // 4. 定义三个新的输入端口
  return providedBasicPorts({
    BT::InputPort<double>("x", 0.0, "Goal X"),
    BT::InputPort<double>("y", 0.0, "Goal Y"),
    BT::InputPort<double>("yaw", 0.0, "Goal Yaw (rad)")
  });
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::SendNav2GoalAction, "SendNav2Goal");