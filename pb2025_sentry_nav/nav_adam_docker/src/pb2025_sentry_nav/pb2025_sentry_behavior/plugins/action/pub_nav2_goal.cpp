// Copyright 2025 Lihan Chen
// Licensed under the Apache License, Version 2.0

#include "pb2025_sentry_behavior/plugins/action/pub_nav2_goal.hpp"
// 1. 必须包含这个头文件，否则无法解析 "x;y;yaw" 字符串！
#include "pb2025_sentry_behavior/custom_types.hpp"

namespace pb2025_sentry_behavior
{

PubNav2Goal::PubNav2Goal(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, config, params) // 2. 修正基类
{
}

BT::PortsList PubNav2Goal::providedPorts()
{
  return providedBasicPorts({
    // 这里使用了自定义类型，需要 custom_types.hpp 支持
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "0;0;0", "Expected goal pose. Format: 'x;y;yaw'")
  });
}

bool PubNav2Goal::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  //getInput 会调用 convertFromString，这需要 custom_types.hpp 可见
  if (!getInput("goal", msg)) {
    return false;
  }
  
  // 确保 frame_id 正确
  if (msg.header.frame_id.empty()) {
    msg.header.frame_id = "map";
  }
  msg.header.stamp = node_->now();

  return true;
}

// 3. 删除 setHaltMessage

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PubNav2Goal, "PubNav2Goal");