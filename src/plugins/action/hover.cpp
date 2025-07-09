#include "plugins/action/hover.h"

Hover::Hover(const std::string &name, const NodeConfig &config)
  : StatefulActionNode(name, config)
{
  mct              = MavRosConnect::getInstance();
  nh               = mct->getROSHandle();
  fcu_pose_ptr     = mct->getFcuPose();
  fcu_state_ptr    = mct->getFcuState();
  tgt_pose_pub_ptr = std::make_shared<ros::Publisher>(*mct->getPosTgtPublier());
  cmd.coordinate_frame = mavros_msgs::PositionTarget::
    FRAME_LOCAL_NED;  // 选择控制坐标系，位置，速度，加速度使用local坐标系，姿态使用的是body坐标系
  cmd.type_mask = 0xfff;
  cmd.type_mask &=
    ~mavros_msgs::PositionTarget::FORCE;  // px4 不响应力的控制方式
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PX;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PY;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
}

PortsList Hover::providedPorts()
{
  return { InputPort<bool>("stop_hover", "emit hover event"),
           InputPort<bool>("hover_is_end",
                           "if exit current node,be set ture") };
}

NodeStatus Hover::onStart()
{
  /*这里参数本可以在行为树里面设置，但是目前黑板参数通过别的节点改不了，需后面排查，这里暂且使用ros
   * 参数服务器*/

  //可能在别的节点或者流程里面把下面的参数改变了，因此再次启动这个节点时，需要回复初始值
  ROS_INFO("in hover node ===================");
  nh->setParam("hover_is_end_port", false);
  nh->setParam("stop_hover_port", false);

  if(fcu_state_ptr->mode != "OFFBOARD" || !fcu_state_ptr->armed)
  {
    ROS_ERROR("current fcu state has error");
    return NodeStatus::FAILURE;
  }
  else
  {
    cmd.position.x = fcu_pose_ptr->pose.position.x;
    cmd.position.y = fcu_pose_ptr->pose.position.y;
    cmd.position.z = fcu_pose_ptr->pose.position.z;
    return NodeStatus::RUNNING;
  }
}

NodeStatus Hover::onRunning()
{
  //悬停节点除了开启或关闭节点，还需要一个退出节点的信号。
  //  auto stop_hover_port = getInput<bool>("stop_hover");
  //  auto is_end_port     = getInput<bool>("hover_is_end");

  bool is_end = false;
  nh->getParam("hover_is_end_port", is_end);

  if(is_end)
  {  //如果某个节点触发了结束当前节点，停止运行
    return NodeStatus::SUCCESS;
  }

  bool stop_hover;
  nh->getParam("stop_hover_port", stop_hover);

  if(!stop_hover)
  {
    ROS_INFO("keep hover ...");
    cmd.position     = fcu_pose_ptr->pose.position;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "hover_debug";
    tgt_pose_pub_ptr->publish(cmd);
    ros::spinOnce();
  }
  else
  {
    //    ROS_INFO("recv end hover command");
    ;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 模拟执行逻辑
  return BT::NodeStatus::
    RUNNING;  //在没有发出终止节点前，即便不是悬停动作，也要挂起当前节点
}

void Hover::onHalted()
{
  //  ROS_INFO("The havor node is end");
  ;
}
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<Hover>("Hover");
}
