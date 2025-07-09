#include "plugins/action/goalyaw.h"

GoalYaw::GoalYaw(const std::string &name, const NodeConfig &config)
    : SyncActionNode(name, config)
{
  mct = MavRosConnect::getInstance();
  nh = mct->getROSHandle();
  fcu_pose_ptr = mct->getFcuPose();
  fcu_state_ptr = mct->getFcuState();
  tgt_pose_pub_ptr = mct->getPosTgtPublier();
  cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  cmd.type_mask = 0xfff;
  cmd.type_mask &=
      ~mavros_msgs::PositionTarget::FORCE;                   // px4 不响应力的控制方式
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW; // 放开yaw控制
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PX;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PY;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
  cmd.header.frame_id = "gaol_yaw";
}

PortsList GoalYaw::providedPorts()
{
  return {InputPort("goal_yaw", "we need to control the fcu yaw")};
}

NodeStatus GoalYaw::tick()
{
  if (!fcu_state_ptr->armed || fcu_state_ptr->mode != "OFFBOARD")
  {
    throw RuntimeError(
        "to control fcu yaw fail, the dorne's state is node [in GoalYaw] "
        "offboard or not armed");
    return NodeStatus::FAILURE;
  };
  ROS_INFO("in GoalYaw node");
  auto yaw_port = getInput<double>("goal_yaw");
  if (!yaw_port)
  {
    throw RuntimeError("error reading prot [goal_yaw]", yaw_port.error());
  }
  ros::Rate loop(20);
  while (ros::ok())
  {
    double yaw = yaw_port.value();
    cmd.yaw = yaw * M_PI / 180;
    cmd.position = fcu_pose_ptr->pose.position;
    tgt_pose_pub_ptr->publish(cmd);
    ros::spinOnce();

    tf2::Quaternion q;
    tf2::fromMsg(fcu_pose_ptr->pose.orientation, q);
    tf2::Matrix3x3 rot(q);
    double yaw_, pitch_, roll_;
    rot.getRPY(roll_, pitch_, yaw_);
    ROS_INFO("fcu yaw: %f", yaw_);
    auto det = abs(cmd.yaw - yaw_);
    ROS_INFO("fcu yaw: %f, det:%f", yaw_, det);
    if (det < 0.1)
    { // 任务已经到达目标点了
      ROS_INFO("current yaw(%f) contrl success", yaw);
      //ros::Duration(2.0).sleep();
      return NodeStatus::SUCCESS;
    }
    loop.sleep();
  }
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<GoalYaw>("GoalYaw");
}
