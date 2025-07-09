#include "plugins/action/plannode.h"


PlanNode::PlanNode(const std::string name, const NodeConfiguration &config)
    : StatefulActionNode(name, config)
{
  mct = MavRosConnect::getInstance();
  nh = mct->getROSHandle();
  fcu_state_ptr = mct->getFcuState();
  fcu_pose_ptr = mct->getFcuPose();
  tgt_pose_pub_ptr = mct->getPosTgtPublier();
  is_enable_planner = false;
  det_flag = 0;
  is_stop_planner = false;
  has_recv_cmd = false;
  nh->param<bool>("is_stop_planner", is_stop_planner, false);
  cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  cmd.type_mask = 0xfff;
  cmd.type_mask &=
      ~mavros_msgs::PositionTarget::FORCE; // px4 不响应力的控制方式
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PX;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PY;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
  // cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;
  last_time = ros::Time::now();
  cmd.header.frame_id = "planner node";
  nh->param<std::string>("det_topic", det_topic, "/objects");
  
  object_sub = nh->subscribe<common_msgs::Objects>(det_topic, 10, bind(&PlanNode::ObjectCallback, this, _1));  // 新增：订阅器

  std::string goal_topic;
  nh->param<std::string>("goal_topic", goal_topic, "/move_base_simple/goal");

  std::string cmd_topic;
  nh->param<std::string>("planner_cmd_topic", cmd_topic, "/pose_cmd");

  planer_goal_pub = nh->advertise<geometry_msgs::PoseStamped>(goal_topic, 10);

  pos_cmd_sub = nh->subscribe<quadrotor_msgs::PositionCommand>(
      cmd_topic, 10, bind(&PlanNode::PositionCmdCB, this, _1));
}

PortsList PlanNode::providedPorts()
{
  return {InputPort("goal_position", "the goal position"),
          InputPort("goal_ori", "the goal orientation, [roll, pitch, yaw]"),
          InputPort("enable_planner", "open or close planner"),
          InputPort("planner_ctrl_type",
                    "choose the control type,0:pose,1:volicty,2:accelerate"),
          InputPort("goal_src", "0: xml set; 1:program inner;3: rviz")};
}

//新增,是否收到气球数据
void PlanNode::ObjectCallback(const common_msgs::Objects::ConstPtr &msg)
{
  if (msg->objects.empty()){
    ROS_INFO("the msg is Null, please check this msg.");
    return;
    }
  
  //ROS_INFO("recv objsize in plannode: %d", msg->objects.size());

  static double score_min = 0.7;
  for (size_t i = 0; i < msg->objects.size(); ++i)
  {
    if (msg->objects[i].class_name != "balloon")
      continue;
    // ROS_INFO("Recv ball object");
    if (msg->objects[i].score > score_min && goal_src == 1)
    {
      ROS_INFO("Balloon Detected:");
      det_flag = 1;
      return;
    }
  }
}

void PlanNode::PositionCmdCB(
    const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
  last_time = ros::Time::now();
  //ROS_INFO("goal_src: %d, det_flag: %d", goal_src, det_flag);
  if (goal_src == 1){
    if (!is_enable_planner || !det_flag )
      return; // 如果不开启路径规划，直接返回
    cmd.position = msg->position;
    cmd.velocity = msg->velocity;
    cmd.acceleration_or_force = msg->acceleration;
    cmd.yaw = msg->yaw;
    cmd.yaw_rate = msg->yaw_dot;
    if (!has_recv_cmd)
      has_recv_cmd = true;
  }
  else{
    if (!is_enable_planner)
      return; // 如果不开启路径规划，直接返回
    cmd.position = msg->position;
    cmd.velocity = msg->velocity;
    cmd.acceleration_or_force = msg->acceleration;
    cmd.yaw = msg->yaw;
    cmd.yaw_rate = msg->yaw_dot;
    if (!has_recv_cmd)
      has_recv_cmd = true;
  }
}

NodeStatus PlanNode::onStart()
{
  if (fcu_state_ptr->mode != "OFFBOARD" || !fcu_state_ptr->armed)
    return NodeStatus::FAILURE;

  ROS_INFO("in planner node");

  auto ret_ = getInput<int>("planner_ctrl_type");
  if (!ret_)
  {
    throw RuntimeError("error reading prot [enable_planner]", ret_.error());
  }

  ROS_INFO("planner ctrl type is %d", ret_.value());

  nh->getParam("is_stop_planner", is_stop_planner);
  if (is_stop_planner)
  { // 如果别的节点结束了当前节点，直接退出
    ROS_WARN("planner node has recv other request , exit");
    is_stop_planner = false;
    return NodeStatus::RUNNING;
  }

  auto goal_src_port = getInput<int>("goal_src"); // 路径规划节点来源
  // 如果目标源来自程序内，这里就不需要接受xml里面传过来的参数
  if (!goal_src_port)
  {
    throw RuntimeError("error reading prot [goal_src]", goal_src_port.error());
  }
  goal_src = goal_src_port.value();

  // 监听开关
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  auto ret = getInput<bool>("enable_planner");
  is_enable_planner = ret.value();
  if (!ret)
  {
    throw RuntimeError("error reading prot [enable_planner]", ret.error());
  }
  if (!is_enable_planner)
  {
    ROS_WARN("is not open planner");
    return NodeStatus::FAILURE;
  }
  int type = ret_.value();
  if (type == 1)
  { // 速度控制
    cmd.type_mask = 0xfff;
    cmd.type_mask &=
        ~mavros_msgs::PositionTarget::FORCE; // px4 不响应力的控制方式
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VX;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VY;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VZ;
  }
  if (type == 2)
  { // 加速度控制
    cmd.type_mask = 0xfff;
    cmd.type_mask &=
        ~mavros_msgs::PositionTarget::FORCE; // px4 不响应力的控制方式
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFX;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFY;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFZ;
  }
  if (goal_src == 1)
  {
    // cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;
  }
  cmd.header.frame_id = "plann_node";
  if (!has_recv_cmd)
  {
    if (goal_src == 0)
    { // 如果是使用外部给的目标点
      auto ret = getInput<Position3D>("goal_position");
      if (!ret)
      {
        throw RuntimeError("reading goal_position fail", ret.error());
      }
      Position3D pos = ret.value();
      goal.pose.position.x = pos.x;
      goal.pose.position.y = pos.y;
      goal.pose.position.z = pos.z;

      auto ret_ = getInput<Position3D>("goal_ori");
      if (!ret_)
      {
        throw RuntimeError("error reading port [goal_ori]:", ret_.error());
      }
      Position3D ori = ret_.value();

      tf2::Quaternion q;
      q.setRPY(ori.x, ori.y, ori.z);
      goal.pose.orientation.x = q.x();
      goal.pose.orientation.y = q.y();
      goal.pose.orientation.z = q.z();
      goal.pose.orientation.w = q.w();
      planer_goal_pub.publish(goal);
      // ROS_INFO("Planner OnStart current fcu pose is (%f,%f,%f)",
      //          fcu_pose_ptr->pose.position.x, fcu_pose_ptr->pose.position.y,
      //          fcu_pose_ptr->pose.position.z);
      ros::spinOnce();
    }
    else if (goal_src == 1)
    { // 使用别的节点给目标点
      return NodeStatus::RUNNING;
    }
    else if (goal_src == 2)
    {                             // 使用rviz
      return NodeStatus::RUNNING; // 如果在onStart()里面返回SUCCESS 节点将会结束
    }
  }
  ROS_INFO("current fcu pose is (%f,%f,%f)",
           fcu_pose_ptr->pose.position.x, fcu_pose_ptr->pose.position.y,
           fcu_pose_ptr->pose.position.z);
  ROS_INFO("in planner node goal position is (%f,%f,%f)",
           goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
  ROS_INFO("in planner node goal orientation is (%f,%f,%f)",
           goal.pose.orientation.x, goal.pose.orientation.y,
           goal.pose.orientation.z);

  return NodeStatus::RUNNING; // 如果在onStart()里面返回SUCCESS 节点将会结束
  ;
}

NodeStatus PlanNode::onRunning()
{
  // planer_goal_pub.publish(goal);
  // ros::spinOnce();
  static bool stop_hover = false;
  static bool hover_is_end = false;

  if (has_recv_cmd)
  {
    if (!stop_hover || !hover_is_end)
    {                                          // 接受到规划的轨迹了，就不需要保持悬停了。
      nh->setParam("stop_hover_port", true);   // 结束悬停，该按照轨迹来飞了
      nh->setParam("hover_is_end_port", true); // 悬停节点退出信号
      nh->getParam("hover_is_end_port", hover_is_end);
      nh->getParam("stop_hover_port", stop_hover);
      ros::spinOnce();
    }
    //  setOutput("stop_hover", false);
    if (!stop_hover || !hover_is_end) // 存在一种可能，就是有轨迹点了，但是hover状态还没切换
    {                                 // 如果悬停节点还没退出不发送轨迹指令
      return NodeStatus::RUNNING;
    }
  }
  else
  {
    if (goal_src == 0)
    { // 使用xml内节点指令，可以继续发
      //  没收到轨迹指令，可以继续发目标点
      planer_goal_pub.publish(goal);
      ros::spinOnce();
    }
    else if (goal_src == 1)
    { // 等待其他节点程序发送gaol
      ;
    }
    return NodeStatus::RUNNING;
  }

  if (is_enable_planner && ros::Time::now() - last_time < ros::Duration(3))
  {
    tgt_pose_pub_ptr->publish(cmd);
    ros::spinOnce();
  }

  if ((abs(fcu_pose_ptr->pose.position.x - goal.pose.position.x) < 0.2 && abs(fcu_pose_ptr->pose.position.y - goal.pose.position.y) < 0.2 && abs(fcu_pose_ptr->pose.position.z - goal.pose.position.z) < 0.2) /*|| ros::Time::now() - last_time > ros::Duration(5)*/)
  { // 有些路径规划节点可能到了终点了也会一直发终点的话题，因此这里不能仅仅用时间去判断是否以及到终点
    ROS_INFO("fcu_pose: %f,%f,%f", fcu_pose_ptr->pose.position.x, fcu_pose_ptr->pose.position.y, fcu_pose_ptr->pose.position.z);
    ROS_INFO("fcu_pose: %f,%f,%f", goal.pose.position.x, goal.pose.position.x, goal.pose.position.x);
    ROS_INFO("now time %f, last_time: %f", ros::Time::now().toSec(), last_time.toSec());
    ROS_INFO("has over 5 seconds not update cmd");
    return NodeStatus::SUCCESS;
  }
  return NodeStatus::RUNNING;
  ;
}

void PlanNode::onHalted()
{
  ROS_WARN(
      "exit planner node ,maybe the planner has too long time not update cmd!");
  ;
}
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<PlanNode>("PlanNode");
}

