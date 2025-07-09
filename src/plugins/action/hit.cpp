#include "plugins/action/hit.h"

Hit::Hit(const std::string &name, const NodeConfig &config)
    : StatefulActionNode(name, config)
{
  mct = MavRosConnect::getInstance();
  // 需要根据当前飞机位姿，计算目标在全局坐标系下的位置
  fcu_pose_ptr = mct->getFcuPose();
  fcu_state_ptr = mct->getFcuState();
  tgt_pose_pub_ptr = mct->getPosTgtPublier();
  nh = mct->getROSHandle();

  nh->param<std::string>("det_topic", det_topic, "/objects");
  nh->param<int>("img_w", rgb_image_w, 640);
  nh->param<int>("img_h", rgb_image_h, 480);
  nh->param<double>("hight_min", hight_min, 0.5);
  nh->param<double>("hight_max", hight_max, 1.0);
  nh->param<double>("obs_dist", obs_dist, 0.5);
  nh->param<double>("v_max", v_max, 2);
  nh->param<double>("fx", fx, 320.0);
  nh->param<double>("fy", fy, 320.0);
  nh->param<double>("cx", cx, 240.0);
  nh->param<double>("cy", cy, 240.0);
  nh->param("len_sequence_objs", len_sequence_objs, 5);
  // std::string odom_topic;
  std::string obs_cloud_topic;
  std::string depth_topic;
  std::string pos_cmd_topic;
  nh->param<std::string>("depth_topic", depth_topic, "/rflysim/sensor2/img_depth");
  // nh->param("odom_topic",odom_topic,"/mavros/local_position/odom");
  nh->param<std::string>("obs_cloud_topic", obs_cloud_topic, "/grid_map/occupancy_inflate");
  nh->param<std::string>("planner_cmd_topic", pos_cmd_topic, "/planning/pos_cmd");

  nh->param("kx", kx, 0.0);
  nh->param("ky", ky, 0.0);
  nh->param("kz", kz, 0.0);
  nh->param("kyaw_rate", k_yaw, 0.0);

  det_sub = nh->subscribe<common_msgs::Objects>(det_topic, 10,
                                                bind(&Hit::RecvObj, this, _1));

  obs_sub = nh->subscribe<sensor_msgs::PointCloud2>(obs_cloud_topic, 10, boost::bind(&Hit::RecvPointCloud, this, _1));
  depth_sub = nh->subscribe<sensor_msgs::Image>(depth_topic, 1, boost::bind(&Hit::DepthCallback, this, _1));

  goal_pub = nh->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  isDetect = false;
  isHit_enable = false;

  cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
  cmd.type_mask = 0xfff;
  cmd.type_mask &=
      ~mavros_msgs::PositionTarget::FORCE; // px4 不响应力的控制方式
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VX;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VY;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VZ;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  cmd.header.frame_id = "Hit";

  ;
}

PortsList Hit::providedPorts()
{

  return {InputPort("object_name", "input hitted object"),
          InputPort("method", "hit object method,direct or other")};
}

NodeStatus Hit::onStart()
{
  ROS_INFO("in hit node");
  if (fcu_state_ptr->mode != "OFFBOARD" || !fcu_state_ptr->armed)
  {
    ROS_ERROR("current fcu state cann't to fly");
    return NodeStatus::FAILURE;
  }
  auto object_name_port = getInput<std::string>("object_name");
  if (!object_name_port)
  {
    throw RuntimeError("error reading prot[balloon_name]", object_name_port.error());
  }
  obj_name = object_name_port.value();
  ROS_INFO("we will hited %s", obj_name.c_str());

  auto hit_method_port = getInput<int>("method");
  if (!hit_method_port)
  {
    throw RuntimeError("error reading port[method in Hit]", hit_method_port.error());
  }
  hit_method = static_cast<Hit::Method>(hit_method_port.value());

  if (hit_method == Hit::Method::direct)
  { // 如是直接撞击目标，直接对isHit_enable赋值
    isHit_enable = true;
  }

  return NodeStatus::RUNNING; // 如果在onStart()里面返回SUCCESS 节点将会结束
  ;
}

NodeStatus Hit::onRunning()
{ // 这个节点结束条件是气球破裂，但是在程序里面不判断气球破裂的条件，
  // 因此这里不会返回成功，如果刺破了气球，人接手飞机控制权,否则任务目标丢失，等待再一次刺破动作
  // 首先确保飞机状态是真确的、

  std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 可适当降低执行频率，来降低计算量

  if (fcu_state_ptr->mode != "OFFBOARD" || !fcu_state_ptr->armed)
  {
    ROS_ERROR("current fcu state cann't to fly");
    return NodeStatus::FAILURE;
  }
  if (hit_method != Hit::Method::direct)
  {
    if (!isDetect || !isHit_enable)
    {
      bool stop_hover;
      nh->getParam("stop_hover_port", stop_hover);
      auto last_t = ros::Time::now();
      if (stop_hover && ros::Time::now() - last_t > ros::Duration(1))
      { // 如过持续1秒丢失目标，保持悬停，等待再次检测,或者主动搜索
        ROS_WARN("lost object ,keep hover [in hit.cpp at RecvObj]");
        nh->setParam("stop_hover_port", false);
        last_t = ros::Time::now();
      }
      return NodeStatus::RUNNING;
    }
  }
  /*使用避障的方式逼近，然后近距离是使用视觉伺服控制*/
  if (abs(goal.pose.position.x) < FLT_MIN || abs(goal.pose.position.y) < FLT_MIN || abs(goal.pose.position.z) < FLT_MIN)
  { // 如果能计算到目标的全局系下的坐标，那就表明目标以及被检查了
    ROS_WARN("the goal is not settings");
    nh->setParam("stop_hover_port", false);
    return NodeStatus::RUNNING;
  }

  double dist = std::sqrt(std::pow(goal.pose.position.x - fcu_pose_ptr->pose.position.x, 2) + std::pow(goal.pose.position.y - fcu_pose_ptr->pose.position.y, 2) + std::pow(goal.pose.position.z - fcu_pose_ptr->pose.position.z, 2));
  ROS_INFO("current ball distance %f", dist);
  static bool is_vision_ctrl = false;
  // if (dist > 0.8 && !is_vision_ctrl)
  { // 如果距离小于一定值，使用视觉控制快速撞击
    goal_pub.publish(goal);
    ros::spinOnce();
    // ROS_INFO("goal at pub: %f,%f,%f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    // ROS_INFO("current_pose: %f,%f,%f", fcu_pose_ptr->pose.position.x, fcu_pose_ptr->pose.position.y, fcu_pose_ptr->pose.position.z);
    return NodeStatus::RUNNING;
  }
  // else
  { // 退出轨迹控制，直接切换到视觉伺服去打击
    nh->setParam("is_stop_planner", true);
    is_vision_ctrl = true;
  }
  // 用与P控制
  int ex = 0;
  int ey = 0;
  double area_rate = 0.0;
  {
    std::unique_lock<std::mutex> lock(mtx, std::try_to_lock);
    if (lock.owns_lock())
    {
      //      ROS_INFO("obj time: %f", obj.header.stamp.toSec());
      //      ROS_INFO("current_time: %f", ros::Time::now().toSec());
      //      if(ros::Time::now() - obj.header.stamp > ros::Duration(2))
      //      {
      //      //如果一段时间没有检测到目标，需要等待检测，如果更长一段时间没有发现目标，该再一次搜索目标（具体需要结合xml文件设计）
      //        ROS_WARN("lost object ,keep hover [in hit.cpp, at onRunning]");
      //        nh->setParam("stop_hover_port", false);
      //        return NodeStatus::RUNNING;
      //      }

      ex = obj.center_x - int(rgb_image_w / 2); // 控制y方向速度
      ey = obj.center_y - int(rgb_image_h / 2); // 控制z方向速度
      //      area_rate = (obj.right_bottom_x - obj.left_top_x) / rgb_image_w;
      std::cout << "ex: " << ex << " ey: " << ey << " ar " << area_rate
                << std::endl;
    }
    else
    {
      ROS_WARN("the onRunning not get lock");
      return NodeStatus::RUNNING;
    }
  }
    // 因为随着目标的靠近，目标中心在图像
    //   auto vy = ky * ex;  //y方向不控制速度，使用 控制偏航角
    auto vz = -kz * ey;

    auto vx = v_max - ky * ex;

    if (vx < 0)
    {
      vx = v_min;
    }

    if (fcu_pose_ptr->pose.position.z > hight_max || fcu_pose_ptr->pose.position.z < hight_min)
    { // 这种情况飞机应该是控制过了，需要调参或者反向控制调换来，
      // 尽管飞机是俯冲式撞击，但是为了保证飞机不被撞毁，pitch角不能倾斜过大（这样必须要保证高速飞行），因此视觉看着像是平行撞击
      if (fcu_pose_ptr->pose.position.z < hight_min)
      {
        vz = v_min;
      }
      else
        vz = -v_min;
    }

    auto yaw_rate =
        -k_yaw * ex; // 当y方向控制左右受到障碍物干扰是，应当控制偏航角

    ROS_INFO("vx:%f,vz:%f,yaw_rate:%f", vx, vz, yaw_rate);

    cmd.velocity.x = vx;
    cmd.velocity.y = 0;
    cmd.velocity.z = vz;
    cmd.yaw_rate = yaw_rate;
    tgt_pose_pub_ptr->publish(cmd);
    ros::spinOnce();
    //  isDetect = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return NodeStatus::RUNNING;

    ;
  }

  void Hit::onHalted()
  {
    ;
  }

  void Hit::DepthCallback(const sensor_msgs::Image::ConstPtr &msg)
  { // 直接拿球的RGB中心位置，再深度图里面做位置解算（当然前提是，使用深度与RGB对齐后的数据）
    if (obj.center_x < 1 || obj.center_y < 1)
    { // 没有检测到目标，不需要计算
      return;
    }
    float depth = 0.0;
    const int pixel_idx = obj.center_y * msg->width + obj.center_x;
    static int bytes = msg->data.size() / (msg->width * msg->height);
    const int byte_offset = pixel_idx * (msg->step / msg->width);
    if (bytes == 2)
    { // 表明是一个像素占两个字节
      uint16_t raw_depth;
      memcpy(&raw_depth, &msg->data[byte_offset], sizeof(uint16_t));
      depth = raw_depth / 1000.0f; // 转换为米
    }
    else if (bytes == 4)
    {
      memcpy(&depth, &msg->data[byte_offset], sizeof(float));
    }

    if (depth <= 0.0 || std::isnan(depth))
    { // 无效点
      return;
    }

    float cam_x = (obj.center_x - cx) / fx;
    float cam_y = (obj.center_y - cy) / fy;
    float cam_z = depth;
    // 将点转换到map坐标系下
    // x = cam_z; y =-cam_x; z = -cam_y;
    geometry_msgs::PoseStamped body_p;
    body_p.pose.position.x = cam_z;
    body_p.header.frame_id = "body";
    body_p.pose.position.y = -cam_x;
    body_p.pose.position.z = -cam_y;
    geometry_msgs::PoseStamped ball_p;
    static tf2_ros::Buffer tf_buffer;
    static tf2_ros::TransformListener tf_listener(tf_buffer);
    try
    { // 这里需要查看TF树
      goal = tf_buffer.transform(body_p, "camera_init");
      ROS_INFO("goal in map: %f,%f,%f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("Transform failure: %s", ex.what());
    }
    // 发送目标点给路径规划节点
  }

  void Hit::RecvObj(const common_msgs::Objects::ConstPtr &msg)
  { // 这里需要的目标是目标分割后的中心位置，而非检测框的中心位置；

    

    if (msg->objects.empty() || obj_name.empty())
      return;
      
    ROS_INFO("recv objsize in hit: %d", msg->objects.size());
    static int idx = -1;
    static double score = -1.0;
    for (size_t i = 0; i < msg->objects.size(); ++i)
    {
      if (msg->objects[i].class_name != obj_name)
        continue;
      // ROS_INFO("Recv ball object");
      if (msg->objects[i].score > score)
      {
        idx = i;
        score = msg->objects[i].score;
      }
    };
    if (msg->objects[idx].score < 0.7 || msg->objects[idx].class_name != obj_name)
    { // 置信度太低了，
      idx = -1;
      score = -1;
      std::unique_lock<std::mutex> lock(mtx, std::try_to_lock);
      if (lock.owns_lock())
      {
        obj.center_x = 0;
        obj.center_y = 0;
      }
      return;
      // ROS_WARN("The object score is %f,or havn't need to hit object", msg->objects[idx].score);
    }
    std::unique_lock<std::mutex> lock(mtx, std::try_to_lock);
    if (lock.owns_lock())
    {
      obj = msg->objects[idx];
      if (hit_method != Hit::Method::direct)
      { // 当单独撞击目标时，不用进入这里，这里仅仅是满足二十届机器人比赛需求，要满足小车从P1->P2->P1这样一个流程后才能去刺
        if (!isHit_enable)
        {
          JudgeHitEnable();
        }
        // ROS_INFO("end hover");
        if (isHit_enable)
        {
          // 当可以攻击的时候，才结束悬停, 如果使用路径规划去避障，需从planner node 里面去控制
          nh->setParam("stop_hover_port", true); // 结束悬停
        }
        isDetect = true;
      }
    }
    idx = 0;
    score = 0;
  }

  /*
  void Hit::RecvPointCloud(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
  {
    // 需要过滤些点云，如雷达罩，远处点，为了减少计算这里过滤出一个正方体的点集
    std::vector<livox_ros_driver2::CustomPoint> tmp;
  #pragma omp parallel shared(obs_ps)
    {
      for (size_t i = 0; i < msg->points.size(); ++i)
      {
        if (abs(msg->points[i].x) < obs_dist && abs(msg->points[i].y) < obs_dist && abs(msg->points[i].z) < obs_dist)
        {
          tmp.push_back(msg->points[i]);
          ;
        }
      }
  #pragma omp barrier
  #pragma omp single
      {   // 只有在需要的时候才会用上这个，也就是当飞机需要在x,y方向的值后开始判断
        ; // 只有一个线程执行打印
        std::unique_lock<std::mutex> lock(pc_mtx);
        obs_ps.swap(tmp);
      }
    }
  }
  */

  void Hit::RecvPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    /*
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    try
    {
      // 这里直接从tf树里面获取到飞机当前的位置，这几从避障模块获得到的点云，使用最新的odom计算是不准确的，准确的来讲应该直接获取到雷达获得深度点云，但是那样会带来更多的计算量
      geometry_msgs::TransformStamped transform =
          tf_buffer.lookupTransform("base_link", "odom", ros::Time(0));

      pcl_ros::transformPointCloud("base_link", transform, *msg, obs_ps);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
    }
    */
  }

  bool Hit::CalFlyOri(double &vx, double &vy, double &vz, double &yaw_rate)
  {
    /*
    if (obs_ps.empty()) // 如果没有障碍物阻挡，可直接使用 速度 + 偏航控制
      return false;

    // 说明安全范围内有障碍物点，应该避让
    std::vector<livox_ros_driver2::CustomPoint> tmp;
    {
      std::unique_lock<std::mutex> lock(pc_mtx);
      tmp.swap(obs_ps);
    }

    if (vx > 0)
    { // 飞机往前飞，扫描前方安全飞行范围内是否有障碍物
      // 仅仅使用yaw_rate 和 vx
      // 已经不能规避障碍物了，需要控制vy方向,加单把点云分成四个象限
      // ，但是点肯定有重叠的地方
      std::vector<livox_ros_driver2::CustomPoint> fp;
      std::vector<livox_ros_driver2::CustomPoint> bp;
      std::vector<livox_ros_driver2::CustomPoint> lp;
      std::vector<livox_ros_driver2::CustomPoint> rp;
      for (size_t i = 0; i < tmp.size(); ++i)
      {
        if (tmp[i].x > 0)
        {
          fp.push_back(tmp[i]);
        }
        else
        {
          bp.push_back(tmp[i]);
        }
        if (tmp[i].y > 0)
        {
          lp.push_back(tmp[i]);
        }
        else
        {
          rp.push_back(tmp[i]);
        }
      }

      if (yaw_rate > 0 && lp.size())
      {             // 飞机向左转向，转为飞机向左飞，转向停止
        vy = v_max; // 这里使用最粗暴的方式，后续根据情况再改
        vx = 0;     // 停止向前飞
        yaw_rate = 0;
      }
    }
    else
    {
      ;
    }
    */
  }

  void Hit::JudgeHitEnable()
  {                                               // 当小车在图像的轨迹从图像底部到顶部的行径方向的时候，任务可以攻击了。因此当飞机飞到P1的，小车应该能走完半段路程，因此，这里需要保证小车有一定的速度
    static std::queue<common_msgs::Obj> pre_objs; // 用来存储一段时间目标
    if (pre_objs.size() < len_sequence_objs)
    {
      pre_objs.push(obj);
    }
    else
    { // 这里判断简单，就拿第一个和最后一个做差值即可,如果出现误判，适当增大len_sequenece_objs的值
      pre_objs.push(obj);
      common_msgs::Obj f = pre_objs.front();
      common_msgs::Obj b = pre_objs.back();
      if (f.center_y - b.center_y)
      {
        isHit_enable = true;
        return;
      }
      pre_objs.pop();
    }
  }

  BT_REGISTER_NODES(factory)
  {
    factory.registerNodeType<Hit>("Hit");
  }
