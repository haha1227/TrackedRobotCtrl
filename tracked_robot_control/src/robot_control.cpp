#include "tracked_robot_control/robot_control.h"
#include "tracked_robot_control/robot_planning.h"

RobotControl::RobotControl()
{
    InitKinematicsSolver();
    InitAllPID();
}

void RobotControl::InitKinematicsSolver()
{
    ik_solver_.reset(new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param, timeout, error, TRAC_IK::Speed));

    if (chain_start=="" || chain_end=="") {
        ROS_FATAL("Missing chain info in header file");
        exit(-1);
    }
    ROS_INFO_STREAM("TRAC-IK found " << chain_start << " solutions (" << chain_end);

    bool valid = ik_solver_->getKDLChain(robot_chain);
    if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
        exit(-1);
    }

    valid = ik_solver_->getKDLLimits(ll, ul);
     std::cout << "limits: " << ll(0) <<" " << ul(0) <<" ";
    if (!valid){
      ROS_ERROR("There were no valid KDL joint limits found");
      exit(-1);
    }

    num_used_joints = robot_chain.getNrOfJoints();
    ROS_INFO("Using %d joints", num_used_joints);
    KDL::JntArray joint_seed(num_used_joints);
    KDL::SetToZero(joint_seed);
    home_jnt = KDL::JntArray(joint_seed);

    //ik_solver_.reset(new TRAC_IK::TRAC_IK(robot_chain,ll, ul));
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain));
    jac_solverJ_.reset(new KDL::ChainJntToJacSolver(robot_chain));
    fk_solverVel_rec_.reset(new KDL::ChainFkSolverVel_recursive(robot_chain));
    ik_solverVel_.reset(new KDL::ChainIkSolverVel_pinv(robot_chain));
    ik_solverKDL_.reset(new KDL::ChainIkSolverPos_NR_JL(robot_chain, ll, ul,*fk_solver_,*ik_solverVel_,100,error));
}

void RobotControl::InitAllPID()
{
    // the order is motor3, 4, 5, 6, 7
    for (int i = 0; i < 6; i++)
    {
        MiniPID temp = MiniPID(joint_p.at(i),200,joint_d.at(i));
        temp.setOutputLimits(joint_vel_abs_limits.at(i)*joint_vel_abs_limits_ratio.at(i));
        //temp.setOutputFilter(0.1);
        joint_pids.push_back(temp);
    }

    for (int i = 0; i < 3; i++)
    {
        MiniPID temp = MiniPID(mobile_p.at(i), 0, mobile_d.at(i));
        temp.setOutputLimits(mobile_vel_abs_limits.at(i));
        mobile_pids.push_back(temp);
    }
}

KDL::Frame RobotControl::Jnt2Cart(KDL::JntArray joint_positions)
{
    bool kinematics_status;
    KDL::Frame cart_pose;
    kinematics_status = fk_solver_->JntToCart(joint_positions, cart_pose);
    if (kinematics_status>=0){
        return cart_pose;
    } else{
        ROS_ERROR("Could not calculate forward kinematics");
        exit(-1);
    }
}

KDL::JntArray RobotControl::Cart2Jnt(KDL::Frame cart_frame, bool random_seed)
{
    KDL::JntArray result(num_used_joints);
    KDL::JntArray q(num_used_joints);
    joint_dist_.resize(num_used_joints);
    for(std::size_t i = 0; i < num_used_joints; ++i)
    {
        q(i) = joint_dist_[i](rand_eng_);
    }
    // q is the current joint position. q = home_jnt means the current state is in the home posiitons.
    if (!random_seed)
        q = home_jnt;
    //int kinematics_status = ik_solver_->CartToJnt(q, cart_frame, result);
    int kinematics_status = ik_solverKDL_->CartToJnt(q, cart_frame, result);

    if(kinematics_status < 0){
        ROS_ERROR("Could not calculate inverse kinematics, %d", kinematics_status);
        //exit(-1);
    } else{
        return result;
    }
}

KDL::JntArray RobotControl::GetCurrentJntPos(std::vector<double> input, bool arm)
{
    int len = input.size();
    KDL::JntArray result;
    result.resize(len);
    for (int i = 0; i < len; i++)
    {
        result.data(i) = input.at(i);
    }
    return result;
}

std::vector<double> RobotControl::Frame2Vector(KDL::Frame frame)
{
    KDL::Vector p = frame.p;
    KDL::Rotation M = frame.M;

    double x, y, z, w;
    M.GetQuaternion(x, y, z, w);

    // return the 3D position x y z and rotation roll pitch yaw
    return std::vector<double>{p(0), p(1), p(2), x, y, z, w};
}

KDL::Frame RobotControl::Vector2Frame(std::vector<double> v)
{
    KDL::Vector p(v.at(0), v.at(1), v.at(2));
    KDL::Rotation M;
    M.RPY(v.at(3), v.at(4), v.at(5));

    return KDL::Frame(M, p);
}

void RobotControl::PrintFrame(KDL::Frame input)
{
    std::vector<double> p = Frame2Vector(input);
    printf("%s \n","KDL FK Success");
    std::cout <<"Origin: " << p.at(0) << "," << p.at(1)  << "," << p.at(2) << std::endl;
    std::cout <<"Quater: " << p.at(3)  << "," << p.at(4)  << "," << p.at(5)  << "," << p.at(6) << std::endl;
}

// id: 1, 2, 3 for mobile case. 4, 5, 6, 7, 8, 9 for arm.
std::string RobotControl::SetJntPosition(int id, double goal_pos, double &vel, double current_pos, MiniPID pid)
{
    std::string axisID = "0300020" + std::to_string(id);

    double out;
    if (id <4)
    {
      out = pid.getOutput(current_pos, goal_pos);
    }
    else if(id >3 && ctrl_style == "pid")
    {
      out = pid.getOutput(current_pos, goal_pos);
    }
    else if(id >3 && ctrl_style == "ptp" && joint_trajectory.points.size() > 1)
    {
      //if (nth_waypoints > joint_trajectory.points.size())
      //  nth_waypoints = 1;
      out = joint_trajectory.points.at(nth_waypoints).velocities.at(id-4) /8.3/rpm2radian;
      //out = joint_trajectory.points.at(nth_waypoints).positions.at(id-4);// /8.3/rpm2radian;

      if (id == int(num_used_joints + 2))
      {
          nth_waypoints++;
      }
      if (nth_waypoints == joint_trajectory.points.size())
      {
        start_planning =true;
        nth_waypoints = 1;
        if (demo)
        {
          for (int i = 0; i<5; i++)
          {
            joint_positions.at(i) = -1 * joint_positions.at(i);
          }
          //motor_positions.at(2) = -1 * motor_positions.at(2) ;
        }
      }
      //out = pid.getOutput(current_pos, out);
    }

    vel = out;
    if (id == 2 || id == 5 || id == 6)
        vel = (-1) * vel;
    std::string vel2 = Dec2Hex(vel, 4);
    vel2 = vel2.substr(2,2) + vel2.substr(0,2);
    return cmd_header + BCCXOR(axisID.append(vel2));
}

std::string RobotControl::SetMotorSpeed(int id, double speed)
{
    std::string axisID = "0300020" + std::to_string(id);
    std::string vel2 = Dec2Hex(speed, 4);
    vel2 = vel2.substr(2,2) + vel2.substr(0,2);
    return cmd_header + BCCXOR(axisID.append(vel2));
}

void RobotControl::planPTP(trajectory_msgs::JointTrajectory& joint_trajectory,
                           double velocity_scaling_factor, const double acceleration_scaling_factor,
                           double sampling_time)
{
  trajectory_msgs::JointTrajectory temp;
  joint_trajectory = temp;
  // current pos
  std::map<std::string, double> start_pos = { {"jnt4", current_jnt_pos.data(0)}, {"jnt5", current_jnt_pos.data(1)},
                                              {"jnt6", current_jnt_pos.data(2)}, {"jnt7", current_jnt_pos.data(3)},
                                              {"jnt8", current_jnt_pos.data(4)},};
                                              //{"jnt8", 0.0}, {"jnt9", 0.0},};

  // goal pos
  std::map<std::string, double> goal_pos = { {"jnt4", joint_positions.at(0)}, {"jnt5", joint_positions.at(1)},
                                              {"jnt6", joint_positions.at(2)}, {"jnt7", joint_positions.at(3)},
                                              {"jnt8", joint_positions.at(4)},};
                                              //{"jnt8", joint_positions.at(4)}, {"jnt9", joint_positions.at(5)},};

  // initialize joint names
  for (const auto& item : goal_pos)
  {
    joint_trajectory.joint_names.push_back(item.first);
  }

  // check if goal already reached
  bool goal_reached = true;
  for (auto const& goal : goal_pos)
  {
    if (fabs(start_pos.at(goal.first) - goal.second) >= MIN_MOVEMENT)
    {
      goal_reached = false;
      break;
    }
  }
  if (goal_reached)
  {
    ROS_INFO_STREAM("Goal already reached, set one goal point explicitly.");
    if (joint_trajectory.points.empty())
    {
      trajectory_msgs::JointTrajectoryPoint point;
      point.time_from_start = ros::Duration(sampling_time);
      for (const std::string& joint_name : joint_trajectory.joint_names)
      {
        point.positions.push_back(start_pos.at(joint_name));
        point.velocities.push_back(0);
        point.accelerations.push_back(0);
      }
      joint_trajectory.points.push_back(point);
    }
    return;
  }

  // compute the fastest trajectory and choose the slowest joint as leading axis
  std::string leading_axis = joint_trajectory.joint_names.front();
  double max_duration = -1.0;

  std::map<std::string, pilz_industrial_motion_planner::VelocityProfileATrap> velocity_profile;
  int i = 0;
  for (const auto& joint_name : joint_trajectory.joint_names)
  {
    // create vecocity profile if necessary
    velocity_profile.insert(std::make_pair(
        joint_name, pilz_industrial_motion_planner::VelocityProfileATrap(velocity_scaling_factor * joint_vel_abs_limits.at(i),
                                         acceleration_scaling_factor * joint_acc_abs_limits.at(i),
                                         acceleration_scaling_factor * joint_dec_abs_limits.at(i))));

    velocity_profile.at(joint_name).SetProfile(start_pos.at(joint_name), goal_pos.at(joint_name));
    if (velocity_profile.at(joint_name).Duration() > max_duration)
    {
      max_duration = velocity_profile.at(joint_name).Duration();
      leading_axis = joint_name;
    }
    i++;
  }

  // Full Synchronization
  // This should only work if all axes have same max_vel, max_acc, max_dec
  // values
  // reset the velocity profile for other joints
  /*
  double acc_time = velocity_profile.at(leading_axis).firstPhaseDuration();
  double const_time = velocity_profile.at(leading_axis).secondPhaseDuration();
  double dec_time = velocity_profile.at(leading_axis).thirdPhaseDuration();

  for (const auto& joint_name : joint_trajectory.joint_names)
  {
    if (joint_name != leading_axis)
    {
      // make full synchronization
      // causes the program to terminate if acc_time<=0 or dec_time<=0 (should
      // be prevented by goal_reached block above)
      // by using the most strict limit, the following should always return true
      if (!velocity_profile.at(joint_name)
               .setProfileAllDurations(start_pos.at(joint_name), goal_pos.at(joint_name), acc_time, const_time,
                                       dec_time))
      // LCOV_EXCL_START
      {
        std::stringstream error_str;
        error_str << "TrajectoryGeneratorPTP::planPTP(): Can not synchronize "
                     "velocity profile of axis "
                  << joint_name << " with leading axis " << leading_axis;
        throw PtpVelocityProfileSyncFailed(error_str.str());
      }
      // LCOV_EXCL_STOP
    }
  }
  */

  // first generate the time samples
  std::vector<double> time_samples;
  for (double t_sample = 0.0; t_sample < max_duration; t_sample += sampling_time)
  {
    time_samples.push_back(t_sample);
  }
  // add last time
  time_samples.push_back(max_duration);

  // construct joint trajectory point
  for (double time_stamp : time_samples)
  {
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(time_stamp);
    int i = 0;
    for (std::string& joint_name : joint_trajectory.joint_names)
    {
      point.positions.push_back(velocity_profile.at(joint_name).Pos(time_stamp));
      point.velocities.push_back(velocity_profile.at(joint_name).Vel(time_stamp)*joint_vel_abs_limits_ratio.at(i));
      point.accelerations.push_back(velocity_profile.at(joint_name).Acc(time_stamp));
      i++;
    }
    joint_trajectory.points.push_back(point);
  }

  // Set last point velocity and acceleration to zero
  std::fill(joint_trajectory.points.back().velocities.begin(), joint_trajectory.points.back().velocities.end(), 0.0);
  std::fill(joint_trajectory.points.back().accelerations.begin(), joint_trajectory.points.back().accelerations.end(),
            0.0);
}

void RobotControl::demo_callback(const std_msgs::Bool::ConstPtr &msg)
{
  demo = msg->data;
  if (demo)
  {
    arm_start_pub = true;

    joint_positions.at(0) = 0.0;
    joint_positions.at(1) = 0.0;
    joint_positions.at(2) = -0.0;
    joint_positions.at(3) = 0.0;
    joint_positions.at(4) = 0.0;

    //motor_positions.at(2) = 1.83;
  }
}

void RobotControl::arm_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    // ROS_INFO_STREAM("Writing arm data to serial port: " << msg->header.seq);

    std_msgs::String cmd;

    current_jnt_pos = GetCurrentJntPos(std::vector<double>({msg->position.begin(), msg->position.begin() + num_used_joints}), true);

    if (ctrl_style == "ptp" && start_planning)
    {
      planPTP(joint_trajectory);
      start_planning = false;
      std::cout << "restart planning, num of waypoints: " << joint_trajectory.points.size()<<std::endl;
    }

    for (int i = 0; i < num_used_joints; i++)
    {
        double jnt_out_vel;
        cmd.data = SetJntPosition(i+4, joint_positions.at(i), jnt_out_vel, msg->position.at(i), joint_pids.at(i));
        if (std::abs(jnt_out_vel) > 2 && arm_start_pub)
        //if (arm_start_pub)
        {
            mobile_pub.publish(cmd);
        }
        rate.sleep();
    }

    if (false)
    {
      KDL::Frame cart_pos = Jnt2Cart(current_jnt_pos);
      geometry_msgs::PoseStamped temp;
      temp.header.stamp = ros::Time::now();
      temp.pose.position.x = cart_pos.p.x();
      temp.pose.position.y = cart_pos.p.y();
      temp.pose.position.z = cart_pos.p.z();
      cart_pos.M.GetQuaternion(temp.pose.orientation.x,
                               temp.pose.orientation.y,
                               temp.pose.orientation.z,
                               temp.pose.orientation.w);

      arm_cart_pub.publish(temp);
      rate.sleep();
    }
}

// set wheel speed for mobile base, here only for the flipper
void RobotControl::mobile_flipper_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    // ROS_INFO_STREAM("Writing mobile data to serial port: " << msg->header.seq);

    std_msgs::String cmd;
    // current_motor_pos = GetCurrentJntPos(std::vector<double>({msg->position.end(), msg->position.end() + 1}), false);

    // set the flipper motor cmd
    double flipper_motor_vel;
    cmd.data = SetJntPosition(2+1, motor_positions.at(2), flipper_motor_vel, msg->position.at(2), mobile_pids.at(2));
    if (std::abs(flipper_motor_vel) > 2 && flipper_start_pub)
    {
        mobile_pub.publish(cmd);
        rate.sleep();
    }
}

// set the flipper target position in radian
void RobotControl::mobile_flipper_target_callback(const std_msgs::Float32::ConstPtr &msg)
{
    motor_positions.at(2) = msg->data;
}

void RobotControl::arm_cart_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // rostopic pub /arm_target_pose geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 0.48366, y: 0.00250347, z: 0.731996}, orientation: {x: 0.22360485, y: -0.67082681, z: -0.22360485, w: -0.67081527}}}'
    geometry_msgs::Pose pos = msg->pose;
    KDL::JntArray result(num_used_joints);
    KDL::Frame cart_pos;
    cart_pos.p = KDL::Vector(pos.position.x, pos.position.y, pos.position.z);
    cart_pos.M = KDL::Rotation::Quaternion(pos.orientation.x, pos.orientation.y, pos.orientation.z, pos.orientation.w);
    result = Cart2Jnt(cart_pos,true);
    for (int i = 0; i < num_used_joints; i++)
    {
        joint_positions.at(i) = result.data(i);
    }
}

// get twist ctrl cmd and compute the wheel speed accordingly
void RobotControl::mobile_ctrl_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // rostopic pub -r 50 /cmd_vel geometry_msgs/TwistStamped '{header: {frame_id: "map"},twist: {linear: {x: 0.1,y: 0,z: 0},angular: {x: 0,y: 0,z: 0}}}'

    x_vel = msg->twist.linear.x;
    z_angVel = msg->twist.angular.z;

    // compute inverse kinematics of the tracked mobile base
    // reference: https://mp.weixin.qq.com/s/M9bIUCKfsAFQ3OGtTwgelQ

    double right_wheel_linear_vel = x_vel + lamda*0.5*wheels_distance*z_angVel;// m/s
    double left_wheel_linear_vel = x_vel - lamda*0.5*wheels_distance*z_angVel;// m/s

    double right_wheel_motor_vel = std::min(3000.0, std::max(-3000.0, -1.0 * (60.0*right_wheel_linear_vel/wheel_radius*0.5/3.141592653*gear_ratio))); // rpm
    double left_wheel_motor_vel = std::min(3000.0, std::max(-3000.0,60.0*left_wheel_linear_vel/wheel_radius*0.5/3.141592653*gear_ratio));// rpm

    std_msgs::String cmd;
    if (std::abs(right_wheel_motor_vel) > 2 && mobile_start_pub)
    {
        cmd.data = SetMotorSpeed(2, right_wheel_motor_vel);
        mobile_pub.publish(cmd);
        rate.sleep();
    }
    if (std::abs(left_wheel_motor_vel) > 2 && mobile_start_pub)
    {
        cmd.data = SetMotorSpeed(1, left_wheel_motor_vel);
        mobile_pub.publish(cmd);
        rate.sleep();
    }

}

void RobotControl::mobile_ctrl_keys_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    x_vel = msg->linear.x;
    z_angVel = msg->angular.z;

    // compute inverse kinematics of the tracked mobile base
    // reference: https://mp.weixin.qq.com/s/M9bIUCKfsAFQ3OGtTwgelQ

    double right_wheel_linear_vel = x_vel + lamda*0.5*wheels_distance*z_angVel;// m/s
    double left_wheel_linear_vel = x_vel - lamda*0.5*wheels_distance*z_angVel;// m/s

    double right_wheel_motor_vel = std::min(3000.0, std::max(-3000.0, -1.0 * (60.0*right_wheel_linear_vel/wheel_radius*0.5/3.141592653*gear_ratio))); // rpm
    double left_wheel_motor_vel = std::min(3000.0, std::max(-3000.0,60.0*left_wheel_linear_vel/wheel_radius*0.5/3.141592653*gear_ratio));// rpm

    std_msgs::String cmd;
    if (std::abs(right_wheel_motor_vel) > 2 && mobile_start_pub)
    {
        cmd.data = SetMotorSpeed(2, right_wheel_motor_vel);
        mobile_pub.publish(cmd);
        rate.sleep();
    }
    if (std::abs(left_wheel_motor_vel) > 2 && mobile_start_pub)
    {
        cmd.data = SetMotorSpeed(1, left_wheel_motor_vel);
        mobile_pub.publish(cmd);
        rate.sleep();
    }

}


/*
// PID position ctrl
void RobotControl::mobile_flipper_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    // ROS_INFO_STREAM("Writing mobile data to serial port: " << msg->header.seq);

    std_msgs::String cmd;
    // current_motor_pos = GetCurrentJntPos(std::vector<double>({msg->position.end(), msg->position.end() + 1}), false);

    // set the flipper motor cmd
    double flipper_motor_vel;
    cmd.data = SetJntPosition(2+1, motor_positions.at(2), flipper_motor_vel, msg->position.at(2), mobile_pids.at(2));
    if (std::abs(flipper_motor_vel) > 2 && mobile_start_pub)
    {
        mobile_pub.publish(cmd);
        rate.sleep();
    }

     bool turn = false;
     if (std::abs(armTargetPos.position.y-robotCurrentPose.position.y) > turnYthreshold) {turn = true;};
    // either set the wheel cmds to follow a line,
    for (int i = 0; i < num_motor-1; i++)
    {
        double motor_vel;
        cmd.data = SetJntPosition(i+1, armTargetPos.position.x, motor_vel, robotCurrentPose.position.x, mobile_pids.at(0));// only use the first pid
        if (std::abs(motor_vel) > 2 && mobile_start_pub && !turn)
        {
            mobile_pub.publish(cmd);
            rate.sleep();
        }
    }

    // or set the wheel rotation cmds.
    for (int i = 0; i < num_motor-1; i++)
    {
        double motor_vel;
        double temp;
        temp = robotCurrentPose.position.y;
        // should turn left or turn right
        if (((armTargetPos.position.y-robotCurrentPose.position.y > 0) && (i == 0)) || ((armTargetPos.position.y-robotCurrentPose.position.y < 0) && (i == 1)))
        {
            temp = armTargetPos.position.y;
        }

        cmd.data = SetJntPosition(i+1, armTargetPos.position.y, motor_vel, temp, mobile_pids.at(i));
        if (std::abs(motor_vel) > 2 && mobile_start_pub && turn)
        {
            mobile_pub.publish(cmd);
            rate.sleep();
        }
    }
}
*/
