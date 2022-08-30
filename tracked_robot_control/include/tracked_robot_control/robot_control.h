#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <random>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/velocityprofile.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "pilz_industrial_motion_planner/trajectory_generator_ptp.h"
#include "pilz_industrial_motion_planner/trajectory_generator_lin.h"
#include "pilz_industrial_motion_planner/trajectory_generator_circ.h"
//#include <eigen3/Eigen/Core>
#include "MiniPID.h"
#include "uart_comm.h"


class RobotControl
{
public:
    RobotControl();
    ~RobotControl(){}

    // arm is controlled based on joint positions.
    // mobile base is vel control.
    // define position control or vel control only for the flipper.
    std::string flipper_ctrl_type = "position";

    // check the urdf/track_robot.urdf, or you can display the robot with the launch/display.launch file
    // we need to set whcih joint can be controlled, here are from the chain_start to chain_end
    std::string chain_start = "base_link";
    std::string chain_end = "link4";
    std::string urdf_param = "/robot_description";
    double timeout = 1;
    const double error = 1e-3;

    KDL::Chain robot_chain;
    // lower joint limits, upper joint limits
    KDL::JntArray ll, ul;

    void InitKinematicsSolver(void);

    // joint coordinates to Cartesian coordinates, forward kinematics
    KDL::Frame Jnt2Cart(KDL::JntArray joint_positions);
    // Cartesian coordinates to joint coordinates, inverse kinematics
    KDL::JntArray Cart2Jnt(KDL::Frame cart_frame, bool random_seed = false);

    // convert the 6-D pose from the KDL::Frame format to std::vector
    std::vector<double> Frame2Vector(KDL::Frame frame);
    KDL::Frame Vector2Frame(std::vector<double> v);
    // bool arm is not used
    KDL::JntArray GetCurrentJntPos(std::vector<double> input, bool arm = true);
    KDL::JntArray current_jnt_pos;
    KDL::JntArray current_motor_pos;
    void PrintFrame(KDL::Frame input);

    // the number of the used joints for the arm kinematics, here it is equal to 5 because our arm has 5 position sensors.
    int num_used_joints;
    // home_jnt means the position of each joint is equal to 0
    KDL::JntArray home_jnt;

    ros::Rate rate = ros::Rate(100);
    ros::Subscriber robotCurrentOdometry_sub;

    ros::Subscriber demo_sub;
    ros::Subscriber arm_sub;
    ros::Subscriber arm_cart_pos_sub;

    ros::Subscriber mobile_flipper_sub;
    ros::Subscriber mobile_flipper_target_sub;

    ros::Subscriber mobile_ctrl_sub;
    ros::Subscriber mobile_ctrl_keys_sub;
    ros::Publisher mobile_pub;
    ros::Publisher arm_cart_pub;

    void demo_callback(const std_msgs::Bool::ConstPtr& msg);
    void arm_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void mobile_flipper_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void mobile_flipper_target_callback(const std_msgs::Float32::ConstPtr& msg);

    void arm_cart_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void mobile_ctrl_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void mobile_ctrl_keys_callback(const geometry_msgs::Twist::ConstPtr& msg);

    bool arm_start_pub = false;
    bool flipper_start_pub = false;
    bool mobile_start_pub = false;
    geometry_msgs::Pose armTargetPos;
    double turnYthreshold = 0.1; // unit: m
    double x_vel = 0.0;
    double z_angVel = 0.0;

    std::string cmd_header = "0A0C";
    int num_joints = 6;
    int num_motor = 3;
    int couter_pub = 0;
    std::vector<int> joint_velocities{std::vector<int>(num_joints,0)}; // for arm, unit rpm
    std::vector<int> motor_velocities{std::vector<int>(num_motor,0)}; // for mobile chasis

    std::vector<double> joint_positions{std::vector<double>(num_joints,0)}; // for arm, unit radians
    std::vector<double> motor_positions{std::vector<double>(num_motor,0)}; // for mobile chasis

    std::string SetJntPosition(int id, double goal_pos, double &vel, double current_pos, MiniPID pid);
    std::string SetMotorSpeed(int id, double speed);

    std::vector<MiniPID> joint_pids, mobile_pids;
    double rpm2radian = 0.104719755;
    // 查看‘额定转速’和'总减速比'， 在 'LZ_TRACK履带机器人开发手册.pdf'中的3.3 关节电机减数比和扭矩 
    // 这里把电机的转速转换成关节的转速，每个关节的最大转速通过8.3这个系数进行了限制，也可以是其他数。200， 200，100，500等也是转速，可以人为更改
    std::vector<double> joint_vel_abs_limits = {rpm2radian*200*8.3/1520, rpm2radian*200*8.3/1660,
                                                rpm2radian*100*8.3/1328, rpm2radian*500*8.3/7128, rpm2radian*500*8.3/1625, 500}; // for remote controller
    //std::vector<double> joint_vel_abs_limits = {3.549, 3.248, 5.334, 1.982, 8.694, 2.23};   // for pc
    // 每个关节的总减速比
    std::vector<int> joint_vel_abs_limits_ratio = {1520, 1660, 1328, 7128, 1625, 6336};   // ratio of motor speed over joint speed

    // 关节加速度的获取是ptp planning中的难点，这里我是通过不断试验得到的。
    std::vector<double> joint_acc_abs_limits = {30 ,34.9, 10.4, 17.4, 1, 2};
    std::vector<double> joint_dec_abs_limits = {30,34.9, 10.4, 17.4, 1, 2};

    // PD control for the arm joints, the 6th joint is not used. 实验得到
    std::vector<int> joint_p = {500, 500, 500, 500, 500, 2000};
    std::vector<int> joint_d = {0 ,0, 0, 0, 0, 200};

    std::vector<int> mobile_vel_abs_limits = {3600, 3600, 2800};
    // std::vector<int> mobile_vel_abs_limits = {1000, 1000, 100};
    // PD control for the mobile joints,
    std::vector<int> mobile_p = {500, 500, 500};
    std::vector<int> mobile_d = {0, 0, 0};
    void InitAllPID();

    // compute inverse kinematics of the tracked mobile base
    // reference: https://mp.weixin.qq.com/s/M9bIUCKfsAFQ3OGtTwgelQ
    double lamda = 1.1;
    double gear_ratio = 40.0;
    double wheel_radius = 0.1+0.01; // m
    double wheels_distance = 0.473; // m

    // check the page:
    // https://ros-planning.github.io/moveit_tutorials/doc/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html#the-ptp-motion-command
    void planPTP(trajectory_msgs::JointTrajectory& joint_trajectory,
                 double velocity_scaling_factor = 1.0,
                 const double acceleration_scaling_factor = 1,
                 double sampling_time = 0.04);
    const double MIN_MOVEMENT = 0.09;

    trajectory_msgs::JointTrajectory joint_trajectory;
    std::string ctrl_style = "ptp"; // pid, ptp, line, or circle.
    bool start_planning = true;
    int nth_waypoints = 1;
    bool demo = false;

private:
    std::shared_ptr<TRAC_IK::TRAC_IK> ik_solver_;
    std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solverKDL_;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_solverVel_;

    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::shared_ptr<KDL::ChainFkSolverVel> fk_solverVel_;
    std::shared_ptr<KDL::ChainFkSolverVel_recursive> fk_solverVel_rec_;

    std::shared_ptr<KDL::ChainJntToJacSolver> jac_solverJ_;

    std::vector<std::uniform_real_distribution<double> > joint_dist_;
    std::default_random_engine rand_eng_;

};


#endif // ROBOT_CONTROL_H
