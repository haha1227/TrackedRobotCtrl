#include "tracked_robot_control/robot_control.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracked_robot_control");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    
    RobotControl robot_control;

    nh_local.param("arm_ctrl", robot_control.arm_start_pub, false);
    nh_local.param("flipper_ctrl", robot_control.flipper_start_pub, false);
    nh_local.param("mobile_ctrl", robot_control.mobile_start_pub, false);


    // here assign the abs vel
//    robot_control.joint_velocities.at(0) = 10;
//    robot_control.joint_velocities.at(1) = 10;
//    robot_control.joint_velocities.at(2) = 10;
//    robot_control.joint_velocities.at(3) = 10;
//    robot_control.joint_velocities.at(4) = 10;
    robot_control.demo = true;
    if (robot_control.demo)
    {
      robot_control.arm_start_pub = true;
      robot_control.joint_positions.at(0) = 0.25;
      robot_control.joint_positions.at(1) = -0.3;
      robot_control.joint_positions.at(2) = 0.1;
      robot_control.joint_positions.at(3) = 0.2;
      robot_control.joint_positions.at(4) = 0.8;
    }else
    {
      robot_control.arm_start_pub = true;
      robot_control.joint_positions.at(0) = 0.0;
      robot_control.joint_positions.at(1) = -0.0;
      robot_control.joint_positions.at(2) = 0.0;
      robot_control.joint_positions.at(3) = 0.0;
      robot_control.joint_positions.at(4) = 0.0;
    }

//    robot_control.motor_positions.at(2) = 1.83;



    robot_control.demo_sub = nh.subscribe("demo_start", 10, &RobotControl::demo_callback, &robot_control);
    robot_control.arm_sub = nh.subscribe("arm_info", 10, &RobotControl::arm_callback, &robot_control);
    robot_control.arm_cart_pos_sub = nh.subscribe("arm_target_pos", 10, &RobotControl::arm_cart_pos_callback, &robot_control);

    //set the flipper pos, now comment it becasue it's broken
    // robot_control.mobile_flipper_sub = nh.subscribe("mobile_info", 10, &RobotControl::mobile_flipper_callback, &robot_control);
    // robot_control.mobile_flipper_target_sub = nh.subscribe("flipper_target_pos", 10, &RobotControl::mobile_flipper_target_callback, &robot_control);


    robot_control.mobile_ctrl_sub = nh.subscribe("cmd_vel", 10, &RobotControl::mobile_ctrl_callback, &robot_control);
    robot_control.mobile_ctrl_keys_sub = nh.subscribe("cmd_vel_keys", 10, &RobotControl::mobile_ctrl_keys_callback, &robot_control);

    robot_control.mobile_pub = nh.advertise<std_msgs::String>("motor_command", 10);
    robot_control.arm_cart_pub = nh.advertise<geometry_msgs::PoseStamped>("arm_current_pose", 10);

    ros::spin();

//    while (ros::ok()) {
//        if (robot_control.current_jnt_pos.rows()>0)
//        {
//            std::cout << "current: " << robot_control.current_jnt_pos.data(0) << " "
//                      << robot_control.current_jnt_pos.data(1) << " "
//                      << robot_control.current_jnt_pos.data(2) << " "
//                      << robot_control.current_jnt_pos.data(3) << std::endl;

//            KDL::Frame cart_pos = robot_control.Jnt2Cart(robot_control.current_jnt_pos);
//            robot_control.PrintFrame(cart_pos);
//            // home pose
//            // Origin: 0.48366,-0.00250347,0.731996
//            // RPY: -2.81984,1.57078,-2.81985
//            // xyzw:

//            // current: 1.00104 0.293749 0.301761 0.985396
//            // KDL FK Success
//            // Origin: 0.0583015,0.481288,0.200664
//            // RPY: -3.14159,-0.0101204,-2.14056

//            KDL::JntArray result(robot_control.num_used_joints);
//            cart_pos.p = KDL::Vector(0.0583015,0.481288,0.200664);
//            cart_pos.M = KDL::Rotation::Quaternion(-0.478985,0.877692,-0.00726987,-0.0133136);
//            //result=robot_control.Cart2Jnt(cart_pos,true);

//            //for (int i = 0; i < robot_control.num_used_joints; i++)
//            //    robot_control.joint_positions.at(i) = result.data(i);

//        }
//        ros::spinOnce();
//    }



/*
    KDL::JntArray jointpositions(robot_control.num_used_joints);

    // Assign some values to the joint positions
    for(unsigned int i=0;i<robot_control.num_used_joints;i++){
        float myinput;
        printf ("Enter the position of joint %i: ",i);
        scanf ("%e",&myinput);
        jointpositions(i)=(double)myinput;
    }

    // Create the frame that will contain the results
    KDL::Frame cartpos = robot_control.Jnt2Cart(jointpositions);

    robot_control.PrintFrame(cartpos);

    cartpos.p = KDL::Vector(-0.245715,0.00671667,0.00310352);
    cartpos.M = KDL::Rotation::Quaternion(0.362132,-0.662884,0.314178,0.575098);

    robot_control.PrintFrame(cartpos);

    // cartpos.p += KDL::Vector(2.0, 0, 0);
    KDL::JntArray result(robot_control.num_used_joints);
    result=robot_control.Cart2Jnt(cartpos, true);

    printf("%s \n","TRAC IK Success");
    for(unsigned int i = 0; i < robot_control.num_used_joints; i++)
        std::cout << result(i) << " ";
    std::cout << std::endl;
*/
    ros::shutdown();
    return 0;
}
