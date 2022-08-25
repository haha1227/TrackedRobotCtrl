#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#define MANUAL_MODE 0
#define AUTO_MODE 1
#define KEYBOARD_MODE 2

class JoystickControl
{
private:
    int mode = MANUAL_MODE;

    // The car parameters
    float speed, steering_angle;
    float max_speed, max_steering_angle;
    float update_cmd_rate;
    float wheelbase;

    // subscribed current of the arm
    geometry_msgs::PoseStamped arm_current_cart_pose;
    // publish the target pose of the arm
    geometry_msgs::PoseStamped arm_target_cart_pose;


    // Joystick parameters
    int joy_speed_axis, joy_angle_axis, joy_manual_button, joy_auto_button, joy_keyboard_button;
    int joy_arm_x_axis, joy_arm_y_axis, joy_arm_z_axis;
    float joy_max_speed;

    // A ROS node
    ros::NodeHandle nh;

    // A timer to update the pose
    ros::Timer update_pose_timer;

    // Listen for drive and joystick commands
    ros::Publisher cmd_pub;
    ros::Publisher cmd_arm_cart_pub;

    ros::Subscriber joy_sub;
    ros::Subscriber auto_sub;
    ros::Subscriber keyboard_sub;
    ros::Subscriber arm_current_cart_sub;

public:
    JoystickControl()
    {
        // Initialize the node handle
        nh = ros::NodeHandle("~");

        // Get joystick parameters
        bool joy;
        nh.getParam("joy", joy);
        nh.getParam("joy_max_speed", joy_max_speed);
        nh.getParam("joy_speed_axis", joy_speed_axis);
        nh.getParam("joy_angle_axis", joy_angle_axis);
        nh.getParam("joy_auto_button", joy_auto_button);
        nh.getParam("joy_manual_button", joy_manual_button);
        nh.getParam("joy_keyboard_button", joy_keyboard_button);

        nh.getParam("update_pose_rate", update_cmd_rate);

        // Get the car parameters
        nh.getParam("wheelbase", wheelbase);
        nh.getParam("max_speed", max_speed);
        nh.getParam("max_steering_angle", max_steering_angle);

        // Get the arm pose increments
        nh.getParam("joy_arm_x_axis", joy_arm_x_axis);
        nh.getParam("joy_arm_y_axis", joy_arm_y_axis);
        nh.getParam("joy_arm_z_axis", joy_arm_z_axis);


        // Start a timer to output the pose
        update_pose_timer = nh.createTimer(ros::Duration(update_cmd_rate), &JoystickControl::update_cmd, this);

        // If the joystick is enabled
        if (joy)
        {
            // Start a subscriber to listen to joystick commands
            joy_sub = nh.subscribe("/joy_nuc", 1, &JoystickControl::joy_callback, this);
        }

        auto_sub = nh.subscribe("/car/ackermann", 1, &JoystickControl::auto_callback, this);
        keyboard_sub = nh.subscribe("/car/keyboard", 1, &JoystickControl::keyboard_callback, this);

        cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 10);

        cmd_arm_cart_pub = nh.advertise<geometry_msgs::PoseStamped>("/arm_target_pos", 10);
        arm_current_cart_sub = nh.subscribe("/arm_current_pose", 1, &JoystickControl::arm_callback, this);
    }

    void update_cmd(const ros::TimerEvent &)
    {
        // publish ackermann messages
        geometry_msgs::TwistStamped cmd_msg;

        cmd_msg.twist.linear.x = speed;
        cmd_msg.twist.angular.z = steering_angle;

        cmd_pub.publish(cmd_msg);
        if (arm_target_cart_pose.pose.position.x != 0.0 &&
            arm_target_cart_pose.pose.position.y != 0.0 &&
            arm_target_cart_pose.pose.position.z != 0.0)
        {
            cmd_arm_cart_pub.publish(arm_target_cart_pose);
        }
    }

    void set_speed(float speed_)
    {
        speed = std::min(std::max(speed_, -max_speed), max_speed);
    }

    void set_steering_angle(float steering_angle_)
    {
        steering_angle = std::min(std::max(steering_angle_, -max_steering_angle), max_steering_angle);
    }

    void keyboard_callback(const geometry_msgs::Twist& cmd_keyboard)
    {
        if (mode == KEYBOARD_MODE)
        {
            set_speed(cmd_keyboard.linear.x);
            set_steering_angle(cmd_keyboard.angular.z);
        }
    }

    void auto_callback(const ackermann_msgs::AckermannDriveStamped& cmd_auto)
    {
        if (mode == AUTO_MODE)
        {
            set_speed(cmd_auto.drive.speed);
            set_steering_angle(cmd_auto.drive.steering_angle);
        }
    }

    void joy_callback(const sensor_msgs::Joy &msg)
    {
        if (msg.buttons[joy_manual_button])
        {
            mode = MANUAL_MODE;
            ROS_INFO("Switched to manual control.");
        }
        else if (msg.buttons[joy_auto_button])
        {
            mode = AUTO_MODE;
            ROS_INFO("Switched to auto control.");
        }
        else if (msg.buttons[joy_keyboard_button])
        {
            mode = KEYBOARD_MODE;
            ROS_INFO("Switched to keyboard control.");
        }

        if (mode == MANUAL_MODE)
        {
            speed = joy_max_speed * msg.axes[joy_speed_axis];
            steering_angle = max_steering_angle * msg.axes[joy_angle_axis];

            float scale = 1.0;
            arm_target_cart_pose.pose.position.x = arm_current_cart_pose.pose.position.x + scale * msg.axes[joy_arm_x_axis];
            arm_target_cart_pose.pose.position.y = arm_current_cart_pose.pose.position.y + scale * msg.axes[joy_arm_y_axis];
            arm_target_cart_pose.pose.position.z = arm_current_cart_pose.pose.position.z + scale * msg.axes[joy_arm_z_axis];

            arm_target_cart_pose.pose.orientation = arm_current_cart_pose.pose.orientation;

        }
    }

    void arm_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (mode == MANUAL_MODE)
        {
            arm_current_cart_pose.header = msg->header;
            arm_current_cart_pose.pose = msg->pose;
        }
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "joystick_control");
    JoystickControl jc;
    ros::spin();
    return 0;
}
