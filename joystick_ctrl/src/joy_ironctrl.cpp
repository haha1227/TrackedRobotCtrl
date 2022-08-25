#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
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

    // Joystick parameters
    int joy_speed_axis, joy_angle_axis, joy_manual_button, joy_auto_button, joy_keyboard_button;
    float joy_max_speed;

    // A ROS node
    ros::NodeHandle nh;

    // A timer to update the pose
    ros::Timer update_pose_timer;

    // Listen for drive and joystick commands
    ros::Publisher cmd_pub;
    ros::Subscriber joy_sub;
    ros::Subscriber auto_sub;
    ros::Subscriber keyboard_sub;

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

    // Start a timer to output the pose
    update_pose_timer = nh.createTimer(ros::Duration(update_cmd_rate), &JoystickControl::update_cmd, this);

    // If the joystick is enabled
    if (joy)
      // Start a subscriber to listen to joystick commands
      joy_sub = nh.subscribe("/joy", 1, &JoystickControl::joy_callback, this);

    auto_sub = nh.subscribe("/car/ackermann", 1, &JoystickControl::auto_callback, this);
    keyboard_sub = nh.subscribe("/car/keyboard", 1, &JoystickControl::keyboard_callback, this);

    cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/cmd_acm", 10);
  }

  void update_cmd(const ros::TimerEvent &)
  {
    // publish ackermann messages
    ackermann_msgs::AckermannDriveStamped acm_msg;
    acm_msg.header.frame_id = "/map_lio";
    acm_msg.header.stamp = ros::Time::now();
    acm_msg.header.seq = 0;

    acm_msg.drive.speed = speed;
    acm_msg.drive.steering_angle = steering_angle;

    cmd_pub.publish(acm_msg);
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
      if (speed)
        set_steering_angle(atan(cmd_keyboard.angular.z * wheelbase / speed));
    }
  }

  void auto_callback(const ackermann_msgs::AckermannDriveStamped& cmd_auto)
  {
    if (mode == AUTO_MODE)
    {
      set_speed(cmd_auto.drive.speed);
      if (speed)
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
        if (speed)
          steering_angle = max_steering_angle * msg.axes[joy_angle_axis];
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
