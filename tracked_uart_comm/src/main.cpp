#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/JointState.h>
#include "uart_comm.h"

UART_Comm ser_port;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port: " << msg->data);
    ser_port.ser.write(Hex2Ascii(msg->data));
}

int main (int argc, char** argv){
    ros::init(argc, argv, "tracked_uart_comm");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("motor_command", 10, write_callback);
    ros::Publisher battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery_info", 10);
    ros::Publisher arm_pub = nh.advertise<sensor_msgs::JointState>("arm_info", 10);
    ros::Publisher mobile_pub = nh.advertise<sensor_msgs::JointState>("mobile_info", 10);

    sensor_msgs::BatteryState battery_info;
    sensor_msgs::JointState arm_info;
    sensor_msgs::JointState mobile_info;

    std::string pc_buffer;// store the past kBufferSize==216 word
    std::string frame;// one frame segmented from the pc_buffer for battery, velocity, or position.

    int error_count = 0;
    while(ros::ok()){
        //ros::spinOnce();

        if(ser_port.ser.available())
        {
            std_msgs::String result;
            result.data = Ascii2Hex(ser_port.ser.readline(2, ""));
            ser_port.ser.flush();

            pc_buffer.append(result.data);
            if (pc_buffer.size()>ser_port.kBufferSize)
            {
                pc_buffer.erase(0,2);
                ros::Time frame_end_time;
                std::string feedback = ser_port.AutoLineData(pc_buffer, frame, frame_end_time);

                if (feedback == "battery")
                {
                    battery_info.header.stamp = frame_end_time;
                    battery_info.percentage = Hex2Dec(frame.substr(2, 2)) * 0.01;
                    battery_info.current = Hex2Dec(frame.substr(8, 2) + frame.substr(6, 2)) * 0.01;// unit: A
                    battery_info.voltage = Hex2Dec(frame.substr(12, 2) + frame.substr(10, 2)) * 0.01;// unit: V

                    if (battery_info.current == 0.0)
                    {
                        error_count++;
                        if (error_count == 400)
                        {
                            ROS_ERROR_STREAM("Please switch the SA button to 'up' status in the romote controller.");
                            error_count = 0;
                        }
                    }

                    battery_pub.publish(battery_info);

                }
                else if (feedback == "velocity")
                {
                    // add 9ms to bridge the gap between 'velocity' timestamp and 'position' timestamp.
                    mobile_info.header.stamp = frame_end_time + ros::Duration(0.009);
                    arm_info.header.stamp = frame_end_time + ros::Duration(0.009);

                    mobile_info.name.resize(3);
                    mobile_info.velocity.resize(3);

                    arm_info.name.resize(6);
                    arm_info.velocity.resize(6);

                    for (int i = 0; i < 3; i++)
                    {
                        mobile_info.name.at(i) = "motor"+std::to_string(i);
                        mobile_info.velocity.at(i) = Hex2Dec(frame.substr(4+4*i, 2) + frame.substr(2+4*i, 2));
                    }
                    for (int i = 0; i < 6; i++)
                    {
                        arm_info.name.at(i) = "joint"+std::to_string(i);
                        arm_info.velocity.at(i) = Hex2Dec(frame.substr(12+4+4*i, 2) + frame.substr(12+2+4*i, 2));
                    }
                }
                else if (feedback == "position")
                {
                    // convert the pos from encode values to radian
                    mobile_info.position = { 0.0, 0.0, -1.0 * (Hex2Dec(frame.substr(4, 2) + frame.substr(2, 2)) - 602.0) / 11.6 * ser_port.kDegree2Radian};
                    if (mobile_info.position.at(2) > M_PI)
                        mobile_info.position.at(2) -= 2.0*M_PI;
                    arm_info.position.resize(6);

                    for (int i = 0; i < 5; i++)
                    {
                        // convert the pos from encode values to radian, which changes the range from [0, 360] degree to [-180,  180]
                        arm_info.position.at(i) = Hex2Dec(frame.substr(8+4*i, 2) + frame.substr(6+4*i, 2));
                        if (i==0)
                        {
                            arm_info.position.at(i) = (5250-arm_info.position.at(i))/45.75 * ser_port.kDegree2Radian;
                        }
                        if (i==1)
                        {
                            arm_info.position.at(i) = (14980-arm_info.position.at(i))/45.75 * ser_port.kDegree2Radian;
                        }
                        if (i==2)
                        {
                            arm_info.position.at(i) = 1.0 * (3.141592653 - (-6350+arm_info.position.at(i))/45.75 * ser_port.kDegree2Radian);
                        }
                        if (i==3)
                        {
                            arm_info.position.at(i) = -1.0 * (5925+45.75*2-arm_info.position.at(i))/45.75 * ser_port.kDegree2Radian;
                        }
                        if (i==4)
                        {
                            arm_info.position.at(i) = (arm_info.position.at(i) - 7889)/45.75 * ser_port.kDegree2Radian;
                        }

                        if (arm_info.position.at(i) > M_PI)
                            arm_info.position.at(i) -= 2.0*M_PI;
                    }
                    arm_pub.publish(arm_info);
                    mobile_pub.publish(mobile_info);
                }
            }
        }
        ros::spinOnce();
    }

    pc_buffer.clear();
    frame.clear();
    ser_port.CloseSerialPort();
    ros::shutdown();

    return 0;
}
