#ifndef UART_COMM_H
#define UART_COMM_H
#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <stdexcept>

class UART_Comm
{
public:
    UART_Comm();
    virtual ~UART_Comm(){}

    serial::Serial ser;
    // the named port
    std::string port_name = "/dev/uart0";
    // the size of the past two frames. Each frame has a 14-D battery info + a 24-D motor speed info + a 16-D motor position info
    // check the chapter 3.	数据反馈 in the document 'LZ_TRACK履带机器人串口通信协议.docx'
    // we retrieve the past two frames in case some data is missing
    const int kBufferSize = 216;//(14+24+16)*2*2=108*2=216

    bool InitSerialPort(void);
    
    // auto segment the last full frame into three different sub-frames for battery info, velocity info, and position info
    std::string AutoLineData(std::string buffer, std::string& frame, ros::Time& frame_end_time);
    void CloseSerialPort(void);

    const float kDegree2Radian = 0.01745329;
    // the starting hex number to enable all motors
    // check the part 2.2	底盘电机供电控制命令 in 'LZ_TRACK履带机器人串口通信协议.docx'
    const std::string kMotorEnableCode = "0A0C0100010101";
    // the starting hex number to disable all motors
    const std::string kMotorDisableCode = "0A0C0100010000";

private:

    bool battery_frame_start = false;
    bool velocity_frame_start = false;
    bool position_frame_start = false;
};

std::string Ascii2Hex(const std::string& input);
int Hex2Dec(std::string input);
int HexValue(unsigned char hex_digit);
std::string Hex2Ascii(const std::string& input);
std::string Dec2Hex(int i, int width = 2);
// 校验和
std::string BCCXOR(const std::string& input);

#endif // UART_COMM_H
