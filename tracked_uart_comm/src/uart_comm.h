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
    std::string port_name = "/dev/uart0";
    const int kBufferSize = 216;//(14+24+16)*2*2=108*2=216

    bool InitSerialPort(void);
    std::string AutoLineData(std::string buffer, std::string& frame, ros::Time& frame_end_time);
    void CloseSerialPort(void);

    const float kDegree2Radian = 0.01745329;
    const std::string kMotorEnableCode = "0A0C0100010101";
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
std::string BCCXOR(const std::string& input);

#endif // UART_COMM_H
