#include "uart_comm.h"

UART_Comm::UART_Comm()
{
    bool sucess = InitSerialPort();
    if(sucess)
        ROS_INFO_STREAM("Starting to communicate with the serial port.");
    else
        ROS_ERROR_STREAM("Failed to communicate with the serial port.");

    sleep(1);
    ser.write(Hex2Ascii(kMotorEnableCode));
    ROS_INFO_STREAM("Enable all the motors.");
    sleep(1);
}

bool UART_Comm::InitSerialPort(void)
{
    try
    {
        ser.setPort(port_name);
        ser.setBaudrate(115200);
        ser.setBytesize(serial::eightbits);
        ser.setParity(serial::parity_none);
        ser.setStopbits(serial::stopbits_one);
        ser.setFlowcontrol(serial::flowcontrol_none);
        serial::Timeout to = serial::Timeout::simpleTimeout(5);// ms
        ser.setTimeout(to);
        ser.open();
        ser.flush();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return false;
    }

    if(ser.isOpen())
        ROS_INFO_STREAM("Serial Port initialized");
    else
        return false;
    return true;
}

std::string UART_Comm::AutoLineData(std::string buffer, std::string& frame, ros::Time& frame_end_time)
{
    std::string frame_header = "0A0C";

    // battery frame
    if (buffer.substr(buffer.size()-10,buffer.size()) == (frame_header+"080069"))
        battery_frame_start = true;

    if (battery_frame_start)
    {
        if (frame.size()== 20)
        {
            frame = "";
            battery_frame_start = false;
            return "";
        }
        frame += buffer.substr(buffer.size()-2, buffer.size());
        if (frame.size()== 20)
        {
            frame_end_time = ros::Time::now();
            return "battery";
        }
    }

    // velocity frame
    if (buffer.substr(buffer.size()-10,buffer.size()) == (frame_header+"120061"))
        velocity_frame_start = true;

    if (velocity_frame_start)
    {
        if (frame.size()== 40)
        {
            frame = "";
            velocity_frame_start = false;
            return "";
        }
        frame += buffer.substr(buffer.size()-2, buffer.size());
        if (frame.size()== 40)
        {
            frame_end_time = ros::Time::now();
            return "velocity";
        }
    }

    // position frame
    if (buffer.substr(buffer.size()-10,buffer.size()) == (frame_header+"0C0062"))
        position_frame_start = true;
    if (position_frame_start)
    {
        if (frame.size()== 28)
        {
            frame = "";
            position_frame_start = false;
            return "";
        }
        frame += buffer.substr(buffer.size()-2, buffer.size());
        if (frame.size()== 28)
        {
            frame_end_time = ros::Time::now();
            return "position";
        }
    }
    return "";
}

void UART_Comm::CloseSerialPort(void)
{
    ser.write(Hex2Ascii(kMotorDisableCode));
    sleep(1);
    ser.close();
}


std::string Ascii2Hex(const std::string& input)
{
    static const char hex_digits[] = "0123456789ABCDEF";

    std::string output;
    output.reserve(input.length() * 2);
    for (unsigned char c : input)
    {
        output.push_back(hex_digits[c >> 4]);
        output.push_back(hex_digits[c & 15]);
    }
    return output;
}

int Hex2Dec(std::string input)
{
    int a = std::stoi(input, 0, 16);
    return a<std::pow(2,15)?a:a-std::pow(2,16);
}

int HexValue(unsigned char hex_digit)
{
    static const signed char hex_values[256] = {
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
         0,  1,  2,  3,  4,  5,  6,  7,  8,  9, -1, -1, -1, -1, -1, -1,
        -1, 10, 11, 12, 13, 14, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, 10, 11, 12, 13, 14, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    };
    int value = hex_values[hex_digit];
    if (value == -1) throw std::invalid_argument("invalid hex digit");
    return value;
}

std::string Hex2Ascii(const std::string& input)
{
    const auto len = input.length();
    if (len & 1) throw std::invalid_argument("odd length");

    std::string output;
    output.reserve(len / 2);
    for (auto it = input.begin(); it != input.end(); )
    {
        int hi = HexValue(*it++);
        int lo = HexValue(*it++);
        output.push_back(hi << 4 | lo);
    }
    return output;
}

std::string Dec2Hex(int i, int width)
{
    std::stringstream ioss;
    std::string s_temp;
    ioss << std::setiosflags(std::ios::uppercase) << std::hex << i;
    ioss >> s_temp;

    if(width > s_temp.size())
    {
        std::string s_0(width - s_temp.size(), '0');
        s_temp = s_0 + s_temp;
    }
    std::string s = s_temp.substr(s_temp.length() - width, s_temp.length());
    return s;
}

std::string BCCXOR(const std::string& input)
{
    int len = input.size() / 2;
    int temp = 0;
    for (int i = 0; i < len; i++)
    {
        temp ^= Hex2Dec(input.substr(2*i, 2));
    }
    //std::cout << "result: " << input+Dec2Hex(temp) << std::endl;
    return input + Dec2Hex(temp);
}
