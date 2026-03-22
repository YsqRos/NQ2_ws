#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
using namespace serial;
serial::Serial sp;
 double RobotV_ = 0;
 double YawRate_ = 0;
 
 const unsigned short crc16_list[256]={ /*CRC 余式表*/
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

//数据头、数据尾
const unsigned char ender[1] = {0x0d};
const unsigned char header[5] = {0xaa, 0x20, 0x20, 0x3a, 0x08};
// const double Circumference = 82.31;  //轮子周长
const double Circumference = 785.398;  //轮子周长  半径250mm



//发送左右轮速控制速度共用体,传感器的X，Z，Angle
union sendData
{
	short d;
	unsigned char data[2];
}leftVelSet1,rightVelSet1,leftVelSet2,rightVelSet2;

const double ROBOT_LENGTH = 74.5;  //两轮之间的半径长度mm  实际间距为496.6  乘以一个参数来近似成二轮差速车进行处理

unsigned short getCrc16(unsigned char *ptr, unsigned char len)
{
    unsigned short crc = 0;
    unsigned char da;
    crc = 0;
    while(len-- != 0)
    {
        da = (unsigned char)(crc/256);
        crc<<=8;
        crc^=crc16_list[da^*ptr];
        ptr++;
    }
    return crc;
}

void serialInit()
{
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    sp.setParity(serial::parity_none);
    sp.setFlowcontrol(serial::flowcontrol_none);
    sp.setStopbits(serial::stopbits_one);
    sp.setBytesize(serial::eightbits);
    //串口设置timeout
    sp.setTimeout(to);
}
    unsigned char buf[16] = {0};//
void writeSpeed(double RobotV, double YawRate)
{

    int i, length = 0;
    double r = RobotV / YawRate;//mm
    unsigned short Crc16Value;
    //  RobotV=RobotV/10.0;YawRate=YawRate/10.0;
    // 计算左右轮期望速度
    // if(RobotV == 0)      //转   由于电机安装的原因。当车轮速度一致时，自然左右轮速度相反
    // {
    //     leftVelSet1.d  = (short)((YawRate * ROBOT_LENGTH * 0.5) * 100 /10.0);//mm/s
    //     rightVelSet1.d = (short)((YawRate * ROBOT_LENGTH * 0.5) * 100 /10.0);//mm/s
    //     leftVelSet2.d  = (short)((YawRate * ROBOT_LENGTH * 0.5)*100.0 /10.0);//mm/s
    //     rightVelSet2.d = (short)((YawRate * ROBOT_LENGTH * 0.5)*100.0 /10.0);//mm/s
    // } 
    // else if(YawRate == 0)//直线
    // {
    //     leftVelSet1.d  = (short)((RobotV / Circumference) * 100);//  r/min
    //     rightVelSet1.d = (short)( - (RobotV / Circumference) * 100);
    //     leftVelSet2.d  = (short)((RobotV / Circumference) * 100);//  r/min
    //     rightVelSet2.d = (short)( - (RobotV / Circumference) * 100);
    //     // leftVelSet1.d  = (short)(10 * 100);//mm/s
    //     // rightVelSet1.d = (short)(-10 * 100);
    //     // rightVelSet2.d  = (short)(-10 * 100);//mm/s
    //     // leftVelSet2.d = (short)(10 * 100);
    // }
    // else                //速度不一致
    // {
    //     leftVelSet1.d  = (short)((RobotV - YawRate * (ROBOT_LENGTH * 0.5)) * 100);
    //     rightVelSet1.d = (short)( - ((RobotV + YawRate * (ROBOT_LENGTH * 0.5)) * 100));
    //     leftVelSet2.d  = (short)((RobotV - YawRate * (ROBOT_LENGTH * 0.5)) * 100);
    //     rightVelSet2.d = (short)( - ((RobotV + YawRate * (ROBOT_LENGTH * 0.5)) * 100));
    // }
    // leftVelSet1.d=5000;
    //    leftVelSet2.d=5000;
    //       rightVelSet1.d=-5000;
    //          rightVelSet2.d=-5000;
        leftVelSet1.d  = (short)((2*RobotV+YawRate*ROBOT_LENGTH/1000.0)*60.0/(2.0*Circumference/1000.0)*100.0);
        rightVelSet1.d = (short)((-1.0)*(2*RobotV-YawRate*ROBOT_LENGTH/1000.0)*60.0/(2.0*Circumference/1000.0)*100.0);
        leftVelSet2.d  = (short)((2*RobotV+YawRate*ROBOT_LENGTH/1000.0)*60.0/(2.0*Circumference/1000.0)*100.0);
        rightVelSet2.d = (short)((-1.0)*(2*RobotV-YawRate*ROBOT_LENGTH/1000.0)*60.0/(2.0*Circumference/1000.0)*100.0);


    // 设置消息头
    for(i = 0; i < 5; i++)
        buf[i] = header[i];             //buf[0] buf[1] buf[2] buf[3] buf[4]
    
    // 设置机器人左右轮速度
    for(i = 0; i < 2; i++)
    {
        buf[i + 5] = leftVelSet1.data[i];  //buf[5] buf[6]
        buf[i + 7] = rightVelSet1.data[i]; //buf[7] buf[8]
        buf[i + 9] = rightVelSet2.data[i];  //buf[9] buf[10]
        buf[i + 11] = leftVelSet2.data[i]; //buf[11] buf[12]
    }

    // 设置校验值、消息尾
    Crc16Value = getCrc16(buf, 13);
    buf[13] = (unsigned char)Crc16Value;   // buf[13]
    buf[14] = (unsigned char)(Crc16Value >> 8);  // buf [14]
    buf[15] = ender[0];   //buf[15]
    sp.write(buf,16);
    

}
// 速度控制消息的回调函数
void cmdCallback(const geometry_msgs::Twist& msg)
{
	RobotV_ = msg.linear.x ;
	YawRate_ = msg.angular.z;
}
bool spinOnce(double RobotV, double YawRate)
{
    // 下发机器人期望速度
    writeSpeed(RobotV, YawRate);

    
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port2");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
    serialInit();
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB1 is opened.");
    }
    else
    {
        return -1;
    }
    ros::Subscriber sub = n.subscribe("cmd_vel", 50, cmdCallback);
    
    ros::Rate loop_rate(5);
    while(ros::ok())
    {
        // unsigned char buf[16] = {0xAA, 0x20, 0x20, 0x3A, 0x08, 0xE8, 0x03, 0x18, 0xFC, 0x18, 0xFC, 0xE8, 0x03, 0xDE, 0x79, 0x0D};
        // unsigned char buf[16] = {0xAA, 0x20, 0x20, 0x3A, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0xCE, 0x0D};
        // sp.write(buf,16);
        ros::spinOnce();
        spinOnce(RobotV_,YawRate_);
        loop_rate.sleep();
    }
    
    //关闭串口
    sp.close();
 
    return 0;
}
