#pragma once
#include <cstdio>
#include <string>
#include <cstring>
// #include <spdlog/fmt/bin_to_hex.h>

#ifdef _WIN32
#define NOMINMAX
#define NOGDI
#include <WinSock2.h>
#include <windows.h>
#else  // linux
#include <stdlib.h>    
#include <unistd.h>    
#include <sys/types.h>  
#include <sys/stat.h>   
#include <fcntl.h>      
#include <termios.h>  
#include <iostream>
#include <errno.h>
#endif // _WIN32

/*#pragma pack(push, 1)
struct FrameHeaderWithCmd
{
    uint8_t SOF;         ///< 起始字节(0xA5)
    uint16_t DataLength; ///< 数据长度
    uint8_t Seq;         ///< 包序号
    uint8_t CRC8;        ///< 帧头CRC8校验
    uint16_t CmdId;      ///< 命令码
};

struct interactive_header_data_t
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
};

struct FrameTail
{
    uint16_t CRC16; ///< 整包校验
};

struct match_status_t
{
    uint8_t game_type : 4;     ///< 比赛类型：1:机甲大师赛 2:单项赛 3:人工智能挑战赛
    uint8_t game_progress : 4; ///< 当前比赛阶段：0:未开始比赛 1:准备阶段 2:自检阶段 3:5s倒计时 4:对战中 5:比赛结算中
    uint16_t stage_remain_time; ///< 当前阶段剩余时间(s)
    uint64_t sync_timestamp;
};

struct robot_status_t
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;
    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;
    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
};

struct graphic_data_struct_t
{
    uint8_t graphic_name[3];
    uint32_t operate_type:3;
    uint32_t graphic_type:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t radius:10;
    uint32_t end_x:11;
    uint32_t end_y:11;
};
#pragma pack(pop)
*/

class SerialPort
{
public:
    SerialPort()  {}
    ~SerialPort() {}
    /**
    *	@Brief:		打开串口,成功返回true，失败返回false
    *	@Param		portname(串口名): 在Windows下是"COM1""COM2"等，在Linux下是"/dev/ttyS1"等
    *	@Param		baudrate(波特率): 9600、19200、38400、43000、56000、57600、115200
    *	@Param		parity(校验位): 0为无校验，1为奇校验，2为偶校验，3为标记校验（仅适用于windows)
    *	@Param		databit(数据位): 4-8(windows),5-8(linux)，通常为8位
    *	@Param		stopbit(停止位): 1为1位停止位，2为2位停止位,3为1.5位停止位
    *	@Param		synchronizeflag(同步、异步,仅适用与windows): 0为异步，1为同步
    *	@Return:	真或假
    */
    bool OpenPort(const std::string& portname, int baudrate, char parity, char databit, char stopbit, char synchronizeflag = 1);

    //发送数据或写数据，成功返回发送数据长度，失败返回0
    int m_Send(const void* buf, int len);

    //接受数据或读数据，成功返回读取实际数据的长度，失败返回0
    int m_Receive(void* buf, int maxlen);
    void Close();


private:
    int pHandle[16];
    char synchronizeflag;

    bool m_Open(const char* portname, int baudrate, char parity, char databit, char stopbit, char synchronizeflag = 1);
    //关闭串口，参数待定
    void m_Close();
};



