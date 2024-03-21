#ifndef GIMBAL
#define GIMBAL

#include "SerialPort.hpp"
#include "Checksum.h"

#include <cmath>
#include <unistd.h>
#include <thread>
#include <memory>

class Gimbal{
public:
    Gimbal();
    ~Gimbal();
    
    bool move(float, float, bool);
    bool set(float, float, bool);
    bool keep();

    float cur_pitch();
    float cur_yaw();
    float cur_v_forward();
    float cur_v_right();
    float cur_v_angular();

private:
    struct ProjectileRx
    {
        uint8_t header;
        float sys_time;
        float pitch;
        float yaw;
        float v_forward;
        float v_right;
        float v_angular;
        uint8_t checksum;
    }  __attribute__((packed));

    ProjectileRx rx_struct_{};
    uint8_t rx_buffer_[1024]{};
    uint8_t buf_to_decode_[64]{};

    float pitch, yaw, bullet_speed;
    float v_forward, v_right, v_angular;
    float last_pitch, last_yaw;
    
    SerialPort sp;

    bool ok;

    std::thread read_thread;


    void GetInfo();
    void Read();
};

#endif