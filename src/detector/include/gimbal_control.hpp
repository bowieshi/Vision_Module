#ifndef GIMBAL
#define GIMBAL

#include "SerialPort.hpp"
#include "Checksum.h"

#include <cmath>
#include <unistd.h>
#include <thread>
#include <memory>
#include <atomic>

class Gimbal{
public:
    Gimbal();
    ~Gimbal();
    
    bool move(float, float, bool);
    bool set(float, float, bool, uint8_t);
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

    std::atomic<float> pitch, yaw, bullet_speed;
    std::atomic<float> v_forward, v_right, v_angular;
    float last_pitch, last_yaw;
    
    SerialPort sp;

    bool ok;

    std::thread read_thread;


    void GetInfo();
    void Read();
    int readn(uint8_t *, size_t);
};

#endif