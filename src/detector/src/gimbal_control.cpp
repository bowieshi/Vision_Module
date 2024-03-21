#include "gimbal_control.hpp"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <math.h>

void Gimbal::GetInfo(){

    if (Crc8Verify(buf_to_decode_, sizeof(ProjectileRx))){
        memcpy(&rx_struct_, buf_to_decode_, sizeof(ProjectileRx));
        pitch = rx_struct_.pitch;
        yaw = rx_struct_.yaw;
        v_forward = rx_struct_.v_forward;
        v_right = rx_struct_.v_right;
        v_angular = rx_struct_.v_angular;
        // std::cout << "In gimbal control: " << rx_struct_.pitch << " " << rx_struct_.yaw << std::endl;
    }
    else{
        printf("CRC error\n");
    }
}

void Gimbal::Read(){
    const int len_buf = sizeof(ProjectileRx)*10;
    while(ok){
        if (sp.m_Receive(rx_buffer_, len_buf)) {
            for(int idx = 0; idx < len_buf - sizeof(ProjectileRx); )
            {
                if (rx_buffer_[idx]==0x3A)
                {
                    
                    memcpy(buf_to_decode_, rx_buffer_+ idx, sizeof(ProjectileRx));
                    GetInfo();
                    idx+=sizeof(ProjectileRx);
                }
                else
                {
                    ++idx;
                }
            }
        }
    }
}

Gimbal::Gimbal():ok(true), read_thread(&Gimbal::Read, this){
    sp.OpenPort("/dev/ttyACM0", 115200, 0, 8, 1);
    pitch = 0;
    yaw = 0;
}

Gimbal::~Gimbal(){
    ok = false;
    read_thread.join();
}

float normalize_angle(float x){
    x = fmod(x, 2*M_PI);
    if (x > M_PI){
        x -= 2*M_PI;
    }
    else if (x < -M_PI){
        x += 2*M_PI;
    }
    return x;
}

bool Gimbal::move(float disp_pitch, float disp_yaw, bool fire = false){        //Input displacement of current state.
    float cur_pitch = pitch;
    float cur_yaw = yaw;
    // std::cout << pitch << " " << yaw << std::endl;

    // Construct data
    unsigned char packet[10];
    packet[0] = 0xA3;
    union {
        float actual;
        uint8_t raw[4];
    } tx_x{}, tx_y{};

    tx_x.actual = cur_pitch + disp_pitch;
    tx_y.actual = cur_yaw + disp_yaw;
    // printf("%f %f\n", cur_pitch, cur_yaw);

    for (int i = 0; i < 4; i++) {
        packet[1 + i] = tx_x.raw[i];  // x
        packet[5 + i] = tx_y.raw[i];  // y
    }
    packet[9] = fire;
    // Crc8Append(packet, 10);
    if (sp.m_Send(packet, sizeof(packet)) > 0) {
        printf("send successfully\n");
        return 1;
    }
    else{
        printf("Failed\n");
        return 0;
    }
}


bool Gimbal::set(float pitch_, float yaw_, bool fire = false){        //Input absolute state.
    // Construct data
    unsigned char packet[10];
    packet[0] = 0xA3;
    union {
        float actual;
        uint8_t raw[4];
    } tx_x{}, tx_y{};

    tx_x.actual = pitch_;
    tx_y.actual = yaw_;

    for (int i = 0; i < 4; i++) {
        packet[1 + i] = tx_x.raw[i];  // x
        packet[5 + i] = tx_y.raw[i];  // y
    }
    packet[9] = fire;
    
    // Crc8Append(packet, 10);
    if (sp.m_Send(packet, sizeof(packet)) > 0) {
        printf("send successfully\n");
        printf("Send set command: %f %f\n", pitch_, yaw_);
        return 1;
    }
    else{
        printf("failed\n");
        return 0;
    }
}

bool Gimbal::keep(){
    return move(0, 0);
}

float Gimbal::cur_pitch(){
    return pitch;
}

float Gimbal::cur_yaw(){
    return yaw;
}

float Gimbal::cur_v_forward(){
    return v_forward;
}

float Gimbal::cur_v_right(){
    return v_right;
}

float Gimbal::cur_v_angular(){
    return v_angular;
}