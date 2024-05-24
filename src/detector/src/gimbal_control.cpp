#include "gimbal_control.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <math.h>
#include <thread>

void Gimbal::GetInfo(){
    // if(sp.m_Receive(rx_buffer_, sizeof(ProjectileRx)) == 0){
    //     return;
    // }
    
    if (Crc8Verify(rx_buffer_, sizeof(ProjectileRx))) {
        memcpy(&rx_struct_, rx_buffer_, sizeof(ProjectileRx));
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

int Gimbal::readn(uint8_t *buf, size_t n){
    while(n > 0){
        int nread = sp.m_Receive(buf, n);
        if(nread < 0){
            // std::cout<<nread<<std::endl;
            // printf("Read Serial Error: \n");
            return -1;
        }
        else if(nread == 0){
            continue;
        }
        n -= nread;
        buf += nread;
    }
    return 0;
}

void Gimbal::Read(){
    const int len_buf = sizeof(ProjectileRx)*10;
    while(ok){
        // std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // if(sp.m_Receive(rx_buffer_, len_buf)) { // todo multi-thread
            // for(int idx = 0; idx < len_buf-sizeof(ProjectileRx); )
            // {
            //     if (rx_buffer_[idx] == 0x3A)
            //     {
            //         memcpy(buf_to_decode_, rx_buffer_+idx, sizeof(ProjectileRx));
            //         for(int i = 0; i < sizeof(ProjectileRx); i++){
            //             printf("%x ", buf_to_decode_[i]);
            //         }
            //         printf("\n");
            //         GetInfo();
            //         idx+=sizeof(ProjectileRx);
            //     }
            //     else
            //     {
            //         ++idx;
            //     }
            // }
            // if (rx_buffer_[0] == 0x3A) {
            //     GetInfo();
            // }
            // else{
            //     std::cout<<"error frame head"<<std::endl;
            // }
        if (readn(rx_buffer_, 1) == 0) {
            // printf("%x", rx_buffer_[0]);
            switch (rx_buffer_[0]) {
                case 0x3A: {
                    if(readn(rx_buffer_+1, sizeof(ProjectileRx)-1) == 0){
                        // for(int i = 0; i < sizeof(ProjectileRx); i++){
                        //     printf("%2x ", rx_buffer_[i]);
                        // }
                        // printf("\n");
                        GetInfo();
                    }
                    else{
                        // printf("Read Serial Error (n = %d)\n", (int)sizeof(ProjectileRx)-1);
                    }
                    break;
                }

                default:
                    break;
            }
        }
        else{
            // printf("Read Serial Error (n = 1)\n");
        }
    }
}

Gimbal::Gimbal():ok(true), read_thread(&Gimbal::Read, this){
    sp.OpenPort("/dev/USB_CBOARD", 115200, 0, 8, 1);
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
    unsigned char packet[11];
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
    Crc8Append(packet, 11);
    if (sp.m_Send(packet, sizeof(packet)) > 0) {
        printf("send successfully\n");
        return 1;
    }
    else{
        printf("Failed\n");
        return 0;
    }
}


bool Gimbal::set(float pitch_, float yaw_, bool fire = false, uint8_t patrol = 0){        //Input absolute state.
    pitch_ = pitch_ / 1.135;
    // Construct data
    unsigned char packet[12];
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
    packet[10] = patrol;
    
    Crc8Append(packet, 12);
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
    return 1.135 * pitch;
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