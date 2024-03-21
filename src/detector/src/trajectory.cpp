#include <math.h> 
#include <stdio.h>

#include "trajectory.hpp"


//Trajectory function
float monoDirectionalAirResistanceModel(float s, float v, float angle, float expa)// exponential acceleration
{
    float z;
    float t; 
    t = (float)((expa - 1) / (trajectory::k * v * cos(angle)));
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    return z;
}
// for solviong Trajectory function, out put the pitch angle
float pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    float expa = exp(trajectory::k * s);
    int i = 0;
    z_temp = z;
    for (i = 0; i < 20; i++)
    {
        angle_pitch = atan2(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch, expa);
        dz = 0.3*(z - z_actual);
        z_temp = z_temp + dz;
        if (fabsf(dz) < 0.00001)
        {
            break;
        }
    }
    return angle_pitch;
}
// main function to solve the trajectory
// input the aim_x, aim_y, aim_z
float trajectory::SolveTrajectory( float aim_x, float aim_y, float aim_z)
{
    float s = sqrt(aim_x * aim_x + aim_y * aim_y);
    float pitch = pitchTrajectoryCompensation(s- trajectory::s_bias, aim_z, trajectory::current_v);
    return pitch; 
}

