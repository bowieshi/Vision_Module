#ifndef __SOLVETRAJECTORY_H__
#define __SOLVETRAJECTORY_H__
#ifndef PI
#define PI 3.1415926535f
#endif
#define GRAVITY 9.78
typedef unsigned char uint8_t;


namespace trajectory {
    const float k = 0.092;
    const float current_v = 30;
    const float s_bias = 0.05;
    float SolveTrajectory(float aim_x, float aim_y, float aim_z);
}

#endif /*__SOLVETRAJECTORY_H__*/