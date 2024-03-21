# Document

This document helps you to comprehend how does the code work.

It introduces details about the realization of auto-aimming algorithm.

## Knowledge will be used

1. C++ Object-oriented programming (OOP)
2. ROS1, ros-noetic, RVIZ
3. OpenCV
4. Linear algebra, calculus and probability theory
5. Simple neural network


## Directory hierarchy

We have 2 ros-nodes:
1. detector_node (in detector_node.cpp)
2. gimbal_node (in gimbal_node.cpp)

### detector_node
`detector_node` is the auto-aimming unit

Firstly, `detector_node.cpp` will use `detector module` to detector armors in image and then use `tracker module` to predictor the movement model of enermy robot.

**detector module**

`mydetector.cpp` -> `detector.cpp`
                 -> `number_classifier.cpp`

**tracker module**


`predictor.cpp`-> `mytracker.cpp` -> `tracker.cpp` -> `Extended_KF.cpp`

### gimbal_node
`gimbal_node` is the unit dealing with reading pitch and yaw message from C-board and send tf transform.

You have no need to look at `Checksum.cpp` and `gimbal_control.cpp`. They are provided by embedded group.


## Detector algorithm

This part is about how to detector the armors in a given image and then classify its number.

Reference: https://gitlab.com/rm_vision/rm_auto_aim/-/tree/main/armor_detector?ref_type=heads

## Tracker algorithm

### Get 3D coordinate from 2D image

After detection, we have the bounding box(the four corner points) of the detected armor. We employ `Perspective-n-Point (PnP) pose computation` method to calculate the 3D coordinate of the armor center in the camera coordinate. We use open-source library `opencv-solvePnP`.

**Reference**

[Pose computation overview](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html)

[solvePnp](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d)

[【姿态表示】旋转向量、旋转矩阵、欧拉角、四元数](https://zhuanlan.zhihu.com/p/93563218)

[[数学]罗德里格旋转公式（Rodrigues' rotation formula）](https://zhuanlan.zhihu.com/p/451579313)

[cv::Rodrigues()](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga61585db663d9da06b68e70cfbf6a1eac)

[旋转向量](https://blog.csdn.net/xuewend/article/details/84645213)


## our C-Board gimbal message:

- `yaw`: `0` at start point, when gimbal turn left(or anticlockwise from above), the yaw increases

- `pitch`: `0` at start point, when gimbal turn down, the pitch increases

## orientationToYaw rectify
After transform, we got the Pose of armor under world coordinate

We use `orientationToYaw` to fetch the `yaw` of armor
```c++
double Predictor::orientationToYaw(const geometry_msgs::Quaternion & q)
{
    // Get armor yaw
    tf2::Quaternion tf_q;
    tf_q.setX(q.x), tf_q.setY(q.y), tf_q.setZ(q.z), tf_q.setW(q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    return yaw;
}
```

The returned `yaw` has the following feature:

- when facing directly, the `yaw` is `-PI/2`
- when the enermy robot spin anticlockwise seeing above, the `yaw` increase

However, in the observation equation, the needed `yaw` should have the following features:
- directly face our camera: `yaw = 0`;
- the tracking robot rotates anticlockwise (look from above). `yaw` increases.
- the tracking robot rotates clockwise (look from above), `yaw` decreases.

So in order to make the `yaw` to be desired. We need add a offset `PI/2`. The revised `orientationToYaw` is:
```c++
double Predictor::orientationToYaw(const geometry_msgs::Quaternion & q)
{
    // Get armor yaw
    tf2::Quaternion tf_q;
    tf_q.setX(q.x), tf_q.setY(q.y), tf_q.setZ(q.z), tf_q.setW(q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    yaw = yaw + PI/2;
    return yaw;
}
```

# Appendix

Please note that in `predictor.cpp`, we have define 


When the armor is facing the camera in the middle, $yaw$ is $-\frac{\pi}{2}$.

When the robot rotates clockwise from the top looking down, $yaw$

Some convention coordinates:

**Camera coordinate**

x: right

y: down

z: forward


**World coordinate**

x: forward

y: left

z: upward


Extended Kalman Filter

state equation:

$$
\boldsymbol{x}_n=[x_c,v_{x_c},y_c,v_{y_c},z_c,v_{z_c},yaw,v_{yaw},r]^T
$$

observation equation:

$$
\boldsymbol{z}_n=[x_a,y_a,z_a,yaw]^T
$$

### State transition function

$$
\hat{\boldsymbol{x}}_{n+1}=f(\boldsymbol{x}_n)=\begin{bmatrix}
x_c + v_{x_c}\Delta t\\
v_{x_c}\\
y_c + v_{y_c}\Delta t\\
v_{y_c}\\
z_c + v_{z_c}\Delta t\\
v_{z_c}\\
yaw + v_{yaw}\Delta t\\
v_{yaw}\\
r\end{bmatrix}
$$

$$
F=\begin{bmatrix}
    1 & \Delta t & 0 & 0 & 0 & 0 & 0 & 0 & 0\\
    0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0\\
    0 & 0 & 1 & \Delta t & 0 & 0 & 0 & 0 & 0\\
    0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0\\
    0 & 0 & 0 & 0 & 1 & \Delta t & 0 & 0 & 0\\
    0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0\\
    0 & 0 & 0 & 0 & 0 & 0 & 1 & \Delta t & 0\\
    0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0\\
    0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1\\
\end{bmatrix}
$$

### Measurement equation (or Observation function)

$$
\boldsymbol{z}_n=h(\boldsymbol{x}_n)=\begin{bmatrix}
x_c-r\cos(yaw)\\
y_c-r\sin(yaw)\\
z_c\\
yaw
\end{bmatrix}
$$

$$
J_H=\begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 & r\sin(yaw) & 0 & -\cos(yaw)\\
0 & 0 & 1 & 0 & 0 & 0 & -r\cos(yaw) & 0 & -\sin(yaw)\\
0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0\\
0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0\\
\end{bmatrix}
$$

# Appendix
## Yaw Rectify
```
double Tracker::orientationToYaw(const geometry_msgs::Quaternion & q)
{
    // Get armor yaw
    tf2::Quaternion tf_q;
    tf_q.setX(q.x);
    tf_q.setY(q.y);
    tf_q.setZ(q.z);
    tf_q.setW(q.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    
    //rectify
    yaw = yaw - PI/2;

    // Make yaw change continuous (-pi~pi to -inf~inf)
    yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
    last_yaw_ = yaw;
    return yaw;
}
```
For $yaw$

$yaw$ is the tracking armor's yaw to the center of the robot

After our pnp and coordinate transform. The $yaw$ we get has the following feature:
- directly face (zhengdui) our camera. $yaw = \frac{\pi}{2}$;
- the tracking robot rotates anticlockwise (look from above). $yaw$ increases.
- the tracking robot rotates clockwise (look from above), $yaw$ decreases.

However, in the observation equation, the needed $yaw$ should have the following features:
- directly face (zhengdui) our camera. $yaw = 0$;
- the tracking robot rotates anticlockwise (look from above). $yaw$ increases.
- the tracking robot rotates clockwise (look from above), $yaw$ decreases.

Thus we need rectify our current $yaw$: `yaw = yaw - PI/2`




For gimbal

pitch, upwards -> increase
down -> decrease

yaw, left -> increase
right ->decrease