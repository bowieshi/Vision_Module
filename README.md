# Astar RoboMaster Vision Module

This is a manual book about how to deploy Vision Module to your mini-nuc

**Author: Shi Boao**

**Date: 26/3/2024**

## Step 1: Clone this Repo

Use `git clone` to clone this `catkin_ws` to your mini-computer.

We use `Linux` operating system, specifically `Ubuntu 20.04`.

## Step 2: Install Dependency

Our codes are dependent on:
1. `ros-noetic` (include some packages, e.g., tf, tf2, sensor_msgs, geometry_msgs)
2. `opencv` (opencv-4.7.0), `cv_bridge`
3. `Eigen3`
4. `MVS` (hikvision driver)

please make sure you have installed all the packages and modify the corresponding path in `CMakeList.txt`.

## Step 3: Ensure your Harwares

Typically, we link `1.camera`, `2.Development Board (Type C)` to mini_computer with two USB ports. please make sure you have given the USB ports permission to read the data.
### how to check the permission of USB ports

To see all the ports: `ls /dev/tty*`.

Then link the camera/Development-Bord to mini-computer and enter `ls /dev/tty*` again.

Look through all the `tty*`, if there is someone new popping out, this is the corresponding port to the linked hardware.

Assume the port is `ttyACM0`. Add permission to `ttyACM0`: `sudo chmod 777 /dev/ttyACM0`.

Or

You can directly add permission to all the port: `sudo chmod 777 /dev/tty*`

### data from development board (type C)

you need check which USB port is currently used. And modify `gimbal_control.cpp`.
If you cannot read the data from development board (c type), you have to check whether you have given permission to USB ports. If not, use `sudo chmod 777 /dev/ttyACM0`.

## Step 4: Adjust Global Parameters

### (1). Camera Parameters
1. `ExposureTime`
2. `Gamma`

First, you need open the `MVS` software, and open the camera. Then, you can adjust the `ExposureTime` and `Gamma` until the armor's lights are bright and clear and easy to recognize.

After it, record the parameters and click into `catkin_ws/src/detector/include/hikrobot_camera.hpp`. And use `ctrl + F` to find `ExposureTime` and `Gamma` to change them into your record parameters.

You have to change it in every different environment before testing. These parameters influence performance of detector very much.

e.g., In inno-wing with light off, suitable parameters are `ExposureTime` = `4000`, `Gamma` = `0.5`.

3. Camera intrinsics and Camera size

please refer to configuration file at `catkin_ws/src/detector/params/camera.yaml`.

You need to download a python program for intrinsics measurement and take dozens of photo with MVS software. Then run the python to get the `K` and `D`. Then you can modify the parameters in `catkin_ws/src/detector/params/camera.yaml`.

It is important to measure the accurate camera intrinsics.

- you need to adjust the focus of camera to make it at suitable position, like 5 meter.

3. Different types of camera have different settings:

- Old camera (e.g., sentry_right & sentry_left) has `1440 * 1080` resolution. bayerRG8

- New camera (e.g., infantry) has `1280 * 1024` resolution. bayerGB8

you need to change some parameters in `hikrobot_camera.hpp`

相机标定一些注意点：

1. 首先调整相机光圈，使相机焦距大约4-5m，即正常有效射击距离，距离太近或太远都可能无法打中目标。即标定板在4-5m处能够非常清晰地拍清楚每个角点；

2. 拍摄清晰的标定板照片，大约15-20张；

3. 利用 `catkin_ws/src/detector/cali/cali.py` 进行标定；

4. 标定完后检查标定后的矩阵，`cx`, `cy` 对应的位置上是否合理，即像素边长的一半；

5. 将标定的结果存到 `camera_calibration` 文件夹对应的位置；(1). 标定图片存到 `camera_calibration/images/`对应文件夹下；(2). 标定输出文件存到 `paras/camera`, 格式参照 `sentry_right.txt`; 

PS: 
- `camera_calibration/images_deprecated` 是不好的标定照片的例子
- `camera_calibration/images/sentry_right` 文件夹下是比较好的例子

## (2). Gimbal Parameters

please refer to configuration file at `catkin_ws/src/detector/params/gimbal.yaml`.

Different robots have different size and rotation axis, we build up three coordinate system to cope with the coordinate transformation from camera coordinate system to world coordinante system.

### Procedure to calibrate camera:
1. save all the captured images in `catkin_ws/src/detector/camera_calibration/images/xxx`
2. modify `catkin_ws/src/detector/camera_calibration/in_VID5.xml`. parameters need modify: `BoardSize_Width`, `BoardSize_Height`, `Square_Size`
3. modify `VID5.xml`: add all pathes of your captured images.
4. run `./build/camera_calibration in_VID5.xml`
5. desired camera intrinsic is in `out_camera_data.xml`

## (i). World coordinate system:

Our robot has two rotation axis, one for `yaw`, the other for `pitch`.

我们机器人有两个旋转轴，一个是`yaw`轴，控制水平旋转，一个是`pitch`轴，控制竖直旋转。

世界坐标系是静止的，车头沿着`yaw`,`pitch`旋转，世界坐标系不动，但关节坐标系和相机坐标系会发生改变。所以世界坐标系的原点应该在`yaw`轴上的某一点，为了方便，我们作一个经过`pitch`轴平行于地面的平面，这个平面于`yaw`轴的交点即为世界坐标系原点。
### How to locate world coordinate system's original point?

Firstly, place the gun horizontally. then move the `pitch` axis towards `yaw` axis horizontally until they have a intersection point. This point is the original point.

### x, y, z axis convention

- `x`: forward (along the gun, where the bullet shoots at)
- `y`: left
- `z`: upward

世界坐标系的`x`轴,`y`轴,`z`轴正方向分别是往前，往左，往上（往前是枪口上电是默认朝向）


## (ii) Joint coordinate system:

We make the center vertical plane of the line connecting the two fixed points of the camera.

关节坐标系是一个从相机坐标系转换到世界坐标系的中介坐标系，简化转换过程。关节坐标系的原点：相机会通过两个螺丝与车头相连接，这两个连接孔上表面（如果相机安装在枪口上方）的圆心连线的中垂面和车头的`pitch`旋转轴的交点就是关节坐标系的原点。

the plance will intersect will the `pitch` axis in a point, which is the Joint coorinante system's orginal points.

Joint coorinante system has the same `x`, `y`, `z` axis convention with world coordinate.

关节坐标系的各个轴正方向与世界坐标系一致。

## (iii). Camera coordiante system:

Camera coordinate system has its orginial point at the optical center of the camera.

相机坐标系的原点在相机的光心处

### x, y, z axis convention

- `x`: right
- `y`: down
- `z`: forward (along the gun, where the bullet shoots at)

相机坐标系的`x`轴,`y`轴,`z`轴正方向分别是往右，往下，往前（往前是相机朝向）

## modify parameters in yaml
知道了每个坐标系之间的关系，我们就需要通过CAD图以及相机的工图找到对应的距离，修改`gimbal.yaml`文件。

`WJ_translation`是世界坐标系到关节坐标系的位移转换，即在世界坐标系的规范下，关节坐标系原点的坐标减去世界坐标系原点的坐标

`JC_translation`是关节坐标系到相机坐标系的位移转换，即在相机坐标系的规范下，相机坐标系原点的坐标减去关节坐标系原点的坐标。

Q:为什么这里不用世界坐标系？

A: 因为从关节坐标系转换到相机坐标系需要转换坐标轴的方向，在`ros`的`tf.transform`中四元数的旋转是先于translation的所以我们直接用相机坐标系了。

**单位都是`m`**


## Transformation

The three coordinate system has 3 orginal points, we need to measure the distance between each points in every robot.

We use world coordinate system as our standard coordinate system for `WJ`, we place $O_W$, $O_J$ in world coordinate system, and measure the distance.

$$
T_{WJ} = \begin{bmatrix}
dX_{WJ}\\
dY_{WJ}\\
dZ_{WJ}\\
\end{bmatrix}=\begin{bmatrix}
x_{O_J}-x_{O_W}\\
y_{O_J}-y_{O_W}\\
z_{O_J}-z_{O_W}\\
\end{bmatrix}
$$

We use camera coordinate system as our standard coordinate system for `JC`, we place $O_W$, $O_J$ in camera coordinate system, and measure the distance.

$$
T_{JC}=\begin{bmatrix}
dX_{JC}\\
dY_{JC}\\
dZ_{JC}\\
\end{bmatrix}=\begin{bmatrix}
x_{O_C}-x_{O_J}\\
y_{O_C}-y_{O_J}\\
z_{O_C}-z_{O_J}\\
\end{bmatrix}
$$

## Step 5: Compile and Run

### How to run the catkin_ws
All the following operations are conducted at directory catkin_ws. please `cd catkin_ws` first,
1. Compile all the files, using `catkin_make`;
2. Open a new terminal, execute `roscore`;
3. Open a new terminal, execute `source ./devel/setup.bash`. Then `rosrun detector detector_node`;
4. Open a new terminal, execute `source ./devel/setup.bash`. Then `rosrun detector gimbal_node`;
5. Open a new terminal, execute `rviz`;
6. In rviz, add the corresponding topic into your panel, like `tf`, `Markerarray`, `image`.

or you can just use roslaunch: `roslaunch detector predict.launch`.



# Appendix

## 1. How to use MVS

### Step 1: Install and test MVS driver

We use Hikvision industry camera

We need to install Hikvision camera driver first from official website

Normally, the driver directory is at ``/opt/MVS/bin``

The executable file ``MVS`` is the driver

using ``./MVS`` to run the driver

please make sure you have used USB line to link camera with you computer.

Then you can see the video from camera in the MVS software

### Step 2: Read image message in your program

``hikrobot_camera.hpp`` is provided by Hikvision. There is no need for you to modify it.

You should include this header in you ``cpp``. 

And use the following code:

```c++
camera::Camera cam;
cv::Mat img;
while(true) {
  cam.ReadImg(img)
  if (img.empty())
    continue;
  //do something to process this image
  //...
}
```

you can read each frame of the video captured by your camera into ``cv::Mat img``.

Possibly your reading can fail and reture a empty image. You should check it.

### Step 3: Visualize your image

We use ROS-noetic and RVIZ to visualize our image.

We use ``cv_bridge`` to transform ``cv::Mat`` into ``sensor_msgs::ImagePtr``. Then we use a ros publisher to publish our image message and you can add this topic in RVIZ to visualise it.

```c++
cv_bridge::CvImage cv_image;
cv_image.header.stamp = ros::Time::now();  // Set the timestamp
cv_image.encoding = sensor_msgs::image_encodings::BGR8;  // Set the encoding (e.g., BGR8 for color image)
cv_image.image = img;
sensor_msgs::ImagePtr ros_image = cv_image.toImageMsg();
img_pub.publish(ros_image);
```



## 2. Trick: How to kill an unstoppable process
Sometimes, you rosrun a node and want to stop it. You try to press `ctrl + c` but with no succuss. You can try the following procedure:
1. `ps -a` show all the on-going process;
2. `kill UID`, where `UID` is the UID of the unstoppable process.


## 3. Trick: use rqt_plot to test code

This is a tool to visualize a wave graph of some data.

reference:
- https://blog.csdn.net/wwsQt/article/details/68925432
- https://blog.csdn.net/qq_42108414/article/details/124407476