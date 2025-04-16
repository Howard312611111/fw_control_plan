# PX4 Autopilot Software In the Loop (SITL) - Fixed Wing
###### tags: `user_guide`


## Environment

Ubuntu 18.04 + ROS, Ubuntu 20.04 + ROS


## Install

### 1. PX4 Environment Setup

<font color="#696969">*Last edited time: 2023/07/10*</font>

Please follow this [document](https://hackmd.io/XPh1n2JaQhq-UUewi_EGSw?view).


### 2. Add New PX4 Model

<font color="#696969">*Last edited time: 2023/08/02*</font>

Please follow this [document](https://hackmd.io/NOT-MylFSwKgVh5PR-ORHA).

> Ref. https://github.com/Leo125Jan/vtol_gimbal_control



### 3. Add A Car Model (prius) in Simulation

<font color="#696969">*Last edited time: 2023/06/05*</font>

```bash
$ cd <your_workspace>/src
or
$ cd catkin_ws/src

$ git clone git@github.com:osrf/car_demo.git
$ cp -r car_demo/prius_msgs/ .
$ cp -r car_demo/prius_description/ .
$ rm -rf car_demo
$ cd ..
$ catkin_make
```

To control the puris, change the file in `prius_description/urdf/prius.urdf` into [this file](https://github.com/vivianbh/fw_control/blob/main/models/vehicle/prius/prius.urdf):link:.
用這個file的urdf檔案（1054行）貼到上一步驟prius_description的urdf中,車子才可以動




> Ref. https://github.com/Leo125Jan/vtol_gimbal_control

### 4. YOLO

<font color="#696969">*Last edited time: 2023/06/05*</font>

Please follow this [document](https://hackmd.io/kqe9M0DlT8OcNQvfBxcFuA).
> Ref. https://github.com/Leo125Jan/vtol_gimbal_control


## Run the Simulation

<font color="#696969">*Last edited time: 2023/08/02*</font>

Download the package under the workspace
```bash
$ cd {your_workspace}/src
$ git clone https://github.com/vivianbh/fw_control.git
```

add the following command `source $HOME/{your_workspace_path}/devel/setup.bash` in `~/.bashrc` file.

```bash
$ source ~/.bashrc

$ cd {your_workspace}/
$ catkin_make
```


Run the following commands in terminal:

- Open the simulation
```bash
$ ./QGroundControl.AppImage  (or double click)

$ cd {your_workspace}
$ roslaunch sim_multi_with_prius.launch vehicle:="fw_gimbal_camera"
```

- UAV loiter
```bash
$ roslaunch fw_control multi_uav_flight.launch
```

- Start image recognition
```bash
$ roslaunch darknet_ros darknet_ros.launch
```

- Target tracking
```bash
$ roslaunch fw_control uav0_gimbal_camera_tracking.launch
```


### 1. How to use

In the ternimal where run `uav0_gimbal_camera_tracking.launch` node, press the below functional keys: 

Default: 
- gimbal lock at current angle
- tracking by 3D position, not yolo measurement



1. Gimbal 

    | Keyboard | Event | 
    | -------- | -------- | 
    | `t`    | activate tracking     |
    | `l`    | lock at current angle     | 


2. Tracking Method

    | Keyboard | Event | 
    | -------- | -------- | 
    | `y`    | use yolo measurement     |
    | `o`    | use 3D position     | 

        



### 2. How to set the parameters in `gimbal_control.cpp`

In `gimbal_control.cpp`, there are three parameters in `TrackingControllerR` function:

![](https://hackmd.io/_uploads/rylWWEKth.png)


Change second param to set which uav used.

- Second param option: `gb.miniyy`, `gb.techpod`
