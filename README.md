# PBVS

This repository contains a ROS driver to control Kuka KR4 R600 robot, including a script to perform Position Based Visual Servo (PBVS).


## How to use

#### TODO
- [ ] Setup network (robot & host)
- [ ] Copy .src files to robot & fix AXIS_SET issue
- [ ] Build docker
- [ ] Execute commands


### Steps on Host
- Connect to the network and check if ping work with the robot IP:
```sh
ping 192.168.50.205
```

- Build the docker image (if the first time running):
```sh
cd pbvs-moveit-kr4
docker-compose build
```

- Start docker container:
```sh
./launch.sh
```

- Execute the following commands in order:
    - This command start a hardware interface that communicates with the robot.
```sh
roslaunch kuka_kvp_hw_interface test_joint_command_interface.launch
```
    - This command start ros moveit that enable to send trajectory to the hardware interface.
```sh
roslaunch kuka_kr4_moveit demo.launch
```


## Third Party
- [Kuka KR4 R600 Model](https://github.com/isys-vision/kuka_experimental/blob/kr4r600/kuka_eki_hw_interface/src/kuka_eki_hw_interface.cpp)
