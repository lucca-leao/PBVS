#!/bin/bash

xhost +

DIR="$( cd "$( dirname "$0" )" && pwd )"
echo $DIR

docker run --rm -it --privileged \
	--net=host \
	--gpus all \
	-e NVIDIA_VISIBLE_DEVICES=0 \
	--env=DISPLAY \
	--env=NVIDIA_DRIVER_CAPABILITIES=all \
	--env=QT_X11_NO_MITSHM=1 \
	--volume=/dev:/dev \
	--volume=$HOME/.Xauthority:/root/.Xauthority:rw \
	--volume=${DIR}/catkin_ws/src/:/catkin_ws/src/ \
	--runtime=nvidia ivision/kuka-kr4-ros:v1.0
