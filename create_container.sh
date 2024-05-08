#!/bin/bash

xhost local:root

XAUTH=/tmp/.docker.xauth

docker run -it \
    --name=$1 \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/dev:/dev" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    dndqodqks/ubuntu-ros2-gazebo-realsense:$2 \
    bash
