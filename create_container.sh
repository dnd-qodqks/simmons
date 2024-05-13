#!/bin/bash

docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

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
    dndqodqks/arm64v8-ubuntu-ros2-realsense:$2 \
    bash
