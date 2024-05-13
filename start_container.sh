#!/bin/bash

docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

xhost local:root

docker start $1

docker attach $1
