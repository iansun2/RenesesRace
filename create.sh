#!/bin/bash

DIR="$( cd "$( dirname "$0" )" && pwd )"
echo "DIR: ${DIR}"

# docker pull arm64v8/ros:humble-perception
export WAYLAND_DISPLAY=wayland-0

docker run -it -d --name ros2 \
    --privileged \
    --net=host \
    -e XDG_RUNTIME_DIR=/tmp \
	-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
	-v $XDG_RUNTIME_DIR/wayland-0:/tmp/wayland-0 \
    -v $XDG_RUNTIME_DIR/wayland-1:/tmp/wayland-1 \
	-e QT_QPA_PLATFORM=wayland \
    -v ${DIR}/workspace:/root/workspace \
    ros2_dev

    # -v ${DIR}/user_data/pip-packages:/usr/local/lib/python3.10/dist-packages \
    # -v ${DIR}/user_data/local:/root/.local \

docker run -it -d --name redis \
    --net=host \
    redis:7.4.1