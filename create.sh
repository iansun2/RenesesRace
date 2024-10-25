#!/bin/bash

DIR="$( cd "$( dirname "$0" )" && pwd )"
echo "DIR: ${DIR}"

# docker pull arm64v8/ros:humble-perception
export WAYLAND_DISPLAY=wayland-1

docker run -it -d --name ros2 \
    --privileged \
    --net=host \
    -e XDG_RUNTIME_DIR=/tmp \
	-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
	-v $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/tmp/$WAYLAND_DISPLAY \
	-e QT_QPA_PLATFORM=wayland \
    -v ${DIR}/workspace:/root/workspace \
    -v ${DIR}/user_data/pip-packages:/usr/local/lib/python3.10/dist-packages \
    ros2_dev

    # -v /usr/lib64:/usr/lib64 \
    # -p 8080:80 \