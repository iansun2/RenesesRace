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
	-v $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/tmp/$WAYLAND_DISPLAY \
	-e QT_QPA_PLATFORM=wayland \
    -v ${DIR}/workspace:/root/workspace \
    -v ${DIR}/user_data/pip-packages:/usr/local/lib/python3.10/dist-packages \
    -v ${DIR}/user_data/local:/root/.local \
    -v ${DIR}/config/code-server-config.yaml:/root/.config/code-server/config.yaml:ro \
    ros2_dev


docker run -it -d --name redis \
    --net=host \
    redis:7.4.1