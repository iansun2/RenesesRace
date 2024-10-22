#!/bin/bash

# docker pull arm64v8/ros:humble-perception

docker run -it -d --name ros2 \
    --privileged \
    --net=host \
    -e XDG_RUNTIME_DIR=/tmp \
	-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
	-v $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/tmp/$WAYLAND_DISPLAY \
	-e QT_QPA_PLATFORM=wayland \
    arm64v8/ros:humble-perception


    # -p 8080:80 \