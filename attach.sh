#!/bin/bash

# docker pull arm64v8/ros:humble-perception

docker exec -w /root -it ros2 bash -c " \
    . /ros_entrypoint.sh; \
    bash"
