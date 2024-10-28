#!/bin/bash

export WAYLAND_DISPLAY=wayland-1
docker start ros2
docker start redis