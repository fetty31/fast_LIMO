#!/bin/bash
set -e

DOCKER_IMG="fastlimo-ros2-image"
PROJECT_ROOT="$(cd "$(dirname "$0")"; pwd)"

echo "----- Building Docker image from ${PROJECT_ROOT}/Dockerfile "
docker build -t ${DOCKER_IMG} ${PROJECT_ROOT}

echo "----- Building ROS workspace "
${PROJECT_ROOT}/run sh -c "cd /home/colcon_ws/ && colcon build --symlink-install"