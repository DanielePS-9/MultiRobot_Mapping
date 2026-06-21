#!/bin/bash
 
IMAGE_NAME="ros"
IMAGE_TAG="prj_lidar"
CONTAINER_NAME="multirobot"
 
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
 
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Removing existing container: ${CONTAINER_NAME}"
    docker rm -f "${CONTAINER_NAME}"
fi
 
echo "Starting container: ${CONTAINER_NAME}"
echo "Image     : ${IMAGE_NAME}:${IMAGE_TAG}"
#echo "Workspace : ${PROJECT_ROOT}/ros_ws/src -> /ros_ws/src"
echo "Workspace : ${PROJECT_ROOT}/ros_ws/:/root/ros_workspace"


docker run -it \
  --name "${CONTAINER_NAME}" \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e ROS_DOMAIN_ID=0 \
  -e TURTLEBOT3_MODEL=waffle \
  -v "${PROJECT_ROOT}/ros_ws/:/root/ros_workspace" \
  --privileged \
  "${IMAGE_NAME}:${IMAGE_TAG}"
 

# -v "${PROJECT_ROOT}/ros_ws/src:/ros_ws/src" \