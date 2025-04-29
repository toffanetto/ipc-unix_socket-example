#!/bin/bash

# For production in target 
docker run -it --rm --ipc=host --name obu-ros-driver obu-ros-driver:V0.1-arm64

# For debug in host
#docker run -it --rm --ipc=host obu-ros-driver:latest_amd64