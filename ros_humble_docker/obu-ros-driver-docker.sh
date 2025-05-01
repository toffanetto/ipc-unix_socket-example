#!/bin/bash

# For production in target 
docker run -it --rm --ipc=host --volume /tmp:/tmp --network host --name obu-ros-driver obu-ros-driver:V0.3-arm64