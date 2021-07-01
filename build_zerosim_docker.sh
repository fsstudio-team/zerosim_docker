#!/bin/bash
set -e
set -u

# --no-cache <<< add this if you want to force rebuild
docker build --no-cache -t zerosim_ros -f docker/base/Dockerfile .
