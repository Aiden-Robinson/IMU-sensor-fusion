#!/bin/bash

# Script to rebuild Docker container with quaternion conversion fix

echo "Rebuilding Docker container with quaternion conversion fix..."

# Stop any running container
docker-compose down 2>/dev/null || docker stop ros2_imu_visualization 2>/dev/null || true

# Rebuild the Docker image
docker build --no-cache -t ros2_imu_visualization .

echo "Container rebuilt successfully!"
echo "Now run: ./docker_setup.sh"
