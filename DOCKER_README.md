# Docker ROS2 IMU Visualization

This Docker-based solution provides a completely portable ROS2 IMU visualization for Linux with minimal setup.

## 🚀 Quick Start

```bash
./docker_setup.sh
```

## 📋 Prerequisites

- Docker
- Docker Compose (optional, but recommended)
- X11 server running (usually automatic on desktop Linux)

## 🐳 What's Included

- **ROS2 Humble Desktop** - Full ROS2 installation
- **RVIZ2** - 3D visualization
- **Custom IMU Package** - Serial reader and TF broadcaster
- **All Dependencies** - Pre-installed and configured

## 🎯 Usage Options

### 1. One-Command Setup (Recommended)

```bash
# Full setup and run
./docker_setup.sh

# With custom serial port
./docker_setup.sh -p /dev/ttyUSB0

# Build only
./docker_setup.sh --build-only

# Run only (if already built)
./docker_setup.sh --run-only
```

### 2. Docker Compose

```bash
# Set your serial port
export SERIAL_PORT=/dev/ttyACM0
docker-compose up --build
```

### 3. Manual Docker Run

```bash
# Build image
docker build -t ros2_imu_visualization .

# Run container
docker run -it --rm \
    --name ros2_imu_visualization \
    --privileged \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e SERIAL_PORT=/dev/ttyACM0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev:/dev \
    --device=/dev/ttyACM0:/dev/ttyACM0 \
    ros2_imu_visualization
```

##  Configuration

### Environment Variables
- `SERIAL_PORT`: Serial port for Arduino (default: `/dev/ttyACM0`)
- `DISPLAY`: X11 display (auto-configured)
- `ROS_DOMAIN_ID`: ROS2 domain ID (default: 0)

### Custom Serial Port
```bash
export SERIAL_PORT=/dev/ttyUSB0
```

## 🐛 Troubleshooting

### Permission denied for serial port:
```bash
sudo usermod -aG dialout $USER
# Then log out and back in
```

### X11 forwarding not working:
```bash
xhost +local:docker
```

### Serial port not found:
```bash
# List available ports
ls /dev/tty*
# or
python3 ros2_imu_package/scripts/list_serial_ports.py
```

### Container won't start:
```bash
# Check logs
docker logs ros2_imu_visualization

# Clean up
docker system prune
```

### Arduino not detected:
1. Check USB connection
2. Verify Arduino is programmed with IMU code
3. Test with serial monitor first

## 📁 Project Structure

```
├── Dockerfile                    # Main Docker image definition
├── docker-compose.yml           # Docker compose configuration
├── docker_setup.sh              # Linux setup script
├── docker/
│   ├── entrypoint.sh            # Container entrypoint
│   └── launch_imu.sh            # IMU launch script
└── ros2_imu_package/            # ROS2 package (copied into container)
```

## 🎭 Benefits of Docker Approach

✅ **Portable**: Works on any Linux system with Docker  
✅ **Consistent**: Same environment everywhere  
✅ **No Dependencies**: Everything included in container  
✅ **Clean**: No system pollution  
✅ **Reproducible**: Identical setup every time  
✅ **Shareable**: Easy to distribute  

## 🚀 Development Workflow

1. **Edit code** in `ros2_imu_package/`
2. **Rebuild container**: `./docker_setup.sh --build-only`
3. **Test changes**: `./docker_setup.sh --run-only`
4. **Iterate** as needed

## 📦 Container Size

- Base image: ~2GB (ROS2 Humble Desktop)
- Final image: ~2.5GB (with all dependencies)
- First build: 5-10 minutes
- Subsequent builds: 1-2 minutes (cached layers)

## 🔄 Updates

To update the container with latest changes:
```bash
./docker_setup.sh
```

The script will automatically rebuild with any code changes.
