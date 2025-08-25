#!/bin/bash

# Docker-based ROS2 IMU Setup Script
# This script builds and runs the ROS2 IMU visualization in Docker

set -e  # Exit on any error

echo "=========================================="
echo "Docker ROS2 IMU Setup Script"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Default values
SERIAL_PORT="/dev/ttyACM0"
BUILD_ONLY=false
RUN_ONLY=false
USE_COMPOSE=true

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -p|--port)
            SERIAL_PORT="$2"
            shift 2
            ;;
        --build-only)
            BUILD_ONLY=true
            shift
            ;;
        --run-only)
            RUN_ONLY=true
            shift
            ;;
        --no-compose)
            USE_COMPOSE=false
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -p, --port PORT       Serial port (default: /dev/ttyACM0)"
            echo "  --build-only         Only build the Docker image"
            echo "  --run-only           Only run (skip build)"
            echo "  --no-compose         Use docker run instead of docker-compose"
            echo "  -h, --help           Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                           # Build and run with docker-compose"
            echo "  $0 -p /dev/ttyUSB0          # Use different serial port"
            echo "  $0 --build-only             # Just build the image"
            echo "  $0 --run-only               # Just run (if already built)"
            echo "  $0 --no-compose             # Use docker run instead"
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

print_status "Configuration:"
echo "  Serial Port: $SERIAL_PORT"
echo "  Build Only: $BUILD_ONLY"
echo "  Run Only: $RUN_ONLY"
echo "  Use Compose: $USE_COMPOSE"
echo ""

# Check Docker installation
print_status "Checking Docker installation..."
if ! command -v docker &> /dev/null; then
    print_error "Docker is not installed"
    echo "Please install Docker first:"
    echo "  curl -fsSL https://get.docker.com -o get-docker.sh"
    echo "  sudo sh get-docker.sh"
    exit 1
fi

if [ "$USE_COMPOSE" = true ]; then
    if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
        print_warning "Docker Compose not found, falling back to docker run"
        USE_COMPOSE=false
    fi
fi

print_success "Docker is available"

# Check if we're on Linux and need X11 forwarding
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    print_status "Setting up X11 forwarding for Linux..."
    xhost +local:docker 2>/dev/null || print_warning "Could not set X11 permissions"
fi

# Build Docker image (if not run-only)
if [ "$RUN_ONLY" = false ]; then
    print_status "Building Docker image..."
    cd "$SCRIPT_DIR"
    
    docker build -t ros2_imu_visualization . || {
        print_error "Docker build failed"
        exit 1
    }
    
    print_success "Docker image built successfully"
fi

# Exit if build-only
if [ "$BUILD_ONLY" = true ]; then
    print_success "Build completed. To run the container:"
    echo "  $0 --run-only"
    exit 0
fi

# Check serial port
print_status "Checking serial port availability..."
if [ ! -e "$SERIAL_PORT" ]; then
    print_warning "Serial port $SERIAL_PORT not found!"
    echo ""
    echo "Available serial ports:"
    ls /dev/tty* 2>/dev/null | grep -E "(ACM|USB)" || echo "No USB/ACM ports found"
    echo ""
    print_warning "Make sure your Arduino is connected"
    echo "You can specify a different port with: $0 -p /dev/ttyUSB0"
    echo ""
    
    # Ask user if they want to continue anyway
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_status "Stopped. Connect Arduino and try again."
        exit 0
    fi
else
    print_success "Serial port $SERIAL_PORT is available"
fi

# Run the container
print_status "Starting ROS2 IMU visualization container..."

if [ "$USE_COMPOSE" = true ]; then
    print_status "Using docker-compose..."
    cd "$SCRIPT_DIR"
    
    # Set environment variables for docker-compose
    export SERIAL_PORT="$SERIAL_PORT"
    export DISPLAY="${DISPLAY:-:0}"
    
    # Stop any existing container
    docker-compose down 2>/dev/null || true
    
    # Start the container
    docker-compose up --build
else
    print_status "Using docker run..."
    
    # Stop any existing container
    docker stop ros2_imu_visualization 2>/dev/null || true
    docker rm ros2_imu_visualization 2>/dev/null || true
    
    # Run the container
    docker run -it --rm \
        --name ros2_imu_visualization \
        --privileged \
        --network host \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e SERIAL_PORT="$SERIAL_PORT" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev:/dev \
        --device="$SERIAL_PORT:$SERIAL_PORT" \
        ros2_imu_visualization
fi

print_success "Container stopped"
