#!/bin/bash

# Complete ROS2 IMU Setup Script
# This script handles everything needed to set up and run ROS2 IMU visualization

set -e  # Exit on any error

echo "=========================================="
echo "ROS2 IMU Complete Setup Script"
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
ROS2_PKG_DIR="$SCRIPT_DIR/ros2_imu_package"

# Default values
SERIAL_PORT="/dev/ttyACM0"
ROS2_WS="$HOME/ros2_ws"
SKIP_DEPS=false
SKIP_BUILD=false
AUTO_LAUNCH=true

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -p|--port)
            SERIAL_PORT="$2"
            shift 2
            ;;
        -w|--workspace)
            ROS2_WS="$2"
            shift 2
            ;;
        --skip-deps)
            SKIP_DEPS=true
            shift
            ;;
        --skip-build)
            SKIP_BUILD=true
            shift
            ;;
        --no-launch)
            AUTO_LAUNCH=false
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -p, --port PORT        Serial port (default: /dev/ttyACM0)"
            echo "  -w, --workspace DIR    ROS2 workspace directory (default: ~/ros2_ws)"
            echo "  --skip-deps           Skip dependency installation"
            echo "  --skip-build          Skip package building"
            echo "  --no-launch           Don't automatically launch visualization"
            echo "  -h, --help            Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Full setup with defaults"
            echo "  $0 -p /dev/ttyUSB0                   # Use different serial port"
            echo "  $0 --skip-deps --skip-build         # Just launch (if already set up)"
            echo "  $0 --no-launch                      # Setup but don't launch"
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
echo "  ROS2 Workspace: $ROS2_WS"
echo "  Skip Dependencies: $SKIP_DEPS"
echo "  Skip Build: $SKIP_BUILD"
echo "  Auto Launch: $AUTO_LAUNCH"
echo ""

# Step 1: Check if ROS2 is installed
print_status "Checking ROS2 installation..."
if ! command -v ros2 &> /dev/null; then
    print_error "ROS2 is not installed or not in PATH"
    echo "Please install ROS2 first: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

# Detect ROS2 distribution
ROS_DISTRO=$(ros2 --version | grep -oP 'ros2 \K\w+' || echo "humble")
print_success "Found ROS2 distribution: $ROS_DISTRO"

# Source ROS2 environment
print_status "Sourcing ROS2 environment..."
source /opt/ros/$ROS_DISTRO/setup.bash

# Step 2: Install dependencies (if not skipped)
if [ "$SKIP_DEPS" = false ]; then
    print_status "Installing system dependencies..."
    
    sudo apt update -qq
    
    # Install ROS2 packages
    print_status "Installing ROS2 packages..."
    sudo apt install -y \
        ros-$ROS_DISTRO-rviz2 \
        ros-$ROS_DISTRO-robot-state-publisher \
        ros-$ROS_DISTRO-tf2-ros \
        ros-$ROS_DISTRO-tf2-tools \
        ros-$ROS_DISTRO-sensor-msgs \
        ros-$ROS_DISTRO-geometry-msgs \
        ros-$ROS_DISTRO-std-msgs \
        python3-colcon-common-extensions \
        python3-rosdep2 || {
        
        print_warning "Some packages may not be available. Continuing anyway..."
    }
    
    # Try to install IMU plugin (may not be available in all distros)
    sudo apt install -y ros-$ROS_DISTRO-rviz-imu-plugin 2>/dev/null || {
        print_warning "rviz-imu-plugin not available for $ROS_DISTRO"
    }
    
    # Install Python dependencies
    print_status "Installing Python dependencies..."
    pip3 install pyserial tf-transformations || {
        print_warning "Some Python packages may already be installed"
    }
    
    # Add user to dialout group for serial port access
    print_status "Adding user to dialout group for serial port access..."
    sudo usermod -aG dialout $USER
    
    print_success "Dependencies installed successfully"
else
    print_status "Skipping dependency installation"
fi

# Step 3: Create ROS2 workspace
print_status "Setting up ROS2 workspace at $ROS2_WS..."
mkdir -p $ROS2_WS/src

# Step 4: Copy or link the package
print_status "Setting up ROS2 package..."
if [ -L "$ROS2_WS/src/ros2_imu_package" ]; then
    print_status "Package already linked, updating..."
    rm "$ROS2_WS/src/ros2_imu_package"
elif [ -d "$ROS2_WS/src/ros2_imu_package" ]; then
    print_status "Package already exists, updating..."
    rm -rf "$ROS2_WS/src/ros2_imu_package"
fi

# Create symbolic link to package
ln -sf "$ROS2_PKG_DIR" "$ROS2_WS/src/ros2_imu_package"
print_success "Package linked to workspace"

# Step 5: Build the package (if not skipped)
if [ "$SKIP_BUILD" = false ]; then
    print_status "Building ROS2 package..."
    cd "$ROS2_WS"
    
    # Initialize rosdep if not already done
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        print_status "Initializing rosdep..."
        sudo rosdep init 2>/dev/null || print_warning "rosdep already initialized"
    fi
    
    print_status "Updating rosdep..."
    rosdep update 2>/dev/null || print_warning "rosdep update failed, continuing..."
    
    # Install package dependencies
    print_status "Installing package dependencies..."
    rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || {
        print_warning "Some dependencies may not be available via rosdep"
    }
    
    # Build the package
    print_status "Compiling package..."
    colcon build --packages-select ros2_imu_package --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    if [ $? -eq 0 ]; then
        print_success "Package built successfully"
    else
        print_error "Package build failed"
        exit 1
    fi
else
    print_status "Skipping package build"
    cd "$ROS2_WS"
fi

# Step 6: Check serial port
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
        print_status "Setup completed. Run the script again when Arduino is connected."
        exit 0
    fi
else
    print_success "Serial port $SERIAL_PORT is available"
fi

# Step 7: Source the workspace
print_status "Sourcing workspace environment..."
source "$ROS2_WS/install/setup.bash"

# Step 8: Create launch script for future use
print_status "Creating launch script..."
cat > "$SCRIPT_DIR/launch_imu_visualization.sh" << EOF
#!/bin/bash
# Quick launch script for IMU visualization

# Source ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
source $ROS2_WS/install/setup.bash

# Launch IMU visualization
echo "Launching IMU visualization on \${1:-$SERIAL_PORT}..."
ros2 launch ros2_imu_package imu_visualization.launch.py serial_port:=\${1:-$SERIAL_PORT}
EOF

chmod +x "$SCRIPT_DIR/launch_imu_visualization.sh"
print_success "Launch script created: $SCRIPT_DIR/launch_imu_visualization.sh"

# Step 9: Launch visualization (if requested)
if [ "$AUTO_LAUNCH" = true ]; then
    print_success "Setup complete! Launching IMU visualization..."
    echo ""
    print_status "Starting ROS2 IMU visualization with the following components:"
    echo "  - IMU Serial Reader (reads from $SERIAL_PORT)"
    echo "  - TF Broadcaster (publishes transforms)"
    echo "  - Robot State Publisher (for visualization)"
    echo "  - RVIZ2 (3D visualization)"
    echo ""
    print_status "Press Ctrl+C to stop the visualization"
    echo ""
    
    # Small delay to let user read the messages
    sleep 2
    
    # Launch the visualization
    ros2 launch ros2_imu_package imu_visualization.launch.py serial_port:=$SERIAL_PORT
else
    print_success "Setup complete!"
    echo ""
    echo "To launch the IMU visualization manually, run:"
    echo "  $SCRIPT_DIR/launch_imu_visualization.sh"
    echo ""
    echo "Or with a different serial port:"
    echo "  $SCRIPT_DIR/launch_imu_visualization.sh /dev/ttyUSB0"
    echo ""
    echo "Or use ROS2 directly:"
    echo "  source $ROS2_WS/install/setup.bash"
    echo "  ros2 launch ros2_imu_package imu_visualization.launch.py serial_port:=$SERIAL_PORT"
fi
