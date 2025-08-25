"""
3D IMU Visualization Script
Reads IMU data from Arduino serial and displays orientation in real-time
"""

import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from scipy.spatial.transform import Rotation as R
import time
import threading
import queue

class IMUVisualizer:
    def __init__(self, port='COM3', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.data_queue = queue.Queue()
        self.running = False
        
        # IMU data storage
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Setup 3D plot
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.setup_plot()
        
    def setup_plot(self):
        """Setup the 3D plot appearance"""
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])
        self.ax.set_zlim([-2, 2])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('IMU 3D Orientation Visualization')
        
        # Create the IMU box (representing your sensor)
        self.create_imu_box()
        
    def create_imu_box(self):
        """Create a 3D box to represent the IMU sensor"""
        # Define box vertices (a rectangular prism)
        self.box_vertices = np.array([
            [-1, -0.5, -0.2],  # vertex 0
            [1, -0.5, -0.2],   # vertex 1
            [1, 0.5, -0.2],    # vertex 2
            [-1, 0.5, -0.2],   # vertex 3
            [-1, -0.5, 0.2],   # vertex 4
            [1, -0.5, 0.2],    # vertex 5
            [1, 0.5, 0.2],     # vertex 6
            [-1, 0.5, 0.2]     # vertex 7
        ])
        
        # Define the 12 edges of the box
        self.edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],  # bottom face
            [4, 5], [5, 6], [6, 7], [7, 4],  # top face
            [0, 4], [1, 5], [2, 6], [3, 7]   # vertical edges
        ]
        
        # Initialize empty line objects
        self.lines = []
        for edge in self.edges:
            line, = self.ax.plot3D([], [], [], 'b-', linewidth=2)
            self.lines.append(line)
            
        # Add coordinate axes
        self.axis_lines = []
        # X-axis (red)
        line, = self.ax.plot3D([], [], [], 'r-', linewidth=3)
        self.axis_lines.append(line)
        # Y-axis (green)
        line, = self.ax.plot3D([], [], [], 'g-', linewidth=3)
        self.axis_lines.append(line)
        # Z-axis (blue)
        line, = self.ax.plot3D([], [], [], 'b-', linewidth=3)
        self.axis_lines.append(line)
        
    def connect_serial(self):
        """Connect to Arduino serial port"""
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Failed to connect to {self.port}: {e}")
            return False
            
    def read_serial_data(self):
        """Read data from serial port in a separate thread"""
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    self.parse_imu_data(line)
            except Exception as e:
                print(f"Serial read error: {e}")
            time.sleep(0.01)  # Small delay to prevent excessive CPU usage
            
    def parse_imu_data(self, line):
        """Parse IMU data from Arduino output"""
        try:
            # Look for orientation data specifically
            if "ORIENTATION:" in line:
                # Extract orientation values
                parts = line.split('\t')
                if len(parts) >= 4:
                    self.roll = float(parts[1])
                    self.pitch = float(parts[2])
                    self.yaw = float(parts[3])
                    
                    # Put data in queue for main thread
                    self.data_queue.put((self.roll, self.pitch, self.yaw))
                    
            # Fallback: Calculate orientation from accelerometer if no ORIENTATION data
            elif "ACC" in line and "ORIENTATION:" not in line:
                # Extract accelerometer values
                parts = line.split('\t')
                if len(parts) >= 4:
                    acc_x = float(parts[1])
                    acc_y = float(parts[2])
                    acc_z = float(parts[3])
                    
                    # Calculate roll and pitch from accelerometer
                    # Note: This is a simplified calculation, yaw cannot be determined from accel alone
                    self.roll = np.arctan2(acc_y, np.sqrt(acc_x**2 + acc_z**2)) * 180 / np.pi
                    self.pitch = np.arctan2(-acc_x, np.sqrt(acc_y**2 + acc_z**2)) * 180 / np.pi
                    
                    # Put data in queue for main thread
                    self.data_queue.put((self.roll, self.pitch, self.yaw))
                    
        except (ValueError, IndexError) as e:
            # Silently ignore parsing errors to avoid spam
            pass
        except Exception as e:
            print(f"Data parsing error: {e}")
            
    def rotate_box(self, roll, pitch, yaw):
        """Rotate the box vertices based on roll, pitch, yaw"""
        # Convert degrees to radians
        roll_rad = np.radians(roll)
        pitch_rad = np.radians(pitch)
        yaw_rad = np.radians(yaw)
        
        # Create rotation matrix using scipy
        rotation = R.from_euler('xyz', [roll_rad, pitch_rad, yaw_rad])
        rotated_vertices = rotation.apply(self.box_vertices)
        
        return rotated_vertices
        
    def update_plot(self, frame):
        """Update the 3D plot with new IMU data"""
        # Get latest data from queue
        try:
            while not self.data_queue.empty():
                self.roll, self.pitch, self.yaw = self.data_queue.get_nowait()
        except queue.Empty:
            pass
            
        # Rotate the box
        rotated_vertices = self.rotate_box(self.roll, self.pitch, self.yaw)
        
        # Update box edges
        for i, edge in enumerate(self.edges):
            start_vertex = rotated_vertices[edge[0]]
            end_vertex = rotated_vertices[edge[1]]
            self.lines[i].set_data_3d([start_vertex[0], end_vertex[0]], 
                                    [start_vertex[1], end_vertex[1]], 
                                    [start_vertex[2], end_vertex[2]])
        
        # Update coordinate axes
        origin = np.array([0, 0, 0])
        axis_length = 1.5
        
        # Create rotation matrix for axis display
        roll_rad = np.radians(self.roll)
        pitch_rad = np.radians(self.pitch)
        yaw_rad = np.radians(self.yaw)
        rotation = R.from_euler('xyz', [roll_rad, pitch_rad, yaw_rad])
        
        # Rotate axis vectors
        x_axis = rotation.apply([axis_length, 0, 0])
        y_axis = rotation.apply([0, axis_length, 0])
        z_axis = rotation.apply([0, 0, axis_length])
        
        # Update axis lines
        self.axis_lines[0].set_data_3d([origin[0], x_axis[0]], 
                                     [origin[1], x_axis[1]], 
                                     [origin[2], x_axis[2]])
        self.axis_lines[1].set_data_3d([origin[0], y_axis[0]], 
                                     [origin[1], y_axis[1]], 
                                     [origin[2], y_axis[2]])
        self.axis_lines[2].set_data_3d([origin[0], z_axis[0]], 
                                     [origin[1], z_axis[1]], 
                                     [origin[2], z_axis[2]])
        
        # Update title with current orientation
        self.ax.set_title(f'IMU Orientation - Roll: {self.roll:.1f}° Pitch: {self.pitch:.1f}° Yaw: {self.yaw:.1f}°')
        
        return self.lines + self.axis_lines
        
    def start_visualization(self):
        """Start the 3D visualization"""
        if not self.connect_serial():
            print("Failed to connect to serial port. Using demo mode.")
            self.demo_mode()
            return
            
        self.running = True
        
        # Start serial reading thread
        serial_thread = threading.Thread(target=self.read_serial_data)
        serial_thread.daemon = True
        serial_thread.start()
        
        # Start animation
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50, blit=False, cache_frame_data=False)
        plt.show()
        
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()
            
    def demo_mode(self):
        """Demo mode with simulated data for testing"""
        self.running = True
        
        def demo_data():
            t = 0
            while self.running:
                # Generate simulated IMU data
                roll = 30 * np.sin(t * 0.5)
                pitch = 20 * np.cos(t * 0.3)
                yaw = 45 * np.sin(t * 0.2)
                
                self.data_queue.put((roll, pitch, yaw))
                time.sleep(0.05)
                t += 0.05
                
        # Start demo data thread
        demo_thread = threading.Thread(target=demo_data)
        demo_thread.daemon = True
        demo_thread.start()
        
        # Start animation
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50, blit=False, cache_frame_data=False)
        plt.show()
        
        self.running = False

def main():
    """Main function"""
    print("IMU 3D Visualizer")
    print("Make sure your Arduino is connected and sending IMU data")
    
    # Create visualizer (change COM port as needed)
    visualizer = IMUVisualizer(port='COM3', baudrate=115200)
    
    try:
        visualizer.start_visualization()
    except KeyboardInterrupt:
        print("\nVisualization stopped by user")

if __name__ == "__main__":
    main()
