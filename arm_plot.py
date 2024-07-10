import threading
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np

# Global variables for storing last received arm angles, initialized to 0
last_arm_angles_msg = Float32MultiArray(data=[0.0, 0.0])
arm_lock = threading.Lock()

def arm_callback(msg):
    global last_arm_angles_msg
    with arm_lock:
        last_arm_angles_msg.data[0] = msg.data[0] + math.pi / 2
        last_arm_angles_msg.data[1] = msg.data[1]

def update_arm_plot(frame, ax):
    ax.clear()

    with arm_lock:
        arm_angles_msg = last_arm_angles_msg
    arm_angles = arm_angles_msg.data
    
    # Arm segment lengths and start points
    segment_lengths = [0.105, 0.13]  # Example lengths for the two segments
    start_point = (0.0, 0.0)

    # Calculate end points of each segment
    x_positions = [start_point[0]]
    y_positions = [start_point[1]]

    current_angle = 0.0
    for i in range(len(segment_lengths)):
        current_angle += arm_angles[i]  # Assuming angles are in radians
        x_end = x_positions[-1] + segment_lengths[i] * np.cos(current_angle)
        y_end = y_positions[-1] + segment_lengths[i] * np.sin(current_angle)
        x_positions.append(x_end)
        y_positions.append(y_end)

    ax.plot(x_positions, y_positions, marker='o', markersize=8, color='green', linewidth=3)
    
    ax.set_xlim(-0.03, 0.3)
    ax.set_ylim(-0.03, 0.3)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title('Robot Arm Configuration')

    plt.draw()
    plt.pause(0.001)

def ros2_arm_node_thread():
    rclpy.init()
    node = rclpy.create_node('arm_plotter_node')

    # Create subscription for arm angles
    node.create_subscription(Float32MultiArray, '/n10/servo_cmd_arm', arm_callback, 10)

    rclpy.spin(node)
    rclpy.shutdown()

def main():
    # Start the ROS2 node thread for arm angles
    arm_thread = threading.Thread(target=ros2_arm_node_thread, daemon=True)
    arm_thread.start()

    # Set up the plot for arm visualization
    fig, ax = plt.subplots()

    # Main plot update loop
    while True:
        update_arm_plot(None, ax)
        time.sleep(0.1)  # Update plot 10 times a second

if __name__ == '__main__':
    main()

