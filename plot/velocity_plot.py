import threading
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np

# Global variables for storing last received messages, initialized to 0
last_motor_vel_msg = Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
last_angles_msg = Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
lock = threading.Lock()

def motor_vel_callback(msg):
    global last_motor_vel_msg
    with lock:
        last_motor_vel_msg = msg

def angles_callback(msg):
    global last_angles_msg
    with lock:
        last_angles_msg = msg

def update_plot(frame, ax, start_points):
    ax.clear()
    legend_handles = []

    with lock:
        motor_vel_msg = last_motor_vel_msg
        angles_msg = last_angles_msg

    motor_velocities = motor_vel_msg.data
    angles = angles_msg.data

    # Wheel positions and corresponding start points
    wheel_positions = [
        (-0.11, 0.16),    # Left Front
        (0.11, 0.16),     # Right Front
        (-0.11, 0),       # Left Middle
        (0.11, 0),        # Right Middle
        (-0.11, -0.16),   # Left Back
        (0.11, -0.16)     # Right Back
    ]

    for i in range(len(motor_velocities)):
        magnitude = motor_velocities[i] / 60 * 2 * math.pi * 0.05
        angle_rad = angles[i]  # Assume angles are in radians
        x_start, y_start = wheel_positions[i]
        x_end = x_start + magnitude / 10 * np.cos(angle_rad + math.pi / 2)
        y_end = y_start + magnitude / 10 * np.sin(angle_rad + math.pi / 2)
        
        arrow = ax.arrow(x_start, y_start, x_end - x_start, y_end - y_start, 
                         head_width=0.01, head_length=0.01, fc='blue', ec='blue', 
                         linewidth=2)
        legend_handles.append(arrow)

        # Annotate the magnitudes
        ax.text(x_end, y_end, f'{magnitude:.2f}', color='red', fontsize=8)

    ax.set_xlim(-0.3, 0.3)
    ax.set_ylim(-0.3, 0.3)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')

    plt.draw()
    plt.pause(0.001)

def ros2_node_thread():
    rclpy.init()
    node = rclpy.create_node('plotter_node')

    # Create subscriptions
    node.create_subscription(Float32MultiArray, '/n10/motor_vel', motor_vel_callback, 10)
    node.create_subscription(Float32MultiArray, '/n10/servo_cmd_wheels', angles_callback, 10)

    rclpy.spin(node)
    rclpy.shutdown()

def main():
    # Start the ROS2 node thread
    ros2_thread = threading.Thread(target=ros2_node_thread, daemon=True)
    ros2_thread.start()

    # Set up the plot
    fig, ax = plt.subplots()

    # Main plot update loop
    while True:
        update_plot(None, ax, [])
        time.sleep(0.1)  # Update plot 10 times a second

if __name__ == '__main__':
    main()
