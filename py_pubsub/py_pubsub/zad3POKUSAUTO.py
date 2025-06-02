import math
import time

class Odometry:
    def __init__(self):
        self.left_wheel_odom = 0
        self.right_wheel_odom = 0
        self.linear_velocity = 0
        self.pose = (0, 0, 0)  # (x, y, theta)

    def update(self, linear_distance, angular_distance):
        self.left_wheel_odom += linear_distance
        self.right_wheel_odom += linear_distance

        # Assuming linear velocity is the same for both wheels
        self.linear_velocity = linear_distance

        # Update pose
        x, y, theta = self.pose
        x += linear_distance * math.cos(theta)
        y += linear_distance * math.sin(theta)
        theta += angular_distance
        self.pose = x, y, theta

    def get_pose(self):
        return self.pose


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as OdometryMsg
from std_msgs.msg import Float64, Int32
import matplotlib.pyplot as plt
import numpy as np
from pynput.keyboard import Key, Listener


class LidarTeleop(Node):
    def __init__(self):
        super().__init__('lidar_teleop_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan,'scan',self.lidar_scan_callback,10)
        self.odom_subscription = self.create_subscription(OdometryMsg,'odom',self.odom_callback,10)
        self.listener = Listener(on_press=self.on_key_press)
        self.listener.start()

        # Setup for Matplotlib
        plt.ion()
        self.fig1, self.ax1 = plt.subplots(figsize=(8, 8))  # LiDAR plot
        self.fig2, self.ax2 = plt.subplots(figsize=(8, 8))  # Path plot
        self.scan_data = {}
        self.odometry = Odometry()  # Initialize odometry
        self.last_scan_time = time.time()  # Initialize the last scan time
        self.path_x = []
        self.path_y = []
        self.total_distance = 0

        # Publisher for total distance traveled
        self.distance_publisher_ = self.create_publisher(Float64, '/robot_distance', 10)

        # Publisher and attribute for mode
        self.mode_publisher_ = self.create_publisher(Int32, 'actual_mode', 10)
        self.mode = '0'  # Default mode is OFF

    def lidar_scan_callback(self, msg):
        current_time = time.time()
        dt = current_time - self.last_scan_time
        self.last_scan_time = current_time

        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Update LiDAR plot
        self.ax1.clear()
        self.ax1.plot(x, y, '.')
        self.ax1.set_xlim([-5, 5])
        self.ax1.set_ylim([-5, 5])
        self.ax1.set_aspect('equal', 'box')

        self.plot_min_range_line(angles, ranges, -5, 5, 'front', 'r-')  # front
        self.plot_min_range_line(angles, ranges, 88, 91, 'left', 'g-')  # left

        plt.draw()
        plt.pause(0.1)

    def plot_min_range_line(self, angles, ranges, angle_min, angle_max, segment_name, line_color):
        indices = np.where((angles >= np.deg2rad(angle_min)) & (angles <= np.deg2rad(angle_max)))
        segment_ranges = ranges[indices]
        if len(segment_ranges) > 0:
            min_index = np.argmin(segment_ranges)
            min_range = segment_ranges[min_index]
            min_angle = angles[indices][min_index]

            # Saving results
            self.scan_data[segment_name] = {
                'distance': min_range,
                'angle': np.rad2deg(min_angle)  # Save angle in degrees for readability
            }

            # Calculation and plotting
            start_x = 0.21 * np.cos(min_angle)
            start_y = 0.21 * np.sin(min_angle)
            end_x = start_x + min_range * np.cos(min_angle)
            end_y = start_y + min_range * np.sin(min_angle)
            self.ax1.plot([start_x, end_x], [start_y, end_y], line_color)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Calculate distance from previous point
        if self.path_x and self.path_y:
            last_x = self.path_x[-1]
            last_y = self.path_y[-1]
            distance = np.sqrt((x - last_x)**2 + (y - last_y)**2)
            self.total_distance += distance

            # Publish total distance traveled
            distance_msg = Float64()
            distance_msg.data = self.total_distance
            self.distance_publisher_.publish(distance_msg)

           # print(f"Total distance traveled: {self.total_distance:.2f} meters")

        # Append the current position to the path
        self.path_x.append(x)
        self.path_y.append(y)

        # Update the path plot
        self.ax2.clear()
        self.ax2.plot(self.path_x, self.path_y, 'b-')  # Blue line for the path
        self.ax2.set_aspect('equal', 'box')

        plt.draw()
        plt.pause(0.1)

    def on_key_press(self, key):
        twist = Twist()
        handled = False
        mode_msg = Int32()  # Initialize mode message

        if hasattr(key, 'char'):
            if key.char == '1':
                self.mode = 'manual'
                mode_msg.data = 1  # Set mode to manual (1)
                print("MANUAL MODE")
                handled = True
            elif key.char == '2':
                self.mode = 'automatic'
                mode_msg.data = 2  # Set mode to automatic (2)
                print("AUTO MODE")
                handled = True
            elif key.char == '0':
                self.mode = 'OFF'
                mode_msg.data = 0  # Set mode to OFF (0)
                print("OFF MODE")
                handled = True

            # Publish the current mode to the topic
            self.mode_publisher_.publish(mode_msg)

        if self.mode == 'manual':
            if key == Key.up:
                twist.linear.x = 1.0
                handled = True
            elif key == Key.down:
                twist.linear.x = -1.0
                handled = True
            elif key == Key.left:
                twist.angular.z = 1.0
                handled = True
            elif key == Key.right:
                twist.angular.z = -1.0
                handled = True
        elif self.mode == 'automatic':
            self.automatic_mode_action()  # Call the method for automatic mode actions
            handled = True

        if handled:
            self.publisher_.publish(twist)
        if key == Key.esc:
            plt.close()
            return False  # Stop the listener

    def automatic_mode_action(self):
        twist = Twist()

        # Define speed and desired distance from the wall based on Lidar data
        linear_speed = 1.0  # Set the speed
        distance_to_wall = self.scan_data.get('front', {}).get('distance', float('inf'))  # Get the distance from the front wall
        print(f"Total distance traveled:.2f meters", distance_to_wall)
        desired_distance = 0.3  # Set the desired distance from the wall

        if distance_to_wall > desired_distance:
            twist.linear.x = linear_speed
        else:
            twist.linear.x = 0.0

        # Track the wall on the left side
        left_wall_distance = self.scan_data.get('left', {}).get('distance', float('inf'))  # Get the distance from the left wall
        desired_wall_distance = 0.3  # Set the desired distance from the wall

        if left_wall_distance > desired_wall_distance:
            # Turn right
            twist.angular.z = -0.5  # Set the angular speed for turning right
        else:
            # Turn left
            twist.angular.z = 0.5  # Set the angular speed for turning left

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    lidar_teleop_node = LidarTeleop()

    try:
        print("DEFAULT MODE IS OFF, PRESS 1 to MANUAL, PRESS 2 to AUTOMATIC MODE")
        print("Press 'ESC' to quit. LiDAR and robot motion plots are shown in real-time.")
        rclpy.spin(lidar_teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the listener
        lidar_teleop_node.listener.stop()
        lidar_teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
