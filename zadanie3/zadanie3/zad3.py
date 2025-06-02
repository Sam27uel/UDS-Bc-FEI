import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as OdometryMsg
from std_msgs.msg import Float64, Int32
import matplotlib.pyplot as plt
import numpy as np
from pynput.keyboard import Key, Listener
import keyboard

class Odometry:
    def __init__(self):
        self.left_wheel_odom = 0
        self.right_wheel_odom = 0
        self.linear_velocity = 0
        self.pose = (0, 0, 0)

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
class Robot_logic(Node):
    def __init__(self):
        super().__init__('lidar_teleop_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan,'scan',self.lidar_scan_callback,10)
        self.odom_subscription = self.create_subscription(OdometryMsg,'odom',self.odom_callback,10)
        self.listener = Listener(on_press=self.handle_key_press)
        self.listener.start()

        self.distance_publisher_ = self.create_publisher(Float64, 'robot_distance', 10)
        self.velocity_publisher_ = self.create_publisher(Float64, 'robot_lin_velocity', 10)
        self.angular_publisher_ = self.create_publisher(Float64, 'robot_ang_velocity', 10)
        self.mode_publisher_ = self.create_publisher(Int32, 'actual_mode', 10)

        self.mode = '0'  # Default mode is OFF
        self.total_distance = 0
        self.linear_velocity = 0.5  # Default linear velocity
        self.angular_velocity = 1.0  # Default angular velocity


        plt.ion()
        self.fig1, self.ax1 = plt.subplots(figsize=(5, 5))  # LiDAR plot
        self.fig2, self.ax2 = plt.subplots(figsize=(5, 5))  # Path plot

        self.scan_data = {}
        self.odometry = Odometry()  # Initialize odometry
        self.last_scan_time = time.time()  # Initialize the last scan time
        self.path_x = []
        self.path_y = []

        self.ax2.set_xlim([-10, 10])
        self.ax2.set_ylim([-10, 10])
        self.ax2.set_aspect('equal', 'box')
        self.ax2.set_xlabel('X')
        self.ax2.set_ylabel('Y')
        self.ax2.set_title('Path')
        self.ax2.grid(True)
        self.distance_line, = self.ax2.plot([], [], 'r-', label='Distance')
        self.ax2.legend()

    def lidar_scan_callback(self, msg):
        current_time = time.time()
        dt = current_time - self.last_scan_time
        self.last_scan_time = current_time
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        self.ax1.clear()
        self.ax1.plot(x, y, '.r')
        self.ax1.set_xlim([-7, 7])
        self.ax1.set_ylim([-7, 7])
        self.ax1.set_aspect('equal', 'box')

        self.lidar_ranges(angles, ranges, -5, 5, 'front', 'g-')  # front
        self.lidar_ranges(angles, ranges, 45, 120, 'left', 'b-')  # left
        self.lidar_ranges(angles, ranges, 270, 300, 'right', 'm-')  # right
        plt.draw()
        plt.pause(0.1)

        if self.mode == 2:
            self.automatic_mode()
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.path_x and self.path_y:
            last_x = self.path_x[-1]
            last_y = self.path_y[-1]
            distance = np.sqrt((x - last_x)**2 + (y - last_y)**2)
            self.total_distance += distance

            distance_msg = Float64()
            distance_msg.data = self.total_distance
            self.distance_publisher_.publish(distance_msg)

            print(f"Total distance traveled: {self.total_distance:.2f} meters")

        # Append the current position to the path
        self.path_x.append(x)
        self.path_y.append(y)

        # Update the path plot
        self.ax2.clear()
        self.ax2.plot(self.path_x, self.path_y, 'b-')  # Blue line for the path
        self.ax2.set_aspect('equal', 'box')
        plt.draw()
        plt.pause(0.1)

    def lidar_ranges(self, angles, ranges, angle_min, angle_max, robot_sides, color):
        indices = np.where((angles >= np.deg2rad(angle_min)) & (angles <= np.deg2rad(angle_max)))
        sides_ranges = ranges[indices]
        if len(sides_ranges) > 0:
            index = np.argmin(sides_ranges)
            rrange = sides_ranges[index]
            angle = angles[indices][index]
            self.scan_data[robot_sides] = {'distance': rrange, 'angle': np.rad2deg(angle)}
            first_x = 0.21 * np.cos(angle)
            first_y = 0.21 * np.sin(angle)
            last_x = first_x + rrange * np.cos(angle)
            last_y = first_y + rrange * np.sin(angle)

            self.ax1.plot(
                [first_x, last_x],
                [first_y, last_y],
                color)

    def handle_key_press(self, key):
        twist = Twist()
        key_pressed = False
        mode_msg = Int32()  # Initialize mode message

        if hasattr(key, 'char'):
            if key.char == '1':
                self.mode = 1
                mode_msg.data = 1  # Set mode to MANUAL (1)
                print("MANUAL MODE")
                key_pressed = True
                if self.linear_velocity == 0.5 and self.angular_velocity == 1.0:
                    print("DEFAULT linear velocity = 0.5, DEFAULT angular velocity = 1.0")

            elif key.char == '2':
                self.mode = 2
                mode_msg.data = 2  # Set mode to AUTOMATIC (2)
                print("AUTOMATIC MODE")
                key_pressed = True

            elif key.char == '0' and (self.mode == 1 or self.mode == 2) and self.linear_velocity > 0.0 and self.angular_velocity > 0.0: #check if robot has velocity and if its in motion mode
                self.mode = 0
                mode_msg.data = 0  # Set mode to OFF (0)
                print("OFF MODE")
                key_pressed = True
                print("Robot is stopped. Turned off.")

            # Publish the current mode to the topic
            self.mode_publisher_.publish(mode_msg)

        if self.mode == 1:  # MANUAL mode
            self.initial_approach_done = False

            if key == Key.up:
                twist.linear.x = self.linear_velocity
                key_pressed = True
            elif key == Key.down:
                twist.linear.x = -self.linear_velocity
                key_pressed = True
            elif key == Key.left:
                twist.angular.z = self.angular_velocity
                key_pressed = True
            elif key == Key.right:
                twist.angular.z = -self.angular_velocity
                key_pressed = True
            elif key == Key.space:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                print("Robot has stopped!")
                key_pressed = True
            elif key == Key.shift_r:  # Increase linear velocity
                self.linear_velocity = min(self.linear_velocity + 0.1, 1.0)
                print(f"Linear velocity increased to {self.linear_velocity:.1f}")
                key_pressed = True
            elif key == Key.ctrl_r:  # Decrease linear velocity
                self.linear_velocity = max(self.linear_velocity - 0.1, 0.1)
                print(f"Linear velocity decreased to {self.linear_velocity:.1f}")
                key_pressed = True
            elif key == Key.shift_l:  # Increase linear velocity
                self.angular_velocity = min(self.angular_velocity + 0.1, 1.0)
                print(f"Angular velocity increased to {self.angular_velocity:.1f}")
                key_pressed = True
            elif key == Key.ctrl_l:  # Decrease linear velocity
                self.angular_velocity = max(self.angular_velocity - 0.1, 0.1)
                print(f"Angular velocity decreased to {self.angular_velocity:.1f}")
                key_pressed = True

        if key_pressed:
            self.publisher_.publish(twist)
            # Publish current linear velocity
            current_velocity_msg = Float64()
            current_velocity_msg.data = twist.linear.x
            self.velocity_publisher_.publish(current_velocity_msg)

            # Publish current angular velocity
            current_angular_msg = Float64()
            current_angular_msg.data = twist.angular.z
            self.angular_publisher_.publish(current_angular_msg)

    def automatic_mode(self):

            front_distance = self.scan_data['front']['distance']
            left_angle = self.scan_data['left']['angle']
            right_angle = self.scan_data['right']['angle']

            # Variable to track if the initial approach to the wall
            if 'initial_approach_done' not in self.__dict__:
                self.initial_approach_done = False

            if not self.initial_approach_done:
                if front_distance < 2.0 and front_distance > 0.7:
                    print(f"Approaching wall to {front_distance:.1f} meters")
                    twist = Twist()
                    twist.linear.x = self.linear_velocity
                    self.publisher_.publish(twist)
                elif front_distance <= 0.7:
                    # Stop and align parallel to the wall
                    print("Initial approach done. Aligning with the wall.")
                    twist = Twist()
                    twist.linear.x = 0.0
                    self.publisher_.publish(twist)
                    self.initial_approach_done = True  # Set the flag to True after initial approach

            if 85 <= left_angle <= 95:
                # paralel, begin movement

                if 'front' in self.scan_data:
                    front_distance = self.scan_data['front']['distance']
                    if front_distance == 0.0:
                        twist = Twist()
                        twist.linear.x = self.linear_velocity
                        self.publisher_.publish(twist)

                    elif front_distance > 0.7:
                        twist = Twist()
                        twist.linear.x = self.linear_velocity
                        self.publisher_.publish(twist)

                    elif front_distance <= 2.0:
                        if front_distance <= 0.7:
                            twist = Twist()
                            twist.linear.x = 0.0
                            time.sleep(0.5)
                            twist.angular.z = -4.5
                            self.publisher_.publish(twist)

            else:
                #not paralel, begin alignment
                if 'left' in self.scan_data:
                    left_distance = self.scan_data['left']['distance']
                    if left_distance == 0.0:
                        twist = Twist()
                        twist.linear.x = 0.0
                        twist.angular.z = self.angular_velocity
                        self.publisher_.publish(twist) # Rotate left

                    elif left_distance <= 2:
                        if left_angle < 90:
                            twist = Twist()
                            twist.linear.x = 0.0
                            twist.angular.z = -self.angular_velocity
                            self.publisher_.publish(twist)  # Rotate right

                        elif left_angle > 90:
                            twist = Twist()
                            twist.linear.x = 0.01
                            twist.angular.z = self.angular_velocity
                            self.publisher_.publish(twist)  # Rotate left

                        elif right_angle < 90:
                            twist = Twist()
                            twist.linear.x = 0.0
                            twist.angular.z = self.angular_velocity
                            self.publisher_.publish(twist)  # Rotate left

                        elif right_angle > 90:
                            twist = Twist()
                            twist.linear.x = 0.0
                            twist.angular.z = -self.angular_velocity
                            self.publisher_.publish(twist)  # Rotate right
                        else:
                            twist = Twist()
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            self.publisher_.publish(twist)

            mode_msg = Int32()
            mode_msg.data = 2  # Set mode to AUTOMATIC (2)
            self.mode_publisher_.publish(mode_msg)
def main(args=None):
    rclpy.init(args=args)
    robot_lidar_node = Robot_logic()

    print("DEFAULT MODE IS OFF, PRESS 1 to MANUAL, PRESS 2 to AUTOMATIC MODE")
    print("Press 'ESC' to quit")

    while True:
        rclpy.spin(robot_lidar_node)

        key = keyboard.read_event(suppress=True)
        if key.name == 'esc':
            print("ESC pressed")
            break  #

        robot_lidar_node.handle_key_press(key)

    robot_lidar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()