import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from pynput.keyboard import Key, Listener
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Int64

class TeleopNode(Node):
    desired_distance = 0.7  # požadovaná vzdialenosť od steny v metroch
    max_linear_speed = 0.2  # maximálna lineárna rýchlosť
    max_angular_speed = 0.3  # maximálna uhlová rýchlosť

    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        #self.distance_line = None
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)
        self.distance_publisher = self.create_publisher(Int64, 'traveled_distance', 10)
        self.listener = Listener(on_press=self.on_press)
        self.listener.start()

        # Setup for Matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.closest_distance = float('inf')
        self.closest_angle = 0
        self.distance_readings = {}
        self.mode = 'manual'  # 'manual' or 'automatic'
        self.wall_approaching_in_progress = False

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Update plot
        self.ax.clear()
        self.ax.plot(x, y, '.')
        self.ax.set_xlim([-5, 5])
        self.ax.set_ylim([-5, 5])
        self.ax.set_aspect('equal', 'box')

        self.plot_min_range_line(angles, ranges, -5, 5, 'front', 'r-')  # front
        self.plot_min_range_line(angles, ranges, 45, 110, 'left', 'g-')  # left
        self.plot_min_range_line(angles, ranges, 270, 300, 'right', 'm-')  # right
        self.plot_min_range_line(angles, ranges, 43, 47, 'left-front', 'y-')  # left-front
        self.plot_min_range_line(angles, ranges, 305, 335, 'right-front', 'c-')  # right-front
        self.plot_min_range_line(angles, ranges, 133, 137, 'left-back', 'k-')  # left-back
        self.plot_min_range_line(angles, ranges, 225, 265, 'right-back', 'k-')  # right-back

        self.print_distance_info()

        if self.wall_approaching_in_progress:
            return  # Do nothing if wall_approaching is in progress

            # Call wall_approaching first
        if self.mode == 'automatic':
            # self.wall_approaching()
            # wall_approaching will set wall_approaching_in_progress flag

            # Execute parallel_approaching only if wall_approaching is not in progress
            # if not self.wall_approaching_in_progress:
            self.parallel_approaching()

        plt.draw()
        plt.pause(0.1)

    def print_distance_info(self):
        if 'front' in self.distance_readings:
            front_distance = self.distance_readings['front']['distance']
            front_angle = self.distance_readings['front']['angle']
            if front_distance > 2:
                print(f"Distance in front: inf, Angle: {front_angle}°")
            else:
                print(f"Distance in front: {front_distance} m, Angle: {front_angle}°")

        if 'left' in self.distance_readings:
            left_distance = self.distance_readings['left']['distance']
            left_angle = self.distance_readings['left']['angle']
            if 88 <= left_angle <= 92:
                print(f"Distance on the left: {left_distance} m, Angle: {left_angle}° - Robot is parallel to the wall")
            else:
                print(f"Distance on the left: {left_distance} m, Angle: {left_angle}°")

        # if 'left-front' in self.distance_readings:
        #     left_front_distance = self.distance_readings['left-front']['distance']
        #     left_front_angle = self.distance_readings['left-front']['angle']
        #     if left_front_distance > 2:
        #         print(f"Distance on the front left: inf, Angle: {left_front_angle}°")
        #     else:
        #         print(f"Distance on the front left: {left_front_distance} m, Angle: {left_front_angle}°")
        #
        # if 'left-back' in self.distance_readings:
        #     left_back_distance = self.distance_readings['left-back']['distance']
        #     left_back_angle = self.distance_readings['left-back']['angle']
        #     print(f"Distance on the back left: {left_back_distance} m, Angle: {left_back_angle}°")

        # if 'left-front' in self.distance_readings and 'left-back' in self.distance_readings:
        #     left_front_distance = self.distance_readings['left-front']['distance']
        #     left_back_distance = self.distance_readings['left-back']['distance']
        #     left_distance = self.distance_readings['left']['distance']
        #     if left_front_distance - left_distance <= 0.25 and left_back_distance - left_distance <= 0.25:
        #         print("Robot is parallel to the wall")

        # if 'right' in self.distance_readings:
        #     right_distance = self.distance_readings['right']['distance']
        #     right_angle = self.distance_readings['right']['angle']
        #     if 268 <= right_angle <= 272:
        #         print(
        # f"Distance on the right: {right_distance} m, Angle: {right_angle}° - Robot is parallel to the wall")
        #     else:
        #         print(f"Distance on the right: {right_distance} m, Angle: {right_angle}°")


    def plot_min_range_line(self, angles, ranges, angle_min, angle_max, segment_name, line_color):
        indices = np.where((angles >= np.deg2rad(angle_min)) & (angles <= np.deg2rad(angle_max)))
        segment_ranges = ranges[indices]
        if len(segment_ranges) > 0:
            relevant_indices = np.where(segment_ranges <= 2)  # Filter out ranges greater than 2 meters
            relevant_ranges = segment_ranges[relevant_indices]
            relevant_angles = angles[indices][relevant_indices]

            if len(relevant_ranges) > 0:
                min_index = np.argmin(relevant_ranges)
                min_range = relevant_ranges[min_index]
                min_angle = relevant_angles[min_index]

                # Ukladanie výsledkov
                self.distance_readings[segment_name] = {
                    'distance': min_range,
                    'angle': np.rad2deg(min_angle)  # Pre prehľadnosť ukladáme uhol v stupňoch
                }

                # Výpočet a vykreslenie
                start_x = 0.21 * np.cos(min_angle)
                start_y = 0.21 * np.sin(min_angle)
                end_x = start_x + min_range * np.cos(min_angle)
                end_y = start_y + min_range * np.sin(min_angle)
                self.ax.plot([start_x, end_x], [start_y, end_y], line_color)

    def on_press(self, key):
        twist = Twist()
        handled = False
        if hasattr(key, 'char'):
            if key.char == '1':
                self.mode = 'manual'
                print("Switched to manual control mode.")
                handled = True
            elif key.char == '2':
                self.mode = 'automatic'
                print("Switched to automatic control mode.")
                handled = True

        if self.mode == 'manual':
            if key == Key.up:
                twist.linear.x = 0.6
                handled = True
            elif key == Key.down:
                twist.linear.x = -0.6
                handled = True
            elif key == Key.left:
                twist.angular.z = 0.5
                handled = True
            elif key == Key.right:
                twist.angular.z = -0.5
                handled = True

        if handled:
            self.publisher_.publish(twist)
        if key == Key.esc:
            plt.close()
            return False  # Stop the listener

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def slow_forward(self):
        twist = Twist()
        twist.linear.x = 0.02
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def rotate_right(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.2
        self.publisher_.publish(twist)

    def rotate_left(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.2
        self.publisher_.publish(twist)

    def wall_approaching(self, desired_distance=0.5):
        if self.wall_approaching_in_progress:
            return  # Do nothing if wall_approaching is in progress

        self.wall_approaching_in_progress = True

        try:
            if 'front' in self.distance_readings:
                front_distance = self.distance_readings['front']['distance']
                print(f"Distance to wall: {front_distance} meters")
                if front_distance > 0.8:
                    self.move_forward()  # Move forward at normal speed if distance is greater than 0.8 meters
                elif 0.5 < front_distance <= 0.7:
                    self.slow_forward()  # Move forward at slower speed if distance is between 0.5 and 0.8 meters
                elif front_distance <= 0.5:
                    self.stop_robot()  # Stop if desired distance is reached
                    print("Desired distance reached. Robot has stopped.")
            else:
                print("No distance data available.")
        finally:
            self.wall_approaching_in_progress = False

    def parallel_approaching(self, desired_angle=90, angle_tolerance=5, angle_high_tolerance=5, obstacle_threshold=0.5):
        if self.wall_approaching_in_progress:
            return  # Do nothing if wall_approaching is in progress

        try:
            # if 'left' in self.distance_readings and 'right' in self.distance_readings:
            if 'left' in self.distance_readings:
                left_angle = self.distance_readings['left']['angle']
                # right_angle = self.distance_readings['right']['angle']
                # print(f"Left angle: {left_angle}°, Right angle: {right_angle}°")

                # if desired_angle - angle_tolerance <= left_angle <= desired_angle + angle_tolerance and \
                #         desired_angle - angle_tolerance <= right_angle <= desired_angle + angle_tolerance:
                if desired_angle - angle_tolerance <= left_angle <= desired_angle + angle_high_tolerance:
                    print("Robot is parallel to the wall")
                    # Check for obstacles while moving forward
                    if 'front' in self.distance_readings:
                        front_distance = self.distance_readings['front']['distance']
                        if front_distance == 0.0:
                            self.move_forward()  # Continue moving forward if distance in front is 0.0
                        elif front_distance > 0.8:
                            self.move_forward()  # Move forward at normal speed if distance is greater than 0.8 meters
                        elif 0.5 < front_distance <= 0.7:
                            self.slow_forward()  # Move forward at slower speed if distance is between 0.5 and 0.8 meters
                        elif front_distance <= 2:  # Added condition for distance <= 2
                            if front_distance < obstacle_threshold:
                                # Rotate right if obstacle detected in front
                                self.rotate_right()
                            else:
                                self.move_forward()  # Move forward if no obstacle detected
                        else:
                            self.move_forward()  # Move forward if no obstacle detected
                    else:
                        self.move_forward()  # Move forward if no front distance data available
                else:
                    print("Robot is not parallel to the wall")
                    # Perform actions to correct the robot's orientation based on angle
                    if 'left' in self.distance_readings:
                        left_distance = self.distance_readings['left']['distance']
                        if left_distance == 0.0:
                            self.rotate_left()  # Rotate left if left distance is 0.0
                        elif left_distance <= 2:  # Added condition for distance <= 2
                            if left_angle < desired_angle:
                                self.rotate_right()  # Rotate right if left side is not aligned with the desired angle
                            elif left_angle > desired_angle:
                                self.rotate_left()  # Rotate left if left side overshoots the desired angle
                            # elif right_angle < desired_angle:
                            #     self.rotate_left()  # Rotate left if right side is not aligned with the desired angle
                            # elif right_angle > desired_angle:
                            #     self.rotate_right()  # Rotate right if right side overshoots the desired angle
                            else:
                                self.stop_robot()  # Stop the robot if it's already aligned but not parallel
                    else:
                        print("Ignoring left distance > 2 meters")
            else:
                print("No angle data available.")
        finally:
            self.wall_approaching_in_progress = False


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()

    try:
        print("Use arrow keys to move the robot. Press 'ESC' to quit. LiDAR plot is shown in real-time.")
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the listener
        teleop_node.listener.stop()
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()