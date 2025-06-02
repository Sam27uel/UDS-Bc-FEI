# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            10)
        self.subscription  # zabrániť varovaniu o nevyužitej premennej

    def listener_callback(self, msg):
        self.get_logger().info('Heard: "%s"' % msg.pose.pose)

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
