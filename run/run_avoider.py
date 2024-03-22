#!/usr/bin/env python

#
# run_avoider.py
#

import sys
sys.dont_write_bytecode = True

import time
import rclpy
import random as rd
from rclpy.node import Node
#from sensor_msgs.msg import LaserScan, Range
from irobot_create_msgs.msg import IrIntensity, IrIntensityVector
from geometry_msgs.msg import Twist

ir_sensors = {
    "ir_intensity_side_left": 0,
    "ir_intensity_left": 1,
    "ir_intensity_front_left": 2,
    "ir_intensity_front_center_left": 3,
    "ir_intensity_front_center_right": 4,
    "ir_intensity_front_right": 5,
    "ir_intensity_right": 6
}

class ObstacleAvoider(Node):
    def __init__(self, random = False, debug = False):
        super().__init__('obstacle_avoider')

        self.random = random
        self.random_counter = 0
        self.get_logger().info('Launch Avoider(random=' + str(self.random) + ')')
        self.log = True

        if debug:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            self.get_logger().debug("Using DEBUG logger.")

        qos_profile = rclpy.qos.QoSProfile(
            depth = 0,  # Set an appropriate depth value
            reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.subscription = self.create_subscription(
            #LaserScan,
            #'/scan',
            IrIntensityVector,
            '/ir_intensity',
            self.scan_callback,
            qos_profile
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.twist_msg = Twist()

    def scan_callback(self, msg):
        # Adjust these parameters based on your sensor characteristics (for LaserScan)
        # min_distance = 0.3  # Minimum distance to obstacle
        # max_distance = 5.0  # Maximum detection distance
        threshold = 150

        degrees = [-0.5, -0.87, -1.22, -1.57, 1.22, 0.87, 0.5]
        debug = [0,0,0,0,0,0,0]
        for sensor in msg.readings:
            debug[ir_sensors[sensor.header.frame_id]] = sensor.value

        self.get_logger().debug(str(debug))
        self.get_logger().debug(str([threshold < x for x in debug]))
        self.get_logger().debug(str(self.random_counter))

        #obstacle_detected = any( threshold < sensor.value for sensor in msg.readings )
        # obstacle_detected = any(
        #     min_distance < range_val < max_distance for range_val in msg.ranges
        # )
        obstacle_detected = any( threshold < x for x in debug )

        if obstacle_detected:
            if self.log:
                self.get_logger().info('Rotating!')
                self.log = False
            self.random_counter = 0
            # Obstacle detected, stop moving
            self.twist_msg.linear.x = 0.0
            # Rotate to avoid obstacle
            # self.twist_msg.angular.z = degrees[[threshold < x for x in debug].index(True)]
            self.twist_msg.angular.z = 1.57
        else:
            if not self.log:
                self.get_logger().info('Moving Forward!')
                self.log = True
            self.random_counter += 1
            # No obstacle, move forward
            self.twist_msg.linear.x = 0.1
            self.twist_msg.angular.z = 0.0

            # Random rotation
            if self.random and self.random_counter > 200 and rd.randint(1, 10) == 5:
                self.get_logger().info('Random small rotation!')
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = [-1.5, 1.5][rd.getrandbits(1)]
                self.random_counter = 0
                self.publisher.publish(self.twist_msg)
                time.sleep(0.5)
                return

        # Publish the twist message
        self.publisher.publish(self.twist_msg)

def main(args = []):
    rclpy.init(args=args)
    avoider = ObstacleAvoider(
        random = '--random' in args,
        debug = '--debug' in args
    )
    rclpy.spin(avoider)
    avoider.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])
