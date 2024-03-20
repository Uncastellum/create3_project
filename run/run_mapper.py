#!/usr/bin/env python

#
# run_mapper.py
#

import sys
sys.dont_write_bytecode = True

import time
import rclpy
import random
from rclpy.node import Node
from rclpy.action.client import ActionClient
from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.action import RotateAngle, DriveDistance

from map import Map

ir_sensors = {
    "ir_intensity_side_left": 0,
    "ir_intensity_left": 1,
    "ir_intensity_front_left": 2,
    "ir_intensity_front_center_left": 3,
    "ir_intensity_front_center_right": 4,
    "ir_intensity_front_right": 5,
    "ir_intensity_right": 6
}


class Create3Mapper(Node):
    def __init__(self, control = False, debug = False):
        super().__init__('create3mapper')

        if debug:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            self.get_logger().debug("Using DEBUG logger.")

        self.map = Map()
        self.random_counter = 0
        self.rotating_desv = 0
        self.last_mov = [None, True]
        self.moving = False

        self.subscription = self.create_subscription(
            IrIntensityVector, '/ir_intensity',
            (self._callback_control if control else self._callback),
            rclpy.qos.QoSProfile(depth = 0,
                reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            )
        )

        # Action clients
        # self.rotate_client = self.create_client(RotateAngle.Impl.SendGoalService, '/rotate_angle')
        # self.drive_client = self.create_client(DriveDistance.Impl.SendGoalService, '/drive_distance')
        self.rotate_client = ActionClient(self, RotateAngle, '/rotate_angle')
        self.drive_client = ActionClient(self, DriveDistance, '/drive_distance')

        while not self.rotate_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('RotateAngle action server not available, waiting again...')
        while not self.drive_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('DriveDistance action server not available, waiting again...')

        self.rotate_goal = RotateAngle.Goal()
        self.drive_goal = DriveDistance.Goal()
        self.get_logger().info('Ready! (manually controlled=' + str(control) + ')')
        self._printmap()

    def _printmap(self):
        self.get_logger().info('Updated map:')
        for p in str(self.map).split('\n'):
            self.get_logger().info('\t\t' + p)

    def _updateStatus(self, res):
        self.moving = False

    def _futureResponse(self, res):
        handle = res.result()
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._updateStatus)

    def _desviation_correction(self, clockwise):
        times = -1 if clockwise else 1
        self.rotating_desv += times
        if self.rotating_desv > 4:
            times = -3
            self.rotating_desv = 1
        elif self.rotating_desv < -4:
            times = 3
            self.rotating_desv = -1
        return times

    def _rotate90(self, clockwise = True):
        self.moving = True
        self.last_mov = [clockwise, self.last_mov[0]]
        # Desviation
        # times = -1 if clockwise else 1
        times = self._desviation_correction(clockwise)
        self.map.rotate(clockwise = clockwise)
        self.rotate_goal.angle = int(times) * 1.5708 # 90 degrees * x times
        self.rotate_goal.max_rotation_speed = 0.8
        future = self.rotate_client.send_goal_async(self.rotate_goal)
        future.add_done_callback(self._futureResponse)

    def _moveFWcell(self):
        self.moving = True
        self.last_mov = [None, self.last_mov[0]]
        self.map.moveFoward()
        self.drive_goal.distance = 0.1 # 0.1 meters
        self.drive_goal.max_translation_speed = 1.2
        future = self.drive_client.send_goal_async(self.drive_goal)
        future.add_done_callback(self._futureResponse)

    def _calculate_thresholds(self, readings):
        thlim1 = 200
        thlim2 = 350
        th_l = readings[1] > thlim1 or readings[2] > thlim2
        th_m = readings[2] > thlim2 or readings[3] > thlim2 or readings[4] > thlim2
        th_r = readings[4] > thlim2 or readings[5] > thlim1
        return [th_l, th_m or (th_l and th_r), th_r]

    def _callback(self, msg):
        if not (self.rotate_client.server_is_ready()
                and self.drive_client.server_is_ready()) or self.moving:
            time.sleep(1)
            return

        readings = [0,0,0,0,0,0,0]
        for sensor in msg.readings:
            readings[ir_sensors[sensor.header.frame_id]] = sensor.value

        obstacles = self._calculate_thresholds(readings)
        self.get_logger().debug(str(readings))
        self.get_logger().debug(str(obstacles))
        self.get_logger().debug(str(self.last_mov))

        # Check front and update
        old_obstacles = self.map.getinfo()
        if any(old_obstacles):
            old_obstacles = [a or b for a, b in zip(old_obstacles, obstacles)]
            obstacles = old_obstacles

        self.map.drawObstacle_R(readings[-1] > 250)
        self.map.drawObstacle_L(readings[0] > 250)

        if any(obstacles):
            self.map.drawObstacle(obstacles)
            self.random_counter //= 2
            if self.last_mov[0] != self.last_mov[1] and self.last_mov[0] != None:
                self._rotate90(clockwise = not bool(self.last_mov[0]))
                self.last_mov.reverse()
            elif obstacles[0] and not obstacles[2]:
                self._rotate90(clockwise = True)
            elif obstacles[2]:
                self._rotate90(clockwise = False)
            else:
                self._rotate90(clockwise = bool(random.getrandbits(1)))
        else:
            self.random_counter += 1
            if bool(self.random_counter & 1) and self.random_counter > 10 and bool(random.getrandbits(1)):
                self.random_counter = 0
                self.get_logger().info('ROLL DICE! Random rotation to avoid dead')
                self._rotate90(clockwise = bool(random.getrandbits(1)))
            else:
                self._moveFWcell()

        self._printmap()
        self.get_logger().info('----------------------------------------------')

    def _callback_control(self, msg):
        if not (self.rotate_client.server_is_ready()
                and self.drive_client.server_is_ready()) or self.moving:
            time.sleep(1)
            return

        # Here read and draw
        readings = [0,0,0,0,0,0,0]
        for sensor in msg.readings:
            readings[ir_sensors[sensor.header.frame_id]] = sensor.value
        obstacles = self._calculate_thresholds(readings)
        self.map.drawObstacle(obstacles)
        self.map.drawObstacle_R(readings[-1] > 250)
        self.map.drawObstacle_L(readings[0] > 250)

        user_input = input("Input (q/e rotate, w translate):")
        if user_input == 'q':
            self._rotate90(clockwise = False)
        elif user_input == 'w':
            if any(self.map.getinfo()) or any(obstacles):
                self.get_logger().debug("Wait to rotate.")
                return
            self._moveFWcell()
        elif user_input == 'e':
            self._rotate90(clockwise = True)
        else:
            return
        self._printmap()
        self.get_logger().info('----------------------------------------------')


def main(args = []):
    rclpy.init(args=args)
    mapper = Create3Mapper(
        control = '--control' in args,
        debug = '--debug' in args
    )
    rclpy.spin(mapper)
    mapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])
