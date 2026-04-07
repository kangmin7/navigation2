#!/usr/bin/env python3
# Copyright (c) 2024 Navigation2 Contributors
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
#
# PX4 / MAVROS bridge node for Nav2.
#
# Behaviour
# ---------
# 1. Odometry relay
#    /mavros/local_position/odom  -->  /odom
#
# 2. Direct goal tracking with pitch + roll (no yaw change)
#    - Goal position comes from /goal_pose  (map frame)
#    - Robot position + yaw come from TF lookup: map -> base_link
#      (ensures goal and position are always in the same frame,
#       even when SLAM's map->odom transform drifts)
#    - Direction vector in map frame decomposed into body-frame x/y:
#          vx_body =  cos(yaw)*vx_map + sin(yaw)*vy_map
#          vy_body = -sin(yaw)*vx_map + cos(yaw)*vy_map
#    - angular.z tracks goal direction via proportional controller (yaw_kp * yaw_error)
#
# Workflow
# --------
#   1. Set goal in RViz2  →  bridge enters ACTIVE, starts 10 Hz setpoints
#   2. Switch PX4 to OFFBOARD mode
#   3. Arm the vehicle
#   4. Drone slides to goal using pitch (linear.x) and roll (linear.y)
#   5. Within goal_tolerance  →  publishes zero vel (drone holds position)

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException


class PX4MavrosBridge(Node):

    def __init__(self):
        super().__init__('px4_mavros_bridge')

        self.declare_parameter('mavros_odom_topic', '/mavros/local_position/odom')
        self.declare_parameter('mavros_cmd_vel_topic',
                               '/mavros/setpoint_velocity/cmd_vel_unstamped')
        self.declare_parameter('setpoint_rate', 10.0)
        self.declare_parameter('max_speed', 0.5)       # m/s
        self.declare_parameter('approach_dist', 1.0)   # m — start slowing
        self.declare_parameter('goal_tolerance', 0.25) # m — stop
        self.declare_parameter('yaw_kp', 1.0)          # proportional gain for yaw
        self.declare_parameter('max_yaw_rate', 1.0)    # rad/s

        mavros_odom_topic = self.get_parameter(
            'mavros_odom_topic').get_parameter_value().string_value
        mavros_cmd_vel_topic = self.get_parameter(
            'mavros_cmd_vel_topic').get_parameter_value().string_value
        setpoint_rate = self.get_parameter(
            'setpoint_rate').get_parameter_value().double_value
        self._max_speed = self.get_parameter(
            'max_speed').get_parameter_value().double_value
        self._approach_dist = self.get_parameter(
            'approach_dist').get_parameter_value().double_value
        self._goal_tolerance = self.get_parameter(
            'goal_tolerance').get_parameter_value().double_value
        self._yaw_kp = self.get_parameter(
            'yaw_kp').get_parameter_value().double_value
        self._max_yaw_rate = self.get_parameter(
            'max_yaw_rate').get_parameter_value().double_value

        # TF — used to get robot pose in map frame
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # State
        self._active = False
        self._goal_x = 0.0
        self._goal_y = 0.0

        # Publishers
        self._setpoint_pub = self.create_publisher(
            Twist, mavros_cmd_vel_topic, 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Subscribers
        self._goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_callback, 10)
        self._odom_sub = self.create_subscription(
            Odometry, mavros_odom_topic, self._odom_callback, 10)

        self._setpoint_timer = self.create_timer(
            1.0 / setpoint_rate, self._setpoint_callback)

        self.get_logger().info(
            f'PX4 MAVROS bridge ready.\n'
            f'  Odom relay : {mavros_odom_topic} --> /odom\n'
            f'  Control    : pitch+roll toward goal in map frame, yaw fixed\n'
            f'  Max speed  : {self._max_speed} m/s  '
            f'Tolerance: {self._goal_tolerance} m')

    # ------------------------------------------------------------------

    def _goal_callback(self, msg: PoseStamped):
        self._goal_x = msg.pose.position.x
        self._goal_y = msg.pose.position.y
        if not self._active:
            self._active = True
            self.get_logger().info(
                'Goal pose received — setpoint stream ACTIVE. '
                'Switch PX4 to OFFBOARD mode now.')
        self.get_logger().info(
            f'New goal (map frame): ({self._goal_x:.2f}, {self._goal_y:.2f})')

    def _odom_callback(self, msg: Odometry):
        self._odom_pub.publish(msg)

    def _setpoint_callback(self):
        if not self._active:
            return

        # Look up robot position and yaw in map frame
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except (LookupException, ConnectivityException, ExtrapolationException):
            # TF not ready yet — publish zero to maintain OFFBOARD heartbeat
            self._setpoint_pub.publish(Twist())
            return

        pos_x = t.transform.translation.x
        pos_y = t.transform.translation.y

        # Yaw from TF quaternion (map frame)
        q = t.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        dx = self._goal_x - pos_x
        dy = self._goal_y - pos_y
        dist = math.sqrt(dx * dx + dy * dy)

        cmd = Twist()  # zero by default

        if dist > self._goal_tolerance:
            # Speed: full until approach_dist, linear ramp to zero inside it
            speed = self._max_speed
            if dist < self._approach_dist:
                speed = self._max_speed * (dist / self._approach_dist)

            # Velocity in map (ENU world) frame — MAVROS cmd_vel_unstamped
            # interprets linear.x/y as ENU velocities, not body-frame.
            cmd.linear.x = speed * dx / dist
            cmd.linear.y = speed * dy / dist

            # Yaw: proportional controller toward goal direction
            desired_yaw = math.atan2(dy, dx)
            yaw_error = math.atan2(math.sin(desired_yaw - yaw),
                                   math.cos(desired_yaw - yaw))
            cmd.angular.z = max(-self._max_yaw_rate,
                                min(self._max_yaw_rate,
                                    self._yaw_kp * yaw_error))

        self._setpoint_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PX4MavrosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
