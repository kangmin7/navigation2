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
# 2. Path-following velocity forwarding
#    - Nav2 (MPPI) computes cmd_vel in body frame (FLU) and publishes to
#      cmd_vel_to_mavros via the velocity smoother.
#    - Bridge rotates linear.x/y from body frame → ENU world frame using
#      current yaw from TF (map → base_link), then forwards to MAVROS.
#    - angular.z tracks goal direction via a proportional yaw controller.
#      (MPPI runs with wz_max=0 so Nav2 never commands yaw directly.)
#
# Frame convention
# ----------------
#   MAVROS setpoint_velocity/cmd_vel_unstamped expects ENU world-frame
#   velocities for linear.x/y (it converts ENU→NED internally).
#   Nav2 cmd_vel is body-frame FLU.  Rotation body→world:
#       vx_enu = cos(yaw)*vx_body - sin(yaw)*vy_body
#       vy_enu = sin(yaw)*vx_body + cos(yaw)*vy_body
#
# Workflow
# --------
#   1. Set goal in RViz2  →  bridge enters ACTIVE, starts 10 Hz setpoints
#   2. Switch PX4 to OFFBOARD mode
#   3. Arm the vehicle
#   4. Nav2 plans a path and MPPI tracks it; bridge converts and forwards
#      the smoothed cmd_vel to MAVROS while adding a yaw rate command.
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
        self.declare_parameter('nav2_cmd_vel_topic', 'cmd_vel_to_mavros')
        self.declare_parameter('setpoint_rate', 10.0)
        self.declare_parameter('goal_tolerance', 0.25)  # m — stop
        self.declare_parameter('yaw_kp', 1.0)           # proportional gain for yaw
        self.declare_parameter('max_yaw_rate', 1.0)     # rad/s

        mavros_odom_topic = self.get_parameter(
            'mavros_odom_topic').get_parameter_value().string_value
        mavros_cmd_vel_topic = self.get_parameter(
            'mavros_cmd_vel_topic').get_parameter_value().string_value
        nav2_cmd_vel_topic = self.get_parameter(
            'nav2_cmd_vel_topic').get_parameter_value().string_value
        setpoint_rate = self.get_parameter(
            'setpoint_rate').get_parameter_value().double_value
        self._goal_tolerance = self.get_parameter(
            'goal_tolerance').get_parameter_value().double_value
        self._yaw_kp = self.get_parameter(
            'yaw_kp').get_parameter_value().double_value
        self._max_yaw_rate = self.get_parameter(
            'max_yaw_rate').get_parameter_value().double_value

        # TF — used to get current yaw for body→world rotation and yaw control
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # State
        self._active = False
        self._goal_x = 0.0
        self._goal_y = 0.0
        self._nav2_cmd = Twist()  # latest cmd_vel from Nav2 (body frame)

        # Publishers
        self._setpoint_pub = self.create_publisher(
            Twist, mavros_cmd_vel_topic, 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Subscribers
        self._goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_callback, 10)
        self._odom_sub = self.create_subscription(
            Odometry, mavros_odom_topic, self._odom_callback, 10)
        self._nav2_cmd_sub = self.create_subscription(
            Twist, nav2_cmd_vel_topic, self._nav2_cmd_callback, 10)

        self._setpoint_timer = self.create_timer(
            1.0 / setpoint_rate, self._setpoint_callback)

        self.get_logger().info(
            f'PX4 MAVROS bridge ready.\n'
            f'  Odom relay  : {mavros_odom_topic} --> /odom\n'
            f'  Nav2 cmd_vel: {nav2_cmd_vel_topic} (body frame) --> {mavros_cmd_vel_topic} (ENU)\n'
            f'  Goal tol    : {self._goal_tolerance} m')

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

    def _nav2_cmd_callback(self, msg: Twist):
        self._nav2_cmd = msg

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
            # Rotate Nav2 body-frame cmd_vel → ENU world frame
            vx_body = self._nav2_cmd.linear.x
            vy_body = self._nav2_cmd.linear.y
            cmd.linear.x = math.cos(yaw) * vx_body - math.sin(yaw) * vy_body
            cmd.linear.y = math.sin(yaw) * vx_body + math.cos(yaw) * vy_body

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
