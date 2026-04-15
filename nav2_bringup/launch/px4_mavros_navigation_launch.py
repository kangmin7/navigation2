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
# Launch file for Nav2 with PX4 + MAVROS integration.
#
# MAVROS topic mapping:
#   Odometry  : /mavros/local_position/odom  -->  nav2 odom_topic (param)
#   Velocity  : nav2 cmd_vel  -->  cmd_vel_to_mavros  -->  px4_mavros_bridge
#               --> /mavros/setpoint_velocity/cmd_vel_unstamped
#   TF        : MAVROS publishes map -> odom -> base_link  (no AMCL needed)
#
# cmd_vel routing goes through px4_mavros_bridge (not directly to MAVROS) so
# the bridge can inject a zero-velocity heartbeat the moment a goal_pose is
# received.  PX4 requires setpoints at >2 Hz before accepting OFFBOARD mode,
# so the bridge starts streaming as soon as the user sets a goal in RViz.
#
# Required MAVROS configuration (mavros/launch or px4_config.yaml):
#   local_position plugin must be enabled and tf_send: true
#   The TF frames must match: map_frame_id, odom_frame_id, base_link_frame_id
#
# Usage (with slam_toolbox — no map file needed):
#   ros2 launch nav2_bringup px4_mavros_navigation_launch.py
#
# Usage (with a pre-built map):
#   ros2 launch nav2_bringup px4_mavros_navigation_launch.py \
#       map:=/path/to/map.yaml \
#       params_file:=/path/to/nav2_px4_params.yaml

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')

    # ---------------------------------------------------------------------------
    # Launch configurations
    # ---------------------------------------------------------------------------
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # MAVROS-specific configurations
    mavros_odom_topic = LaunchConfiguration('mavros_odom_topic')
    mavros_cmd_vel_topic = LaunchConfiguration('mavros_cmd_vel_topic')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    # TF remappings required when using namespaces so the node's namespace
    # can be prepended to the relative topic names.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        # Nav2 nodes use /odom — the bridge relays MAVROS odom there with
        # RELIABLE QoS so Nav2's default RELIABLE subscribers don't mismatch.
        'odom_topic': '/odom',
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    # ---------------------------------------------------------------------------
    # Declare arguments
    # ---------------------------------------------------------------------------
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (false for real PX4 hardware)')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_px4_params.yaml'),
        description='Full path to the Nav2 parameters file')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically activate the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Respawn nodes on crash')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level')

    declare_mavros_odom_topic_cmd = DeclareLaunchArgument(
        'mavros_odom_topic',
        default_value='/mavros/local_position/odom',
        description='MAVROS odometry topic (used as odom_topic for bt_navigator '
                    'and velocity_smoother)')

    declare_mavros_cmd_vel_topic_cmd = DeclareLaunchArgument(
        'mavros_cmd_vel_topic',
        default_value='/mavros/setpoint_velocity/cmd_vel_unstamped',
        description='MAVROS velocity setpoint topic that receives Nav2 cmd_vel')

    # ---------------------------------------------------------------------------
    # Navigation nodes
    # ---------------------------------------------------------------------------
    # Explicit use_sim_time override applied on top of the YAML for every node.
    # RewrittenYaml rewrites the value in the file but the node's internal ROS
    # clock can only be set via the parameters list — this dict guarantees it.
    sim_time_param = {'use_sim_time': use_sim_time}

    load_nodes = GroupAction(actions=[
        # Controller server
        # cmd_vel output is kept as 'cmd_vel_nav' here; velocity_smoother
        # produces the final cmd_vel that the bridge forwards to MAVROS.
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, sim_time_param],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, sim_time_param],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, sim_time_param],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, sim_time_param],
            arguments=['--ros-args', '--log-level', log_level],
            # Recovery behaviors publish cmd_vel directly; route through bridge.
            remappings=remappings + [('cmd_vel', 'cmd_vel_to_mavros')]),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, sim_time_param],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, sim_time_param],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        # Velocity smoother: subscribes to cmd_vel_nav (controller output),
        # publishes smoothed result to cmd_vel_to_mavros so the bridge can
        # inject it into the MAVROS setpoint stream.
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, sim_time_param],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel_to_mavros'),
            ]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            }]),

        # MAVROS bridge: republishes /mavros/local_position/odom → /odom
        # for compatibility with RViz and any external tools that expect /odom.
        Node(
            package='nav2_bringup',
            executable='px4_mavros_bridge.py',
            name='px4_mavros_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'mavros_odom_topic': mavros_odom_topic,
            }],
        ),
    ])

    # ---------------------------------------------------------------------------
    # Build the launch description
    # ---------------------------------------------------------------------------
    ld = LaunchDescription()

    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_mavros_odom_topic_cmd)
    ld.add_action(declare_mavros_cmd_vel_topic_cmd)

    ld.add_action(load_nodes)

    return ld
