"""Isaac Sim counterpart of sambot_description/gazebo_display.launch.py.

Starts Isaac Sim (scripts/run_sim.py) plus the same ROS side as the Gazebo setup:
robot_state_publisher, robot_localization EKF, RViz, and the cmd_vel relay
(/demo/cmd_vel TwistStamped -> /isaac/cmd_vel Twist).

    ros2 launch sambot_isaac isaac_display.launch.py [headless:=true] [world:=/abs/path/world.usda]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

# Variables from a sourced ROS workspace that must not leak into Isaac's bundled Python/ROS.
_ROS_ENV_VARS = ("PYTHONPATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH",
                 "LD_LIBRARY_PATH", "ROS_PACKAGE_PATH")


def _isaac_process(context):
    isaac_path = os.path.expanduser(LaunchConfiguration('isaac_sim_path').perform(context))
    pkg_share = get_package_share_directory('sambot_isaac')

    env = {k: v for k, v in os.environ.items() if k not in _ROS_ENV_VARS}
    env.update({
        'ROS_DISTRO': 'jazzy',
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'LD_LIBRARY_PATH': os.path.join(isaac_path, 'exts', 'isaacsim.ros2.core', 'jazzy', 'lib'),
    })

    cmd = [os.path.join(isaac_path, 'python.sh'), os.path.join(pkg_share, 'scripts', 'run_sim.py')]
    if LaunchConfiguration('headless').perform(context).lower() == 'true':
        cmd.append('--headless')
    world = LaunchConfiguration('world').perform(context)
    if world:
        cmd += ['--world', world]
    return [ExecuteProcess(cmd=cmd, env=env, output='screen', name='isaac_sim')]


def _check_ros_env():
    # The ROS nodes inherit this shell's environment. Isaac's bundled ROS libs on LD_LIBRARY_PATH
    # (e.g. left over from running python.sh by hand) make every system ROS node crash on load.
    lib_path = os.environ.get('LD_LIBRARY_PATH', '').split(':')
    if any('isaacsim' in p for p in lib_path):
        raise RuntimeError(
            "LD_LIBRARY_PATH contains Isaac Sim's bundled ROS libs, which breaks the system ROS nodes.\n"
            "Launch from a fresh terminal: source /opt/ros/jazzy/setup.zsh && source install/setup.zsh\n"
            "(this launch file sets Isaac's own environment for the Isaac process).")


def generate_launch_description():
    _check_ros_env()
    desc_share = get_package_share_directory('sambot_description')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
                    {'use_sim_time': use_sim_time}],
    )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(desc_share, 'config', 'ekf.yaml'), {'use_sim_time': use_sim_time}],
    )
    cmd_vel_relay_node = Node(
        package='sambot_isaac',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use Isaac Sim /clock'),
        # Same model as the Gazebo launch: it also defines imu_link, which the URDF lacks.
        DeclareLaunchArgument('model', default_value=os.path.join(desc_share, 'urdf', 'sambot_odometry_sensors.sdf'),
                              description='Robot description for robot_state_publisher'),
        DeclareLaunchArgument('rvizconfig', default_value=os.path.join(desc_share, 'rviz', 'config.rviz'),
                              description='Absolute path to rviz config file'),
        DeclareLaunchArgument('rviz', default_value='True', description='Start RViz'),
        DeclareLaunchArgument('headless', default_value='False', description='Run Isaac Sim without a window'),
        DeclareLaunchArgument('world', default_value='',
                              description='World USD (default: sambot_isaac usd/worlds/square_room.usda)'),
        DeclareLaunchArgument('isaac_sim_path', default_value='~/apps/robotics/isaacsim',
                              description='Isaac Sim install directory'),
        OpaqueFunction(function=_isaac_process),
        robot_state_publisher_node,
        robot_localization_node,
        cmd_vel_relay_node,
        rviz_node,
    ])
