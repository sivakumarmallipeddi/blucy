import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, LogInfo
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_blucy_path = get_package_share_directory('blucy')
    urdf_file = os.path.join(pkg_blucy_path, 'urdf', 'blucy.urdf')
    world_file = os.path.join(pkg_blucy_path, 'world', 'underwater.world')
    gazebo_models_path = os.path.expanduser('~/.gazebo/models')
    ign_path = f"{pkg_blucy_path}:{gazebo_models_path}"

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        # Set environment variables
        SetEnvironmentVariable(name='GZ_VERSION', value='fortress'),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=ign_path
        ),
        LogInfo(msg=['IGN_GAZEBO_RESOURCE_PATH set to: ', ign_path]),
        SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'),

        # Start Gazebo Fortress (Ignition GUI + Server)
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_file, '-v', '4'],
            output='screen'
        ),

        # Publish robot TFs from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(robot_description, value_type=str)
            }]
        ),

        # Spawn Blucy drone after delay to ensure world is loaded
        TimerAction(
            period=5.0,
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_blucy',
                arguments=[
                    '-file', urdf_file,
                    '-name', 'blucy',
                    '-x', '0.0', '-y', '0.0', '-z', '5.0',
                    '-R', '0.0',   # roll 
                    '-P', '0.0',   # pitch
                    '-Y', '0.0'    # yaw
                ],
                output='screen'
            )]
        ),
    ])