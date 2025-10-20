import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Use actual path for reading URDF file
    pkg_blucy_path = get_package_share_directory('blucy')
    urdf_file = os.path.join(pkg_blucy_path, 'urdf', 'blucy_test.urdf')

    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('blucy'),
                    'launch',
                    'gazebo.launch.py'
                )
            ])
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(robot_description_config, value_type=str)
            }]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'blucy',
                '-file', urdf_file,
                'x', '0.0', 'y', '0.0', 'z', '10.0'
            ],
            output='screen'
        )
    ])