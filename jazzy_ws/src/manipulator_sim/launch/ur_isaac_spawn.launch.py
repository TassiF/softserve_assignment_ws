import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    start_isaac_sim = LaunchConfiguration('start_isaac_sim')
    start_planner = LaunchConfiguration('start_planner')
    random_seed = LaunchConfiguration('random_seed')
    max_cycles = LaunchConfiguration('max_cycles')

    manipulator_share = get_package_share_directory('manipulator_sim')
    urdf_xacro = os.path.join(
        manipulator_share, 'urdf', 'ur_with_gripper_tcp.urdf.xacro'
    )
    moveit_controllers = os.path.join(
        manipulator_share, 'config', 'moveit_controllers.yaml'
    )
    pick_place_config = os.path.join(
        manipulator_share, 'config', 'pick_place.yaml'
    )

    # The arm model is shared by ROS and Isaac. Isaac adds the integrated
    # Robotiq 2F-140 variant; MoveIt receives a conservative attached proxy.
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='ur', package_name='ur_moveit_config'
        )
        .robot_description(
            file_path=urdf_xacro,
            mappings={
                'name': 'ur',
                'ur_type': ur_type,
                'force_abs_paths': 'true',
            },
        )
        .robot_description_semantic(
            file_path='srdf/ur.srdf.xacro',
            mappings={'name': 'ur'},
        )
        .robot_description_kinematics(
            file_path='config/kinematics.yaml'
        )
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path=moveit_controllers)
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    launch_arguments = [
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur10e',
            description=(
                "UR arm model; the Robotiq scene requires Isaac's integrated "
                'ur10e asset.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the /clock published by Isaac Sim.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz with the UR MoveIt configuration.',
        ),
        DeclareLaunchArgument(
            'start_isaac_sim',
            default_value='true',
            description='Start the Isaac Sim standalone scene.',
        ),
        DeclareLaunchArgument(
            'start_planner',
            default_value='true',
            description='Start automatic random pick/place planning.',
        ),
        DeclareLaunchArgument(
            'random_seed',
            default_value='-1',
            description=(
                'Planner RNG seed; use -1 for nondeterministic targets.'
            ),
        ),
        DeclareLaunchArgument(
            'max_cycles',
            default_value='0',
            description='Number of pick/place cycles; 0 runs indefinitely.',
        ),
        DeclareLaunchArgument(
            'initial_joint_positions',
            default_value='-1.5708,-1.5708,1.5708,-1.5708,-1.5708,0.0',
            description=(
                'Initial positions in shoulder_pan, shoulder_lift, elbow, '
                'wrist_1, wrist_2, wrist_3 order.'
            ),
        ),
        DeclareLaunchArgument(
            'version',
            default_value='5.1.0',
            description='Isaac Sim version used when install_path is empty.',
        ),
        DeclareLaunchArgument(
            'install_path',
            default_value='/isaac-sim',
            description='Isaac Sim installation root inside the container.',
        ),
        DeclareLaunchArgument(
            'use_internal_libs',
            default_value='true',
            description=(
                "Use Isaac Sim's Python 3.11-compatible ROS libraries "
                '(required by the 5.1 container).'
            ),
        ),
        DeclareLaunchArgument(
            'dds_type',
            default_value='',
            description='Optional middleware override: fastdds, cyclonedds, or zenoh.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='',
            description='Optional USD path for non-standalone Isaac startup.',
        ),
        DeclareLaunchArgument(
            'standalone',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('manipulator_sim'),
                    'scripts',
                    'spawn_ur30.py',
                ]
            ),
            description='Isaac Python scene that spawns and bridges the UR robot.',
        ),
        DeclareLaunchArgument(
            'play_sim_on_start',
            default_value='true',
            description='Start the simulation timeline immediately.',
        ),
        DeclareLaunchArgument(
            'ros_distro',
            default_value='jazzy',
            description='ROS distribution exposed to Isaac Sim.',
        ),
        DeclareLaunchArgument(
            'ros_installation_path',
            default_value='',
            description='Optional custom ROS setup paths for Isaac Sim.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='',
            description='Set to native or webrtc for headless Isaac Sim.',
        ),
        DeclareLaunchArgument(
            'custom_args',
            default_value='',
            description='Additional Isaac Sim command-line arguments.',
        ),
        DeclareLaunchArgument(
            'exclude_install_path',
            default_value='',
            description='Paths to remove from the Isaac Sim process environment.',
        ),
    ]

    environment = [
        SetEnvironmentVariable(name='UR_ROBOT_TYPE', value=ur_type),
        SetEnvironmentVariable(
            name='UR_INITIAL_JOINTS',
            value=LaunchConfiguration('initial_joint_positions'),
        ),
        SetEnvironmentVariable(
            name='UR_HEADLESS', value=LaunchConfiguration('headless')
        ),
        SetEnvironmentVariable(name='UR_SCENE_CONFIG', value=pick_place_config),
        SetEnvironmentVariable(name='UR_RANDOM_SEED', value=random_seed),
    ]

    isaac_sim = Node(
        package='isaacsim_bringup',
        executable='run_isaacsim',
        name='isaacsim_bringup',
        output='screen',
        condition=IfCondition(start_isaac_sim),
        parameters=[
            {
                'version': LaunchConfiguration('version'),
                'install_path': LaunchConfiguration('install_path'),
                'use_internal_libs': LaunchConfiguration('use_internal_libs'),
                'dds_type': LaunchConfiguration('dds_type'),
                'gui': LaunchConfiguration('gui'),
                'standalone': LaunchConfiguration('standalone'),
                'play_sim_on_start': LaunchConfiguration('play_sim_on_start'),
                'ros_distro': LaunchConfiguration('ros_distro'),
                'ros_installation_path': LaunchConfiguration(
                    'ros_installation_path'
                ),
                'headless': LaunchConfiguration('headless'),
                'custom_args': LaunchConfiguration('custom_args'),
                'exclude_install_path': LaunchConfiguration(
                    'exclude_install_path'
                ),
            }
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': use_sim_time},
        ],
    )

    trajectory_controller = Node(
        package='manipulator_sim',
        executable='isaac_joint_trajectory_controller',
        name='isaac_joint_trajectory_controller',
        output='screen',
        parameters=[pick_place_config, {'use_sim_time': use_sim_time}],
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': use_sim_time,
                'allow_trajectory_execution': True,
                'publish_robot_description': True,
                'publish_robot_description_semantic': True,
            },
        ],
    )

    planner = Node(
        package='manipulator_sim',
        executable='random_pick_place_planner',
        name='random_pick_place_planner',
        output='screen',
        condition=IfCondition(start_planner),
        parameters=[
            moveit_config.to_dict(),
            pick_place_config,
            {
                'use_sim_time': use_sim_time,
                'random_seed': ParameterValue(random_seed, value_type=int),
                'max_cycles': ParameterValue(max_cycles, value_type=int),
            },
        ],
    )

    planner_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=planner,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason='pick/place planner exited')
                )
            ],
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('ur_moveit_config'),
                'config',
                'moveit.rviz',
            ),
        ],
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription(
        launch_arguments
        + environment
        + [
            planner_exit,
            robot_state_publisher,
            trajectory_controller,
            move_group,
            planner,
            rviz,
            isaac_sim,
        ]
    )
