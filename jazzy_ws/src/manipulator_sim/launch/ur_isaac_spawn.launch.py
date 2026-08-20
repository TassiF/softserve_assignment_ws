from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource


launch_arguments = [
    DeclareLaunchArgument('ur_type', default_value='ur30', description='UR robot model type'),
    DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
    DeclareLaunchArgument('use_mock_hardware', default_value='true', description='Let Isaac Sim handle the hardware interface'),
    DeclareLaunchArgument('mock_sensor_commands', default_value='false', description='Forward sensor commands from ROS to Isaac Sim'),
    DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz for debugging'),

    DeclareLaunchArgument('version', default_value='6.0.1', description='Specify the version of Isaac Sim to use. Isaac Sim will be run from default install root folder for the specified version. Leave empty to use latest version of Isaac Sim.'),
    
    DeclareLaunchArgument('install_path', default_value='', description='If Isaac Sim is insalled in a non-default location, provide a specific path to Isaac Sim installation root folder. (If defined, "version" parameter will be ignored)'),
    
    DeclareLaunchArgument('use_internal_libs', default_value='false', description='Set to true if you wish to use internal ROS libraries shipped with Isaac Sim.'),
    
    DeclareLaunchArgument('dds_type', default_value='', description='Set to "fastdds", "cyclonedds", or "zenoh" to override RMW_IMPLEMENTATION. If left empty, the surrounding environment\'s RMW_IMPLEMENTATION is used.'),

    # UR30 Robot
    DeclareLaunchArgument('gui', default_value='https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/UniversalRobots/ur30/ur30.usd', description='Provide the path to a usd file to open it when starting Isaac Sim in standard gui mode. If left empty, Isaac Sim will open an empty stage in standard gui mode.'),
    # Franka Panda Robot
    # DeclareLaunchArgument('gui', default_value='https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd', description='Provide the path to a usd file to open it when starting Isaac Sim in standard gui mode. If left empty, Isaac Sim will open an empty stage in standard gui mode.'),
    
    DeclareLaunchArgument('standalone', default_value='', description='Provide the path to the python file to open it and start Isaac Sim in standalone workflow. If left empty, Isaac Sim will open an empty stage in standard Gui mode.'),
    
    DeclareLaunchArgument('play_sim_on_start', default_value='false', description='If enabled and Isaac Sim will start playing the scene after it is loaded. (Only applicable when in standard gui mode and loading a scene)'),
    
    DeclareLaunchArgument('ros_distro', default_value='jazzy', description='Provide ROS version to use. Only Humble and Jazzy is supported.'),
    
    DeclareLaunchArgument('ros_installation_path', default_value='', description='Comma-separated list of ROS installation paths. If ROS is installed in a non-default location (as in not under /opt/ros/), provide the path to your main setup.bash file for your ROS install. (/path/to/custom/ros/install/setup.bash). Similarly add the path to your local_setup.bash file for your workspace installation. (/path/to/custom_ros_workspace/install/local_setup.bash)'),

    DeclareLaunchArgument('headless', default_value='', description='Set to "native" or "webrtc" to run Isaac Sim with different headless modes, if left blank, Isaac Sim will run in the regular GUI workflow. This parameter can be overridden by "standalone" parameter.'),

    DeclareLaunchArgument('custom_args', default_value='', description='Add any custom Isaac Sim args that you want to forward to isaac-sim.sh during run time.'),

    DeclareLaunchArgument('exclude_install_path', default_value='', description='Comma-separated list of installation paths to exclude from LD_LIBRARY_PATH, PYTHONPATH, and PATH environment variables.'),

]

def robot_description_content():
    ur_type = LaunchConfiguration('ur_type')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')

    ur_desc_share = FindPackageShare(package='ur_description')
    urdf_xacro = PathJoinSubstitution([ur_desc_share, 'urdf', 'ur.urdf.xacro'])

    return Command([
        'xacro', ' ', urdf_xacro, ' ',
        'name:=', 'ur', ' ',
        'ur_type:=', ur_type, ' ',
        'use_mock_hardware:=', use_mock_hardware, ' ',
        'mock_sensor_commands:=', mock_sensor_commands, ' ',
        'force_abs_paths:=true'
    ])

def launch_setup(context):
    # Run isaac sim as a ROS2 node with default parameters. Parameters can be overridden here or via launch arguments from other launch files. 
    isaacsim_node = Node(
        package='isaacsim_bringup', executable='run_isaacsim',
        name='isaacsim_bringup', output="screen",
        parameters=[{
            'version': LaunchConfiguration('version'),
            'install_path': LaunchConfiguration('install_path'),
            'use_internal_libs': LaunchConfiguration('use_internal_libs'),
            'dds_type': LaunchConfiguration('dds_type'),
            'gui': LaunchConfiguration('gui'),
            'standalone': LaunchConfiguration('standalone'),
            'play_sim_on_start': LaunchConfiguration('play_sim_on_start'),
            'ros_distro': LaunchConfiguration('ros_distro'),
            'ros_installation_path': LaunchConfiguration('ros_installation_path'),
            'headless': LaunchConfiguration('headless'),
            'custom_args': LaunchConfiguration('custom_args'),
            'exclude_install_path': LaunchConfiguration('exclude_install_path')
        }]
    )
    return [isaacsim_node]

def generate_launch_description():
    # ur_type = LaunchConfiguration('ur_type')
    # use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    # mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    # urdf_xacro = PathJoinSubstitution([ur_desc_share, 'urdf', 'ur.urdf.xacro'])


    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    ur_desc_share = FindPackageShare(package='ur_description')
    rviz_cfg = PathJoinSubstitution([ur_desc_share, 'rviz', 'view_robot.rviz'])

    
    # # IsaacSim node 
    # isaacsim_node = Node(
    #     package='isaacsim_bringup', executable='run_isaacsim',
    #     name='isaacsim_bringup', output="screen",
    #     parameters=[{
    #         'version': LaunchConfiguration('version'),
    #         'install_path': LaunchConfiguration('install_path'),
    #         'use_internal_libs': LaunchConfiguration('use_internal_libs'),
    #         'dds_type': LaunchConfiguration('dds_type'),
    #         'gui': LaunchConfiguration('gui'),
    #         'standalone': LaunchConfiguration('standalone'),
    #         'play_sim_on_start': LaunchConfiguration('play_sim_on_start'),
    #         'ros_distro': LaunchConfiguration('ros_distro'),
    #         'ros_installation_path': LaunchConfiguration('ros_installation_path'),
    #         'headless': LaunchConfiguration('headless'),
    #         'custom_args': LaunchConfiguration('custom_args'),
    #         'exclude_install_path': LaunchConfiguration('exclude_install_path')
    #     }]
    # )
    # opfunc = OpaqueFunction(function = isaacsim_node)
    
    #Robot state publisher node
    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content(),
                'use_sim_time': use_sim_time
            }]
        )
    ]

    #Rviz2 node
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(use_rviz),
            parameters=[{'use_sim_time': use_sim_time}]
        )
    )

    #Joint state publisher node
    nodes.append(
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='initial_joint_states_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'zeros': {
                    'shoulder_pan_joint': 0.0,
                    'shoulder_lift_joint': -1.5708,  # -90 degrees
                    'elbow_joint': 1.5708,           #  90 degrees
                    'wrist_1_joint': -1.5708,        # -90 degrees
                    'wrist_2_joint': -1.5708,        # -90 degrees
                    'wrist_3_joint': 0.0
                }
            }]
        )
    )


    # opfunc = OpaqueFunction(function=launch_setup)
    # ld = LaunchDescription(launch_arguments)
    
    # for node in nodes:
    #     ld.add_action(node)
        
    # ld.add_action(opfunc)

    #UniversalRobots node
    ur_view = [
                IncludeLaunchDescription(
                    FrontendLaunchDescriptionSource(
                        [
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("ur_description"),
                                    "launch",
                                    "view_ur.launch.xml",
                                ]
                            )
                        ]
                    ),
                    launch_arguments={
                        "ur_type": LaunchConfiguration("ur_type"),
                    }.items(),
                )
            ]

    opfunc = OpaqueFunction(function = launch_setup)
    # ld = LaunchDescription(launch_arguments + nodes)
    ld = LaunchDescription(launch_arguments + ur_view)
    ld.add_action(opfunc)
    return ld