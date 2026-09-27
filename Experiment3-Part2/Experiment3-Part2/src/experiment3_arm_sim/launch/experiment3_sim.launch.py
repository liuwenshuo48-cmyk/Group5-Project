import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('experiment3_arm_sim')
    desc_pkg = get_package_share_directory('mycobot_description')

    world = PathJoinSubstitution([
        pkg, 'worlds', LaunchConfiguration('world')
    ])

    xacro_file = os.path.join(
        pkg, 'urdf', 'mecharm_270_gazebo.urdf.xacro'
    )

    controllers = os.path.join(
        pkg, 'config', 'controllers.yaml'
    )

    sorting_params = os.path.join(
        pkg, 'config', 'sorting_params.yaml'
    )

    gui = LaunchConfiguration('gui')

    # Gazebo needs both Experiment 3 and mycobot mesh resources.
    share_roots = [
        os.path.dirname(pkg),
        os.path.dirname(desc_pkg),
    ]

    # Allow Ignition Gazebo to resolve model:// resources used by
    # Experiment 3 visual models, e.g. model://xuanmai_box/...
    exp3_models = os.path.join(share_roots[0], 'experiment3_arm_sim', 'models')

    resource_env = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        ':'.join(
            share_roots +
            [exp3_models] +
            [os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]
        )
    )

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' controllers_file:=',
            controllers
        ]),
        value_type=str
    )

    gz_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world],
        output='screen',
        condition=IfCondition(gui)
    )

    gz_headless = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', world],
        output='screen',
        condition=UnlessCondition(gui)
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        output='screen'
    )

    # Experiment 3 Part 2: Grid -> Bin Action Server
    pick_and_place_server = Node(
        package='experiment3_arm_sim',
        executable='pick_and_place_server',
        parameters=[sorting_params],
        output='screen'
    )

    # Experiment 3 Part 2: Detection -> Grid -> Bin -> Action manager
    task_manager = Node(
        package='experiment3_task_manager',
        executable='task_manager',
        parameters=[{
            'image_width': 640,
            'image_height': 480,
            'use_sim_time': True
        }],
        output='screen'
    )

    # Gazebo camera -> ROS 2 image bridge
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/experiment3_camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image'
        ],
        output='screen'
    )

    # Experiment 3 Part 1: YOLO detector
    detector = Node(
        package='experiment3_vision',
        executable='detector_node',
        parameters=[{
            'image_topic': '/experiment3_camera/image_raw',
            'detection_topic': '/detections',
            'conf_threshold': 0.80
        }],
        output='screen'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # Same verified base height as Experiment 2.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mecharm',
            '-x', '0',
            '-y', '0',
            '-z', '0.75'
        ],
        output='screen'
    )

    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                name,
                '--controller-manager-timeout',
                '60'
            ],
            output='screen'
        )

    jsb = spawner('joint_state_broadcaster')
    arm = spawner('arm_controller')
    hand = spawner('hand_controller')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='experiment3_sorting_world.sdf'
        ),

        resource_env,
        gz_gui,
        gz_headless,
        clock_bridge,
        camera_bridge,
        detector,
        pick_and_place_server,
        task_manager,
        rsp,
        spawn,

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn,
                on_exit=[jsb]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=jsb,
                on_exit=[arm, hand]
            )
        ),
    ])
