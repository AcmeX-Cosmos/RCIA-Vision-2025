# camera_driver.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import ComposableNodeContainer, Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import yaml

def generate_launch_description():
    # 加载参数文件
    params_path = os.path.join(
        get_package_share_directory('rcia_bringup'),
        'config',
        'camera_info.yaml'
    )
    camera_params = yaml.safe_load(open(params_path))['camera_node']['ros__parameters']

    # 图像节点配置
    image_node = ComposableNode(
        package='rcia_camera_driver',
        plugin='rcia::camera_driver::CameraNode',
        name='camera_node',
        parameters=[camera_params],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    gimbal_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('rcia_bringup'), 'config', 'node_params', 'gimbal_params.yaml')))
    
    robot_gimbal_description = Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('rcia_robot_description'),
                'urdf',
                'rcia_gimbal.urdf.xacro'
            ]),
            ' xyz:="', gimbal_params['gimbal']['xyz'],
            '" rpy:="', gimbal_params['gimbal']['rpy'],
            '"'
        ])

    robot_gimbal_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_gimbal_description,
                    'publish_frequency': 1000.0}]
    )

    def get_params(name):
        return os.path.join(get_package_share_directory('rcia_bringup'), 'config', 'node_params', '{}_params.yaml'.format(name))


    # 图像订阅节点配置
    rcia_vision_detector_node = ComposableNode(
        package='rcia_vision_detector',
        plugin='rcia::vision_identify::ArmorIdentifier',
        name='rcia_vision_detector',
        parameters=[get_params("armor_identify"), camera_params],
        extra_arguments=[{'use_intra_process_comms': True}]
    )


    # 动态构建容器
    def get_camera_container(*detector_nodes):
        nodes_list = [
            image_node,
            rcia_vision_detector_node,
            *detector_nodes
        ]
        return ComposableNodeContainer(
            name='camera_container',
            namespace='camera',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=nodes_list,
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO']
        )

    # 确保组件容器优先启动
    container = ComposableNodeContainer(
        name='ComponentManager',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='both',                         # 同时输出到屏幕和日志
        ros_arguments=['--log-level', 'info']  # 调整日志级别
    )

    # 定义 SerialDriverNode 组件
    serial_driver_composable = ComposableNode(
        package='rcia_serial_driver',
        plugin='rcia::serial_driver::SerialDriverNode',
        name='serial_driver_node',
        parameters=[get_params("serial_driver")],
        extra_arguments=[{'use_intra_process_comms': True}]  # 启用进程内通信
    )

    # 定义 MathSolverNode 组件
    math_solver_composable = ComposableNode(
        package='rcia_math_solver',
        plugin='rcia::math_solver::MathSolverNode',
        name='math_solver_node',
        parameters=[get_params("math_solver_node"), camera_params],
        extra_arguments=[{'use_intra_process_comms': True}]  # 启用进程内通信
    )

    # 定义 VisionTrackerNode 组件
    vision_tracker_composable = ComposableNode(
        package='rcia_vision_tracker',
        plugin='rcia::vision_tracker::VisionTrackerNode',
        name='vision_tracker_node',
        parameters=[get_params("gimbal_control")],
        extra_arguments=[{'use_intra_process_comms': True}]  # 启用进程内通信
    )
    
    # 使用TimerAction确保容器初始化完成
    load_action = TimerAction(
        period=1.0,
        actions=[
            LoadComposableNodes(
                target_container='/ComponentManager',
                composable_node_descriptions=[serial_driver_composable, math_solver_composable, vision_tracker_composable]
            )
        ]
    )

    # 带延迟启动的容器
    delayed_camera_container = TimerAction(
        period=1.0,  # 延迟1秒启动
        actions=[get_camera_container()]
    )


    launch_description_list = [
        robot_gimbal_publisher,
        delayed_camera_container,
        container,
        load_action
    ]


    return LaunchDescription(launch_description_list)
