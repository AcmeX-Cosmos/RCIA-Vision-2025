import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node, SetParameter
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    def launch_setup(context): 
        # 读取YAML配置参数
        config_path = os.path.join(
            get_package_share_directory('rcia_bringup'),
            'config',
            'node_params',
            'gimbal_params.yaml'
        )

        with open(config_path, 'r') as f:
            gimbal_params = yaml.safe_load(f)

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

        return [
            SetParameter(name='gimbal_config', value=gimbal_params),
            
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_gimbal_publisher',
                parameters=[
                    {
                        'robot_description': robot_gimbal_description,
                        'publish_frequency': 1000.0
                    }
                ],
                remappings=[] # ('/robot_description', '/gimbal_description')
            )

            # Node(
            #     package='joint_state_publisher',
            #     executable='joint_state_publisher',
            #     name='joint_state_publisher',
            #     output='screen',
            #     parameters=[{
            #         'source_list': ['/gimbal_joint_states'],  # 订阅自定义话题
            #         'rate': 100,  # 设置发布频率
            #         'use_sim_time': True  # 若使用仿真时间需开启
            #     }]
            # ),

        ]

    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
