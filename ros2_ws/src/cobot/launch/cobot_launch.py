#!/usr/bin/env python3
"""
cobot Launch File
================

myCobot280制御ノード起動ファイル

Author: Takashi Otsuka (takatronix@gmail.com)
License: Apache-2.0
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パッケージディレクトリ取得
    pkg_dir = get_package_share_directory('cobot')
    
    # 起動引数定義
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyAMA0',
        description='Serial port for myCobot280'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate', 
        default_value='1000000',
        description='Baudrate for myCobot280'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Status publish rate (Hz)'
    )
    
    enable_safety_arg = DeclareLaunchArgument(
        'enable_safety',
        default_value='true',
        description='Enable safety system'
    )
    
    auto_startup_arg = DeclareLaunchArgument(
        'auto_startup',
        default_value='true', 
        description='Auto startup to AUTO mode'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )
    
    # メインノード
    cobot_node = Node(
        package='cobot',
        executable='cobot_node',
        name='cobot_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'enable_safety': LaunchConfiguration('enable_safety'),
            'auto_startup': LaunchConfiguration('auto_startup'),
        }],
        remappings=[
            ('cobot/joint_states', 'joint_states'),
        ]
    )
    
    # joint_state_publisher（URDFのTF配信用）
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'robot_description': open(os.path.join(pkg_dir, 'urdf', 'cobot_with_gripper.urdf')).read()
        }]
    )
    
    # robot_state_publisher（TF配信用）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(os.path.join(pkg_dir, 'urdf', 'cobot_with_gripper.urdf')).read()
        }]
    )
    
    # RViz2（オプション）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'cobot_simple.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        # 起動引数
        port_arg,
        baudrate_arg, 
        publish_rate_arg,
        enable_safety_arg,
        auto_startup_arg,
        rviz_arg,
        
        # 起動情報
        LogInfo(msg=['🚀 Launching cobot280 control system...']),
        LogInfo(msg=['📍 Port: ', LaunchConfiguration('port')]),
        LogInfo(msg=['📊 Rate: ', LaunchConfiguration('publish_rate'), 'Hz']),
        LogInfo(msg=['🛡️ Safety: ', LaunchConfiguration('enable_safety')]),
        
        # ノード
        cobot_node,
        joint_state_publisher,
        robot_state_publisher,
        rviz_node,
        
        LogInfo(msg=['✅ cobot system launched successfully']),
    ])
