#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression, OrSubstitution
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess



def generate_launch_description():
    
    loader_manager_node = Node(
        package='warehouse_mas',
        executable='loader_manager.py',
        name='loader_manager_agent',
        output='screen',
    )

    robot_manager_node = Node(
        package='warehouse_mas',
        executable='robot_manager.py',
        name='robot_manager_agent',
        output='screen',
    )

    system_manager_node = Node(
        package='warehouse_mas',
        executable='system_manager.py',
        name='system_manger_agent',
        output='screen',

    )
    loaders = []
    # start loader nodes
    for i in range(1, 4):
        loader_node = Node(
            package='warehouse_mas',
            executable='loader.py',
            name=f'loader_{i}',
            output='screen',
            parameters=[{'loader_id': i}]
        )
        loaders.append(loader_node)
    
    # start courier nodes
    couriers = []
    for i in range(1, 4):
        courier_node = Node(
            package='warehouse_mas',
            executable='robot.py',
            name=f'courier_{i}',
            output='screen',
            parameters=[{'courier_id': i}]
        )
        couriers.append(courier_node)

    unloading_point_node = Node(
        package='warehouse_mas',
        executable='unloading_point.py',
        name='unloading_point_agent',
        output='screen',
    )
    return LaunchDescription([
        # system_manager_server,
        system_manager_node,
        loader_manager_node,
        robot_manager_node,
        unloading_point_node
    ] + loaders + couriers)
