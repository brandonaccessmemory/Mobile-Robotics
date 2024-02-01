#! /usr/bin/python 
import launch
import launch_ros.actions

def generate_launch_description(): 
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='r2',
            executable='pub',
            output='screen',
            name='publisher'
        ),
        launch_ros.actions.Node(
            package='r2',
            executable='sub',
            output='screen',
            name='subscriber'
        ),
    ])