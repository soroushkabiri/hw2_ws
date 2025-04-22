from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction

def generate_launch_description():
    # Declare launch argument for leader turtle name
    leader_name_arg = DeclareLaunchArgument(
        'current_leader',
        default_value='turtle1',
        description='Name of the leader turtle'
    )
    # Declare how many robots to spawn
    num_robots_arg=DeclareLaunchArgument(
        'num_robots',
        default_value='2',
        description='Number of turtles (followers+leader>=2)'
    )

    return LaunchDescription([
        leader_name_arg,num_robots_arg,
        
        Node(
            package='turtlesim',
            executable='turtlesim_node',
           name='sim'
        ),
        Node(package='dynamic_leader_follower',
             executable='turtle_tf2_broadcaster',
             name='turtle_tf2_broadcaster',
             parameters=[
                 {'num_turtles': LaunchConfiguration('num_robots')}
        ])
        ,
        Node(package='dynamic_leader_follower',
             executable='turtle_tf2_listener',
             name='turtle_tf2_listener_random',
             parameters=[
                 {'num_turtles': LaunchConfiguration('num_robots')},
                 {'current_leader': LaunchConfiguration('current_leader')}
        ])
    ])