import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare launch arguments
    echo_arg = launch.actions.DeclareLaunchArgument(
        'echo',
        default_value='I finally broke down and got myself a plasma TV',
        description='String to echo'
    )

    # Create action server node
    action_srv_node = Node(
        package='tp_pkg',
        executable='action_srv',
        name='action_srv',
        output='screen',
        parameters=[ ],
    )

    # Create action client node
    action_client_node = Node(
        package='tp_pkg',
        executable='action_client',
        name='action_client',
        output='screen',
        parameters=[
            {'echo':LaunchConfiguration('echo')}
        ],
    )

    # Create launch description and add nodes and arguments
    ld = launch.LaunchDescription()
    ld.add_action(echo_arg)
    ld.add_action(action_srv_node)
    ld.add_action(action_client_node)
    
    return ld