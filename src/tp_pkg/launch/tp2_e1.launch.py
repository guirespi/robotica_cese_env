import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Args
    count_limit_arg = launch.actions.DeclareLaunchArgument(
        'count_limit',
        default_value='60',
        description='Limit for the service count.'
    )

    timer_period_arg = launch.actions.DeclareLaunchArgument(
        'timer_period',
        default_value='0.2',
        description='Timer period for service count'
    )

    count_trigger_arg = launch.actions.DeclareLaunchArgument(
        'count_trigger',
        default_value='50',
        description='Count trigger for reset_count service'
    )


    # Nodes
    srv_node = Node(
        package='tp_pkg',
        executable='srv',
        name='srv',
        output='screen',
        parameters=[
            {'count_limit':LaunchConfiguration('count_limit')},
            {'timer_period':LaunchConfiguration('timer_period')}
        ],
        on_exit=launch.actions.Shutdown()
    )

    # Create subscriber node
    client_node = Node(
        package='tp_pkg',
        executable='client',
        name='client',
        output='screen',
        parameters=[
            {'count_trigger':LaunchConfiguration('count_trigger')}
        ],
    )

    # Create launch description and add nodes and arguments
    ld = launch.LaunchDescription()
    ld.add_action(count_limit_arg)
    ld.add_action(timer_period_arg)
    ld.add_action(count_trigger_arg)
    ld.add_action(srv_node)
    ld.add_action(client_node)
    
    return ld