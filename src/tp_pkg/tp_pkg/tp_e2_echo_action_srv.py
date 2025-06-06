import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.action import CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from tp_custom_interfaces.action import Echo

import time

class EchoActionServer(Node):
    def __init__(self):
        super().__init__('echo_action_server')
        
        self.action_server = ActionServer(
            self, 
            Echo, 
            "echo", 
            execute_callback = self.execute_callback,
            goal_callback    = self.goal_callback,    
            cancel_callback  = self.cancel_callback   
            )
        self.get_logger().info('[Server]: Let\'s echo some words!')

    def goal_callback(self, goal_handle: ServerGoalHandle):
        to_echo = goal_handle.echo.strip()
        if not to_echo:
            self.get_logger().warn('Reject: Empty goal')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT


    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('[Server]: Cancel requested.')
        return CancelResponse.ACCEPT


    def execute_callback(self, goal_handle: ServerGoalHandle):
        to_echo = goal_handle.request.echo.strip()
        echo_words = to_echo.split()

        self.get_logger().info(f'[Server]: Echoing \'{to_echo}\'')

        feedback = Echo.Feedback()
        result = Echo.Result()

        words_qty = len(echo_words)

        for word in echo_words:
            feedback.echo_current = word
            goal_handle.publish_feedback(feedback)
            time.sleep(1.0)

        result.echoed_words = words_qty
        goal_handle.succeed()
        self.get_logger().info(f'Finished echoing {words_qty} words.')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = EchoActionServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Destruir el servidor y el nodo antes de apagar
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()