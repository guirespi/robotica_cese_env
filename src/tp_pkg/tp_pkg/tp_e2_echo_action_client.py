import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from tp_custom_interfaces.action import Echo


class EchoActionClient(Node):
    def __init__(self):
        super().__init__('echo_action_client')
        
        self.declare_parameter('echo', 'That is enough, Mr.West. Please, no more for today.')
        self.param_text = self.get_parameter('echo').get_parameter_value().string_value

        self.action_client = ActionClient(self, Echo, 'echo')


    def sendGoal(self):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "[Client]: No echo action server available. "
            )
            exit()

        goal = Echo.Goal()
        goal.echo = self.param_text

        async_goal = self.action_client.send_goal_async(
            goal,
            feedback_callback = self.feedback_callback
        )
        async_goal.add_done_callback(self.goal_response_callback)


    def feedback_callback(self, feedback):
        echo_current = feedback.feedback.echo_current
        self.get_logger().info(f'[Client]: "{echo_current}"')


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('[Client]: Server rejected goal.')
            exit()

        result_async = goal_handle.get_result_async()
        result_async.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info(
                f'[Client]: Echo completed!'
            )
        else:
            self.get_logger().info(
                f'[Client]: Status is {status}.'
            )
        
        exit()


def main(args=None):
    rclpy.init(args=args)
    node = EchoActionClient()
    node.sendGoal()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Destruir el cliente y el nodo antes de apagar
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()