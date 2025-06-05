import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import Int32
from std_srvs.srv import Trigger  


class CountClient(Node):
    def __init__(self):
        super().__init__('count_client')

        self.declare_parameter('count_trigger', 50)
        self.count_trigger = self.get_parameter('count_trigger').get_parameter_value().integer_value
        self.get_logger().info(f'Reset set at ({self.count_trigger})')

        self.client = self.create_client(Trigger, '/reset_count')

        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                'No service /reset_count available. Closing...'
            )
            exit()

        self.request = Trigger.Request()

        self.subscription = self.create_subscription(
            Int32,
            'count_topic',
            self.sub_callback,
            10)

    def sub_callback(self, msg):
        self.get_logger().info(f'Count received: {msg.data}')
        if  msg.data >= self.count_trigger:
            self.get_logger().info(f'Count ({self.count_trigger})')
            self.future = self.client.call_async(self.request)
            self.future.add_done_callback(self.srv_res_cb)
    
    def srv_res_cb(self, future):
        if self.future.done() and not self.future.cancelled():
            response = self.future.result()
            if response is not None:
                self.get_logger().info(f'Success={response.success} Message="{response.message}"')
            else:
                self.get_logger().error('No response from server')


def main(args=None):
    rclpy.init(args=args)
    count_client = CountClient()
    try:
        rclpy.spin(count_client)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        count_client.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()