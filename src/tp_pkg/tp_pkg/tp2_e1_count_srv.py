import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Int32
from std_srvs.srv import Trigger

class CountSrv(Node):

    def __init__(self):
        super().__init__('count_srv')
        # Create publisher for count topic.
        self.publisher_ = self.create_publisher(Int32, 'count_topic', 10)

        # Argument for timer period. Defaul is 0.2 seconds
        self.declare_parameter('timer_period', 0.2)
        timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        # Create timer and init count.
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
        # Argument for count limit. Default is 100.
        self.declare_parameter('count_limit', 100)
        self.count_limit = self.get_parameter('count_limit').get_parameter_value().integer_value

        # Create service to reset count.
        self.srv = self.create_service(Trigger, 'reset_count', self.reset_count_callback)

    def reset_count_callback(self, request, response):
        self.get_logger().info('Resetting count to 0.')
        self.count = 0
        response.success = True
        response.message = 'Count reset.'
        return response

    def timer_callback(self):
        msg = Int32()
        msg.data = self.count
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.data)
        self.count += 1

        if self.count >= self.count_limit:
            self.get_logger().info('Count limit reached. Bye!')
            self.timer.cancel()
            exit()


def main(args=None):
    rclpy.init(args=args)
    count_srv = CountSrv()

    try:
        rclpy.spin(count_srv)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        count_srv.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
