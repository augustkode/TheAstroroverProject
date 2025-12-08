import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 2.0  # hvert 2. sekund bytter vi retning
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.forward = True

    def timer_callback(self):
        msg = Twist()
        if self.forward:
            msg.linear.x = 0.5   # fremover
            self.get_logger().info('Publiserer: fremover')
        else:
            msg.linear.x = -0.5  # bakover
            self.get_logger().info('Publiserer: bakover')
        self.pub.publish(msg)
        self.forward = not self.forward

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
