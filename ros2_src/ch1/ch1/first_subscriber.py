import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from example_interfaces.msg import String

class PointsSubscriber(Node):
    def __init__(self):
        super().__init__('ron_sub')
        self.msg_cnt = 0
        #creating a subscriber create_susbcription inherited from Node class
        self.subscriber = self.create_subscription(String, '/points', self.callback, qos_profile_system_default)

    def callback(self, msg:String):
        #define method that will be called when a message is received
        self.get_logger().info(f"I heard #{self.msg_cnt}: {msg.data}")
        self.msg_cnt += 1

def main(args=None):
    rclpy.init(args=args)
    subscriber = PointsSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()