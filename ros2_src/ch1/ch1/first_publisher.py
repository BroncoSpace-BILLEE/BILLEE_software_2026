import rclpy
from rclpy.node import Node #import the node class from rclpy library

from example_interfaces.msg import String #import the string message type from example_interfaces package
from rclpy.qos import qos_profile_system_default #import the default qos profile

#qos - quality of service profiles
#String - predifned message type from example_interrfaces
#example_interfaces.msg.String

class PointsPublisher(Node):
    #PointsPublisher (our name) is a subclass of the Node class
    def __init__(self):
        super().__init__('dumbledore')

        self.points = 0
        self.publisher = self.create_publisher(msg_type=String, topic='/points',qos_profile=qos_profile_system_default)
        #create_publisher is method of node class
        self.timer = self.create_timer(1.0, self.publish_points)

    def publish_points(self):
        self.points += 10
        message = String()
        message.data = f'{self.points} points for burgerdor' #format message
        self.publisher.publish(message) #publishes message

def main(args=None):
    rclpy.init(args=args)
    publisher = PointsPublisher()
    rclpy.spin(publisher)
    #after spin shutdown
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()