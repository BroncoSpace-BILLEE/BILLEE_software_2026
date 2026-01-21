import rclpy
from rclpy.node import Node


#firstnode is name NOde is class of rclpy..node
class FirstNode(Node):
    def __init__(self) -> None:
        #pass node name parameter
        super().__init__('burger')
        #timer callback for every second
        self.timer = self.create_timer(timer_period_sec=1.0, callback=self. timer_callback)

    def timer_callback(self) -> None:
        #logger method
        self.get_logger().info('do you want a burger with that')

#ROS2 entrypoint
def main(args=None):
    #initialize ROS communication
    rclpy.init(args=args)
    #initialize instance of firstnode class
    node = FirstNode()
    #spin node and wait for events
    rclpy.spin(node)
    #clan node
    node.destroy_node()
    #shutdown the libarry
    rclpy.shutdown()

#entrypoint if started via python
if __name__ == '__main__':
    main()