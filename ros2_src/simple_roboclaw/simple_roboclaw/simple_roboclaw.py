#!/usr/bin/env python3
"""
Reads joystick input and sends percentage speed commands to RoboClaw
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
#from roboclaw_3 import RoboClaw

ROBOCLAW_MAX_SPEED = 127

class RoboClawJoy(Node):
    
    def __init__(self):
        super().__init__('simple_roboclaw')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 38400)
        self.declare_parameter('address', 128)
        self.declare_parameter('axis_left', 1)  # Left stick vertical
        self.declare_parameter('axis_right', 0)     # Left stick horizontal

        self.declare_parameter('publish_hz', 20.0) #TODO: modify after testing if needed
        self.declare_parameter('roboclaw_left_encoder_topic', "/roboclaw/left_encoder_data")
        self.declare_parameter('roboclaw_right_encoder_topic', "/roboclaw/right_encoder_data")
        
        # Get parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.address = self.get_parameter('address').value
        self.axis_left = self.get_parameter('axis_left').value
        self.axis_right = self.get_parameter('axis_right').value
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.roboclaw_left_encoder_topic = self.get_parameter('roboclaw_left_encoder_topic').value
        self.roboclaw_right_encoder_topic = self.get_parameter('roboclaw_right_encoder_topic').value
        
        # Open serial port
        '''
        try:
            self.roboclaw = RoboClaw(port, baudrate)
            self.roboclaw.Open()
            self.get_logger().info(f'Connected to RoboClaw on {port}')

        except Exception as e:
            self.get_logger().error(f'Failed to connect to RoboClaw: {e}')
            raise

        try:
            self.roboclaw.ResetEncoders(self.address)
            self.get_logger().info('Successfully reset encoders')

        except Exception as e:
            self.get_logger().error(f'Failed to reset encoders: {e}')
        '''
        
        # Subscribe to joystick
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.left_encoder_pub = self.create_publisher(Float32, self.roboclaw_left_encoder_topic, 10)
        self.right_encoder_pub = self.create_publisher(Float32, self.roboclaw_right_encoder_topic, 10)

        period = 1.0 / max(self.publish_hz, 1e-3)
        self.timer = self.create_timer(period, self.on_timer)

        
        self.get_logger().info('Simple RoboClaw controller ready!')
    
    def joy_callback(self, msg: Joy):
        """Convert joystick to motor speeds and send to RoboClaw"""
        
        # Get joystick values [-1.0, 1.0]
        left_pctg = msg.axes[self.axis_left] if len(msg.axes) > 2 else 0.0
        right_pctg = msg.axes[self.axis_right] if len(msg.axes) > 2 else 0.0
        
        # Convert to output range: [-127, 127]
        left_speed = int(left_pctg * ROBOCLAW_MAX_SPEED)
        right_speed = int(right_pctg * ROBOCLAW_MAX_SPEED)
        

        self.get_logger().info(f'left: {left_pctg}% Right: {right_pctg}% -> Left: {left_speed}% Right: {right_speed}%')
        
        self.drive_motors(left_speed, right_speed)
        
    
    def drive_motors(self, left_speed:float, right_speed:float):
        """
        Send percentage speed commands to RoboClaw motors
        Speed: [-127, 127]

        """
        self.get_logger().info(f'Driving Motors with the following speeds: Left: {left_speed}% Right: {right_speed}%')

        if left_speed > 0:
            self.roboclaw.ForwardM1(self.address, left_speed)
        else:
            self.roboclaw.BackwardM1(self.address, left_speed)

        if right_speed > 0:
            self.roboclaw.ForwardM1(self.address, right_speed)
        else:
            self.roboclaw.BackwardM1(self.address, right_speed)

    
    
    def __del__(self):
        """Stop motors on shutdown"""
        '''
        try:
            if hasattr(self, 'serial') and self.roboclaw.is_open: #TODO: check how it extends seriallib
                self.drive_motors(0, 0)  # Stop both motors
                self.roboclaw.ResetEncoders(self.address)
        except Exception as e:
            self.get_logger().error(f'System had issues shutting down: {e}')
            pass
        ''' 

    def on_timer(self):
        pass
        #self.left_encoder_pub.publish(self.roboclaw.ReadEncM1(self.address))
        #self.right_encoder_pub.publish(self.roboclaw.ReadEncM2(self.address))

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RoboClawJoy()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()




if __name__ == '__main__':
    main()

    #~/ros2_ws$ ros2 pkg create simple_roboclaw  --build-type ament_python --dependencies rclpy --node-name simple_roboclaw