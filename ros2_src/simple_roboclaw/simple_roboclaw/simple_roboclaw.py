#!/usr/bin/env python3
"""
Reads joystick input and sends percentage speed commands to RoboClaw
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from .roboclaw_3 import Roboclaw
import time

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
        self.roboclaw = Roboclaw(comport=port, rate=baudrate, retries=1)
        self.connected = self.roboclaw.Open()
        if self.connected:
            self.get_logger().info(f'Connected to RoboClaw on {port} at {baudrate} baud')
            time.sleep(0.2)  # Give RoboClaw time to respond
            
            # Try to reset encoders with timeout - skip if RoboClaw not responding
            try:
                result = self.roboclaw.ResetEncoders(self.address)
                if result:
                    self.get_logger().info(f'Successfully reset encoders on address {self.address}')
                else:
                    self.get_logger().warn(f'ResetEncoders failed - RoboClaw address {self.address} not responding. Check:')
                    self.get_logger().warn(f'  1. Is RoboClaw powered on?')
                    self.get_logger().warn(f'  2. Is the address set correctly in BasicMicro Studio?')
                    self.get_logger().warn(f'  3. Is serial mode set correctly (Packet vs Simple)?')
            except Exception as e:
                self.get_logger().error(f'Failed to reset encoders: {e}')
        else:
            self.get_logger().error(f'Failed to open serial port {port}')

        
        # Subscribe to joystick
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.left_encoder_pub = self.create_publisher(Float32, self.roboclaw_left_encoder_topic, 10)
        self.right_encoder_pub = self.create_publisher(Float32, self.roboclaw_right_encoder_topic, 10)

        period = 1.0 / max(self.publish_hz, 1e-3)
        self.timer = self.create_timer(period, self.on_timer)

        if self.connected:
            self.get_logger().info('Simple RoboClaw controller ready!')
        
        # Track command failures
        self.command_failures = 0

        
    
    def joy_callback(self, msg: Joy):
        """Convert joystick to motor speeds and send to RoboClaw"""
        
        if not self.connected:
            return
        
        # Get joystick values [-1.0, 1.0]
        left_pctg = msg.axes[self.axis_left] if len(msg.axes) > self.axis_left else 0.0
        right_pctg = msg.axes[self.axis_right] if len(msg.axes) > self.axis_right else 0.0
        
        # Apply deadzone to filter noise
        DEADZONE = 0.05
        if abs(left_pctg) < DEADZONE:
            left_pctg = 0.0
        if abs(right_pctg) < DEADZONE:
            right_pctg = 0.0
        
        # Convert to output range: [-127, 127]
        left_speed = int(left_pctg * ROBOCLAW_MAX_SPEED)
        right_speed = int(right_pctg * ROBOCLAW_MAX_SPEED)
        
        # Only log if speeds changed to reduce spam
        if not hasattr(self, '_last_speeds') or (left_speed, right_speed) != self._last_speeds:
            self.get_logger().info(f'Joystick -> Left: {left_speed} Right: {right_speed}')
            self._last_speeds = (left_speed, right_speed)
        
        self.drive_motors(left_speed, right_speed)
        
    
    def drive_motors(self, left_speed:float, right_speed:float):
        """
        Send duty cycle commands to RoboClaw motors
        Speed: [-127, 127] where negative is backward, positive is forward
        """
        if not self.connected:
            return
        
        try:
            result_m1 = False
            result_m2 = False
            
            if left_speed > 0:
                result_m1 = self.roboclaw.ForwardM1(self.address, int(left_speed))
            elif left_speed < 0:
                result_m1 = self.roboclaw.BackwardM1(self.address, int(abs(left_speed)))
            else:
                result_m1 = self.roboclaw.ForwardM1(self.address, 0)

            if right_speed > 0:
                result_m2 = self.roboclaw.ForwardM2(self.address, int(right_speed))
            elif right_speed < 0:
                result_m2 = self.roboclaw.BackwardM2(self.address, int(abs(right_speed)))
            else:
                result_m2 = self.roboclaw.ForwardM2(self.address, 0)
            
            if not result_m1 or not result_m2:
                self.command_failures += 1
                if self.command_failures % 10 == 0:  # Log every 10 failures
                    self.get_logger().warn(f'Motor commands failing: M1={result_m1}, M2={result_m2} ({self.command_failures} failures)')
            else:
                if self.command_failures > 0:
                    self.get_logger().info(f'Motor commands responding again (had {self.command_failures} failures)')
                    self.command_failures = 0
                    
        except Exception as e:
            self.get_logger().error(f'Motor control error: {e}')

    
    
    def __del__(self):
        """Stop motors on shutdown"""
        try:
            if hasattr(self, 'serial') and self.roboclaw.is_open: #TODO: check how it extends seriallib
                self.drive_motors(0, 0)  # Stop both motors
                self.roboclaw.ResetEncoders(self.address)
        except Exception as e:
            self.get_logger().error(f'System had issues shutting down: {e}')
            pass

    def on_timer(self):
        if self.connected:
            try:
                left_enc = self.roboclaw.ReadEncM1(self.address)
                right_enc = self.roboclaw.ReadEncM2(self.address)
                
                if left_enc[0]:  # Check status byte
                    left_msg = Float32()
                    left_msg.data = float(left_enc[1])
                    self.left_encoder_pub.publish(left_msg)
                
                if right_enc[0]:  # Check status byte
                    right_msg = Float32()
                    right_msg.data = float(right_enc[1])
                    self.right_encoder_pub.publish(right_msg)
            except Exception as e:
                self.get_logger().debug(f'Encoder read error: {e}')

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
