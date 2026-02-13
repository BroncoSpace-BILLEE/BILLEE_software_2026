#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Int32
from odrive_control.msgs import ControlMessage

CONTROL_MODES = {
    0: 'VOLTAGE_CONTROL',
    1: 'TORQUE_CONTROL',
    2: 'VELOCITY_CONTROL',
    3: 'POSITION_CONTROL'
}

INPUT_MODES = {
    1: 'PASSTHROUGH',
    2: 'VEL_RAMP',
    3: 'POS_FILTER',
    4: 'MIX_CHANNELS',
    5: 'TRAPEZOID_PROFILE',
    6: 'TORQUE_RAMP',
    7: 'MIRROR',
    8: 'TUNING'
}


class JoyToControlMessageNode(Node):

    def __init__(self):
        super().__init__('joy_to_control_message')
        
        # Declare parameters
        self.declare_parameter('node_id', 1)      
        self.declare_parameter('joy_axis_vel', 1)   
        self.declare_parameter('max_velocity', 1.0) # set to one rotation per second for now
        self.declare_parameter('control_mode', 2) # velocity control
        self.declare_parameter('input_mode', 1)        # 1 = passthrough mode
        
        # Get parameters
        self.node_id = self.get_parameter('node_id').value
        self.joy_axis_vel = self.get_parameter('joy_axis_vel').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.control_mode = self.get_parameter('control_mode').value
        self.input_mode = self.get_parameter('input_mode').value
        
        # publish to odrive control messages
        self.vel_publisher = self.create_publisher(ControlMessage, f'/odrive_axis{self.node_id}/control_message', 10)

        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        self.get_logger().info(
            f'Joy to Control Message Node initialized - '
            f'Node ID: {self.node_id}, '
        )

    def joy_callback(self, msg: Joy):
        
        if len(msg.axes) <= self.joy_axis_vel:
            self.get_logger().warn(f'Joy message missing axis {self.joy_axis_vel}')
            return
        
        axis_value = msg.axes[self.joy_axis_vel]
        velocity_command = axis_value * self.max_velocity

        msg = ControlMessage()
        msg.control_mode = self.control_mode
        msg.input_vel = velocity_command
        msg.input_pos = 0.0
        msg.input_torque = 0.0
        self.input_mode = self.input_mode
        
        self.vel_publisher.publish(msg)
        
        self.get_logger().debug(
            f'Published - velocity: {velocity_command:.2f}, '
            f'control_mode: {CONTROL_MODES.get(self.control_mode, "UNKNOWN")}, input_mode: {INPUT_MODES.get(self.input_mode, "UNKNOWN")}'
        )


def main():
    rclpy.init()
    node = JoyToControlMessageNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.vel_publisher.publish(ControlMessage(input_mode=0))  # disable motor
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
