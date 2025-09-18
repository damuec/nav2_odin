#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class SteeringNode(Node):
    def __init__(self):
        super().__init__('steering_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/esp32')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('command_timeout', 1.0)  # Fixed: 1.0 instead of 1,0
        
        #  parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value
        self.command_timeout = self.get_parameter('command_timeout').value
        
        # serial connection
        self.serial_conn = None
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.connect_serial()
        
        #  subscription to nav_vel
        self.subscription = self.create_subscription(
            Twist,
            'nav_vel',
            self.cmd_vel_callback,
            10)
        
        self.get_logger().info('Subscribed to nav_vel topic')
        
        #  last command time
        self.last_command_time = self.get_clock().now().nanoseconds / 1e9
        
        # timer for safety check
        self.timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('Steering node initialized')

    def connect_serial(self):
        """Connect to ESP32 serial port with retry mechanism"""
        max_retries = 20
        retry_delay = 1  # second
        
        for attempt in range(max_retries):
            try:
                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    timeout=self.timeout,
                    write_timeout=self.timeout
                )
                self.get_logger().info(f'Connected to ESP32 on {self.serial_port}')
                
                # Wait for ESP32 to be ready
                time.sleep(2)
                
                # Read initial message
                if self.serial_conn.in_waiting > 0:
                    initial_msg = self.serial_conn.readline().decode().strip()
                    self.get_logger().info(f'ESP32 says: {initial_msg}')
                
                return
                
            except (serial.SerialException, OSError) as e:
                self.get_logger().warn(
                    f'Attempt {attempt + 1}/{max_retries}: '
                    f'Failed to connect to {self.serial_port}: {str(e)}'
                )
                time.sleep(retry_delay)
        
        self.get_logger().error(f'Could not connect to ESP32 after {max_retries} attempts')
        self.serial_conn = None

    def cmd_vel_callback(self, msg):
        """Process incoming Twist messages"""
        # Log received message for debugging
        self.get_logger().info(f'Received command: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
        
        # Convert linear and angular velocity to throttle and steering
        throttle = int(msg.linear.x * 255)  # Scale to -255 to 255
        steering = float(msg.angular.z)     # Radians
        
        # Constrain values
        throttle = max(-255, min(255, throttle))
        steering = max(-0.5, min(0.5, steering))
        
        # Send command to ESP32
        command = f"T{throttle},S{steering:.3f}\n"
        
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.write(command.encode())
                self.last_command_time = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info(f'Sent command: {command.strip()}')
                
                # Read response (non-blocking)
                if self.serial_conn.in_waiting > 0:
                    response = self.serial_conn.readline().decode().strip()
                    if response.startswith('OK:'):
                        self.get_logger().debug(f'ESP32 response: {response}')
                        self.get_logger().info(f"Converted - Throttle: {throttle}, Steering: {steering}")
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial communication error: {str(e)}')
            self.reconnect_serial()

    def safety_check(self):
        """Stop the robot if no commands received recently"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_command_time > self.command_timeout:
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.write("T0,S0\n".encode())
                    # Only log warning once to avoid spamming
                    if current_time - self.last_command_time < self.command_timeout + 1.0:
                        self.get_logger().warn('No commands received - stopping robot')
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Safety check error: {str(e)}')
                self.reconnect_serial()

    def reconnect_serial(self):
        """Reconnect to serial port"""
        self.get_logger().warn('Attempting to reconnect to ESP32...')
        try:
            if self.serial_conn:
                self.serial_conn.close()
        except:
            pass
        
        time.sleep(1)
        self.connect_serial()

    def destroy_node(self):
        """Cleanup before shutdown"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.write("T0,S0\n".encode())  # Stop robot
                time.sleep(0.1)
                self.serial_conn.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    steering_node = SteeringNode()
    
    try:
        rclpy.spin(steering_node)
    except KeyboardInterrupt:
        steering_node.get_logger().info('Keyboard interrupt, shutting down.')
    except Exception as e:
        steering_node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        try:
            steering_node.destroy_node()
        except:
            pass
        try:
            rclpy.try_shutdown()
        except:
            pass

if __name__ == '__main__':
    main()