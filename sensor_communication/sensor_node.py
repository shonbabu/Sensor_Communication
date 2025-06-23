#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32, Int16
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Temperature
import socket
import threading
import struct
import time

class SensorCommunicationNode(Node):
    def __init__(self):
        super().__init__('sensor_communication_node')
        
        # Declare parameters
        self.declare_parameter('sensor_host', 'localhost')
        self.declare_parameter('sensor_port', 2000)
        self.declare_parameter('default_interval', 1000)
        self.declare_parameter('auto_start', True)
        
        # Get parameters
        self.sensor_host = self.get_parameter('sensor_host').value
        self.sensor_port = self.get_parameter('sensor_port').value
        self.default_interval = self.get_parameter('default_interval').value
        self.auto_start = self.get_parameter('auto_start').value
        
        # Socket connection
        self.socket = None
        self.connected = False
        self.receiving = False
        self.receive_thread = None
        
        # Create publishers for sensor data
        self.setup_publishers()
        
        # Create subscribers for control commands
        self.setup_subscribers()
        
        # Connect to sensor
        self.connect_to_sensor()
        
        # Auto-start if enabled
        if self.auto_start:
            self.get_logger().info(f"Auto-starting sensor with {self.default_interval}ms interval")
            self.send_start_command(self.default_interval)
        
        self.get_logger().info("Sensor Communication Node initialized")
    
    def setup_publishers(self):
        """Setup ROS2 publishers for sensor data"""
        # Supply voltage (Float32) - in Volts
        self.supply_voltage_pub = self.create_publisher(
            Float32, 
            'sensor/supply_voltage', 
            10
        )
        
        # Environment temperature (Temperature message)
        self.env_temp_pub = self.create_publisher(
            Temperature,
            'sensor/environment_temperature',
            10
        )
        
        # Orientation angles (Vector3) - in degrees
        self.orientation_pub = self.create_publisher(
            Vector3,
            'sensor/orientation',
            10
        )
        
        # Individual angle publishers for convenience
        self.yaw_pub = self.create_publisher(Int16, 'sensor/yaw', 10)
        self.pitch_pub = self.create_publisher(Int16, 'sensor/pitch', 10)
        self.roll_pub = self.create_publisher(Int16, 'sensor/roll', 10)
        
        self.get_logger().info("Publishers created:")
        self.get_logger().info("  - /sensor/supply_voltage (std_msgs/Float32)")
        self.get_logger().info("  - /sensor/environment_temperature (sensor_msgs/Temperature)")
        self.get_logger().info("  - /sensor/orientation (geometry_msgs/Vector3)")
        self.get_logger().info("  - /sensor/yaw (std_msgs/Int16)")
        self.get_logger().info("  - /sensor/pitch (std_msgs/Int16)")
        self.get_logger().info("  - /sensor/roll (std_msgs/Int16)")
    
    def setup_subscribers(self):
        """Setup subscribers for control commands"""
        # Subscribe to start command with interval
        self.start_sub = self.create_subscription(
            Int16,
            'sensor/start_command',
            self.start_command_callback,
            10
        )
        
        # Subscribe to stop command
        self.stop_sub = self.create_subscription(
            Int16,  # Using Int16 but value doesn't matter for stop
            'sensor/stop_command',
            self.stop_command_callback,
            10
        )
        
        self.get_logger().info("Subscribers created:")
        self.get_logger().info("  - /sensor/start_command (std_msgs/Int16) - interval in ms")
        self.get_logger().info("  - /sensor/stop_command (std_msgs/Int16) - any value to stop")
    
    def connect_to_sensor(self):
        """Connect to the TCP sensor server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)  # 5 second timeout
            self.socket.connect((self.sensor_host, self.sensor_port))
            self.connected = True
            self.get_logger().info(f"Connected to sensor at {self.sensor_host}:{self.sensor_port}")
            
            # Start receiving thread
            self.start_receiving()
            
        except Exception as e:
            self.get_logger().error(f"Failed to connect to sensor: {e}")
            self.connected = False
    
    def start_receiving(self):
        """Start the receiving thread"""
        if self.receive_thread and self.receive_thread.is_alive():
            return
            
        self.receiving = True
        self.receive_thread = threading.Thread(target=self.receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
    
    def receive_loop(self):
        """Main receiving loop"""
        buffer = ""
        
        while self.receiving and self.connected:
            try:
                data = self.socket.recv(1024).decode('ascii')
                if not data:
                    break
                    
                buffer += data
                
                # Process complete messages (ending with \r\n)
                while '\r\n' in buffer:
                    message, buffer = buffer.split('\r\n', 1)
                    if message:
                        self.process_message(message)
                        
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Receive error: {e}")
                break
        
        self.get_logger().warn("Receive loop ended")
    
    def process_message(self, message):
        """Process received message from sensor"""
        if not message.startswith('$'):
            return
        
        try:
            if len(message) < 3:
                return
                
            command_id = message[1:3]
            
            if command_id == '11':  # Status message
                self.process_status_message(message)
            else:
                self.get_logger().warn(f"Unknown response command ID: {command_id}")
                
        except Exception as e:
            self.get_logger().error(f"Message processing error: {e}")
    
    def process_status_message(self, message):
        """Process status message and publish data"""
        try:
            # Extract payload (everything after $11 and before \r\n)
            payload = message[3:]
            
            if len(payload) != 20:  # 10 bytes = 20 hex characters
                self.get_logger().warn(f"Invalid payload length: {len(payload)}")
                return
            
            # Convert hex string to bytes
            payload_bytes = bytes.fromhex(payload)
            
            # Unpack data (all little-endian)
            supply_voltage_raw = struct.unpack('<H', payload_bytes[0:2])[0]  # uint16
            env_temp_raw = struct.unpack('<h', payload_bytes[2:4])[0]        # int16
            yaw_raw = struct.unpack('<h', payload_bytes[4:6])[0]             # int16
            pitch_raw = struct.unpack('<h', payload_bytes[6:8])[0]           # int16
            roll_raw = struct.unpack('<h', payload_bytes[8:10])[0]           # int16
            
            # Convert and publish data
            self.publish_sensor_data(supply_voltage_raw, env_temp_raw, 
                                   yaw_raw, pitch_raw, roll_raw)
            
        except Exception as e:
            self.get_logger().error(f"Status message processing error: {e}")
    
    def publish_sensor_data(self, voltage_mv, temp_dc, yaw_dd, pitch_dd, roll_dd):
        """Publish processed sensor data"""
        timestamp = self.get_clock().now().to_msg()
        
        # Supply voltage (convert from millivolts to volts)
        voltage_msg = Float32()
        voltage_msg.data = voltage_mv / 1000.0
        self.supply_voltage_pub.publish(voltage_msg)
        
        # Environment temperature (convert from deci-celsius to celsius)
        temp_msg = Temperature()
        temp_msg.header.stamp = timestamp
        temp_msg.header.frame_id = "sensor_frame"
        temp_msg.temperature = temp_dc / 10.0
        self.env_temp_pub.publish(temp_msg)
        
        # Orientation (convert from deci-degrees to degrees)
        orientation_msg = Vector3()
        orientation_msg.x = yaw_dd / 10.0    # Yaw
        orientation_msg.y = pitch_dd / 10.0  # Pitch  
        orientation_msg.z = roll_dd / 10.0   # Roll
        self.orientation_pub.publish(orientation_msg)
        
        # Individual angle messages (keep as deci-degrees for precision)
        yaw_msg = Int16()
        yaw_msg.data = yaw_dd
        self.yaw_pub.publish(yaw_msg)
        
        pitch_msg = Int16()
        pitch_msg.data = pitch_dd
        self.pitch_pub.publish(pitch_msg)
        
        roll_msg = Int16()
        roll_msg.data = roll_dd
        self.roll_pub.publish(roll_msg)
        
        # Log data periodically
        self.get_logger().info(
            f"Sensor data - Voltage: {voltage_msg.data:.3f}V, "
            f"Temp: {temp_msg.temperature:.1f}°C, "
            f"Orientation: Y={orientation_msg.x:.1f}° "
            f"P={orientation_msg.y:.1f}° R={orientation_msg.z:.1f}°",
            throttle_duration_sec=2.0
        )
    
    def start_command_callback(self, msg):
        """Handle start command from topic"""
        interval = msg.data
        self.get_logger().info(f"Received start command with interval: {interval}ms")
        self.send_start_command(interval)
    
    def stop_command_callback(self, msg):
        """Handle stop command from topic"""
        self.get_logger().info("Received stop command")
        self.send_stop_command()
    
    def send_start_command(self, interval_ms):
        """Send start command to sensor"""
        if not self.connected:
            self.get_logger().error("Not connected to sensor")
            return False
        
        try:
            # Create command: #03[interval]\r\n
            # Pack interval as little-endian uint16
            interval_bytes = struct.pack('<H', interval_ms)
            interval_hex = interval_bytes.hex().upper()
            
            command = f"#03{interval_hex}\r\n"
            self.socket.send(command.encode('ascii'))
            
            self.get_logger().info(f"Sent start command: {command.strip()}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to send start command: {e}")
            return False
    
    def send_stop_command(self):
        """Send stop command to sensor"""
        if not self.connected:
            self.get_logger().error("Not connected to sensor")
            return False
        
        try:
            command = "#09\r\n"
            self.socket.send(command.encode('ascii'))
            
            self.get_logger().info(f"Sent stop command: {command.strip()}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to send stop command: {e}")
            return False
    
    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.receiving = False
        
        if self.connected and self.socket:
            try:
                self.send_stop_command()  # Stop sensor before disconnecting
                self.socket.close()
            except:
                pass
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SensorCommunicationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
