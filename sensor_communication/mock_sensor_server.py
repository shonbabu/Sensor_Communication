#!/usr/bin/env python3

import socket
import threading
import time
import struct
import random
import signal
import sys

class MockSensorServer:
    def __init__(self, host='localhost', port=2000):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.running = False
        self.sending_status = False
        self.status_interval = 1000  # Default 1000ms
        self.status_thread = None
        
        # Mock sensor data
        self.supply_voltage = 2500  # 2.5V in millivolts
        self.env_temp = 235         # 23.5°C in deci-celsius
        self.yaw = 0                # 0° in deci-degrees
        self.pitch = 0              # 0° in deci-degrees  
        self.roll = 0               # 0° in deci-degrees
        
    def start_server(self):
        """Start the TCP server"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.running = True
            
            print(f"Mock sensor server listening on {self.host}:{self.port}")
            
            while self.running:
                try:
                    self.client_socket, addr = self.server_socket.accept()
                    print(f"Client connected from {addr}")
                    self.handle_client()
                except socket.error:
                    if self.running:
                        print("Socket error occurred")
                    break
                    
        except Exception as e:
            print(f"Server error: {e}")
        finally:
            self.cleanup()
    
    def handle_client(self):
        """Handle client commands"""
        try:
            while self.running and self.client_socket:
                data = self.client_socket.recv(1024)
                if not data:
                    break
                    
                message = data.decode('ascii').strip()
                print(f"Received: {message}")
                
                if message.startswith('#'):
                    self.process_command(message)
                    
        except Exception as e:
            print(f"Client handling error: {e}")
        finally:
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None
    
    def process_command(self, message):
        """Process incoming commands"""
        if len(message) < 3:
            return
            
        try:
            command_id = message[1:3]
            
            if command_id == '03':  # Start command
                if len(message) >= 7:  # #03XXXX\r\n
                    interval_hex = message[3:7]
                    # Convert from little-endian hex to integer
                    interval_bytes = bytes.fromhex(interval_hex)
                    self.status_interval = struct.unpack('<H', interval_bytes)[0]
                    print(f"Start command received, interval: {self.status_interval}ms")
                    self.start_status_messages()
                    
            elif command_id == '09':  # Stop command
                print("Stop command received")
                self.stop_status_messages()
                
        except Exception as e:
            print(f"Command processing error: {e}")
    
    def start_status_messages(self):
        """Start sending periodic status messages"""
        self.sending_status = True
        if self.status_thread and self.status_thread.is_alive():
            return
            
        self.status_thread = threading.Thread(target=self.send_status_loop)
        self.status_thread.daemon = True
        self.status_thread.start()
    
    def stop_status_messages(self):
        """Stop sending status messages"""
        self.sending_status = False
    
    def send_status_loop(self):
        """Send status messages periodically"""
        while self.sending_status and self.client_socket:
            try:
                # Update mock sensor data with some variation
                self.update_mock_data()
                
                # Create status message
                status_msg = self.create_status_message()
                
                self.client_socket.send(status_msg.encode('ascii'))
                print(f"Sent status: {status_msg.strip()}")
                
                time.sleep(self.status_interval / 1000.0)
                
            except Exception as e:
                print(f"Status sending error: {e}")
                break
    
    def update_mock_data(self):
        """Update mock sensor data with realistic variations"""
        # Add some random variation to simulate real sensor
        self.supply_voltage = 2500 + random.randint(-50, 50)  # ±0.05V
        self.env_temp = 235 + random.randint(-20, 20)         # ±2°C
        self.yaw = (self.yaw + random.randint(-10, 10)) % 3600  # Slowly changing
        self.pitch = max(-900, min(900, self.pitch + random.randint(-5, 5)))  # ±90°
        self.roll = max(-1800, min(1800, self.roll + random.randint(-5, 5)))  # ±180°
    
    def create_status_message(self):
        """Create a status message following the protocol"""
        # Pack data in little-endian format
        supply_voltage_bytes = struct.pack('<H', self.supply_voltage)
        env_temp_bytes = struct.pack('<h', self.env_temp)
        yaw_bytes = struct.pack('<h', self.yaw)
        pitch_bytes = struct.pack('<h', self.pitch)
        roll_bytes = struct.pack('<h', self.roll)
        
        # Convert to hex string
        payload = (supply_voltage_bytes + env_temp_bytes + 
                  yaw_bytes + pitch_bytes + roll_bytes).hex().upper()
        
        # Create message: $11[payload]\r\n
        message = f"$11{payload}\r\n"
        return message
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        self.sending_status = False
        
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
            
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print("\nShutting down mock sensor server...")
        self.cleanup()
        sys.exit(0)

def main():
    server = MockSensorServer()
    
    # Handle Ctrl+C gracefully
    signal.signal(signal.SIGINT, server.signal_handler)
    signal.signal(signal.SIGTERM, server.signal_handler)
    
    try:
        server.start_server()
    except KeyboardInterrupt:
        print("\nServer interrupted")
    finally:
        server.cleanup()

if __name__ == '__main__':
    main()
