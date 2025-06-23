#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

import tkinter as tk
from tkinter import ttk, messagebox
import threading

class SimpleSensorGUI:
    def __init__(self):
        # Initialize ROS2
        rclpy.init()
        self.node = Node('sensor_gui_node')
        
        # ROS2 Publishers for control commands
        self.start_pub = self.node.create_publisher(Int16, 'sensor/start_command', 10)
        self.stop_pub = self.node.create_publisher(Int16, 'sensor/stop_command', 10)
        
        # Setup GUI
        self.setup_gui()
        
        # Start ROS2 spinning in separate thread
        self.ros_thread = threading.Thread(target=self.ros_spin_thread, daemon=True)
        self.ros_thread.start()
    
    def setup_gui(self):
        """Setup the simple GUI interface"""
        self.root = tk.Tk()
        self.root.title("Sensor Control")
        self.root.geometry("350x200")
        self.root.resizable(False, False)
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        title_label = ttk.Label(main_frame, text="Sensor Control Panel", 
                               font=('Arial', 14, 'bold'))
        title_label.pack(pady=(0, 20))
        
        # Interval input frame
        interval_frame = ttk.Frame(main_frame)
        interval_frame.pack(pady=10)
        
        ttk.Label(interval_frame, text="Interval (ms):").pack(side=tk.LEFT)
        self.interval_var = tk.StringVar(value="1000")
        interval_entry = ttk.Entry(interval_frame, textvariable=self.interval_var, width=8)
        interval_entry.pack(side=tk.LEFT, padx=(10, 0))
        
        # Buttons frame
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(pady=20)
        
        # Start button
        start_button = ttk.Button(button_frame, text="Start Sensor", 
                                 command=self.start_sensor, width=12)
        start_button.pack(side=tk.LEFT, padx=(0, 10))
        
        # Stop button
        stop_button = ttk.Button(button_frame, text="Stop Sensor", 
                                command=self.stop_sensor, width=12)
        stop_button.pack(side=tk.LEFT)
        
        # Status label
        self.status_var = tk.StringVar(value="Ready")
        status_label = ttk.Label(main_frame, textvariable=self.status_var, 
                               font=('Arial', 10, 'italic'))
        status_label.pack(pady=10)
        
        # Instructions
        instructions = "Enter interval in milliseconds (100-10000)\nand click Start/Stop to control sensor"
        inst_label = ttk.Label(main_frame, text=instructions, 
                              font=('Arial', 9), justify=tk.CENTER)
        inst_label.pack(pady=10)
        
        # Bind window close event
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def start_sensor(self):
        """Send start command to sensor"""
        try:
            interval = int(self.interval_var.get())
            
            # Validate interval
            if interval < 100 or interval > 10000:
                messagebox.showwarning("Invalid Interval", 
                                     "Interval must be between 100 and 10000 milliseconds")
                return
            
            # Create and publish start command
            msg = Int16()
            msg.data = interval
            self.start_pub.publish(msg)
            
            # Update status
            self.status_var.set(f"Started with {interval}ms interval")
            self.node.get_logger().info(f"GUI: Start command sent - interval: {interval}ms")
            
        except ValueError:
            messagebox.showerror("Invalid Input", 
                               "Please enter a valid number for interval")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to start sensor: {e}")
            self.node.get_logger().error(f"GUI: Start command failed: {e}")
    
    def stop_sensor(self):
        """Send stop command to sensor"""
        try:
            # Create and publish stop command
            msg = Int16()
            msg.data = 0  # Data value doesn't matter for stop command
            self.stop_pub.publish(msg)
            
            # Update status
            self.status_var.set("Stop command sent")
            self.node.get_logger().info("GUI: Stop command sent")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to stop sensor: {e}")
            self.node.get_logger().error(f"GUI: Stop command failed: {e}")
    
    def ros_spin_thread(self):
        """ROS2 spinning thread"""
        while rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception as e:
                self.node.get_logger().error(f"ROS spin error: {e}")
                break
    
    def on_closing(self):
        """Handle window closing"""
        self.cleanup()
        self.root.destroy()
    
    def run(self):
        """Run the GUI"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        try:
            if hasattr(self, 'node'):
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Cleanup error: {e}")

def main(args=None):
    try:
        gui = SimpleSensorGUI()
        gui.run()
    except Exception as e:
        print(f"GUI Error: {e}")
        # Try to cleanup ROS2 if it was initialized
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
