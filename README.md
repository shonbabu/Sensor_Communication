# Sensor Communication ROS2 Package

This package provides TCP communication with a robotic sensor using the specified protocol, along with a GUI for user control.

## Features

- **Mock Sensor Server**: Simulates the TCP sensor for testing without hardware
- **ROS2 Sensor Node**: Handles TCP communication and publishes sensor data
- **GUI Control**: Simple interface for starting/stopping sensor with custom intervals
- **Auto-start capability**: Automatically starts sensor on launch with configurable interval

## Package Structure

```
sensor_communication/
├── sensor_communication/
│   ├── sensor_node.py          # Main ROS2 communication node
│   ├── mock_sensor_server.py   # Mock TCP sensor server
│   └── sensor_gui.py           # GUI for user control
├── launch/
│   └── sensor_launch.py        # Launch file for complete system
├── config/
│   └── sensor_params.yaml      # Configuration parameters
└── README.md
```

## Published Topics

## Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/sensor/supply_voltage` | `std_msgs/Float32` | Supply voltage in Volts |
| `/sensor/environment_temperature` | `sensor_msgs/Temperature` | Environment temperature |
| `/sensor/orientation` | `geometry_msgs/Vector3` | Yaw, Pitch, Roll in degrees |
| `/sensor/yaw` | `std_msgs/Int16` | Yaw angle in deci-degrees |
| `/sensor/pitch` | `std_msgs/Int16` | Pitch angle in deci-degrees |
| `/sensor/roll` | `std_msgs/Int16` | Roll angle in deci-degrees |


## Subscribed Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/sensor/start_command` | `std_msgs/Int16` | Start sensor with interval (ms) |
| `/sensor/stop_command` | `std_msgs/Int16` | Stop sensor (any value) |

