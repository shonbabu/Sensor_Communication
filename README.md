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

## Running the System

### Option 1: Complete System Launch (Recommended)

```bash
# Source your workspace
source ~/ros2_ws/install/setup.bash

# Launch everything (mock server, sensor node, and GUI)
ros2 launch sensor_communication sensor_launch.py
```

### Option 2: Individual Components

**Start Mock Sensor Server:**
```bash
ros2 run sensor_communication mock_sensor_server
```

**Start Sensor Node (in new terminal):**
```bash
ros2 run sensor_communication sensor_node
```

**Start GUI (in new terminal):**
```bash
ros2 run sensor_communication sensor_gui
```


## Question 1: Auto-Start Capability

**Requirement:** Automatically sends the start command (with configurable interval) to the sensor on launch.

### Configuration

The system includes auto-start functionality configured through ROS2 parameters:

```yaml
sensor_communication_node:
  ros__parameters:
    # Sensor connection settings
    sensor_host: "localhost"
    sensor_port: 2000
    
    # Default interval for auto-start (milliseconds)
    default_interval: 1000
    
    # Auto-start sensor on launch
    auto_start: true
```

### Auto-Start Process

1. **On Node Launch**: When the sensor node starts, it checks the `auto_start` parameter
2. **If Enabled**: The node automatically sends a start command using the `default_interval` value
3. **Configurable**: Both auto-start behavior and interval can be customized

### Launch Options

```bash
# Launch with custom auto-start settings
ros2 launch sensor_communication sensor_launch.py auto_start:=true default_interval:=500

# Launch without auto-start
ros2 launch sensor_communication sensor_launch.py auto_start:=false
```

---

## Question 2: Data Decoding & Topic Publishing

**Requirement:** Decodes status messages received and publishes various parameters to appropriate topics.

### Communication Protocol

#### Command Message Format (Controller → Sensor)
```
#<CommandID><Payload><CR><LF>
```

| Component | Description |
|-----------|-------------|
| `#` | Start character |
| `CommandID` | 2 ASCII characters (hex-encoded) |
| `Payload` | Optional; size depends on command |
| `<CR><LF>` | End of message (0x0D 0x0A) |

#### Supported Commands

| Command | ID (Hex) | Payload | Description |
|---------|----------|---------|-------------|
| Start | 03 | 2 bytes | Start sending status at specified interval (ms, uint16, little-endian) |
| Stop | 09 | None | Stop sending status messages |

**Example Start Command (1000ms interval):**
```
#03E803<CR><LF>
```
- `03` → Start Command
- `E803` → 1000ms (little-endian of 0x03E8)

#### Response Message Format (Sensor → Controller)
```
$<CommandID><Payload><CR><LF>
```

### Status Message Decoding

**Command ID:** 11  
**Payload Size:** 10 bytes (20 ASCII hex characters)  
**Data Format:** Little-endian

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| SUPPLY_VOLTAGE | uint16 | millivolts (mV) | Supply voltage to sensor |
| ENV_TEMP | int16 | deci-degrees Celsius | Environmental temperature |
| YAW | int16 | deci-degrees | Yaw angle |
| PITCH | int16 | deci-degrees | Pitch angle |
| ROLL | int16 | deci-degrees | Roll angle |

**Example Response:**
```
$11E803FA00F4010A000500<CR><LF>
```
- `SUPPLY_VOLTAGE` → 0x03E8 = 1000 mV = 1.0 V
- `ENV_TEMP` → 0x00FA = 250 deci-degrees = 25.0°C
- `YAW` → 0x01F4 = 500 deci-degrees = 50.0°
- `PITCH` → 0x000A = 10 deci-degrees = 1.0°
- `ROLL` → 0x0005 = 5 deci-degrees = 0.5°

### Data Processing Pipeline

1. **Receives TCP data** from sensor
2. **Buffers incomplete messages** until complete
3. **Processes complete messages** ending with `\r\n`
4. **Converts hex string** back to binary data
5. **Unpacks 5 values** (2 bytes each) using little-endian format
6. **Extracts and converts** voltage, temperature, and orientation data
7. **Creates appropriate ROS2 message types**
8. **Publishes to multiple topics** simultaneously

---

## Question 3: User Control Interface

**Requirement:** Provide options for users to start (with custom interval) and stop the sensor.

### Option 1: GUI Interface 
Launch the graphical user interface:
```bash
ros2 run sensor_communication sensor_gui
```

**GUI Features:**
- Simple tkinter-based interface
- Interval input field (milliseconds)
- Start button with custom interval
- Stop button for immediate control


**Usage:**
1. Enter desired interval in milliseconds
2. Click "Start" to begin sensor data streaming
3. Click "Stop" to halt sensor operation

### Option 2: Command Line Interface

**Start sensor with custom interval:**
```bash
# Start sensor with 500ms interval
ros2 topic pub /sensor/start_command std_msgs/Int16 "data: 500"
```

**Stop sensor:**
```bash
# Stop sensor
ros2 topic pub /sensor/stop_command std_msgs/Int16 "data: 0"
```

### Control Topics

| Topic Name | Message Type | Purpose |
|------------|--------------|---------|
| `/sensor/start_command` | `std_msgs/Int16` | Start sensor with specified interval (ms) |
| `/sensor/stop_command` | `std_msgs/Int16` | Stop sensor operation |

---



