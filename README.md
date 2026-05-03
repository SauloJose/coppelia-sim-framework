# Brainbyte - Professional Robot Simulation Framework

A comprehensive Python framework for controlling and testing robot simulations in CoppeliaSim using a high-performance ZeroMQ bridge with CBOR2 binary serialization.

## Features

**Batch Dataflow Architecture** - Queue commands, send once per frame, receive all sensor data in binary format  
**Professional Logging System** - Standardized, emoji-free logging to console and file  
**BaseApp Lifecycle Management** - Complete simulation lifecycle with setup, post_start, loop, and stop phases  
**High-Performance ZMQ Bridge** - Synchronous REQ/REP communication with CBOR2 binary serialization for minimal latency  
**Robot & Sensor Abstractions** - Extensible base classes (BaseBot, BaseSensor) for rapid robot/sensor development  
**Built-in Visualization** - Standardized Plot2D() and Plot3D() functions for trajectory visualization  
**Interactive CLI** - Beautiful terminal-based menu for launching projects with automatic discovery  
**Production Ready** - Proper packaging with setup.py, pyproject.toml, professional logging, and test suite

## Project Structure

```text
brainbyte/                       # Main package
├── logs/                        # Log files directory
├── core/
│   ├── base_app.py              # BaseApp - simulation lifecycle (setup, loop, stop)
│   ├── bridge.py                # SimulationBridge - ZMQ batch dataflow communication
│   ├── paths.py                 # Path utilities
│   └── __init__.py
├── gui/
│   ├── cli.py                   # Interactive terminal menu (brainGUI)
│   ├── auxF.py                  # CLI utilities (BOT_say, BOT_print, get_key)
│   └── __init__.py
├── robots/
│   ├── base/
│   │   ├── base_bot.py          # BaseBot abstract class (pose, sensors, controls)
│   │   └── __init__.py
│   ├── movel/                   # Mobile robots
│   │   ├── TurtleBot.py         # Differential drive robot (v_max=0.31 m/s)
│   │   ├── Robotino.py          # Omnidirectional robot
│   │   ├── PioneerBot.py        # Another differential robot
│   │   └── Manta.py             # Hexapod robot
│   ├── arms/                    # Robotic arm implementations
│   ├── humanoid/                # Humanoid robot implementations
│   ├── models/                  # CoppeliaSim .ttm model files
│   └── __init__.py
├── sensors/
│   ├── base/
│   │   ├── base_sensor.py       # BaseSensor abstract class
│   │   └── __init__.py
│   ├── HokuyoSensor.py          # Hokuyo LIDAR sensor
│   ├── LDS_02.py                # TurtleBot LIDAR sensor
│   └── __init__.py
├── utils/
│   ├── logging.py               # setup_logger() - professional logging
│   ├── plotting.py              # Plot2D(), Plot3D() - trajectory visualization
│   ├── math.py                  # Mathematical utilities (gram_schmidt, etc)
│   ├── basics/                  # Templates and base files
│   │   ├── app.txt              # Application template
│   │   └── scene.ttt            # Basic simulation scene
│   └── __init__.py
├── control/
│   ├── automatic.py             # Controllers (PID, OnOff, DifferentialController)
│   ├── manual.py                # Manual control implementations
│   └── __init__.py
└── __init__.py

projects/                        # Project storage organized by topics
├── Trajetoria/
│   ├── malha_aberta_exemplo/    # Open-loop control example
│   ├── malha_fechada_exemplo/   # Closed-loop control example
│   ├── PID_exemple/             # PID controller example
│   └── robotino_exemple/        # Robotino robot example
├── Sensores/
│   ├── lidar_exemple/           # LIDAR sensor example
│   └── turtleBot/               # TurtleBot project
├── Obstaculos/                  # Obstacle avoidance projects
└── controle/
    └── Controle_de_caminho/     # Path control example

config.json                      # Configuration (installed dependencies)
main.py                          # Interactive menu launcher
setup.py                         # Package installation
pyproject.toml                   # Project metadata
requirements.txt                 # Runtime dependencies
requirements-dev.txt             # Development dependencies
LICENSE                          # License file

docs/                            # Documentation
├── README.md                    # This file
├── API.md                       # API reference
├── ARCHITECTURE.md              # Architecture documentation
└── CONTRIBUTING.md              # Contribution guidelines
```

## Installation

### Requirements

- Python 3.8+
- CoppeliaSim 4.4.0 or later (with ZMQ RemoteAPI plugin)
- zmq (libzmq) system library

### Setup

```bash
# Clone or download the repository
cd UFCG/ESTUDO/Framework

# Install package with dependencies
pip install -e .

# Or install from requirements
pip install -r requirements.txt

# With development dependencies (testing, linting)
pip install -r requirements-dev.txt
```

### Verify Installation

```bash
# Run the main launcher
python main.py
```
This will display an interactive menu to select and run available projects.

## Quick Start

### Running a Project

```bash
python main.py
```

You'll see an interactive terminal menu showing available projects organized by topic. Use arrow keys to navigate and Enter to select.

**Controls during execution:**
- Press **'x'** during execution to stop the simulation gracefully
- Press **Ctrl+C** for emergency stop

### Creating a New Simulation

1. **Create project folder structure** inside `projects/` with a topic category:
   ```
   projects/
   └── MyTopic/
       └── my_project/
           ├── my_project.py       # Must match folder name
           └── my_project.ttt      # CoppeliaSim scene file
   ```

2. **Implement your simulation class** inheriting from `BaseApp`:

```python
from brainbyte import BaseApp
from brainbyte.robots import TurtleBot
from brainbyte.sensors import LDS_02
from brainbyte.utils.logging import setup_logger
import numpy as np

logger = setup_logger(__name__, '[MYAPP]')

class MyProjectSimulation(BaseApp):
    def __init__(self):
        """Initialize with scene file and simulation time."""
        super().__init__(
            scene_file="my_project.ttt",
            sim_name="MyProject",
            sim_time=60.0  # seconds
        )

    def setup(self):
        """Called once before simulation starts."""
        # Instantiate robots and sensors
        self.robot = TurtleBot(
            bridge=self.bridge,
            robot_name='MyRobot'
        )
        self.lidar = LDS_02(bridge=self.bridge, base_name='MyRobot')
        self.robot.add_sensor('lidar', self.lidar)
        
        # Initialize handshake with CoppeliaSim
        self.handshake()

    def handshake(self):
        """Define which paths to monitor and control."""
        monitor_paths = [
            '/MyRobot_pos',
            '/MyRobot_ori',
            '/MyRobot/Lidar_bin'
        ]
        actuator_paths = [
            '/MyRobot/left_Motor',
            '/MyRobot/right_Motor'
        ]
        self.bridge.initialize(monitor_paths, actuator_paths, self.sim)

    def post_start(self):
        """Called after first simulation step."""
        logger.info(f"Robot initial pose: {self.robot.pose}")

    def loop(self, t):
        """Main control loop called every simulation step."""
        # Read sensors from cache (no network overhead)
        lidar_data = self.lidar.get_bridge_data('_bin')
        robot_pos = self.robot.pose
        
        # Simple forward motion
        self.bridge.queue_velocity('/MyRobot/left_Motor', 1.0)
        self.bridge.queue_velocity('/MyRobot/right_Motor', 1.0)

    def stop(self):
        """Called when simulation ends."""
        logger.info("Simulation finished. Cleaning up...")
        # Motors are stopped automatically by BaseApp

def app():
    """Entry point for main.py launcher."""
    simulation = MyProjectSimulation()
    simulation.run()

if __name__ == "__main__":
    app()
```

3. **Scene file setup** - Create your `.ttt` scene in CoppeliaSim:
   - Add robots with names matching your code (e.g., 'MyRobot')
   - Add motors as children: `/MyRobot/left_Motor`, `/MyRobot/right_Motor`
   - Add sensors as children: `/MyRobot/Lidar`, etc.
   - Save as `my_project.ttt` in the same folder as the `.py` file

4. **Launch from menu** - Run `python main.py` and your project will appear in the menu

## Core Components

### SimulationBridge (ZMQ Batch Dataflow)

**Location:** `brainbyte/core/bridge.py`

The bridge implements a high-performance synchronous communication pattern:

1. **ZeroMQ REQ/REP Pattern** (Port 23001)
   - Python (REQ): Sends command batch and waits for response
   - CoppeliaSim Lua (REP): Receives commands, steps physics, replies with sensor data
   - This guarantees perfect synchronization between Python logic and simulator physics

2. **Batch Dataflow Execution**
   ```python
   # Frame N: Queue commands (no network overhead)
   bridge.queue_velocity('/Robot/left_motor', 2.0)
   bridge.queue_velocity('/Robot/right_motor', 1.5)
   bridge.queue_position('/Robot/arm_joint1', 0.5)
   
   # Send batch once, receive all data once (1 network RTT)
   state = bridge.step()  # Binary CBOR2 transfer
   
   # Frame N+1: Read cached data (zero network overhead)
   position = bridge.get_sensor_data('/Robot_pos')  # O(1) dict lookup
   lidar_cloud = state['/Robot/lidar_bin']  # Already numpy.float32
   ```

3. **CBOR2 Binary Serialization**
   - Dramatically faster than JSON for large data (LiDAR point clouds, camera frames)
   - Automatic conversion: Binary → `numpy.float32` arrays
   - Network payload typically 10-100x smaller than JSON equivalent

### BaseApp Lifecycle

**Location:** `brainbyte/core/base_app.py`

Manages complete simulation lifecycle:

```
__init__()           → Connects to CoppeliaSim RemoteAPI (port 23000)
                      Waits 10 seconds for simulator to open
                      Initializes logging

run()               → Loads scene file
                    → Starts simulation
                    → Creates SimulationBridge
                    → Calls setup()
                    → Calls post_start()
                    → Main loop: calls loop(t) and bridge.step()
                    → Calls stop() on exit or Ctrl+C

Properties:
- self.sim            → RemoteAPIClient reference
- self.bridge         → SimulationBridge instance  
- self.dt             → Simulation time step
- self.st             → Current simulation time
```

### BaseBot & Robots

**Location:** `brainbyte/robots/base/base_bot.py` and `brainbyte/robots/movel/`

Abstract base class for all robots:

```python
class BaseBot(ABC):
    # Properties
    @property
    def pose(self):              # Returns [x, y, theta]
    @pose.setter
    def pose(self, new_pose):    # Teleports robot
    
    # Methods
    def get_pose()               # Reads from cache
    def add_sensor(name, obj)    # Attach sensor
    def add_control(name, obj)   # Attach controller
```

**Available Robots:**
- `TurtleBot` - Differential drive (v_max=0.31 m/s, wheelbase=0.287m)
- `Robotino` - Omnidirectional 3-wheel platform
- `PioneerBot` - Similar to TurtleBot
- `Manta` - Hexapod robot

### BaseSensor & Sensors

**Location:** `brainbyte/sensors/base/base_sensor.py` and `brainbyte/sensors/`

Abstract base class for all sensors:

```python
class BaseSensor(ABC):
    def get_monitor_paths() -> list   # Declares what to monitor
    def get_bridge_data(suffix='')    # Reads from cache
    def update()                      # Process sensor data
```

**Available Sensors:**
- `LDS_02` - TurtleBot LIDAR (360°, ~30m range)
- `HokuyoSensor` - Hokuyo LIDAR

### Controllers

**Location:** `brainbyte/control/automatic.py`

- `PID_Controller` - Proportional-Integral-Derivative control
- `On_Off_Controller` - Binary on/off control with hysteresis
- `DifferentialController` - Navigation controller for differential robots

### Utilities

- **Logging** (`utils/logging.py`) - Professional logger with file output
- **Plotting** (`utils/plotting.py`) - `Plot2D()` and `Plot3D()` for trajectory visualization  
- **Math** (`utils/math.py`) - Vector operations, Gram-Schmidt orthogonalization

## Architecture Highlights

### Path-Based Object Addressing

All objects in CoppeliaSim are addressed using hierarchical paths:
```
/RobotName/base_link          → Robot root position/orientation
/RobotName/left_motor         → Left wheel joint
/RobotName/Lidar              → LIDAR sensor
/RobotName/Lidar_bin          → LIDAR point cloud (binary)
```

This allows:
- Flexible scene organization
- Automatic handle caching by Lua script
- O(1) data lookup in Python cache

### Why Batch Dataflow?

Traditional approach (1 request per command):
```
Python → CoppeliaSim (set velocity)     [10ms RTT]
       ← CoppeliaSim (read position)    [10ms RTT]
       → CoppeliaSim (set torque)       [10ms RTT]
       ← CoppeliaSim (read sensor)      [10ms RTT]
Total: 40ms per frame (25 FPS max) ❌
```

Batch approach (1 request, 1 reply):
```
Python → CoppeliaSim (all commands)     [1ms RTT]
       ← CoppeliaSim (all sensor data)  [1ms RTT]
Total: 2ms per frame (500 FPS possible) ✓
```

### Why CBOR2?

- **Compact**: LiDAR 360×4 bytes → 1440 bytes CBOR vs 5KB+ JSON
- **Fast**: Native binary format, no parsing overhead
- **Native Arrays**: Transmitted as numpy arrays directly
- **Type Preservation**: Floats, ints, binaries encoded efficiently

## Documentation

For more detailed information, see:
- [API Reference](docs/API.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Contributing Guidelines](docs/CONTRIBUTING.md)

## License

MIT License

## Contact

For questions or issues, please open a GitHub issue or contact the maintainers.
