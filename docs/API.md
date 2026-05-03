# Brainbyte API Reference

Complete API documentation for the Brainbyte robotics simulation framework.

---

## 1. Core Module (`brainbyte.core`)

### BaseApp

**Path:** `brainbyte.core.base_app.BaseApp`

Abstract base class that manages the simulation lifecycle and network communication. All simulation projects must inherit from this class.

#### Constructor

```python
def __init__(self, scene_file: str = None, 
             sim_name: str = None,
             sim_time: float = 10.0, 
             log_file: str = None)
```

**Parameters:**
- `scene_file` (str, optional): Path to `.ttt` scene file (relative to script location)
- `sim_name` (str, optional): Name for logging and identification
- `sim_time` (float): Maximum simulation duration in seconds (default: 10.0)
- `log_file` (str, optional): Path to log file. If None, creates temp file.

**Example:**
```python
class MySimulation(BaseApp):
    def __init__(self):
        super().__init__(
            scene_file="my_scene.ttt",
            sim_name="MyRobotTest",
            sim_time=60.0
        )
```

#### Lifecycle Methods (Override in subclass)

##### `setup(self) -> None`

Called once before simulation starts. Use this to:
- Create robot and sensor instances
- Initialize controllers
- Set up data logging
- Call handshake protocol

**Example:**
```python
def setup(self):
    self.robot = TurtleBot(bridge=self.bridge, robot_name='MyRobot')
    self.lidar = LDS_02(bridge=self.bridge, base_name='MyRobot')
    self.robot.add_sensor('lidar', self.lidar)
    self.handshake()
```

##### `post_start(self) -> None`

Called once immediately after simulation starts. Use this for:
- Capturing initial sensor readings
- Setting initial robot positions
- Running diagnostic checks

**Example:**
```python
def post_start(self):
    initial_pose = self.robot.pose
    logger.info(f"Starting from {initial_pose}")
```

##### `loop(self, t: float) -> None`

Called at every simulation step. Contains your main control logic.

**Parameters:**
- `t` (float): Current simulation time in seconds

**Example:**
```python
def loop(self, t):
    # Read sensor data (from cache, no network latency)
    robot_pos = self.robot.pose
    
    # Queue motor commands (enqueued, not sent yet)
    self.bridge.queue_velocity('/MyRobot/left_Motor', 1.0)
    self.bridge.queue_velocity('/MyRobot/right_Motor', 1.0)
    
    # Note: self.bridge.step() is called automatically by BaseApp
```

##### `stop(self) -> None`

Called after simulation ends or on interruption. Use for:
- Saving data
- Plotting results
- Resource cleanup

**Example:**
```python
def stop(self):
    logger.info("Saving trajectory...")
    np.save('trajectory.npy', self.trajectory_data)
    Plot2D(self.trajectory_data, 'X (m)', 'Y (m)', title='Robot Path')
```

#### Properties

**`dt` → float**
```python
@property
def dt(self) -> float:
    """Simulation time step (seconds). Typically 0.05s."""
    return self.sim.getSimulationTimeStep()
```

**`st` → float**
```python
@property
def st(self) -> float:
    """Current simulation time (seconds)."""
    return self.sim.getSimulationTime()
```

#### Public Methods

##### `run(self) -> None`

Starts the simulation lifecycle. Should only be called in the `app()` entry point.

```python
def app():
    sim = MySimulation()
    sim.run()  # Blocks until simulation ends
```

##### `d_time(self) -> float`

Returns the simulation time step in seconds.

---

### SimulationBridge

**Path:** `brainbyte.core.bridge.SimulationBridge`

Handles ZeroMQ/CBOR2 batch dataflow communication with CoppeliaSim.

#### Constructor

```python
def __init__(self, host: str = "127.0.0.1", 
             port: int = 23001, 
             timeout: int = 10000)
```

**Parameters:**
- `host` (str): IP address of CoppeliaSim machine
- `port` (int): ZMQ socket port (CoppeliaSim Lua listens here)
- `timeout` (int): Socket timeout in milliseconds

#### Initialization

##### `initialize(monitor_paths: list, actuator_paths: list, simulation) -> dict`

Performs INIT handshake with CoppeliaSim. Call this in `setup()` to declare which paths to monitor.

**Parameters:**
- `monitor_paths` (list): List of sensor/state paths to read each frame
  - Examples: `['/Robot_pos', '/Robot_ori', '/Lidar_bin']`
- `actuator_paths` (list): List of motor/servo paths to command
  - Examples: `['/Robot/left_motor', '/Robot/right_motor']`
- `simulation`: Reference to `self.sim` from BaseApp

**Returns:**
- Dictionary with initialization confirmation

**Example:**
```python
def setup(self):
    monitor_paths = [
        '/MyRobot_pos',      # Position
        '/MyRobot_ori',      # Orientation
        '/Lidar/data_bin'    # LIDAR point cloud
    ]
    actuator_paths = [
        '/MyRobot/left_Motor',
        '/MyRobot/right_Motor'
    ]
    self.bridge.initialize(monitor_paths, actuator_paths, self.sim)
```

#### Queuing Methods (Batch Dataflow)

All queue methods **do not** send data immediately. They accumulate commands in a buffer that are sent when `step()` is called.

##### `queue_velocity(path: str, velocity: float) -> None`

Schedule a velocity target for a motor joint.

**Parameters:**
- `path` (str): Hierarchical path to the joint (e.g., `'/Robot/motor1'`)
- `velocity` (float): Target velocity in rad/s (for rotating joints)

**Example:**
```python
self.bridge.queue_velocity('/TurtleBot3/left_Motor', 2.5)
self.bridge.queue_velocity('/TurtleBot3/right_Motor', 2.5)
```

##### `queue_position(path: str, position: float) -> None`

Schedule a position target for a joint (typically for arms/servos).

**Parameters:**
- `path` (str): Hierarchical path to the joint
- `position` (float): Target position in radians

**Example:**
```python
self.bridge.queue_position('/Arm/joint1', 1.57)  # ~90 degrees
self.bridge.queue_position('/Gripper/servo', 0.0)
```

##### `queue_command(category: str, path: str, value: any) -> None`

Schedule a custom command of any category.

**Parameters:**
- `category` (str): Command category (e.g., `'teleports'`, `'velocities'`, `'custom'`)
- `path` (str): Hierarchical path to target object
- `value` (any): Command data (can be dict, list, float, etc.)

**Example - Teleport Robot:**
```python
self.bridge.queue_command(
    'teleports', 
    '/MyRobot',
    {
        'pos': [1.0, 2.0, 0.0],     # X, Y, Z
        'ori': [0.0, 0.0, 1.57]     # Roll, Pitch, Yaw
    }
)
```

#### Communication

##### `step(self) -> dict`

Send all queued commands to CoppeliaSim, advance simulation by one time step, and receive sensor data.

**Returns:**
- Dictionary with sensor data keyed by path names
- Binary data (ending in `_bin`) automatically converted to `numpy.float32` arrays

**Example:**
```python
# All queued commands are sent when step() is called
self.bridge.queue_velocity('/Motor1', 1.0)
self.bridge.queue_velocity('/Motor2', 1.0)

state = self.bridge.step()  # Single network RTT

# Access data from response
position = state.get('/Robot_pos', [0, 0, 0])
lidar_data = state.get('/Lidar_bin')  # numpy array
```

**Note:** `BaseApp.run()` automatically calls `self.bridge.step()` after each `loop(t)` iteration.

#### Data Access

##### `get_sensor_data(path: str) -> any`

Retrieve the latest cached sensor data without network overhead.

**Parameters:**
- `path` (str): Sensor path (must be in `monitor_paths`)

**Returns:**
- Cached sensor value from last `step()` call
- Returns `None` if path not found

**Example:**
```python
# In loop()
robot_position = self.bridge.get_sensor_data('/Robot_pos')  # O(1) dict lookup
if robot_position:
    print(f"Robot at {robot_position}")
```

#### Cleanup

##### `close(self) -> None`

Close ZMQ socket and clean up resources.

**Example:**
```python
# Called automatically by BaseApp.run() in finally block
self.bridge.close()
```

---

## 2. Robots Module (`brainbyte.robots`)

### BaseBot

**Path:** `brainbyte.robots.base.base_bot.BaseBot`

Abstract base class for all robots. Provides common interface for sensors, controls, and pose management.

#### Constructor

```python
def __init__(self, bridge: SimulationBridge, robot_name: str)
```

**Parameters:**
- `bridge`: SimulationBridge instance
- `robot_name`: Name of robot root object in scene (e.g., `'Turtlebot3'`)

#### Properties

##### `pose` → np.ndarray

Returns current robot pose as `[x, y, theta]`.

**Getter:**
```python
pose = self.robot.pose  # Returns [1.5, 2.3, 0.785]
```

**Setter (Teleportation):**
```python
self.robot.pose = [0.0, 0.0, 0.0]  # Teleport to origin
```

#### Methods

##### `get_pose(self) -> tuple`

Returns `(position, orientation)` from cache.

**Returns:**
- `position`: [x, y, z] list
- `orientation`: [alpha, beta, gamma] (Euler angles in radians)

**Example:**
```python
pos, ori = self.robot.get_pose()
print(f"Position: {pos}, Orientation: {ori}")
```

##### `add_sensor(sensor_name: str, sensor_instance: BaseSensor) -> None`

Attach a sensor to the robot.

**Parameters:**
- `sensor_name` (str): Friendly name for later retrieval
- `sensor_instance`: Initialized sensor object

**Example:**
```python
lidar = LDS_02(bridge=self.bridge, base_name='MyRobot')
self.robot.add_sensor('front_lidar', lidar)
```

##### `get_sensor(sensor_name: str) -> BaseSensor`

Retrieve an attached sensor by name.

**Example:**
```python
lidar = self.robot.get_sensor('front_lidar')
data = lidar.get_bridge_data('_bin')
```

##### `add_control(control_name: str, control_instance) -> None`

Attach a controller to the robot.

**Example:**
```python
controller = DifferentialController(...)
self.robot.add_control('path_follower', controller)
```

### TurtleBot

**Path:** `brainbyte.robots.movel.TurtleBot.TurtleBot`

Differential drive mobile robot (2 independent wheels).

#### Constructor

```python
def __init__(self, bridge: SimulationBridge,
             robot_name: str = 'Turtlebot3',
             left_motor: str = 'left_Motor',
             right_motor: str = 'right_Motor',
             base_link: str = 'base_link')
```

#### Physical Parameters

```python
# Read-only (set at init)
robot._R = 0.0066                # Wheel radius (m)
robot._L = 0.287                 # Wheelbase (m)
robot._v_max = 0.31              # Max linear velocity (m/s)
robot._w_max = 4.69              # Max angular velocity (rad/s)
```

#### Properties

**`wheel_velocities` → np.ndarray**
```python
wl, wr = self.robot.wheel_velocities  # [rad/s, rad/s]
```

**`robot_velocity` → np.ndarray**
```python
v, omega = self.robot.robot_velocity  # [m/s, rad/s]
```

---

## 3. Sensors Module (`brainbyte.sensors`)

### BaseSensor

**Path:** `brainbyte.sensors.base.base_sensor.BaseSensor`

Abstract base class for all sensors.

#### Constructor

```python
def __init__(self, bridge: SimulationBridge, sensor_path: str)
```

**Parameters:**
- `bridge`: SimulationBridge instance
- `sensor_path`: Path to sensor object in scene

#### Methods

##### `get_monitor_paths(self) -> list`

Declare which data paths the Lua script should monitor for this sensor.

**Returns:**
- List of paths (e.g., `['/Robot/Lidar_bin']`)

**Example (override in subclass):**
```python
def get_monitor_paths(self):
    return [
        f"{self.sensor_path}_pos",    # Position
        f"{self.sensor_path}_bin"     # Binary data
    ]
```

##### `get_bridge_data(suffix: str = "") -> any`

Read the latest sensor data from cache.

**Parameters:**
- `suffix` (str): Suffix to append to sensor path

**Example:**
```python
lidar_cloud = self.sensor.get_bridge_data('_bin')  # Returns numpy array
```

##### `update(self) -> None`

Abstract method. Must be overridden to process sensor data.

### LDS_02

**Path:** `brainbyte.sensors.LDS_02.LDS_02`

TurtleBot3's integrated LIDAR sensor (2D 360° scanner).

#### Constructor

```python
def __init__(self, bridge: SimulationBridge, base_name: str)
```

**Parameters:**
- `bridge`: SimulationBridge instance
- `base_name`: Robot name (e.g., `'Turtlebot3'`)

#### Methods

##### `get_monitor_paths(self) -> list`

Returns paths for angle ranges and distance data.

---

## 4. Control Module (`brainbyte.control`)

### DifferentialController

**Path:** `brainbyte.control.automatic.DifferentialController`

Navigation controller for differential drive robots using Lyapunov-based control law.

#### Constructor

```python
def __init__(self, pos_init: np.ndarray,
             set_point: np.ndarray,
             k_rho: float,
             k_alpha: float,
             k_beta: float,
             dt: float = 0.05)
```

**Parameters:**
- `pos_init`: Initial robot pose `[x, y, theta]`
- `set_point`: Goal pose `[x_goal, y_goal, theta_goal]`
- `k_rho`: Distance error gain
- `k_alpha`: Heading error gain
- `k_beta`: Final heading gain
- `dt`: Control time step

#### Methods

##### `set_SP(set_point: np.ndarray) -> None`

Update goal pose.

```python
controller.set_SP([2.0, 3.0, 0.0])
```

##### `get_control(actual_point: np.ndarray) -> tuple`

Compute control inputs.

**Parameters:**
- `actual_point`: Current robot pose `[x, y, theta]`

**Returns:**
- `(v, omega)`: Linear and angular velocities

**Example:**
```python
v, w = controller.get_control(robot.pose)
bridge.queue_velocity('/left_motor', (v - w * L/2) / R)
bridge.queue_velocity('/right_motor', (v + w * L/2) / R)
```

---

## 5. Utilities (`brainbyte.utils`)

### Logging

**Path:** `brainbyte.utils.logging.setup_logger`

```python
def setup_logger(name: str, 
                 origin_prefix: str = '[APP]',
                 log_file: str = None) -> logging.Logger
```

Create a professional logger with console and file output.

**Parameters:**
- `name`: Logger name (typically `__name__`)
- `origin_prefix`: Prefix for log messages (e.g., `'[MAIN]'`, `'[BRIDGE]'`)
- `log_file`: Optional file path for logging

**Returns:**
- Configured Python Logger

**Example:**
```python
from brainbyte.utils.logging import setup_logger

logger = setup_logger(__name__, '[MYAPP]', log_file='sim.log')
logger.info("Simulation started")
logger.error("Critical error!")
```

### Plotting

**Path:** `brainbyte.utils.plotting`

#### `Plot2D()`

```python
def Plot2D(data, x_label: str, y_label: str,
           tamanho_janela: tuple = (8, 6),
           limite_x: tuple = None,
           limite_y: tuple = None,
           title: str = None) -> None
```

Plot 2D trajectory with start (green) and end (red) markers.

**Parameters:**
- `data`: Array of shape `(N, 2)` with `[x, y]` coordinates
- `x_label`: X-axis label
- `y_label`: Y-axis label
- `tamanho_janela`: Figure size in inches
- `limite_x`/`limite_y`: Axis limits `(min, max)`
- `title`: Plot title

**Example:**
```python
from brainbyte.utils.plotting import Plot2D
import numpy as np

trajectory = np.array([
    [0, 0],
    [1, 1],
    [2, 0]
])
Plot2D(trajectory, 'X (m)', 'Y (m)', title='Robot Path')
```

#### `Plot3D()`

```python
def Plot3D(data, x_label: str, y_label: str, z_label: str,
           tamanho_janela: tuple = (8, 6),
           limite_x: tuple = None,
           limite_y: tuple = None,
           limite_z: tuple = None,
           title: str = None) -> None
```

Plot 3D trajectory.

**Parameters:**
- `data`: Array of shape `(N, 3)` with `[x, y, z]` coordinates
- Other parameters same as `Plot2D()`

---

## Data Flow Summary

```
setup()
  ├── Create robots/sensors
  ├── bridge.initialize(monitor_paths, actuator_paths)
  └── return

post_start()
  └── Optional initial checks

loop(t)  [called repeatedly]
  ├── read: bridge.get_sensor_data()    → O(1) dict lookup
  ├── compute: control logic
  ├── queue: bridge.queue_velocity()
  ├── queue: bridge.queue_position()
  └── [BaseApp calls bridge.step() after loop]

bridge.step()  [happens after each loop]
  ├── Sends: CBOR2 batch with all queued commands
  ├── Physics: CoppeliaSim advances 1 time step
  ├── Receives: CBOR2 batch with all sensor data
  └── Caches: state dictionary for next loop

stop()
  ├── Save results
  ├── Plot trajectories
  └── Cleanup
```

---

## Common Paths Structure

```
/RobotName                           # Robot root
├── _pos                             # Position [x, y, z]
├── _ori                             # Orientation [alpha, beta, gamma]
├── left_Motor (or similar)          # Joint/motor
├── right_Motor
├── base_link                        # Reference frame
├── Lidar                            # Sensor
│   └── _bin                         # Binary data
├── Camera                           # Another sensor
│   └── _rgb_bin                     # RGB image data
└── Arm                              # Arm/manipulator
    ├── Joint1
    ├── Joint2
    └── Gripper
```

