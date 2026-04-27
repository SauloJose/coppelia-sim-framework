
# Brainbyte API Reference

This document provides a technical reference for the core classes, methods, and utilities available in the Brainbyte framework.

---

## 1. Core Module (`brainbyte.core`)

### BaseApp
**Path:** `brainbyte.core.base_app.BaseApp`

The abstract base class that manages the simulation lifecycle and network communication. All simulation projects must inherit from this class.

```python
def __init__(self, scene_file: str = None, sim_time: float = 10.0)
```
* **`scene_file`** (`str`, optional): Path to the `.ttt` scene file. Usually placed alongside the simulation script.
* **`sim_time`** (`float`): Maximum simulation duration in seconds.

#### Lifecycle Methods (To be overridden)

* **`setup(self) -> None`**
    Called once before the simulation starts. Use this to establish simulation handles, define variables, and instantiate controllers or robot objects.
* **`post_start(self) -> None`**
    Called exactly once immediately after the first simulation step. Ideal for capturing initial ground-truth poses or initializing algorithms that require the physics engine to be active.
* **`loop(self, t: float) -> None`**
    Called continuously at each simulation step. Contains the main control loop.
    * **`t`**: Current simulation time in seconds.
* **`stop(self) -> None`**
    Called after the simulation terminates. Use this to safely stop robot motors, process final data arrays, and generate plots.

#### Internal Execution
* **`run(self) -> None`**
    Triggers the framework's internal event loop. Should only be called within the `app()` entry point.

---

### SimulationBridge
**Path:** `brainbyte.core.bridge.SimulationBridge`

Handles the ZeroMQ/CBOR data batching. Accessed internally via `self.bridge` inside your `BaseApp` instance.

#### Queuing Methods (Batch Dataflow)
* **`queue_velocity(self, path: str, velocity: float) -> None`**
    Schedules a velocity target for a specific joint.
* **`queue_position(self, path: str, position: float) -> None`**
    Schedules a position target for a specific joint or servo.
* **`queue_command(self, category: str, path: str, value: any) -> None`**
    Schedules a custom command (e.g., teleportation, custom Lua function triggers).

#### Data Retrieval
* **`get_sensor_data(self, path: str) -> any`**
    Retrieves the latest cached data for a specific path from the current frame without triggering network I/O. Returns deserialized CBOR data (e.g., native `numpy` arrays for LiDAR).

---

## 2. Robots & Controllers (`brainbyte.robots`)

### Control Interfaces
**Path:** `brainbyte.robots.base`

* **`AutoController`**: Base class for implementing autonomous behaviors, path planning (e.g., A*, RRT), and trajectory following algorithms.
* **`ManualController`**: Base class mapping keyboard/joystick inputs to robot velocity queues.

### Pre-built Robot Models
**Path:** `brainbyte.robots.movel`

Standardized classes wrapping differential, omnidirectional, or ackermann kinematics.

```python
from brainbyte.robots.movel.turtlebot import TurtleBot

# Typical usage within BaseApp.setup()
self.robot = TurtleBot(base_path='/TurtleBot', bridge=self.bridge)
```

---

## 3. Utilities (`brainbyte.utils`)

### Plotting
**Path:** `brainbyte.utils.plotting`

Provides standardized Matplotlib wrappers configured for robotics visualization.

```python
def Plot2D(data, x_label: str, y_label: str, tamanho_janela=(8, 6), limite_x=None, limite_y=None, title=None) -> None
```
* **`data`**: `array_like` of shape `(N, 2)` representing [x, y] coordinates.
* **Visuals**: Automatically adds a green circle at `data[0]` (start) and a red star at `data[-1]` (end).

```python
def Plot3D(data, x_label: str, y_label: str, z_label: str, tamanho_janela=(8, 6), limite_x=None, limite_y=None, limite_z=None, title=None) -> None
```
* **`data`**: `array_like` of shape `(N, 3)` representing [x, y, z] coordinates.

### Math Operations
**Path:** `brainbyte.utils.math`

* **`normalize_angle(angle: float) -> float`**: Normalizes an angle to the $[-\pi, \pi]$ range.
* **`euler_to_quaternion(roll, pitch, yaw) -> list`**: Converts Euler angles to quaternion representation.

---

## 4. Logging (`brainbyte.core.logging`)

### setup_logger
**Path:** `brainbyte.core.logging.setup_logger`

Instantiates a strictly formatted logger that outputs to both the console and `brainbyte/logs/`.

```python
def setup_logger(name: str, origin_prefix: str = '[APP]') -> logging.Logger
```
* **`name`**: Typically `__name__` of the calling module.
* **`origin_prefix`**: Sub-system identifier (e.g., `[BRIDGE]`, `[CLI]`, `[ROBOT]`).

**Usage:**
```python
from brainbyte.core.logging import setup_logger
logger = setup_logger(__name__, '[APP]')

logger.debug("Debugging values.")
logger.info("Standard execution info.")
logger.warning("Non-critical issue detected.")
logger.error("Critical failure.")
```
