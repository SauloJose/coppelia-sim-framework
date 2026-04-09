# API Reference

## Core Module

### BaseApp

Base class for all CoppeliaSim simulations.

```python
class BaseApp:
    def __init__(self, scene_file: str = None, sim_time: float = 10.0)
```

**Parameters:**
- `scene_file` (str, optional): Path to .ttt scene file relative to `scenes/` folder
- `sim_time` (float): Maximum simulation time in seconds. Default is 10.0 seconds

**Methods:**

#### run()
```python
def run(self) -> None
```
Main execution method. Orchestrates the complete simulation lifecycle.

#### setup()
```python
def setup(self) -> None
```
Override this method to configure resources before simulation starts.
- Get object handles
- Initialize sensors
- Set initial parameters

#### post_start()
```python
def post_start(self) -> None
```
Override this method to execute code immediately after simulation starts (first post-step).
- Capture initial sensor readings
- Log initial robot pose

#### loop(t)
```python
def loop(self, t: float) -> None
```
Override this method to run control logic at each simulation step.

**Parameters:**
- `t` (float): Current simulation time in seconds

#### stop()
```python
def stop(self) -> None
```
Override this method to execute cleanup code after simulation ends.
- Save trajectory data
- Plot results
- Stop motors

---

## Logging Module

### ProfessionalFormatter

Custom logging formatter for professional output.

```python
class ProfessionalFormatter(logging.Formatter):
    def __init__(self, origin_prefix: str = '[APP]')
```

**Parameters:**
- `origin_prefix` (str): Logging origin indicator, e.g., '[MAIN]' or '[APP]'

**Output Format:**
```
[LEVEL] [ORIGIN] [HH:MM:SS] message
```

### setup_logger()

Factory function to create configured loggers.

```python
def setup_logger(name: str, origin_prefix: str = '[APP]') -> logging.Logger
```

**Parameters:**
- `name` (str): Logger name, typically `__name__`
- `origin_prefix` (str): Origin indicator prefix

**Returns:**
- `logging.Logger`: Configured logger instance

**Example:**
```python
from coppelia_sim_framework import setup_logger

logger = setup_logger(__name__, '[APP]')
logger.info("Application started")  # [INFO] [APP] [14:32:45] Application started
```

---

## Utils Module

### Plot2D()

Plot 2D robot trajectory with marked start and end points.

```python
def Plot2D(
    data: array_like,
    x_label: str,
    y_label: str,
    tamanho_janela: tuple = (8, 6),
    limite_x: tuple = None,
    limite_y: tuple = None,
    title: str = None
) -> None
```

**Parameters:**
- `data` (array_like): 2D array of shape (N, 2) with [x, y] coordinates
- `x_label` (str): Label for X-axis
- `y_label` (str): Label for Y-axis
- `tamanho_janela` (tuple): Figure size in inches (width, height). Default: (8, 6)
- `limite_x` (tuple, optional): X-axis limits (min, max)
- `limite_y` (tuple, optional): Y-axis limits (min, max)
- `title` (str, optional): Plot title. Default: "{y_label} vs {x_label}"

**Features:**
- Green circle: trajectory start point
- Red star: trajectory end point
- Blue line: trajectory path
- Grid and equal aspect ratio enabled

**Example:**
```python
from coppelia_sim_framework import Plot2D
import numpy as np

trajectory = np.array([[0, 0], [1, 1], [2, 0.5]])
Plot2D(trajectory, 'Position X (m)', 'Position Y (m)', title='Robot Path')
```

### Plot3D()

Plot 3D robot trajectory with marked start and end points.

```python
def Plot3D(
    data: array_like,
    x_label: str,
    y_label: str,
    z_label: str,
    tamanho_janela: tuple = (8, 6),
    limite_x: tuple = None,
    limite_y: tuple = None,
    limite_z: tuple = None,
    title: str = None
) -> None
```

**Parameters:**
- `data` (array_like): 2D array of shape (N, 3) with [x, y, z] coordinates
- `x_label`, `y_label`, `z_label` (str): Axis labels
- `tamanho_janela` (tuple): Figure size. Default: (8, 6)
- `limite_x`, `limite_y`, `limite_z` (tuple, optional): Axis limits
- `title` (str, optional): Plot title

**Features:**
- 3D visualization of trajectory
- Same marking conventions as Plot2D
- Full 3D rotation and zoom support

**Example:**
```python
from coppelia_sim_framework import Plot3D
import numpy as np

trajectory_3d = np.array([
    [0, 0, 0],
    [1, 1, 0.5],
    [2, 0.5, 1.0]
])
Plot3D(trajectory_3d, 'X (m)', 'Y (m)', 'Z (m)', title='3D Robot Motion')
```

---

## Common Patterns

### Creating a Simulation

```python
from coppelia_sim_framework import BaseApp, setup_logger

logger = setup_logger(__name__, '[APP]')

class MySimulation(BaseApp):
    def __init__(self):
        super().__init__(scene_file="my_scene.ttt", sim_time=30.0)
        
    def setup(self):
        logger.info("Configuring simulation...")
        self.robotHandle = self.sim.getObject('/MyRobot')
        
    def loop(self, t):
        position = self.sim.getObjectPosition(self.robotHandle, -1)
        logger.debug(f"Position at t={t}: {position}")
        
    def stop(self):
        logger.info("Simulation ended")

def app():
    sim = MySimulation()
    sim.run()

if __name__ == "__main__":
    app()
```

### Handling Sensor Data

```python
import numpy as np

def loop(self, t):
    try:
        # Read sensor data
        raw_data = self.sensor.getData()
        data = np.asarray(raw_data)
        
        # Validate
        if data is None or data.size == 0:
            logger.warning("Empty sensor data")
            return
        
        if data.ndim == 0:
            logger.error(f"Scalar instead of array: {data}")
            return
        
        # Use data
        distance = data[0, 1]
        logger.debug(f"Distance: {distance:.2f}m")
        
    except Exception as e:
        logger.error(f"Sensor read error: {e}")
```

### Kinematics for Differential Robots

```python
# Convert linear (v) and angular (w) velocities to wheel velocities
v = 0.5        # m/s
w = 0.2        # rad/s
L = 0.381      # distance between wheels (m)
r = 0.0975     # wheel radius (m)

wl = (v / r) - (w * L) / (2 * r)  # left wheel
wr = (v / r) + (w * L) / (2 * r)  # right wheel

self.sim.setJointTargetVelocity(self.l_wheel, wl)
self.sim.setJointTargetVelocity(self.r_wheel, wr)
```

---

## Error Handling

### Common Exceptions

**RemoteAPIClient connection errors:**
```python
try:
    client = RemoteAPIClient()
except Exception as e:
    logger.error(f"Connection failed: {e}")
    sys.exit(1)
```

**Invalid handles:**
```python
handle = self.sim.getObject('/NonExistentObject')
if handle == -1:
    logger.error("Object not found in scene")
    raise RuntimeError("Missing object")
```

**Scene loading errors:**
```python
try:
    self.sim.loadScene(scene_path)
except Exception as e:
    logger.error(f"Failed to load scene: {e}")
    raise
```

---

## Version History

**v1.1.0** (Current)
- Added `post_start()` lifecycle method
- Improved sensor data validation patterns
- Enhanced documentation

**v1.0.0**
- Initial framework release
- Core BaseApp class
- Professional logging system
- Plot2D/Plot3D visualization

