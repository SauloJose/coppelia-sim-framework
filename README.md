# CoppeliaSim Framework - Professional Robot Simulation

A comprehensive Python framework for controlling and testing robot simulations in CoppeliaSim using the RemoteAPI ZMQ interface.

## Features

✨ **Professional Logging System** - Standardized, emoji-free logging without compromises  
🎯 **BaseApp Architecture** - Complete simulation lifecycle management  
📊 **Built-in Visualization** - Plot2D/Plot3D standardized plotting functions  
🔧 **DRY Principles** - Reduced code duplication and improved maintainability  
🚀 **Production Ready** - Proper packaging with setup.py, pyproject.toml, and tests  
📚 **Well Documented** - Comprehensive docstrings and example implementations

## Project Structure

```text
coppelia_sim_framework/          # Main package
├── core/
│   ├── base_app.py             # BaseApp - simulation lifecycle management
│   └── logging.py              # Professional logging system
├── utils/
│   └── plotting.py             # Plot2D() and Plot3D() visualization
├── sensors/                    # Sensor implementations (extensible)
├── robots/                     # Robot models (extensible)
└── gui/                        # GUI components (future)

examples/                        # Example applications
├── locomotion_example.py       # Lissajous trajectory following
└── obstacle_avoidance_example.py  # LIDAR obstacle avoidance

tests/                          # Unit tests
docs/                           # Documentation
config/                         # Configuration files
scripts/                        # Utility scripts

main.py                         # Interactive menu launcher
setup.py                        # Package installation
pyproject.toml                  # Project metadata
requirements-dev.txt            # Development dependencies
.gitignore                      # Git ignore rules
```

## Installation

### Requirements

- Python 3.8+
- CoppeliaSim 4.4.0 or later
- ZMQ Remote API enabled in CoppeliaSim

### Setup

```bash
# Using pip
pip install -e .

# Or with development dependencies
pip install -e ".[dev]"

# Or using requirements
pip install -r requirements.txt
```

## Quick Start

### Running an Example

```bash
python main.py
```

Select from the interactive menu to run available examples. 
* **Graceful Stop:** Press 's' during execution to stop the simulation.
* **Emergency Stop:** Press Ctrl+C.

### Creating a New Simulation

1. **Create a new file** in `examples/` (e.g., `my_robot_example.py`)
2. **Inherit from BaseApp**:

```python
from coppelia_sim_framework import BaseApp, setup_logger

logger = setup_logger(__name__, '[APP]')

class MyRobotSimulation(BaseApp):
    def __init__(self):
        super().__init__(scene_file="my_scene.ttt", sim_time=30.0)
    
    def setup(self):
        """Called once before simulation starts."""
        logger.info("Setting up robot...")
        self.robot_handle = self.sim.getObject('/MyRobot')
        self.left_motor = self.sim.getObject('/MyRobot/leftMotor')
        self.right_motor = self.sim.getObject('/MyRobot/rightMotor')
    
    def loop(self, t):
        """Called at each simulation step."""
        # Control logic here
        self.sim.setJointTargetVelocity(self.left_motor, 0.5)
        self.sim.setJointTargetVelocity(self.right_motor, 0.5)
    
    def stop(self):
        """Called after simulation ends."""
        logger.info("Stopping motors...")
        self.sim.setJointTargetVelocity(self.left_motor, 0)
        self.sim.setJointTargetVelocity(self.right_motor, 0)

def app():
    """Entry point for main.py menu."""
    sim = MyRobotSimulation()
    sim.run()

if __name__ == "__main__":
    app()
```

3. **Add to menu** - The file will automatically appear in the launcher menu.

## Core Components

### Simulation Lifecycle

The framework implements a standardized lifecycle for all simulations:
1. **Connection:** Connects to CoppeliaSim via ZMQ RemoteAPI.
2. **Scene Loading:** Loads the specified `.ttt` file.
3. **Setup:** Executes `setup()` to get object handles, sensors, and actuators.
4. **Post-Start:** Executes `post_start()` for tests/diagnostics after the simulation starts.
5. **Main Loop:** Executes `loop(t)` repeatedly while `t < sim_time`.
6. **Stop:** Executes `stop()` for cleanup and safe shutdown.

### BaseApp Class

Manages the complete simulation orchestration:

| Method | Mandatory | When it runs | Description |
|--------|-----------|--------------|-------------|
| `setup()` | Yes | Once, before simulation starts | Get handles, initialize variables |
| `loop(t)` | Yes | Every simulation step | Control logic and sensor reading |
| `stop()` | No | After simulation ends | Safe motor shutdown, analysis, plotting |
| `post_start()` | No | Right after simulation starts | Initial tests and diagnostics |

**Useful Simulator Methods (`self.sim`):**
- `getObject(path)` - Gets the handle of an object by its path.
- `getObjectPosition(handle, ref_handle)` - Gets the object's position.
- `getObjectOrientation(handle, ref_handle)` - Gets orientation (Euler angles).
- `setJointTargetVelocity(handle, velocity)` - Sets joint velocity.
- `getSimulationTimeStep()` - Gets the simulation delta time (dt).

### Professional Logging

All modules use a standardized, emoji-free logging system for traceability:

```python
from coppelia_sim_framework import setup_logger

logger = setup_logger(__name__, '[APP]')

logger.info("Starting...")    # [INFO] [APP] [14:32:45] Starting...
logger.error("Failed...")     # [ERROR] [APP] [14:32:47] Failed...
```

**Format**: `[LEVEL] [ORIGIN] [HH:MM:SS] message`

### Visualization Functions

```python
from coppelia_sim_framework import Plot2D, Plot3D
import numpy as np

# 2D trajectory plot
trajectory = np.array([[0, 0], [1, 1], [2, 0.5]])
Plot2D(trajectory, 'X (m)', 'Y (m)', title='Robot Path')

# 3D trajectory plot  
trajectory_3d = np.array([[0, 0, 0], [1, 1, 0.5], [2, 0.5, 1.0]])
Plot3D(trajectory_3d, 'X (m)', 'Y (m)', 'Z (m)', title='3D Robot Motion')
```

Features include a green circle for the start point, a red star for the end point, automatic scaling, and grid generation.

## Best Practices

### 1. Pre-calculate Loop-Invariant Values
Calculate static values in `setup()` rather than in `loop()`.
**Benefit**: Saves ~180 operations/second on 60 Hz loops.

### 2. Robust Sensor Data Validation
Always validate arrays returned by sensors before processing them to avoid scalar/empty format crashes (e.g., `if data is None or data.size == 0:`).

### 3. Use `post_start()` for Initial Diagnostics
Run initial tests or capture initial poses using `post_start()` instead of using unnecessary `_first_exec` flags inside your `loop()`.

### 4. Separate Real vs. Reference Trajectories
Keep isolated lists for simulated coordinates and reference/desired coordinates to make plotting and error calculation easier.

## Examples

1. **Lissajous Trajectory Following (`locomotion_example.py`)**
   - Trajectory generation, differential kinematics, velocity control, and visualization.
2. **Obstacle Avoidance (`obstacle_avoidance_example.py`)**
   - LIDAR sensor integration, deviation logic, and real-time decision making.

## Running Tests

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=coppelia_sim_framework
```

## Troubleshooting

- **"Program freezes at connection":** CoppeliaSim is not running. Start the application first.
- **"Handle not found (-1 returned)":** Ensure the object path is correct (must start with `/`), the scene is loaded properly, and the name matches the `.ttt` file.
- **"ZMQ Communication Error":** Enable RemoteAPI in CoppeliaSim via `Tools > Remote API server` (must be ON).
- **"TypeError: len() of unsized object":** The sensor returned an invalid data format. Ensure proper data validation with `numpy` arrays.

## Version History

**v1.1.0** (Current)
- Professional folder structure for distribution and packaging (`setup.py`, `pyproject.toml`).
- Completely revamped logging system (standardized, traceable, no emojis).
- Added `post_start()` lifecycle method.
- Comprehensive documentation and unit test framework.

**v1.0.0**
- Initial framework with BaseApp class.
- Basic plotting functionality (Plot2D/Plot3D).

## Contributing

1. Fork the repository
2. Create a feature branch
3. Follow the code style guidelines
4. Add tests for new features
5. Submit a pull request

See [CONTRIBUTING.md](docs/CONTRIBUTING.md) for details.

## License & Support

- **License:** MIT License - see LICENSE file for details.
- **Documentation:** Check the [docs/](docs/) folder.
- **Support:** Report issues on GitHub or ask questions in discussions.
```