# Brainbyte - Professional Robot Simulation Framework

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
brainbyte/                       # Main package
├── logs/                        # Log files directory
│   ├── main.log                 # GUI interface logs
│   └── simulation.log           # Simulation execution logs
├── core/
│   ├── base_app.py              # BaseApp - simulation lifecycle management
│   ├── bridge.py                # ZMQ communication bridge
│   └── logging.py               # Core logging setup
├── utils/
│   ├── basics/                  # Templates and base files
│   │   ├── app.txt              # Application template
│   │   └── sim.ttt              # Basic simulation scene
│   ├── logging.py               # Utility logging functions
│   ├── plotting.py              # Plot2D() and Plot3D() visualization
│   └── math.py                  # Mathematical calculations and utilities
├── robots/                      # Robot models and classes
│   ├── base/
│   │   └── base_robot.py        # Parent class for all robot models
│   ├── arms/                    # Robotic arm implementations
│   ├── humanoid/                # Humanoid robot implementations
│   ├── models/                  # .ttt models to import into CoppeliaSim
│   └── movel/                   # Mobile robots (Pioneer, TurtleBot, Manta, Robotino, etc.)
├── gui/
│   ├── auxF.py                  # Auxiliary functions for the GUI
│   └── cli.py                   # Beautiful CLI-based GUI implementation
└── sensors/                     # Sensor implementations (extensible)

projects/                        # Project storage organized by topics
├── locomotion/                  # Topic category
│   └── my_locomotion_project/   # Specific project folder
│       ├── my_locomotion_project.py # Main script (matches folder name)
│       └── scene.ttt            # CoppeliaSim scene file for this project
└── obstacle_avoidance/          # Topic category
    └── lidar_avoidance/         # Specific project folder
        ├── lidar_avoidance.py   
        └── lidar_avoidance.ttt  

tests/                           # Unit tests
docs/                            # Documentation
config/                          # Configuration files

main.py                          # Interactive menu launcher
setup.py                         # Package installation
pyproject.toml                   # Project metadata
requirements-dev.txt             # Development dependencies
.gitignore                       # Git ignore rules
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

### Running a Project

```bash
python main.py
```

Select from the interactive CLI menu to run available projects. 
* **Graceful Stop:** Press 's' during execution to stop the simulation.
* **Emergency Stop:** Press Ctrl+C.

### Creating a New Simulation

1. **Create a new project folder and files** inside a topic category in `projects/` (e.g., `projects/locomotion/my_robot/`). The Python script must match the folder's name, and the scene file should be placed alongside it:
   * `projects/locomotion/my_robot/my_robot.py`
   * `projects/locomotion/my_robot/my_robot.ttt`
2. **Inherit from BaseApp** inside your `.py` file:

```python
from brainbyte.core.base_app import BaseApp
from brainbyte.core.logging import setup_logger

logger = setup_logger(__name__, '[APP]')

class MyRobotSimulation(BaseApp):
    def __init__(self):
        # Point to the .ttt file in the same directory
        super().__init__(scene_file="my_robot.ttt", sim_time=30.0)
    
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

### Communication Bridge Architecture

To ensure fast, reliable, and concise data exchange between Python and the simulator, the framework utilizes a highly optimized ZeroMQ (ZMQ) bridge architecture located in `core/bridge.py`:

- **CoppeliaSim Thread Script:** Inside the CoppeliaSim environment, a dedicated threaded script actively listens for incoming requests.
- **Variable Caching:** When initialized, this script receives a list of parameters and saves in cache exactly which variables and sensors it needs to monitor.
- **CBOR2 Serialization:** Data is packaged and transmitted using the binary `cbor2` format, which is drastically lighter and faster to process than standard JSON.
- **Path-based Keying:** When the simulator replies, the payload uses the actual simulation "path" of the variable (e.g., `/MyRobot/Lidar`) as the dictionary key. This allows the Python logic to instantly map incoming binary data to the correct virtual components without heavy parsing overhead.

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

All modules use a standardized, emoji-free logging system for traceability. Logs are automatically saved in the `brainbyte/logs/` directory.

```python
from brainbyte.core.logging import setup_logger

logger = setup_logger(__name__, '[APP]')

logger.info("Starting...")    # [INFO] [APP] Starting...
logger.error("Failed...")     # [ERROR] [APP] Failed...
```

**Format**: `[LEVEL] [ORIGIN] [HH:MM:SS] message`

### Visualization Functions

```python
from brainbyte.utils.plotting import Plot2D, Plot3D
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

## Running Tests

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=brainbyte
```

## Troubleshooting

- **"Program freezes at connection":** CoppeliaSim is not running. Start the application first.
- **"Handle not found (-1 returned)":** Ensure the object path is correct (must start with `/`), the scene is loaded properly, and the name matches the `.ttt` file.
- **"ZMQ Communication Error":** Enable RemoteAPI in CoppeliaSim via `Tools > Remote API server` (must be ON).
- **"TypeError: len() of unsized object":** The sensor returned an invalid data format. Ensure proper data validation with `numpy` arrays.

## Version History

**v1.1.60** (Current)
- Main package renamed to `brainbyte`.
- Reorganized module structure: `core`, `utils`, `robots`, `gui`.
- Moved logging directory to root (`brainbyte/logs/main.log` and `simulation.log`).
- Grouped robot modules logically (`arms`, `base`, `humanoid`, `models`, `movel`).
- Re-architected `projects/` directory into a Topic > Project > Files hierarchy. Main scripts must now match their parent project folder name.
- Added beautiful CLI implementation in `gui/cli.py` with auxiliary functions in `auxF.py`.
- Removed deprecated `scripts/` directory.
- Included base template files inside `utils/basics/`.

**v1.1.0**
- Professional folder structure for distribution and packaging.
- Completely revamped logging system.
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
