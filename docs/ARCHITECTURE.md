# CoppeliaSim Framework Architecture

## Overview

The CoppeliaSim Framework is a professional Python package designed to simplify robot control and simulation using CoppeliaSim's ZMQ remote API. It provides:

- **BaseApp**: A base class that handles the simulation lifecycle
- **Professional Logging**: Standardized, emoji-free logging system
- **Visualization Tools**: Built-in 2D and 3D plotting functions
- **Best Practices**: Structured examples and documentation

## Directory Structure

```
coppelia_sim_framework/
├── __init__.py              # Package exports
├── core/
│   ├── __init__.py
│   ├── base_app.py          # BaseApp class - simulation lifecycle management
│   └── logging.py           # ProfessionalFormatter and setup_logger()
├── utils/
│   ├── __init__.py
│   └── plotting.py          # Plot2D() and Plot3D() visualization functions
├── sensors/                 # Sensor implementations (future)
├── robots/                  # Robot definitions (future)
└── gui/                     # GUI components (future)

examples/                     # Example applications
├── __init__.py
├── locomocao_example.py     # Lissajous trajectory following
└── obstacle_avoidance_example.py  # LIDAR-based obstacle avoidance

tests/                        # Unit and integration tests
├── __init__.py
└── test_framework.py

docs/                         # Documentation
├── ARCHITECTURE.md           # This file
├── API.md                    # API reference
├── CONTRIBUTING.md           # Contributing guidelines
└── TUTORIALS.md              # Step-by-step tutorials

scripts/                      # Utility scripts
config/                       # Configuration files

main.py                       # Interactive menu for running examples
setup.py                      # Package setup for pip installation
pyproject.toml               # Modern Python project metadata
requirements-dev.txt         # Development dependencies
README.md                    # Main documentation
.gitignore                   # Git ignore rules
```

## Module Responsibilities

### core.base_app

**Purpose**: Manages the simulation lifecycle

- Handles connection to CoppeliaSim
- Loads scenes
- Manages simulation steps
- Provides hooks: `setup()`, `post_start()`, `loop(t)`, `stop()`

**Key Class**: `BaseApp`

```python
class BaseApp:
    def __init__(self, scene_file=None, sim_time=10.0)
    def setup(self)           # Override: configure before simulation
    def post_start(self)      # Override: execute after simulation starts
    def loop(self, t)         # Override: called each simulation step
    def stop(self)            # Override: cleanup after simulation
    def run(self)             # Main loop - orchestrates everything
```

### core.logging

**Purpose**: Professional logging system without emojis

- Standardized log format: `[LEVEL] [ORIGIN] [TIMESTAMP] message`
- Easy logger creation with `setup_logger()`

**Key Classes/Functions**:
- `ProfessionalFormatter`: Custom logging formatter
- `setup_logger(name, origin_prefix)`: Logger factory function

### utils.plotting

**Purpose**: Visualization of robot trajectories

- `Plot2D(data, x_label, y_label, ...)`: 2D trajectory visualization
- `Plot3D(data, x_label, y_label, z_label, ...)`: 3D trajectory visualization

Both functions automatically mark start (green dot) and end (red star) points.

## Execution Flow

```
┌─────────────────────────────────────────┐
│  main.py                                │
│  (Interactive menu to select examples)  │
└──────────────────┬──────────────────────┘
                   │
                   ├─→ locomocao_example.py
                   │   └→ LocomocaoTeste(BaseApp)
                   │       └→ app() function triggers .run()
                   │
                   └─→ obstacle_avoidance_example.py
                       └→ ObstacleAvoidanceTester(BaseApp)
                           └→ app() function triggers .run()

When .run() is called:
1. Load scene
2. Configure ZMQ stepping mode
3. Call setup() [OVERRIDE THIS]
4. Start simulation
5. Call post_start() [OVERRIDE THIS]
6. Loop:
   - Check for user interrupt ('s' key)
   - Call loop(t) [OVERRIDE THIS]
   - Advance simulation step
7. On exit:
   - Call stop() [OVERRIDE THIS]
   - Stop simulation
```

## Logging Pattern

All modules use the professional logging system:

```python
from coppelia_sim_framework import setup_logger

logger = setup_logger(__name__, '[APP]')  # or '[MAIN]' for main.py

logger.info("Starting simulation...")      # [INFO] [APP] [HH:MM:SS] Starting simulation...
logger.warning("Sensor returned invalid data")  # [WARNING] [APP] [HH:MM:SS] ...
logger.error("Failed to set motor velocity")    # [ERROR] [APP] [HH:MM:SS] ...
```

## Best Practices Implemented

1. **Constant Extraction**: All magic numbers are defined as class constants
2. **Pre-calculation**: Loop-invariant computations are done in `setup()` or `post_start()`
3. **Validation**: Robust array/sensor data validation before use
4. **Separation of Concerns**: Real vs. reference trajectories are tracked separately
5. **Error Handling**: Try-except blocks with informative log messages
6. **Code Organization**: Methods grouped by responsibility (setup, loop, visualization)

## Adding a New Example

1. Create a new file in `examples/` (e.g., `my_example.py`)
2. Inherit from `BaseApp`
3. Implement `setup()`, `loop()`, and `stop()`
4. Define an `app()` function that instantiates and runs your example
5. The example will automatically appear in the `main.py` menu

Example template:

```python
from coppelia_sim_framework import BaseApp, setup_logger

logger = setup_logger(__name__, '[APP]')

class MyTest(BaseApp):
    def __init__(self):
        super().__init__(scene_file="my_scene.ttt", sim_time=30.0)
    
    def setup(self):
        logger.info("Setting up...")
        # Get handles, configure sensors
    
    def loop(self, t):
        # Control logic, sensor reading, data logging
        pass
    
    def stop(self):
        logger.info("Cleaning up...")
        # Save results, plot data

def app():
    teste = MyTest()
    teste.run()

if __name__ == "__main__":
    app()
```

## Future Extensions

- **More Sensors**: IMU, Ultrasonic, Camera integration
- **Robot Classes**: Predefined robot models (Pioneer variants, etc.)
- **GUI**: Real-time visualization during simulation
- **Advanced Control**: Trajectory tracking, motion planning modules
- **Distributed Testing**: CI/CD pipeline for regression testing

## References

- [CoppeliaSim Documentation](https://www.coppeliarobotics.com/helpFiles/)
- [ZMQ Remote API Guide](https://www.coppeliarobotics.com/helpFiles/en/zmqRemoteAPIOverview.htm)
- [Pioneer P3DX Specifications](http://www.mobilerobots.com/ResearchRobots/PioneerP3DX.aspx)
