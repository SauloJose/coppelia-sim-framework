# Contributing Guidelines

Thank you for your interest in contributing to Brainbyte! This document explains how to set up your development environment, follow our code style, and contribute effectively.

---

## Development Setup

### Prerequisites

- Python 3.8 or higher
- CoppeliaSim 4.4.0 or later
- Git for version control
- pip or conda for package management

### Environment Setup

```bash
# Clone the repository
git clone https://github.com/your-org/brainbyte.git
cd brainbyte

# Create a virtual environment
python -m venv venv
source venv/bin/activate      # On Windows: venv\Scripts\activate

# Install development dependencies
pip install -r requirements-dev.txt
pip install -e .               # Install in editable mode
```

### Verify Installation

```bash
# Run basic test
python -c "from brainbyte import BaseApp; print('Installation OK')"

# Run test suite
pytest tests/ -v
```

---

## Code Style & Standards

### Python Style Guide

We follow **PEP 8** with these specifications:

- **Line length**: Maximum 100 characters
- **Indentation**: 4 spaces (never tabs)
- **Imports**: Absolute imports, organized by groups (stdlib, third-party, local)
- **Naming**: 
  - Classes: `PascalCase` (e.g., `DifferentialController`)
  - Functions/Methods: `snake_case` (e.g., `queue_velocity`)
  - Constants: `UPPER_SNAKE_CASE` (e.g., `DEFAULT_TIMEOUT`)
  - Private members: `_leading_underscore` (e.g., `_internal_cache`)

### Docstring Format

Use Google-style docstrings:

```python
def process_trajectory(trajectory: np.ndarray, smooth: bool = True) -> list:
    """Process robot trajectory data.
    
    Applies optional smoothing and interpolation to trajectory points.
    
    Args:
        trajectory (np.ndarray): Array of shape (N, 3) with [x, y, theta] waypoints
        smooth (bool): Whether to apply Savitzky-Golay smoothing. Defaults to True.
    
    Returns:
        list: Smoothed waypoints as list of [x, y, theta] arrays
    
    Raises:
        ValueError: If trajectory shape is invalid
        
    Example:
        >>> traj = np.array([[0, 0, 0], [1, 1, 0.785], [2, 0, 1.57]])
        >>> smooth_traj = process_trajectory(traj, smooth=True)
        >>> print(len(smooth_traj))
        3
    """
    if trajectory.ndim != 2 or trajectory.shape[1] != 3:
        raise ValueError("Trajectory must have shape (N, 3)")
    
    if smooth:
        # Apply smoothing...
        pass
    
    return trajectory.tolist()
```

### Type Hints

Add type hints to function signatures where beneficial:

```python
from typing import List, Tuple, Optional
import numpy as np

def compute_wheel_velocities(
    linear_velocity: float,
    angular_velocity: float,
    wheelbase: float = 0.287,
    wheel_radius: float = 0.0066
) -> Tuple[float, float]:
    """Compute individual wheel velocities from chassis kinematics.
    
    Args:
        linear_velocity: Forward speed (m/s)
        angular_velocity: Rotation rate (rad/s)
        wheelbase: Distance between wheels (m)
        wheel_radius: Wheel radius (m)
    
    Returns:
        Tuple of (left_wheel_vel, right_wheel_vel) in rad/s
    """
    # Implementation...
    pass
```

---

## Code Quality Tools

### Formatting with Black

```bash
# Format entire project
black brainbyte/ tests/

# Format single file
black brainbyte/core/bridge.py

# Check without modifying
black brainbyte/ --check
```

**Configuration** (in `pyproject.toml`):
```toml
[tool.black]
line-length = 100
target-version = ['py38', 'py39', 'py310']
```

### Linting with Flake8

```bash
# Check code quality
flake8 brainbyte/ tests/

# Check specific file
flake8 brainbyte/robots/base/base_bot.py

# Ignore specific rules (in .flake8):
ignore = E203, W503, E501  # Line too long (handled by black)
max-line-length = 100
exclude = .git,__pycache__,venv
```

### Type Checking with Mypy

```bash
# Run type checker
mypy brainbyte/

# Check specific module
mypy brainbyte/core/bridge.py
```

### Run All Checks

```bash
# Complete quality check
./scripts/check_quality.sh

# Or manually:
black brainbyte/ tests/
flake8 brainbyte/ tests/
mypy brainbyte/
pytest tests/ -v
```

---

## Logging Standards

All logging must use the professional framework without emojis:

```python
from brainbyte.utils.logging import setup_logger

logger = setup_logger(__name__, '[MODULE_PREFIX]', log_file='module.log')

# Correct logging (professional):
logger.info("Simulation started successfully")
logger.warning("Deprecated parameter used: 'old_param'")
logger.error("Failed to load scene: file not found")
logger.debug("Cache updated with 42 sensor readings")

# Avoid (unprofessional):
logger.info("✓ Simulation started!")           # Don't use emojis
print("Starting simulation...")                 # Don't use print()
logger.info(f"Result: {result} 🎉")            # No emojis in messages
```

**Log Levels:**
- `DEBUG`: Detailed information for diagnosing problems (variable values, intermediate results)
- `INFO`: General informational messages (state changes, important events)
- `WARNING`: Warning messages (deprecated usage, recoverable errors)
- `ERROR`: Error messages (failures that prevent operation, exceptions)
- `CRITICAL`: Critical failures (simulation cannot continue)

---

## Creating New Components

### Adding a Robot Model

Create a new file in `brainbyte/robots/movel/`:

```python
# brainbyte/robots/movel/MyRobot.py
import numpy as np
from brainbyte.robots.base.base_bot import BaseBot
from brainbyte.utils.logging import setup_logger

logger = setup_logger(__name__, '[MYROBOT]')

class MyRobot(BaseBot):
    """Omnidirectional mobile robot with 3 wheels.
    
    Attributes:
        _v_max (float): Maximum linear velocity (m/s)
        _w_max (float): Maximum angular velocity (rad/s)
    """
    
    def __init__(self, bridge, robot_name='MyRobot', **kwargs):
        """Initialize robot.
        
        Args:
            bridge: SimulationBridge instance
            robot_name: Name in CoppeliaSim scene
        """
        super().__init__(bridge, robot_name)
        
        # Physical parameters
        self._v_max = 0.5           # m/s
        self._w_max = 5.0           # rad/s
        
        logger.info(f"Initialized {robot_name}")
```

**Checklist:**
- [ ] Inherits from `BaseBot`
- [ ] Has docstring explaining the robot
- [ ] Initializes bridge in `__init__`
- [ ] Includes physical parameters as properties
- [ ] Added to `brainbyte/robots/__init__.py`

### Adding a Sensor

Create a new file in `brainbyte/sensors/`:

```python
# brainbyte/sensors/MySensor.py
from brainbyte.sensors.base.base_sensor import BaseSensor
from brainbyte.utils.logging import setup_logger

logger = setup_logger(__name__, '[MYSENSOR]')

class MySensor(BaseSensor):
    """Custom sensor reading measurement data.
    
    Monitors:
        - sensor_path_data: Raw measurement
        - sensor_path_timestamp: Measurement timestamp
    """
    
    def __init__(self, bridge, sensor_path):
        """Initialize sensor.
        
        Args:
            bridge: SimulationBridge instance
            sensor_path: Path in CoppeliaSim scene
        """
        super().__init__(bridge, sensor_path)
        self.last_value = None
    
    def get_monitor_paths(self):
        """Declare paths to monitor."""
        return [
            f"{self.sensor_path}_data",
            f"{self.sensor_path}_timestamp"
        ]
    
    def update(self):
        """Process latest sensor reading."""
        data = self.get_bridge_data('_data')
        if data is not None:
            self.last_value = data
            logger.debug(f"Sensor reading: {data}")
```

**Checklist:**
- [ ] Inherits from `BaseSensor`
- [ ] Implements `get_monitor_paths()`
- [ ] Implements `update()`
- [ ] Has clear docstring
- [ ] Added to `brainbyte/sensors/__init__.py`

### Adding a Controller

Create a new file in `brainbyte/control/`:

```python
# brainbyte/control/automatic.py (add to this file)
import numpy as np
from brainbyte.utils.logging import setup_logger

logger = setup_logger(__name__, '[CONTROL]')

class MyController:
    """Custom control law for robot navigation.
    
    Implements feedback control based on state error.
    """
    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0):
        """Initialize controller gains.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_error = 0.0
        self.last_error = 0.0
    
    def compute_control(self, error: float, dt: float) -> float:
        """Compute control output.
        
        Args:
            error: Current state error
            dt: Time step
        
        Returns:
            Control command
        """
        P = self.kp * error
        self.integral_error += error * dt
        I = self.ki * self.integral_error
        D = self.kd * (error - self.last_error) / dt if dt > 0 else 0.0
        
        self.last_error = error
        return P + I + D
```

---

## Testing

### Unit Tests

Create tests in `tests/`:

```python
# tests/test_bridge.py
import unittest
import numpy as np
from brainbyte.core.bridge import SimulationBridge

class TestSimulationBridge(unittest.TestCase):
    """Test SimulationBridge functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.bridge = SimulationBridge()
    
    def test_queue_velocity(self):
        """Test velocity queuing."""
        self.bridge.queue_velocity('/motor1', 2.5)
        
        # Check that command is in buffer
        self.assertIn('/motor1', self.bridge.command_buffer['velocities'])
        self.assertEqual(
            self.bridge.command_buffer['velocities']['/motor1'],
            2.5
        )
    
    def test_queue_position(self):
        """Test position queuing."""
        self.bridge.queue_position('/servo1', 1.57)
        
        self.assertIn('/servo1', self.bridge.command_buffer['positions'])
    
    def test_get_sensor_data_empty_cache(self):
        """Test retrieval from empty cache."""
        data = self.bridge.get_sensor_data('/nonexistent')
        self.assertIsNone(data)

if __name__ == '__main__':
    unittest.main()
```

### Running Tests

```bash
# Run all tests
pytest tests/ -v

# Run specific test file
pytest tests/test_bridge.py -v

# Run specific test case
pytest tests/test_bridge.py::TestSimulationBridge::test_queue_velocity -v

# Run with coverage report
pytest tests/ --cov=brainbyte --cov-report=html
```

---

## Documentation

### Updating Documentation

When adding new features:

1. **Update docstrings** in the code (Google style)
2. **Update API.md** if adding public methods
3. **Update ARCHITECTURE.md** if changing core design
4. **Update README.md** if affecting quick start

### Example Documentation Entry

For a new robot:

**README.md:**
```markdown
### TurtleBot
- Differential drive robot
- Max velocity: 0.31 m/s
- Wheelbase: 0.287 m
```

**API.md:**
```markdown
### TurtleBot

Path: `brainbyte.robots.movel.TurtleBot.TurtleBot`

Constructor:
...
```

**ARCHITECTURE.md:**
```markdown
### TurtleBot Implementation

TurtleBot uses differential kinematics with...
```

---

## Git Workflow

### Branch Naming

```
feature/robot-name        → New robot model
sensor/sensor-name        → New sensor
fix/issue-description     → Bug fix
docs/documentation-update → Documentation
```

### Commit Messages

Use clear, descriptive commit messages:

```
✓ Good:
"Add TurtleBot robot model with differential kinematics"
"Fix LIDAR data conversion in SimulationBridge"
"Update API documentation for BaseApp lifecycle"

✗ Avoid:
"fix bug"
"update code"
"changes"
```

### Pull Request Template

```markdown
## Description
Brief description of changes

## Type of Change
- [ ] New feature
- [ ] Bug fix
- [ ] Documentation update
- [ ] Code refactoring

## Testing
- [ ] Unit tests pass
- [ ] Code style checks pass
- [ ] Type checking passes
- [ ] Manual testing completed

## Checklist
- [ ] Docstrings added/updated
- [ ] Type hints added
- [ ] Tests added
- [ ] Documentation updated
```

---

## Performance Considerations

### Avoid Common Pitfalls

```python
# ✗ DON'T: Call bridge.step() multiple times in loop
def loop(self, t):
    self.bridge.step()      # Network call
    data1 = self.bridge.get_sensor_data('/path1')
    self.bridge.step()      # Another network call ❌
    data2 = self.bridge.get_sensor_data('/path2')

# ✓ DO: Queue all commands before one step()
def loop(self, t):
    self.bridge.queue_velocity('/motor1', 1.0)
    self.bridge.queue_velocity('/motor2', 1.0)
    state = self.bridge.step()  # Single network call ✓
    data1 = state.get('/path1')
    data2 = state.get('/path2')
```

```python
# ✗ DON'T: Create new logger instances
def loop(self, t):
    logger = setup_logger(__name__)  # Inefficient ❌

# ✓ DO: Create once in module scope
logger = setup_logger(__name__)

def loop(self, t):
    logger.debug(f"Time: {t}")  # Efficient ✓
```

---

## Troubleshooting

### Common Issues

**Issue: "Connection refused" on bridge.step()**
```
Solution:
1. Verify CoppeliaSim is running
2. Check ZMQ server is listening on port 23001
3. Check firewall settings
4. Verify scene is loaded before bridge.initialize()
```

**Issue: "Timeout: CoppeliaSim did not respond"**
```
Solution:
1. Check if Lua script crashed (check CoppeliaSim console)
2. Verify handshake was called
3. Increase timeout: SimulationBridge(..., timeout=20000)
4. Check if simulation is paused
```

**Issue: Tests fail with "No module named 'brainbyte'"**
```
Solution:
pip install -e .  # Install in editable mode
```

---

## Resources

- [Python PEP 8 Style Guide](https://pep8.org)
- [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html)
- [ZeroMQ Documentation](https://zeromq.org)
- [CBOR Specification](https://cbor.io)
- [CoppeliaSim RemoteAPI](https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm)

---

## Questions?

- Open an issue on GitHub
- Check existing documentation in `docs/`
- Review example projects in `projects/`
- Contact maintainers

Thank you for contributing! 🚀
