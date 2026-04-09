# Contributing Guidelines

## Code Style

We follow PEP 8 guidelines with a line length limit of 100 characters (configurable in `pyproject.toml`).

### Formatting

Use `black` for automatic formatting:

```bash
black coppelia_sim_framework/
```

### Linting

Use `flake8` to check code quality:

```bash
flake8 coppelia_sim_framework/
```

### Type Hints

Add type hints where beneficial (not strictly enforced):

```python
def process_sensor_data(data: np.ndarray) -> dict:
    """Process LaDAR sensor readings."""
    pass
```

## Logging Standards

All logging should use the professional framework:

```python
from coppelia_sim_framework import setup_logger

logger = setup_logger(__name__, '[APP]')
logger.info("This is an info message")
```

**Never use**:
- ❌ `print()` for application output
- ❌ Emojis or special characters in logs  
- ❌ `logging.basicConfig()`

## Creating New Examples

1. Create file in `examples/` folder
2. Inherit from `BaseApp`
3. Implement required methods
4. Include docstring explaining the example
5. Implement `app()` function for menu integration

## Testing

Run tests with pytest:

```bash
pytest tests/
pytest tests/ --cov=coppelia_sim_framework
```

Write tests for new features:

```python
import unittest
from coppelia_sim_framework import setup_logger

class TestMyFeature(unittest.TestCase):
    def test_something(self):
        logger = setup_logger('test', '[TEST]')
        assert logger is not None
```

## Documentation

Update documentation when adding features:

1. **Docstrings**: Google-style docstrings in code
2. **Examples**: Add usage examples to `examples/`
3**Tutorials**: Add step-by-step guides to `docs/TUTORIALS.md`
4. **README**: Update main README if changing public API

### Docstring Format

```python
def plot_trajectory(data, title=None):
    """Plot robot trajectory in 2D space.
    
    Args:
        data (np.ndarray): Array of shape (N, 2) with [x, y] coordinates
        title (str, optional): Graph title. Defaults to None.
        
    Returns:
        None: Displays matplotlib figure
        
    Raises:
        ValueError: If data shape is invalid
        
    Example:
        >>> data = np.array([[0, 0], [1, 1], [2, 0]])
        >>> plot_trajectory(data, title="My Path")
    """
    pass
```

## Pull Request Process

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Make changes following style guide
4. Add tests for new functionality
5. Run linters: `black`, `flake8`, `mypy`
6. Run tests: `pytest`
7. Commit with clear messages
8. Push to fork and create pull request

## Reporting Issues

Include:
- Description of the problem
- Steps to reproduce
- Expected vs actual behavior
- Environment (Python version, CoppeliaSim version, etc.)
- Error logs/tracebacks

## Questions?

Open an issue or contact the maintainers through the GitHub repository.
