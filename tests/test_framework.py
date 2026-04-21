"""Unit tests for CoppeliaSim Framework core module."""

import unittest
from unittest.mock import Mock, patch, MagicMock
import numpy as np

# Por enquanto, vamos criar testes básicos
# Estes serão expandidos conforme o framework evoluir


class TestBaseAppInitialization(unittest.TestCase):
    """Tests for BaseApp initialization and connection handling."""
    
    @patch('coppelia_sim_framework.core.base_app.RemoteAPIClient')
    def test_successful_connection(self, mock_client):
        """Test successful connection to CoppeliaSim."""
        from brainbyte import BaseApp
        
        mock_instance = MagicMock()
        mock_client.return_value = mock_instance
        mock_instance.require.return_value.getSimulationState.return_value = 0
        
        # Should not raise an exception
        app = BaseApp(scene_file="test.ttt", sim_time=10.0)
        assert app.scene_file == "test.ttt"
        assert app.sim_time == 10.0
    
    def test_base_app_methods_exist(self):
        """Test that BaseApp has required methods."""
        from brainbyte import BaseApp
        
        required_methods = ['setup', 'loop', 'stop', 'post_start', 'run']
        for method in required_methods:
            assert hasattr(BaseApp, method), f"BaseApp missing method: {method}"


class TestLogging(unittest.TestCase):
    """Tests for professional logging system."""
    
    def test_setup_logger_returns_logger(self):
        """Test that setup_logger returns a logger instance."""
        from brainbyte import setup_logger
        import logging
        
        logger = setup_logger(__name__, '[TEST]')
        assert isinstance(logger, logging.Logger)
    
    def test_professional_formatter(self):
        """Test that ProfessionalFormatter formats logs correctly."""
        from brainbyte import ProfessionalFormatter
        import logging
        
        formatter = ProfessionalFormatter('[TEST]')
        
        # Create a log record
        record = logging.LogRecord(
            name='test',
            level=logging.INFO,
            pathname='test.py',
            lineno=10,
            msg='Test message',
            args=(),
            exc_info=None
        )
        
        formatted = formatter.format(record)
        assert '[INFO]' in formatted
        assert '[TEST]' in formatted
        assert 'Test message' in formatted


class TestPlotting(unittest.TestCase):
    """Tests for visualization functions."""
    
    @patch('matplotlib.pyplot.show')
    def test_plot2d_creates_figure(self, mock_show):
        """Test that Plot2D creates and configures a figure."""
        from brainbyte import Plot2D
        
        data = np.array([[0, 0], [1, 1], [2, 0]])
        
        # Should not raise an exception
        Plot2D(data, 'X', 'Y', title='Test')
        mock_show.assert_called_once()
    
    @patch('matplotlib.pyplot.show')
    def test_plot3d_creates_figure(self, mock_show):
        """Test that Plot3D creates and configures a figure."""
        from brainbyte import Plot3D
        
        data = np.array([[0, 0, 0], [1, 1, 1], [2, 0, 1]])
        
        # Should not raise an exception
        Plot3D(data, 'X', 'Y', 'Z', title='Test 3D')
        mock_show.assert_called_once()


if __name__ == '__main__':
    unittest.main()
