"""CoppeliaSim Framework - Professional Python Framework for Robot Control.

A comprehensive framework for simulating and controlling robots using CoppeliaSim
with a professional logging system, standardized visualization, and best practices.

Key Components:
    - BaseApp: Base class for all simulations
    - setup_logger(): Professional logging system
    - Plot2D/Plot3D: Standardized visualization functions
    
Example:
    from coppelia_sim_framework import BaseApp, setup_logger, Plot2D
    
    class MySimulation(BaseApp):
        def setup(self):
            logger = setup_logger(__name__, '[APP]')
        
        def loop(self, t):
            pass
"""

from brainbyte.core.base_app import BaseApp
from brainbyte.core.logging import setup_logger, ProfessionalFormatter
from brainbyte.utils.plotting import Plot2D, Plot3D

__version__ = "1.1.0"
__all__ = [
    'BaseApp',
    'setup_logger',
    'ProfessionalFormatter',
    'Plot2D',
    'Plot3D',
]
