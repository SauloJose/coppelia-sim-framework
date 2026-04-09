"""Core module for CoppeliaSim Framework."""

from coppelia_sim_framework.core.base_app import BaseApp
from coppelia_sim_framework.core.logging import setup_logger, ProfessionalFormatter

__all__ = ['BaseApp', 'setup_logger', 'ProfessionalFormatter']
