"""Utilities and base class for applications controlling CoppeliaSim via ZMQ.

`BaseApp` manages the common lifecycle for tests/experiments:
- Load scene (.ttt)
- Configure synchronous mode
- Execute `setup()` (once)
- Iterate `loop(t)` until `sim_time` or interruption

Comments:
- This layer separates the control logic (in tests) from the simulation execution/control
  mechanics, facilitating testing and reuse.
"""

import os
import sys
import time
import tempfile
import subprocess
import platform
import shutil
import keyboard
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from brainbyte.core.paths import *
from brainbyte.sensors import *
from brainbyte.robots import * 
from brainbyte.utils.logging import setup_logger
from brainbyte.utils import *

    
class BaseApp:
    """Base class providing the minimal lifecycle for a simulation application.

    Subclasses must override `setup()` and `loop(t)` to implement the test logic.
    """
    def __init__(self, scene_file=None, sim_time=10.0, log_file =None):
        self.scene_file = scene_file
        self.sim_time = sim_time

        # Configurar arquivo de log
        if log_file is None:
            fd, log_file = tempfile.mkstemp(prefix='sim_log_', suffix='.log')
            os.close(fd)
            self._temp_log_file = log_file
        else:
            self._temp_log_file = None

        # Warn the user BEFORE the code potentially hangs
        self.logger = setup_logger(__name__, '[MAIN]', log_file=LOG_APP_FILE)
        self.log_file = log_file
        
        self.logger.info("Attempting to connect to CoppeliaSim engine...")
        self.logger.info("If the terminal freezes, please open CoppeliaSim!")
        
        try:
            # The code will "freeze" here if the simulator is closed
            self.client = RemoteAPIClient()
            self.sim = self.client.require('sim')

            # Stop the simulation if it is already running
            initial_sim_state = self.sim.getSimulationState()
            if initial_sim_state != 0:
                self.sim.stopSimulation()
                time.sleep(1)

            # If execution reaches this point, the connection was successful!
            self.logger.info("Successfully connected to the simulator!")
            
        except Exception as e:
            self.logger.error("CONNECTION ERROR: Could not establish communication.")
            self.logger.error(f"Details: {e}")
            sys.exit(1)

    def run(self):
        try:
            # Carregar cena
            if self.scene_file:
                scene_path = os.path.abspath(f"scenes/{self.scene_file}")
                if not os.path.exists(scene_path):
                    raise FileNotFoundError(f"Scene not found: {scene_path}")
                self.logger.info(f"Loading scene: {self.scene_file}...")
                self.sim.loadScene(scene_path)
            
            self.client.setStepping(True)
            self.setup()
            self.logger.info("Starting simulation...")
            self.sim.startSimulation()
            self.post_start()
            
            # Loop principal
            while (t := self.sim.getSimulationTime()) < self.sim_time:
                try:
                    if keyboard.is_pressed('s'):
                        self.logger.warning(f"Simulation interrupted by user at t={t:.2f}s")
                        break
                except ImportError as e:
                    # Pega erros caso a pessoa não tenha instalado ou não tenha permissão de root
                    self.logger.error("Erro ao acessar o teclado (requer sudo no Linux/Mac). Pressione Ctrl+C para parar.")
                    
                self.loop(t)
                self.client.step()
                
        except KeyboardInterrupt:
            self.logger.warning("Simulation manually interrupted from terminal.")
        except Exception as e:
            self.logger.error(f"Unexpected error: {e}", exc_info=True)
        finally:
            self.logger.info("Stopping simulation...")
            self.stop()
            self.sim.stopSimulation()
            
            # Limpar arquivo temporário, se usado
            if self._temp_log_file and os.path.exists(self._temp_log_file):
                try:
                    os.remove(self._temp_log_file)
                except OSError:
                    pass
    # ==========================================
    # METHODS TO BE OVERRIDDEN IN CHILD CLASSES
    # ==========================================
    def setup(self):
        """Executed once BEFORE the simulation starts (ideal for fetching handles)."""
        pass

    def post_start(self):
        """Executed right after startSimulation() (ideal for initial sensor capture)."""
        pass

    def loop(self, t): 
        """Executed at each simulation step (ideal for control logic and reading sensors)."""
        pass

    def stop(self):
        """Executed after the simulation finishes."""
        pass