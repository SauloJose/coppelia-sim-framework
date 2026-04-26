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
from brainbyte.core.bridge import SimulationBridge 
import traceback
    
class BaseApp:
    """Base class providing the minimal lifecycle for a simulation application.

    Subclasses must override `setup()` and `loop(t)` to implement the test logic.
    """
    def __init__(self, scene_file=None, 
                 sim_name=None, 
                 sim_time=10.0, 
                 log_file =None):
        
        self.sim_name = sim_name
        self.scene_file = scene_file
        self.sim_time = sim_time

        # Configure log file
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
            self.logger.exception(f"Error detected in __init__ BaseApp: Details: {e}")
            sys.exit(1)

    def run(self):
        try:
            # Load scene
            if self.scene_file:
                # Find the absolute path of the script that called BaseApp (e.g., your_app.py)
                try:
                    child_module = sys.modules[self.__class__.__module__]
                    base_dir = os.path.dirname(os.path.abspath(child_module.__file__))
                except (KeyError, AttributeError):
                    # Safety fallback
                    base_dir = os.getcwd()

                scene_path = os.path.join(base_dir, self.scene_file)

                if not os.path.exists(scene_path):
                    raise FileNotFoundError(f"Scene not found: {scene_path}")
                self.logger.info(f"Loading scene: {self.scene_file}...")
                self.sim.loadScene(scene_path)
            

            self.logger.info("Starting simulation...")
            self.sim.startSimulation()

            time.sleep(0.5)
            
            # Dentro de BaseApp.run(), depois de self.sim.startSimulation() e time.sleep(0.5)
            self.logger.info("Acordando o servidor Lua com um passo vazio...")
            for _ in range(3):
                self.sim.step()           # faz o Lua executar sysCall_sensing()
                time.sleep(0.05)
            
            self.bridge = SimulationBridge()
            self.setup()


            
            self.post_start()
            
            current_state = self.bridge.step()
            t = current_state.get('sim_time', 0.0)

            # Main loop
            while t  < self.sim_time:
                try:
                    if keyboard.is_pressed('x'):
                        self.logger.warning(f"Simulation interrupted by user at t={t:.2f}s")
                        break
                except ImportError as e:
                    # Catch errors if the user lacks the module or root privileges
                    self.logger.error("Error detected on Keyboard input (request sudo in Linux/Mac). Press Ctrl+C to stop.")
                
                self.loop(t)

                self.bridge.step()

                # Atualizamos o relógio com o tempo novo que veio do Lua
                t = current_state.get('sim_time', t + 0.05)

        except KeyboardInterrupt:
            self.logger.warning("Simulation manually interrupted from terminal.")
        except Exception as e:
            msg = traceback.format_exc()
            self.logger.exception(f"Unexpected error in run() from BaseApp: {e}\n => Traceback: \n\n{msg}")

        finally:    
            self.logger.info("Stopping simulation in finally...")
            try:
                self.stop()
                if hasattr(self, 'bridge'):
                    self.bridge.close() # Fecha a nossa porta serial adequadamente
                self.sim.stopSimulation()
            except:
                return 
    
    # Fetch standard information 
    def d_time(self):
        """
        Returns the simulation time step.
        """
        return self.sim.getSimulationTimeStep()
    
    def simu_time(self):
        """
        Returns the current simulation time.
        """
        return self.sim.getSimulationTime()
    
    # ==========================================
    # METHODS TO BE OVERRIDDEN IN CHILD CLASSES
    # ==========================================
    def setup(self):
        """Executed once BEFORE the simulation starts (ideal for fetching handles)."""
        pass

    def post_start(self):
        """Executed right after startSimulation() (ideal for initial sensor capture)."""
        pass

    def loop(self, t, actual_state = None): 
        """Executed at each simulation step (ideal for control logic and reading sensors)."""
        pass

    def stop(self):
        """Executed after the simulation finishes."""
        pass