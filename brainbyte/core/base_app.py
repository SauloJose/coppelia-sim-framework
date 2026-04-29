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
import socket
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
        self.logger.info("You have 10 seconds to open CoppeliaSim!")

        if not self._wait_for_simulator(timeout=10.0):
            self.logger.error("TIMEOUT: The simulator failed to open within 10 seconds. Closing.")
            sys.exit(1)

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
            
    def _wait_for_simulator(self, host='localhost', port=23000, timeout=10.0):
        """
        Tenta estabelecer uma conexão TCP simples para verificar se o simulador está online.
        Pinga a porta a cada 0.5 segundos até o tempo limite acabar.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                # Tenta criar uma conexão. O timeout de 0.5 impede que fique preso.
                with socket.create_connection((host, port), timeout=0.5):
                    return True
            except (ConnectionRefusedError, socket.timeout, OSError):
                # Se a porta estiver fechada, espera meio segundo e tenta de novo
                time.sleep(0.5)
        
        return False
    
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
                self.sim.loadScene(scene_path) # load the Scene in the Coppelia
            

            self.logger.info("Starting simulation...")
            self.sim.startSimulation() #Initialize the simulation
 
            time.sleep(0.5)
            
            for _ in range(3):
                self.sim.step()           # make te Coppelia advance one step, callin the system_sensing()
                time.sleep(0.05)
            
            # NECESSARY: Make the bridge to communicate with the Coppelia via ZeroMQ using cbor2
            self.bridge = SimulationBridge()

            # NECESSARY: Setup my simulation
            self.setup()
            
            # NECESSARY: post_start logic to init my configurations
            self.post_start()
            
            current_state = self.bridge.step()
            t = current_state.get('sim_time', 0.0)

            # Main loop of the simulation. Here we can add the logic from loop and the communication with the bridge.
            while t  < self.sim_time:
                try:
                    if keyboard.is_pressed('x'):
                        self.logger.warning(f"Simulation interrupted by user at t={t:.2f}s")
                        break
                except ImportError as e:
                    # Catch errors if the user lacks the module or root privileges
                    self.logger.error("Error detected on Keyboard input (request sudo in Linux/Mac). Press Ctrl+C to stop.")
                
                # IMPORTANT: Here is the logic of my simulation! The child-class have to make this.
                self.loop(self.simu_time())

                # IMPORTANT: Here is a very useful thing. Using self.bridge.step() we call the ZeroMQ API to communicate the 
                # state of the simulation, so, this is VERY IMPORTANT AND NECESSARY to mantain here!
                self.bridge.step()

                # att the time
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
                    self.bridge.close() # stop de bridge connection
                self.sim.stopSimulation()
            except:
                return 
    
    # Fetch standard information 
    def d_time(self):
        """
        Returns the simulation time step.
        """
        return self.sim.getSimulationTimeStep()
    
    @property
    def dt(self):
        """
        Propriedade que chama dinamicamente self.d_time().
        Permite acessar o tempo de simulação usando apenas self.dt nas classes filhas.
        """
        return self.d_time()
    
    def simu_time(self):
        """
        Returns the current simulation time.
        """
        return self.sim.getSimulationTime()
    
    @property
    def st(self):
        """
        Propriedade que chama dinamicamente self.d_time().
        Permite acessar o tempo de simulação usando apenas self.dt nas classes filhas.
        """
        return self.simu_time()
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