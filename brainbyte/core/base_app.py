"""Utilities and base class for applications controlling CoppeliaSim via ZMQ.

`BaseApp` manages the common lifecycle for tests/experiments:
- Load scene (.ttt)
- Configure synchronous mode
- Execute `setup()` (once)
- Iterate `loop(t)` until `sim_time` or interruption
"""

import os
import sys
import time
import tempfile
import subprocess
import platform
import shutil
import socket
import traceback
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from brainbyte.core.paths import *
from brainbyte.sensors import *
from brainbyte.robots import * 
from brainbyte.utils.logging import setup_logger
from brainbyte.utils import *
from brainbyte.core.bridge import SimulationBridge 

# Tenta importar o keyboard de forma segura para Linux/Windows
try:
    import keyboard
    # No Linux, mesmo importando, precisamos testar se temos permissão de acesso
    if platform.system() == 'Linux' and os.getuid() != 0:
        HAS_KEYBOARD = False
    else:
        HAS_KEYBOARD = True
except (ImportError, Exception):
    HAS_KEYBOARD = False
    
class BaseApp:
    """Base class providing the minimal lifecycle for a simulation application.

    Subclasses must override `setup()` and `loop(t)` to implement the test logic.
    """
    def __init__(self, scene_file=None, 
                 sim_name=None, 
                 sim_time=10.0, 
                 log_file=None):
        
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

        self.logger = setup_logger(__name__, '[MAIN]', log_file=LOG_APP_FILE)
        self.log_file = log_file
        
        self.logger.info("Attempting to connect to CoppeliaSim engine...")
        self.logger.info("You have 10 seconds to open CoppeliaSim!")

        if not self._wait_for_simulator(timeout=10.0):
            self.logger.error("TIMEOUT: The simulator failed to open within 10 seconds. Closing.")
            sys.exit(1)

        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.require('sim')

            # Stop the simulation if it is already running
            initial_sim_state = self.sim.getSimulationState()
            if initial_sim_state != 0:
                self.sim.stopSimulation()
                time.sleep(1)

            self.logger.info("Successfully connected to the simulator!")
            
        except Exception as e:
            self.logger.error("CONNECTION ERROR: Could not establish communication.")
            self.logger.exception(f"Error detected in __init__ BaseApp: Details: {e}")
            sys.exit(1)
            
    def _wait_for_simulator(self, host='localhost', port=23000, timeout=10.0):
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                with socket.create_connection((host, port), timeout=0.5):
                    return True
            except (ConnectionRefusedError, socket.timeout, OSError):
                time.sleep(0.5)
        return False
    
    def run(self):
        import signal 

        self.is_running = True

        def handle_sigint(sig, frame):
            self.logger.warning("\n[SIGNAL] Ctrl+C detected. Cleaning up and exiting the loop...")
            self.is_running = False

        # Override the OS-level interrupt signal (SIGINT) with our custom handler
        signal.signal(signal.SIGINT, handle_sigint)
        try:
            # Load scene
            if self.scene_file:
                try:
                    child_module = sys.modules[self.__class__.__module__]
                    base_dir = os.path.dirname(os.path.abspath(child_module.__file__))
                except (KeyError, AttributeError):
                    base_dir = os.getcwd()
    
                # os.path.abspath garante a limpeza de barras redundantes do SO atual
                scene_path = os.path.abspath(os.path.join(base_dir, self.scene_file))
                
                if not os.path.exists(scene_path):
                    raise FileNotFoundError(f"Scene not found: {scene_path}")
                
                self.logger.info(f"Loading scene: {self.scene_file}...")
                
                # --- AJUSTE MULTIPLATAFORMA / DUALBOOT (Z:/) ---
                try:
                    # Tenta carregar o caminho padrão (Funciona nativo no Windows e Linux comum)
                    self.sim.loadScene(scene_path)
                except Exception as e:
                    # Se falhar no Linux, tenta a conversão para o drive virtual Z: (Wine/Partições NTFS)
                    if platform.system() == 'Linux':
                        self.logger.warning("Native Linux path failed in CoppeliaSim. Trying Z:\\ drive mapping...")
                        mapped_path = "Z:" + scene_path.replace("/", "\\")
                        try:
                            self.sim.loadScene(mapped_path)
                            self.logger.info(f"Successfully loaded scene using mapped path: {mapped_path}")
                        except Exception:
                            # Se a gambiarra também falhar, joga o erro original na tela
                            raise e
                    else:
                        raise e
                # -----------------------------------------------

            self.logger.info("Starting simulation...")
            self.sim.startSimulation()
 
            time.sleep(0.5)
            
            for _ in range(3):
                self.sim.step()
                time.sleep(0.05)
            
            self.bridge = SimulationBridge()
            self.setup()
            self.post_start()
            
            t = self.simu_time()
            self.logger.info("Simulation loop started. Press Ctrl+C in terminal to interrupt.")

            if not HAS_KEYBOARD:
                self.logger.warning("Keyboard monitoring disabled (requires sudo on Linux). Use Ctrl+C to stop.")

            # Main loop
            while t < self.sim_time:
                if HAS_KEYBOARD:
                    try:
                        if keyboard.is_pressed('x'):
                            self.logger.warning(f"Simulation interrupted by user at t={t:.2f}s")
                            break
                    except Exception:
                        pass
                
                self.loop(self.simu_time())
                current_state = self.bridge.step()
                t = current_state.get('sim_time', t + 0.05)

        except KeyboardInterrupt:
            self.logger.warning("Simulation manually interrupted from terminal (Ctrl+C).")
        except Exception as e:
            msg = traceback.format_exc()
            self.logger.exception(f"Unexpected error in run() from BaseApp: {e}\n => Traceback: \n\n{msg}")

        finally:    
            self.logger.info("Stopping simulation in finally...")
            try:
                self.logger.info("trying to finish the simulation with the stop() method")
                self.stop()
                if hasattr(self, 'bridge'):
                    self.bridge.close()
                self.sim.stopSimulation()
            except:
                return
    
    def d_time(self):
        return self.sim.getSimulationTimeStep()
    
    @property
    def dt(self):
        return self.d_time()
    
    def simu_time(self):
        return self.sim.getSimulationTime()
    
    @property
    def st(self):
        return self.simu_time()

    # ==========================================
    # METHODS TO BE OVERRIDDEN IN CHILD CLASSES
    # ==========================================
    def setup(self):
        pass

    def post_start(self):
        pass

    def loop(self, t, actual_state=None): 
        pass

    def stop(self):
        pass