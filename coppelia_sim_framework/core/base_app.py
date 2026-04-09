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
import logging
import keyboard
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from coppelia_sim_framework.core.logging import setup_logger

logger = setup_logger(__name__, '[MAIN]')

class BaseApp:
    """Base class providing the minimal lifecycle for a simulation application.

    Subclasses must override `setup()` and `loop(t)` to implement the test logic.
    """
    def __init__(self, scene_file=None, sim_time=10.0):
        self.scene_file = scene_file
        self.sim_time = sim_time
        
        # Warn the user BEFORE the code potentially hangs
        logger.info("Attempting to connect to the CoppeliaSim engine...")
        logger.info("If the terminal freezes on this message, the simulator is CLOSED. Please open CoppeliaSim!")
        
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
            logger.info("Successfully connected to the simulator!")
            
        except Exception as e:
            logger.error("CONNECTION ERROR: Could not establish communication.")
            logger.error(f"Details: {e}")
            sys.exit(1)

    def run(self):
        """Main method that manages the simulation lifecycle."""
        # Load the scene (if specified)
        if self.scene_file:
            # CoppeliaSim requires ABSOLUTE paths to load scenes
            scene_path = os.path.abspath(f"scenes/{self.scene_file}")
            if not os.path.exists(scene_path):
                raise FileNotFoundError(f"Scene not found: {scene_path}")
            
            logger.info(f"Loading scene: {self.scene_file}...")
            self.sim.loadScene(scene_path)  # Load a .ttt scene file

        # Configure synchronous mode
        self.client.setStepping(True)

        # Execute the test-specific setup (defined by the child class)
        self.setup()

        # Start the simulation
        logger.info("Starting simulation...")
        self.sim.startSimulation()

        # Execute post-initialization (e.g., capturing the first sensor reading)
        self.post_start()

        # Main loop running for the scheduled simulation time
        try:
            while (t := self.sim.getSimulationTime()) < self.sim_time:
                
                # Quick user interruption check.
                # Pressing 's' interrupts the simulation immediately.
                if keyboard.is_pressed('s'):
                    logger.warning(f"Simulation interrupted by the user at time {t:.2f}s.")
                    break
                
                self.loop(t)
                
                # Advance one step in the simulator
                self.client.step()
                
        except KeyboardInterrupt:
            logger.warning("Simulation manually interrupted from the terminal.")
            
        finally:
            # Stop the simulation regardless of error or success
            logger.info("Stopping simulation...")

            self.stop() # Execute custom stop procedures
            self.sim.stopSimulation()

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