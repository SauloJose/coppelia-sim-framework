"""Laser visualization and obstacle evasion example.

This example demonstrates:
1. Real-time laser sensor data reading from Hokuyo sensor
2. Laser data visualization with polar plot
3. Obstacle avoidance with reactive control logic
4. Diagnostic pulse functionality for motor validation
"""

import math
import time
import numpy as np
import matplotlib.pyplot as plt
import traceback
import logging

from brainbyte import BaseApp
from brainbyte.sensors import HokuyoSensorSim


def draw_laser_data(laser_data, max_sensor_range=5, show=False, save_path=None):
    """Plots laser scan data in polar coordinates.

    Args:
        laser_data: Array of [angle, distance] pairs from Hokuyo sensor
        max_sensor_range: Maximum range for axis limit (meters)
        show: If True, attempts to display figure (may fail without GUI)
        save_path: If provided, saves figure to this path
    
    Note:
        - Angles >= 0 are plotted in red
        - Angles < 0 are plotted in blue
        - Sensor origin marked with black triangle
        - By default saves to timestamped file to avoid blocking
    """
    fig = plt.figure(figsize=(6, 6), dpi=100)
    ax = fig.add_subplot(111, aspect='equal')

    if laser_data is None:
        return 

    # Combine angle and distance data for plotting
    for ang, dist in laser_data:
        # Filter out readings that are at the maximum range
        if (max_sensor_range - dist) > 0.1:
            x = dist * np.cos(ang)
            y = dist * np.sin(ang)
            c = 'r' if ang >= 0 else 'b'
            ax.plot(x, y, 'o', color=c)

    # Plot the sensor's origin
    ax.plot(0, 0, 'k>', markersize=10)

    ax.grid(True)
    ax.set_xlim([-max_sensor_range, max_sensor_range])
    ax.set_ylim([-max_sensor_range, max_sensor_range])

    if save_path:
        fig.savefig(save_path)
        plt.close(fig)
        return

    if show:
        try:
            plt.show(block=False)
            plt.pause(0.1)
        except Exception:
            # Environment without GUI
            print("Failed to show figure (no GUI). Saving to 'laser_plot.png'.")
            fig.savefig('laser_plot.png')
            plt.close(fig)
    else:
        # Default: save to a timestamped file to avoid blocking
        timestamp = int(time.time())
        filename = f'laser_plot_{timestamp}.png'
        fig.savefig(filename)
        plt.close(fig)
        print(f"Laser plot saved to: {filename}")


class LaserVisualizationExample(BaseApp):
    """Laser visualization and obstacle avoidance example.

    Demonstrates:
    - Reading Hokuyo LIDAR sensor data in real-time
    - Real-time laser data visualization (polar plot)
    - Reactive obstacle avoidance with distance-based control
    - Motor diagnostic pulse for hardware validation
    
    The robot uses a simple reactive control:
    - If front distance > 0.6m: move forward
    - If front distance <= 0.6m: reverse and turn towards free side
    """

    def __init__(self):
        """Initialize laser visualization example.
        
        Configuration:
        - Scene: labirinto.ttt (labyrinth scene in CoppeliaSim)
        - Duration: 60 seconds
        - Auto-diagnostic: Disabled by default (set True to run motor test)
        """
        self.auto_diagnostic = False
        super().__init__(scene_file="labirinto.ttt", sim_time=60.0)
        self._first_exec = True  # Flag to draw laser plot on first loop

    def setup(self):
        """Configure robot resources before simulation starts.

        This method:
        1. Gets robot and motor handles from CoppeliaSim
        2. Initializes Hokuyo sensor
        3. Logs initial position
        4. Pre-calculates kinematic constants (L, r)
        """
        self.logger.info("Configuring robot for laser visualization...")
        self.robotname = 'PioneerP3DX'

        # Get handles for robot and wheels
        self.robotHandle = self.sim.getObject('/' + self.robotname)
        self.l_wheel = self.sim.getObject('/' + self.robotname + '/leftMotor')
        self.r_wheel = self.sim.getObject('/' + self.robotname + '/rightMotor')

        # Verify handles are valid
        bad = []
        for name, h in (('robot', self.robotHandle), ('leftWheel', self.l_wheel), ('rightWheel', self.r_wheel)):
            if h == -1:
                bad.append(name)

        if bad:
            self.logger.error(f"Invalid handles: {bad}. Verify object names in scene.")
            raise RuntimeError(f"Invalid handles: {bad}")
        else:
            self.logger.debug(f"Handles: robot={self.robotHandle}, left={self.l_wheel}, right={self.r_wheel}")
        
        # Initialize sensor (don't read data yet, simulation hasn't started)
        self.hokuyo_sensor = HokuyoSensorSim(self.sim, "/" + self.robotname + "/fastHokuyo")

        # Get initial robot position
        pos = self.sim.getObjectPosition(self.robotHandle, self.sim.handle_world)
        self.logger.info(f'Initial robot position: {pos}')
        
        # Pioneer P3DX kinematics
        self.L = 0.381   # Wheel base (meters)
        self.r = 0.0975  # Wheel radius (meters)

    def post_start(self):
        """Executed right after simulation starts.

        Useful for diagnostics and sensor initialization after startSimulation().
        If auto_diagnostic=True, runs motor validation pulse.
        """
        if getattr(self, 'auto_diagnostic', False):
            self.logger.info("Running automatic diagnostic: motor pulse.")
            try:
                self.diagnostic_pulse(duration=1.0, speed=0.6)
            except Exception:
                self.logger.exception("Automatic diagnostic failed")

    def diagnostic_pulse(self, duration=1.0, speed=0.6):
        """Send velocity pulse to motors for hardware validation.

        Args:
            duration: Pulse duration (seconds)
            speed: Linear velocity reference (m/s) - converted to wl/wr
        
        This method:
        1. Calculates wheel velocities from linear/angular velocity
        2. Sends pulse via motor commands
        3. Ensures motors stop after pulse completes
        """
        if any(h == -1 for h in (self.l_wheel, self.r_wheel)):
            self.logger.error("Cannot run diagnostic: invalid wheel handles")
            return

        # Calculate wl/wr for v=speed, w=0
        v = float(speed)
        w = 0.0
        wl = v / self.r - (w * self.L) / (2 * self.r)
        wr = v / self.r + (w * self.L) / (2 * self.r)

        self.logger.debug(f"Diagnostic pulse: wl={wl:.3f}, wr={wr:.3f}, duration={duration}s")

        start = self.sim.getSimulationTime()
        try:
            while (self.sim.getSimulationTime() - start) < duration:
                self.sim.setJointTargetVelocity(self.l_wheel, wl)
                self.sim.setJointTargetVelocity(self.r_wheel, wr)
                # Step to advance simulation during pulse
                self.sim.step()

        finally:
            # Ensure motors stop
            try:
                self.sim.setJointTargetVelocity(self.l_wheel, 0)
                self.sim.setJointTargetVelocity(self.r_wheel, 0)
            except Exception:
                self.logger.exception("Failed to zero velocities after diagnostic")
            self.logger.info("Diagnostic complete: pulse finished")

    def loop(self, t):
        """Main control loop executed at each simulation step.

        Args:
            t: Current simulation time (seconds)
        
        This method:
        1. Reads laser data from Hokuyo sensor
        2. Visualizes laser data on first iteration
        3. Extracts distance readings from front/left/right directions
        4. Implements reactive obstacle avoidance
        5. Commands differential drive velocities
        """
        # Read laser data
        try:
            laser_data = np.asarray(self.hokuyo_sensor.getSensorData())
            
            # Plot laser data on first execution
            if self._first_exec is True:
                draw_laser_data(laser_data, 5, True)
            self._first_exec = False 
        except Exception:
            self.logger.exception("Error reading sensor data in loop.")
            return

        n = len(laser_data)
        
        # If camera hasn't rendered yet (first few steps), just wait
        if n == 0: 
            return

        # Calculate indices for front/left/right based on array size (typically 684 points)
        frente = int(n / 2)
        lado_direito = int(n * 1 / 4)
        lado_esquerdo = int(n * 3 / 4)

        # Extract distance readings (column 1 has distances, column 0 has angles)
        dist_frente = laser_data[frente, 1]
        dist_esq = laser_data[lado_esquerdo, 1]
        dist_dir = laser_data[lado_direito, 1]

        # Log sensor readings (DEBUG level to avoid console spam in production)
        self.logger.debug(f"[{t:.2f}s] Sensor -> Left: {dist_esq:.2f}m | Front: {dist_frente:.2f}m | Right: {dist_dir:.2f}m")

        # === OBSTACLE AVOIDANCE LOGIC ===
        v = 0.0
        w = 0.0
        
        # 0.6m considered as danger distance
        if dist_frente > 0.6:
            # Path is clear! Move forward
            v = 0.4
            w = 0.0
        else:
            # Obstacle ahead! Check which side has more space
            v = -1  # Reverse
            if dist_esq > dist_dir:
                # Left is clearer, turn left (positive)
                w = np.deg2rad(40)
            else:
                # Right is clearer, turn right (negative)
                w = np.deg2rad(-40)

        # Pioneer P3DX differential kinematics
        wl = v / self.r - (w * self.L) / (2 * self.r)
        wr = v / self.r + (w * self.L) / (2 * self.r)

        # Send velocity commands to motors
        self.logger.debug(f"Velocity command: wl={wl:.3f}, wr={wr:.3f}")
        try:
            self.sim.setJointTargetVelocity(self.l_wheel, wl)
            self.sim.setJointTargetVelocity(self.r_wheel, wr)
        except Exception:
            self.logger.exception("Failed to apply velocities to motors.")
        

def app():
    """Entry point expected by main.py.

    Creates instance and starts execution via BaseApp.run().
    """
    example = LaserVisualizationExample()
    example.run()
