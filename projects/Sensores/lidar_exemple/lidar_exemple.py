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
from brainbyte.robots.movel.PioneerBot import PioneerBot

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
        super().__init__(scene_file="labirinto.ttt", 
                         sim_name="lidar_exemple",
                         sim_time=60.0)
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
        
        # Instancia o robô abstraindo handles e cinemática
        self.robot = PioneerBot(
            bridge=self.bridge, 
            robot_name='PioneerP3DX',
            left_motor='leftMotor',
            right_motor='rightMotor'
        )

        # Initialize sensor (don't read data yet, simulation hasn't started)
        # Instancia o sensor (usando o nome dinâmico do robô)
        self.hokuyo_sensor = HokuyoSensorSim(self.bridge, 
                                             f"/{self.robot.robot_name}/fastHokuyo",True)

        self.robot.add_sensor("LIDAR",self.hokuyo_sensor)
        
        # Junta os caminhos de monitoramento do robô e do sensor num só pacote
        monitor_paths = self.robot.get_monitor_paths()
        actuator_paths = self.robot.get_actuator_paths()
        self.bridge.initialize(monitor_paths, actuator_paths, self.sim)

        # TESTE DE DEBUG:
        print("--- CHAVES NO CACHE DA BRIDGE ---")
        # Tenta rodar um step manual para forçar a atualização
        self.bridge.step() 
        print(self.bridge.latest_state.keys())
        print("---------------------------------")

        # Pré-calcular índices do sensor (economiza operações no loop)
        self.sensor_n_points = 684
        self.idx_frente = self.sensor_n_points // 2
        self.idx_esq = (3 * self.sensor_n_points) // 4
        self.idx_dir = self.sensor_n_points // 4
        self.logger.debug(f"Índices do sensor pré-calculados: frente={self.idx_frente}, esq={self.idx_esq}, dir={self.idx_dir}")


    def post_start(self):
        """Executed right after simulation starts."""
        
        # AJUSTE 4: Lê a pose inicial aqui, pois a ponte já terá os dados no cache
        pos = self.robot.pose
        self.logger.info(f'Initial robot position: x={pos[0]:.2f}, y={pos[1]:.2f}')

        if getattr(self, 'auto_diagnostic', False):
            self.logger.info("Running automatic diagnostic: motor pulse.")
            try:
                self.diagnostic_pulse(duration=1.0, speed=0.6)
            except Exception:
                self.logger.exception("Automatic diagnostic failed")

    def diagnostic_pulse(self, duration=1.0, speed=0.6):
        """Send velocity pulse to motors for hardware validation."""
        self.logger.debug(f"Diagnostic pulse: v={speed}, w=0.0, duration={duration}s")
        
        # AJUSTE 5: Usa o tempo embutido no pacote da bridge e avança usando self.bridge.step()
        start = self.bridge.latest_state.get('sim_time', 0.0)
        try:
            self.robot.set_wheel_velocity(speed, 0.0)
            
            while (self.bridge.latest_state.get('sim_time', 0.0) - start) < duration:
                self.bridge.step() # Avança a física e envia os comandos em fila
        finally:
            try:
                self.robot.stop()
                self.bridge.step() # Garante que a fila de "stop" seja enviada pro simulador
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
            laser_data = np.asarray(self.hokuyo_sensor.update())
            
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
        #self.logger.debug(f"[{t:.2f}s] Sensor -> Left: {dist_esq:.2f}m | Front: {dist_frente:.2f}m | Right: {dist_dir:.2f}m")

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

        # Aplica velocidades usando a classe PioneerBot (ela cuida da matemática!)
        try:
            self.robot.set_wheel_velocity(v, w)
            wl, wr = self.robot.wheel_velocities
            #self.logger.debug(f"Velocity command: wl={wl:.3f}, wr={wr:.3f}")
        except Exception:
            self.logger.exception("Failed to apply velocities to motors.")

    def stop(self):
        """Executed after the simulation finishes to ensure safe shutdown."""
        self.logger.info("Simulation stopping. Halting robot...")
        try:
            self.robot.stop()
        except Exception as e:
            self.logger.warning(f"Error while stopping robot: {e}")
def app():
    """Entry point expected by main.py.

    Creates instance and starts execution via BaseApp.run().
    """
    example = LaserVisualizationExample()
    example.run()
