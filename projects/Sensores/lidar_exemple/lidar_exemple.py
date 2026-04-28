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
import os 

plt.ion()

def draw_laser_data(laser_data, max_sensor_range=5, show=False, save_path=None):
    """Plots laser scan data in polar coordinates de forma eficiente."""
    
    if laser_data is None or len(laser_data) == 0:
        return 

    # Criamos a figura
    fig = plt.figure(figsize=(6, 6), dpi=100)
    ax = fig.add_subplot(111, aspect='equal')

    # Convertendo para coordenadas cartesianas de forma vetorizada (muito mais rápido)
    angles = laser_data[:, 0]
    distances = laser_data[:, 1]
    
    # Filtro de alcance
    mask_range = (max_sensor_range - distances) > 0.1
    
    # Máscaras para cores (Red para >= 0, Blue para < 0)
    mask_red = (angles >= 0) & mask_range
    mask_blue = (angles < 0) & mask_range

    # Plotando os pontos em blocos
    ax.plot(distances[mask_red] * np.cos(angles[mask_red]), 
            distances[mask_red] * np.sin(angles[mask_red]), 'ro', markersize=2, label='Esquerda/Frente')
    ax.plot(distances[mask_blue] * np.cos(angles[mask_blue]), 
            distances[mask_blue] * np.sin(angles[mask_blue]), 'bo', markersize=2, label='Direita')

    # Origem do robô
    ax.plot(0, 0, 'k>', markersize=10)

    ax.grid(True)
    ax.set_xlim([-max_sensor_range, max_sensor_range])
    ax.set_ylim([-max_sensor_range, max_sensor_range])
    ax.set_title(f"Lidar Scan - {time.strftime('%H:%M:%S')}")

    # Lógica de exibição/salvamento
    if save_path:
        # Garante que a pasta existe
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        fig.savefig(save_path)
        plt.close(fig)
    elif show:
        try:
            plt.show(block=False)
            plt.pause(0.5) # Tempo para o SO renderizar a janela
            # Se você quiser que a janela feche após mostrar, use plt.close(fig) aqui
        except Exception:
            print("Ambiente sem interface gráfica. Salvando em 'laser_plot.png'...")
            fig.savefig('laser_plot.png')
            plt.close(fig)
    else:
        timestamp = int(time.time())
        filename = f'laser_plot_{timestamp}.png'
        fig.savefig(filename)
        plt.close(fig)
        print(f"Laser plot salvo como: {filename}")



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

        # If camera hasn't rendered yet (first few steps), just wait
        if len(laser_data) == 0: 
            return

        # Calculate indices for front/left/right based on array size (typically 684 points)
        dist_frente = laser_data[self.idx_frente, 1]
        dist_esq = laser_data[self.idx_esq, 1]
        dist_dir = laser_data[self.idx_dir, 1]

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
            plt.close('all')
        except Exception as e:
            self.logger.warning(f"Error while stopping robot: {e}")
def app():
    """Entry point expected by main.py.

    Creates instance and starts execution via BaseApp.run().
    """
    example = LaserVisualizationExample()
    example.run()
