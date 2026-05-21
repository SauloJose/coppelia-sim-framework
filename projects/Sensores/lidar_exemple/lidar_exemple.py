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
from brainbyte.robots.movel.pioneerBot import PioneerBot
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

        self.DIST_SEGURA = 0.3
        self.VEL_LINEAR = 0.8
        self.ANGULO_GIRO = np.deg2rad(45)

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
        """Executado a cada passo da simulação com lógica de evasão."""
        try:
            # 1. Puxar dados do cache da Bridge (Zero-Lag)
            # O .update() retorna o que a Bridge capturou no último sim.step()
            raw_data = self.hokuyo_sensor.update()
            
            if raw_data is None:
                # Se a Bridge ainda não recebeu nada do Coppelia, saímos cedo
                return

            # Converter para Numpy para processamento matemático
            laser_data = np.asarray(raw_data)

            # 2. VALIDAÇÃO CRÍTICA (Evita o IndexError)
            if laser_data.ndim != 2 or laser_data.shape[0] == 0:
                self.logger.warning(f"LIDAR vazio ou malformado. Shape recebido: {laser_data.shape}")
                return # Interrompe a execução deste loop específico

            # 3. DIVISÃO EM SETORES (ZONAS)
            n_points = laser_data.shape[0]
            terco = n_points//3 

            # 4. Extração de distâncias (Coluna 0: Ângulo, Coluna 1: Distância)
            setor_dir = laser_data[0 : terco, 1]
            setor_frente = laser_data[terco : 2 * terco, 1]
            setor_esq = laser_data[2 * terco :, 1]

            # Extrair o menor valor de cada setor, para ter entendimento
            min_dir = np.min(setor_dir)
            min_frente = np.min(setor_frente)
            min_esq = np.min(setor_esq)
            
            self.logger.debug(f"Zonas Mínimas -> Esq: {min_esq:.2f}m | Frente: {min_frente:.2f}m | Dir: {min_dir:.2f}m")

            # 5. LÓGICA DE DECISÃO PROPORCIONAL
            margem_lateral = 1.0 # Distância a partir da qual o robô começa a se afastar das paredes

            if min_frente > self.DIST_SEGURA:
                v = self.VEL_LINEAR

                # Desvio Proporcional suave
                if min_esq < margem_lateral or min_dir < margem_lateral:
                    w = (min_esq - min_dir) * 0.8
                else:
                    w = 0.2 
            else:
                # Obstáculo iminente à frente: Parar ou Recuar e girar forte
                self.logger.info("Obstáculo frontal! Manobra evasiva...")
                v = -0.5 
                
                # Gira no próprio eixo em direção ao lado mais livre
                if min_esq > min_dir:
                    w = self.ANGULO_GIRO * 0.5 # Gira esquerda
                else:
                    w = -self.ANGULO_GIRO * 0.5 # Gira direita

            # 6. ENVIAR COMANDOS (Enfileira no buffer da Bridge)
            self.robot.set_wheel_velocity(linear_vel=v, angular_vel=w)

        except Exception as e:
            self.logger.error(f"Falha catastrófica no loop de controle: {e}")
            # Opcional: self.stop_simulation() se o erro for persistente

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
