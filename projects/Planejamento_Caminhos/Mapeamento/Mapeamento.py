import numpy as np
import matplotlib.pyplot as plt 
import matplotlib.patches as patches
import matplotlib.transforms as transforms

from brainbyte import BaseApp
from brainbyte.robots.movel.TurtleBot import *
from brainbyte.control.automatic import * 
from brainbyte.sensors.LDS_02 import *
from brainbyte.gui.auxF import get_key
from brainbyte.control.manual import *
from brainbyte.utils.environment import *

class PathPlanning(BaseApp):
    def __init__(self, show_lidar=True):
        super().__init__(scene_file="mapa.ttt", sim_name="PathPlanning", sim_time=120)
        self.obstacles_data = []
        
        # Flag global para habilitar ou desabilitar a visão do Lidar
        self.show_lidar = show_lidar

    def setup(self):
        self.logger.info("Configuring Robot, Sensor and Controllers..")

        self.robot = TurtleBot(
            bridge=self.bridge,
            robot_name='Turtlebot3', 
            left_motor='left_motor', 
            right_motor='right_motor',
            base_link='base_link'
        )
        
        self.Lidar = LDS_02(bridge=self.bridge, base_name='Turtlebot3')
        self.robot.add_sensor(sensor_name='LIDAR', sensor_instance=self.Lidar)

        monitor_paths = self.robot.get_monitor_paths()
        actuator_paths = self.robot.get_actuator_paths()
        self.bridge.initialize(monitor_paths, actuator_paths, self.sim)

        self.buffer = PointCloudAccumulator(max_point=100000)
        self.define_plot_configs()

        self.show_lidar = False
        
        # Implementar um conjunto de perguntas para configurar a simulação aqui no Começo
        self.command_lines()

    def command_lines(self):
        self.logger.warning("Aqui ainda será implementado uma lógica para entrar com variáveis para a simulação")

    def post_start(self):
        super().post_start()
        
        pos = self.robot.pose
        self.logger.info(f'Initial robot position: x={pos[0]:.2f}, y={pos[1]:.2f}')

        # 1. Puxa o raio dinamicamente usando a função externa corrigida para V4.1
        self.bot_radius = get_robot_radius(self.sim, 'Turtlebot3/base_link')
        self.bot_radius += 0.03  # Adiciona a folga desejada

        # Desenha o robô
        self.plot_robot_body = patches.Circle(
            (pos[0], pos[1]), radius=self.bot_radius,
            edgecolor='r', facecolor='none', linewidth=2, label='Contorno do Robô', zorder=5
        )
        self.ax.add_patch(self.plot_robot_body)
        self.ax.legend(loc='upper right')

        # 2. CHAMADA REUTILIZÁVEL: Passamos o raio do robô direto como argumento!
        # A função externa agora se encarrega de calcular as somas de Minkowski automaticamente.
        self.obstacles_data = get_environment_obstacles(self.sim, robot_radius=self.bot_radius)
        
        # 3. Renderização simples na tela
        for obs in self.obstacles_data:
            # Desenha a zona de colisão INFLADA (C-Space) em vermelho bem suave
            polygon_inflated = patches.Polygon(
                obs['corners'], 
                closed=True,
                linewidth=1.2, 
                edgecolor='red', 
                facecolor='red',
                alpha=0.15,
                linestyle='--',
                zorder=2
            )
            self.ax.add_patch(polygon_inflated)

            # Desenha o obstáculo REAL/FÍSICO por cima em cinza escuro
            polygon_real = patches.Polygon(
                obs['corners_originals'], 
                closed=True,
                linewidth=1.5, 
                edgecolor='#333333', 
                facecolor='#666666',
                alpha=0.9,
                zorder=3
            )
            self.ax.add_patch(polygon_real)
    def define_plot_configs(self):
        plt.ion() 
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-7, 7)
        self.ax.set_ylim(-7, 7)
        
        self.ax.set_title("Mapa de Planejamento de Caminhos", fontsize=14, pad=15)
        self.ax.set_xlabel("X (metros)")
        self.ax.set_ylabel("Y (metros)")
        self.ax.grid(True, linestyle='--', alpha=0.5, zorder=0)

        # Mantemos o ponto central apenas para destacar o centro do robô
        self.plot_robot_center, = self.ax.plot([], [], 'ro', markersize=4, zorder=6)
        
        # Plot do Lidar para debug
        self.plot_lidar, = self.ax.plot([], [], 'r.', markersize=2, alpha=0.6, label='Lidar', zorder=3)
            
        self.ax.legend(loc='upper right')

    def loop(self, t, actual_state=None):
        try:
            # Captura a nuvem de pontos atual do sensor (Frame atual)
            data_sensor = self.robot.get_sensor(sensor_name='LIDAR').update() 
            
            if data_sensor is not None and data_sensor.size > 0:
                self.buffer.add(data_sensor)
            
            # Recupera todos os pontos limpos acumulados no Voxel Grid até agora
            accumulated_points = self.buffer.get_all()
            
            # Passa os pontos acumulados para a renderização em tela
            self.plot_result(ds=accumulated_points, robot=self.robot, plot_lidar=self.show_lidar)
            
        except Exception as e:
            self.logger.error(f"Erro detectado in loop(): {e}")
    
    def plot_result(self, ds, robot, plot_lidar=True):
        # Se plot_lidar for True e houverem dados, atualiza os pontos
        if plot_lidar and ds is not None and len(ds) > 0:
            self.plot_lidar.set_data(ds[:, 0], ds[:, 1])
        else:
            self.plot_lidar.set_data([], [])
        
        # Captura a posição real atual do robô
        pos = robot.pose
        
        # Move o círculo vermelho para a nova coordenada central (x, y)
        self.plot_robot_body.set_center((pos[0], pos[1]))
        
        # Atualiza o ponto central discretamente
        self.plot_robot_center.set_data([pos[0]], [pos[1]])
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def stop(self):
        try:
            self.robot.stop()
            plt.ioff()
            plt.close(self.fig)
        except Exception as e:
            self.logger.error(f"Erro detectado in stop(): {e}")
    
def app():
    # Mude para show_lidar=False para ocultar os pontos do Lidar
    aplicacao = PathPlanning(show_lidar=True)
    aplicacao.run()