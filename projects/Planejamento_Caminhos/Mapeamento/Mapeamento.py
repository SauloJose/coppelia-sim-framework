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

    def post_start(self):
        super().post_start()
        
        pos = self.robot.pose
        self.logger.info(f'Initial robot position: x={pos[0]:.2f}, y={pos[1]:.2f}')

        # Chamada da função externa passando a API do CoppeliaSim
        self.obstacles_data = get_environment_obstacles(self.sim)
        
        # Renderização estática do mapa de obstáculos
        for obs in self.obstacles_data:
            rect = patches.Rectangle(
                (-obs['w'] / 2, -obs['h'] / 2), 
                obs['w'], 
                obs['h'], 
                linewidth=1.5, 
                edgecolor='#333333', 
                facecolor='#999999',
                alpha=0.8,
                zorder=2
            )
            t = transforms.Affine2D().rotate_deg(obs['angle']).translate(obs['x'], obs['y']) + self.ax.transData
            rect.set_transform(t)
            self.ax.add_patch(rect)

    def define_plot_configs(self):
        plt.ion() 
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-7, 7)
        self.ax.set_ylim(-7, 7)
        
        # Melhorias visuais no plot
        self.ax.set_title("Mapa de Planejamento de Caminhos", fontsize=14, pad=15)
        self.ax.set_xlabel("X (metros)")
        self.ax.set_ylabel("Y (metros)")
        self.ax.grid(True, linestyle='--', alpha=0.5, zorder=0)

        # Marcadores do Robô
        self.plot_robot, = self.ax.plot([], [], 'ro', markersize=8, label='Robô', zorder=5)
        
        # Sempre cria o Lidar na memória (vazio por enquanto) para evitar erros no set_data futuramente
        self.plot_lidar, = self.ax.plot([], [], 'c.', markersize=2, alpha=0.6, label='Lidar', zorder=3)
            
        self.ax.legend(loc='upper right')

    def loop(self, t, actual_state=None):
        try:
            data_sensor = self.robot.get_sensor(sensor_name='LIDAR').update() 
            
            # Passa a flag global do construtor para o método do plot
            self.plot_result(ds=data_sensor, robot=self.robot, plot_lidar=self.show_lidar)
            
        except Exception as e:
            self.logger.error(f"Erro detectado in loop(): {e}")
    
    # Adicionamos a opção diretamente no input do método
    def plot_result(self, ds, robot, plot_lidar=True):
        
        # Se plot_lidar for True e houverem dados, atualiza os pontos
        if plot_lidar and ds is not None and len(ds) > 0:
            self.plot_lidar.set_data(ds[:, 0], ds[:, 1])
        # Se plot_lidar for False, zera os dados para limpar a tela
        else:
            self.plot_lidar.set_data([], [])
        
        # Atualiza a posição do robô
        pos = robot.pose
        self.plot_robot.set_data([pos[0]], [pos[1]])
        
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