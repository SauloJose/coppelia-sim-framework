import numpy as np
import matplotlib
# Garante o backend interativo para funcionar os cliques na tela
matplotlib.use('TkAgg') 

import matplotlib.pyplot as plt 
import matplotlib.patches as patches
import os
import platform
import subprocess

from brainbyte import BaseApp
from brainbyte.robots.movel.TurtleBot import *
from brainbyte.control.automatic import *
from brainbyte.sensors.LDS_02 import *
from brainbyte.utils.environment import *

from brainbyte.planner.path.rrt import RRTPlanner

class PathPlanning(BaseApp):
    def __init__(self, show_lidar=False):
        super().__init__(scene_file="mapa.ttt", sim_name="PathPlanning", sim_time=120)
        self.obstacles_data = []
        self.show_lidar = show_lidar
        self.bot_radius = 0.15

        self.target_point = None 
        self.waypoints = []
        self.current_waypoint_idx = 0 

        # Parâmetros do RRT
        self.max_iter = 2000
        self.step_size = 0.5 
        
        self.plot_skip_counter = 0

    def setup(self):
        # =====================================================================
        # ENTRADA DE DADOS DO USUÁRIO PARA O RRT
        # =====================================================================
        self.logger.info("Aguardando configurações do usuário no terminal...")
        print("\n--- CONFIGURAÇÕES DO RRT ---")
        try:
            iter_input = input("Digite o limite de tentativas (max_iter) [Padrão: 2000]: ")
            self.max_iter = int(iter_input) if iter_input.strip() != "" else 2000

            step_input = input("Digite o tamanho do passo (step_size) [Padrão: 0.5]: ")
            self.step_size = float(step_input) if step_input.strip() != "" else 0.5

            b_input = input("Digite a expansão dos objetos (raio do robô) [Padrão: 0.15]: ")
            self.bot_radius = float(b_input) if b_input.strip() != "" else 0.15

        except ValueError:
            self.logger.warning("Entrada inválida detectada! Usando os valores padrão.")
            self.max_iter = 2000
            self.step_size = 0.5
            self.bot_radius = 0.15
            
        print("----------------------------\n")
        self.logger.info(f"Parâmetros RRT -> Max Iter: {self.max_iter}, Step: {self.step_size}")

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
        self.bridge.step()
        
        position = self.robot.pose  
        self.target_point = np.array([position[0], position[1], 0.0])

        self.control = DifferentialController(pos_init=position,
                                              set_point=self.target_point,
                                              k_alpha=1,
                                              k_beta=-0.05,
                                              k_rho=0.15,
                                              dt=self.dt)  

        self.robot.add_control(control_name='AUTO_DIFF', control_instance=self.control)
        
        self.rrt = None
        self.define_plot_configs()

    def on_map_click(self, event):
        """Callback acionado quando o usuário clica no gráfico do Matplotlib"""
        if event.xdata is None or event.ydata is None:
            return
            
        x_clicado = event.xdata
        y_clicado = event.ydata
        
        self.target_point = np.array([x_clicado, y_clicado, 0.0])
        self.plot_target_marker.set_data([x_clicado], [y_clicado])
        
        self.logger.info(f"Novo ponto selecionado (X={x_clicado:.2f}, Y={y_clicado:.2f})... Procurando caminho...")
        
        if self.rrt is not None:
            pos_atual = self.robot.pose
            start_2d = np.array([pos_atual[0], pos_atual[1]])
            goal_2d = np.array([x_clicado, y_clicado])
            
            # Planeja a rota
            caminho_rrt = self.rrt.find_path(start_2d, goal_2d)
            
            # 1. Atualiza o plot da Árvore gerada
            xs, ys = [], []
            if self.rrt.nodes:
                for node in self.rrt.nodes:
                    if node.parent is not None:
                        xs.extend([node.x, node.parent.x, np.nan])
                        ys.extend([node.y, node.parent.y, np.nan])
            self.plot_tree.set_data(xs, ys)

            # 2. Avalia o caminho encontrado
            if caminho_rrt is not None and len(caminho_rrt) > 0:
                self.logger.info("Caminho encontrado!")
                caminho_array = np.array(caminho_rrt)
                
                dist_inicio = np.linalg.norm(caminho_array[0] - start_2d)
                dist_fim = np.linalg.norm(caminho_array[-1] - start_2d)
                if dist_inicio > dist_fim:
                    caminho_array = caminho_array[::-1]
                
                self.waypoints = caminho_array.tolist()
                self.current_waypoint_idx = 0
                
                self.plot_path.set_data(caminho_array[:, 0], caminho_array[:, 1])
                self.plot_wp.set_data(caminho_array[:, 0], caminho_array[:, 1])
            else:
                self.logger.warning("Caminho não encontrado!")
                self.waypoints = []
                self.plot_path.set_data([], [])
                self.plot_wp.set_data([], [])

            # CORREÇÃO: Atualiza a interface sem congelar o código
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    def post_start(self):
        super().post_start()
        self.bridge.step() 

        pos = self.robot.pose
        self.logger.info(f'Initial robot position: x={pos[0]:.2f}, y={pos[1]:.2f}, theta ={np.rad2deg(pos[2]):.2f}')

        self.bot_radius = get_robot_radius(self.sim, 'Turtlebot3/base_link')

        self.plot_robot_body = patches.Circle(
            (pos[0], pos[1]), radius=self.bot_radius,
            edgecolor='r', facecolor='none', linewidth=2, zorder=6
        )
        self.ax.add_patch(self.plot_robot_body)
        self.plot_robot_dir, = self.ax.plot([], [], color='b', linewidth=2, zorder=7)

        self.obstacles_data, self.boundary_vertices, _ = get_environment_obstacles(
            self.sim, robot_radius=self.bot_radius
        )
        
        if self.boundary_vertices and len(self.boundary_vertices) > 0:
            boundary_np = np.array(self.boundary_vertices)
            x_min, y_min = np.min(boundary_np, axis=0)
            x_max, y_max = np.max(boundary_np, axis=0)
        else:
            x_min, x_max, y_min, y_max = -7.0, 7.0, -7.0, 7.0

        self.ax.set_xlim(x_min - 0.5, x_max + 0.5)
        self.ax.set_ylim(y_min - 0.5, y_max + 0.5)

        obstacles_list = []
        for obs in self.obstacles_data:
            v = obs['corners']
            if v is not None and len(v) >= 3:
                obstacles_list.append(v)
                polygon_patch = patches.Polygon(v, closed=True, linewidth=1.5, edgecolor='#2c3e50', facecolor='#7f8c8d', alpha=0.7, zorder=3)
                self.ax.add_patch(polygon_patch)

        self.rrt = RRTPlanner(
            bounds=(x_min, x_max, y_min, y_max), 
            obstacles=obstacles_list,
            step_size=self.step_size,
            max_iter=self.max_iter
        )

        if self.boundary_vertices:
            polygon_boundary = patches.Polygon(np.array(self.boundary_vertices), closed=True, linewidth=4, edgecolor="#000000", facecolor='none', zorder=4)
            self.ax.add_patch(polygon_boundary)
        
        # CORREÇÃO: Inicialização limpa da janela
        plt.show(block=False)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        self.logger.info("Mapa gerado! CLIQUE em um ponto no mapa para o robô navegar.")

    def define_plot_configs(self):
        plt.ion() 
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_aspect('equal')
        
        self.ax.set_title("RRT Interativo: Clique para definir o objetivo", fontsize=13, pad=12)
        self.ax.set_xlabel("X (metros)")
        self.ax.set_ylabel("Y (metros)")
        self.ax.grid(True, linestyle='--', alpha=0.2, zorder=0)

        self.plot_target_marker, = self.ax.plot([], [], 'g.', markersize=14, zorder=8)
        self.plot_tree, = self.ax.plot([], [], color='#bdc3c7', linewidth=0.5, zorder=1)
        self.plot_path, = self.ax.plot([], [], 'c-', linewidth=3.0, zorder=4)
        self.plot_wp, = self.ax.plot([], [], 'yo', markersize=5, zorder=5)

        self.fig.canvas.mpl_connect('button_press_event', self.on_map_click)

    def loop(self, t, actual_state=None):
        try:
            data_sensor = self.robot.get_sensor(sensor_name='LIDAR').update() 
            actual_pos = self.robot.pose 

            if self.waypoints and self.current_waypoint_idx < len(self.waypoints):
                ponto_alvo = self.waypoints[self.current_waypoint_idx]
                self.control.set_point = np.array([ponto_alvo[0], ponto_alvo[1], 0.0])
                
                dist_ao_ponto = np.sqrt((actual_pos[0] - ponto_alvo[0])**2 + (actual_pos[1] - ponto_alvo[1])**2)
                
                if dist_ao_ponto < 0.15:
                    self.current_waypoint_idx += 1
                    
                    if self.current_waypoint_idx >= len(self.waypoints):
                        self.logger.info("O robô completou a rota RRT e chegou ao destino! Aguardando novo clique...")
                        self.waypoints = [] 
                        self.target_point = np.array([actual_pos[0], actual_pos[1], 0.0]) 
            else:
                self.control.set_point = self.target_point

            v_cmd, w_cmd = self.control.get_control(actual_point=actual_pos, dt=self.dt)
            self.robot.set_wheel_velocity(linear_vel=v_cmd, angular_vel=w_cmd)

            # --- Atualiza a posição do Robô na Interface Gráfica ---
            self.plot_skip_counter += 1
            if self.plot_skip_counter % 3 == 0:
                self.plot_robot_body.set_center((actual_pos[0], actual_pos[1]))
                dx = self.bot_radius * np.cos(actual_pos[2]) * 2
                dy = self.bot_radius * np.sin(actual_pos[2]) * 2
                self.plot_robot_dir.set_data([actual_pos[0], actual_pos[0] + dx], [actual_pos[1], actual_pos[1] + dy])
                
                # CORREÇÃO CRÍTICA: Atualiza o plot e processa novos cliques SEM dar pause no código
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events()

        except Exception as e:
            self.logger.error(f"Erro detectado no loop(): {e}")

    def stop(self):
        try:
            self.robot.stop()
            plt.ioff()
            plt.show()
        except Exception as e:
            self.logger.error(f"Erro no stop(): {e}")
    
def app():
    aplicacao = PathPlanning(show_lidar=False)
    aplicacao.run()

if __name__ == '__main__':
    app()