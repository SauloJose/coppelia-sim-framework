import numpy as np
import matplotlib
matplotlib.use('Agg') # Força o Matplotlib a trabalhar em background sem abrir janelas
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

from brainbyte.planner.path.prm import *

class PathPlanning(BaseApp):
    def __init__(self, show_lidar=False):
        super().__init__(scene_file="mapa.ttt", sim_name="PathPlanning", sim_time=120)
        self.obstacles_data = []
        self.show_lidar = show_lidar
        self.bot_radius = 0.15

        # Valores iniciais padrão (serão sobrescritos no setup)
        self.target_point = np.array([4.0, 4.0, 0.0]) 
        self.waypoints = []
        self.current_waypoint_idx = 0 

        self.N = 1000
        self.K = 10 
        self.filename_img = ""

    def abrir_imagem(self):
        """Abre a imagem renderizada usando o visualizador padrão do Sistema Operacional"""
        try:
            if platform.system() == 'Windows':
                os.startfile(self.filename_img)
            elif platform.system() == 'Darwin': # macOS
                subprocess.call(('open', self.filename_img))
            else: # Linux
                subprocess.call(('xdg-open', self.filename_img))
        except Exception as e:
            self.logger.error(f"Aviso: Não foi possível abrir a imagem automaticamente: {e}")

    def setup(self):
        # =====================================================================
        # ENTRADA DE DADOS DO USUÁRIO
        # =====================================================================
        self.logger.info("Aguardando configurações do usuário no terminal...")
        print("\n--- CONFIGURAÇÕES DO PRM ---")
        try:
            n_input = input("Digite o número de pontos (N) [Padrão: 1000]: ")
            self.N = int(n_input) if n_input.strip() != "" else 1000

            k_input = input("Digite o número de vizinhos (K) [Padrão: 10]: ")
            self.K = int(k_input) if k_input.strip() != "" else 10

            x_input = input("Digite a coordenada X do objetivo [Padrão: 0.0]: ")
            tx = float(x_input) if x_input.strip() != "" else 0.0

            y_input = input("Digite a coordenada Y do objetivo [Padrão: 0.0]: ")
            ty = float(y_input) if y_input.strip() != "" else 0.0

            b_input = input("Digite a expansão dos objetos (raio do robô) [Padrão: 0.15]: ")
            self.bot_radius = float(b_input) if b_input.strip() != "" else 0.15

            self.target_point = np.array([tx, ty, 0.0])

        except ValueError:
            self.logger.warning("Entrada inválida detectada! Usando os valores padrão.")
            self.N = 1000
            self.K = 10
            self.target_point = np.array([4.0, 4.0, 0.0])
            self.bot_radius = 0.15
            
        print("----------------------------\n")
        self.logger.info(f"Parâmetros definidos -> N: {self.N}, K: {self.K}, Objetivo: X={self.target_point[0]}, Y={self.target_point[1]}")

        # =====================================================================
        # CONTINUAÇÃO DO SETUP
        # =====================================================================
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

        self.control = DifferentialController(pos_init=position,
                                              set_point=self.target_point,
                                              k_alpha=1,
                                              k_beta=-0.05,
                                              k_rho=0.15,
                                              dt=self.dt)  

        self.robot.add_control(control_name='AUTO_DIFF', control_instance=self.control)
        
        self.prm = None
        self.define_plot_configs()

    def post_start(self):
        super().post_start()
        self.bridge.step() 

        pos = self.robot.pose
        self.logger.info(f'Initial robot position: x={pos[0]:.2f}, y={pos[1]:.2f}, theta ={np.rad2deg(pos[2]):.2f}')

        self.bot_radius = get_robot_radius(self.sim, 'Turtlebot3/base_link')

        # Plota a posição inicial do robô (estático)
        plot_robot_body = patches.Circle(
            (pos[0], pos[1]), radius=self.bot_radius,
            edgecolor='r', facecolor='none', linewidth=2, label='Posição Inicial', zorder=5
        )
        self.ax.add_patch(plot_robot_body)
        
        dx = self.bot_radius * np.cos(pos[2])*2
        dy = self.bot_radius * np.sin(pos[2])*2
        self.ax.plot([pos[0], pos[0]+dx], [pos[1], pos[1]+dy], color='b', linewidth=2, zorder=6)

        # 1. Obtendo dados do ambiente
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

        # 2. Convertendo os obstáculos para a lista esperada pelo PRM e Plotando
        obstacles_list = []
        for obs in self.obstacles_data:
            v = obs['corners']
            if v is not None and len(v) >= 3:
                obstacles_list.append(v)
                polygon_patch = patches.Polygon(v, closed=True, linewidth=1.5, edgecolor='#2c3e50', facecolor='#7f8c8d', alpha=0.7, label='Obstáculo', zorder=3)
                self.ax.add_patch(polygon_patch)

        # 3. Inicializando o PRM
        self.prm = PRMPlanner(
            num_samples=self.N, 
            k_neighbors=self.K, 
            bounds=(x_min, x_max, y_min, y_max), 
            obstacles=obstacles_list
        )
        
        start_2d = np.array([pos[0], pos[1]])
        goal_2d = np.array([self.target_point[0], self.target_point[1]])

        self.logger.info("Mapeando rotas pelo espaço livre (PRM)...")
        self.prm.build_roadmap(start=start_2d, goal=goal_2d)

        self.logger.info("Calculando o melhor caminho com A*...")
        caminho_prm = self.prm.find_path()

        if caminho_prm is not None and len(caminho_prm) > 0:
            caminho_array = np.array(caminho_prm)

            dist_inicio = np.linalg.norm(caminho_array[0] - start_2d)
            dist_fim = np.linalg.norm(caminho_array[-1] - start_2d)
            
            if dist_inicio > dist_fim:
                caminho_array = caminho_array[::-1]
            
            self.waypoints = caminho_array.tolist()
            self.current_waypoint_idx = 0

            # Plota o caminho selecionado
            self.ax.plot(caminho_array[:, 0], caminho_array[:, 1], 'c-', linewidth=3.0, label='Caminho Selecionado', zorder=4)
            self.ax.plot(caminho_array[:, 0], caminho_array[:, 1], 'yo', markersize=5, label='Waypoints', zorder=5)
        else:
            self.waypoints = []

        # 4. Plotagem dos nós (Pontos) e arestas (Caminhos do PRM)
        if len(self.prm.nodes) > 0:
            self.ax.plot(self.prm.nodes[:, 0], self.prm.nodes[:, 1], '.', color='#bdc3c7', markersize=4, label='Nós PRM', zorder=2)
            for i, vizinhos in self.prm.graph.items():
                p1 = self.prm.nodes[i]
                for j, _ in vizinhos:
                    p2 = self.prm.nodes[j]
                    self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='#e2e2e2', linewidth=0.6, zorder=1)

        # Plot das bordas do cenário
        if self.boundary_vertices:
            polygon_boundary = patches.Polygon(np.array(self.boundary_vertices), closed=True, linewidth=4, edgecolor="#000000", facecolor='none', zorder=4)
            self.ax.add_patch(polygon_boundary)
        
        # Salva a imagem gerada na pasta do projeto e fecha a figura para liberar memória
        diretorio_modulo = os.path.dirname(os.path.abspath(__file__))
        self.filename_img = os.path.join(diretorio_modulo, 'caminhoPRM.png')
        self.fig.savefig(self.filename_img, dpi=300, bbox_inches='tight')
        plt.close(self.fig)

        # =====================================================================
        # DECISÃO PÓS-PLANEJAMENTO
        # =====================================================================
        if not self.waypoints:
            self.logger.error("Simulação abortada: O PRM não encontrou conexão até o alvo.")
            self.logger.info("Exibindo os pontos gerados...")
            self.abrir_imagem()
            self.stop()
            os._exit(0) # Força o fim da simulação se não tiver caminho
        else:
            self.logger.info(f"Caminho do PRM carregado com {len(self.waypoints)} waypoints brutos.")
            self.logger.info("Caminho encontrado. Iniciando trajetória.")
            self.abrir_imagem()

    def define_plot_configs(self):
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_aspect('equal')
        
        self.ax.set_title("Planejamento com PRM e Busca A* (Mapa Estático)", fontsize=13, pad=12)
        self.ax.set_xlabel("X (metros)")
        self.ax.set_ylabel("Y (metros)")
        self.ax.grid(True, linestyle='--', alpha=0.2, zorder=0)

        # Plota o objetivo final
        self.ax.plot([self.target_point[0]], [self.target_point[1]], 'g.', markersize=12, label='Objetivo', zorder=7)

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
                    
                    # QUANDO CHEGAR NO ÚLTIMO PONTO:
                    if self.current_waypoint_idx >= len(self.waypoints):
                        self.logger.info("O robô completou a rota PRM e chegou ao destino!")
                        self.robot.set_wheel_velocity(linear_vel=0.0, angular_vel=0.0) # Freia o robô
                        self.logger.info("finalizando simulação...")
                        self.stop()
            else:
                self.control.set_point = self.target_point

            # Executa o controlador (apenas se a simulação ainda estiver rodando)
            v_cmd, w_cmd = self.control.get_control(actual_point=actual_pos, dt=self.dt)
            
            self.robot.set_wheel_velocity(linear_vel=v_cmd, angular_vel=w_cmd)

        except Exception as e:
            self.logger.error(f"Erro detectado no loop(): {e}")

    def stop(self):
        try:
            self.robot.stop()
        except Exception as e:
            self.logger.error(f"Erro no stop(): {e}")
    
def app():
    aplicacao = PathPlanning(show_lidar=False)
    aplicacao.run()

if __name__ == '__main__':
    app()