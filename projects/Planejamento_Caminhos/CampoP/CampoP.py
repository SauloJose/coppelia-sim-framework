import numpy as np
import matplotlib.pyplot as plt 
import matplotlib.patches as patches
import matplotlib.transforms as transforms

from brainbyte import BaseApp
from brainbyte.robots.movel.TurtleBot import *
from brainbyte.control.automatic import * 
from brainbyte.sensors.LDS_02 import *
from brainbyte.utils.environment import *

from brainbyte.planner.path.potential import PotentialFieldPlanner

class PathPlanning(BaseApp):
    def __init__(self, show_lidar=True):
        super().__init__(scene_file="mapa.ttt", sim_name="PathPlanning", sim_time=120)
        self.obstacles_data = []
        self.boundary_vertices = []
        self.wall_polygons = []
        self.show_lidar = show_lidar
        self.bot_radius = 0.15 

        self.target_point = np.array([0.0, 0.0, 0.0])
        self.planner = PotentialFieldPlanner(k_att=2, k_rep=0.5, rho_0=0.2, step_size=0.2)
        
        # Atributo para guardar o mapa de calor
        self.field_heatmap = None

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
        
        position = self.robot.pose
        self.target_point = np.array([position[0], position[1], position[2]])
        
        # MANTÉM O CONTROLADOR (Ele será responsável por girar as rodas)
        self.control = DifferentialController(pos_init=position,
                                              set_point=self.target_point,
                                              k_alpha=3,
                                              k_beta=-0.5,
                                              k_rho=0.5,   
                                              dt=self.dt)

        self.control.set_max_values(v_max=self.robot._v_max, w_max=self.robot._w_max)
        self.robot.add_control(control_name='AUTO_DIFF', control_instance=self.control)
        
        self.define_plot_configs()
        self.command_lines()
    def on_map_click(self, event):
        if event.xdata is None or event.ydata is None:
            return
            
        x_clicado = event.xdata
        y_clicado = event.ydata
        
        self.target_point = np.array([x_clicado, y_clicado, 0.0])
        self.plot_target_marker.set_data([x_clicado], [y_clicado])
        self.logger.info(f"Novo objetivo global definido em: X={x_clicado:.2f}, Y={y_clicado:.2f}")

    def command_lines(self):
        self.logger.warning("Aqui ainda será implementado uma lógica para entrar com variáveis para a simulação")

    def post_start(self):
        super().post_start()
        self.bridge.step() 

        pos = self.robot.pose
        self.logger.info(f'Initial robot position: x={pos[0]:.2f}, y={pos[1]:.2f}, theta ={np.rad2deg(pos[2]):.2f}')
        self.target_point = pos 
        self.plot_target_marker.set_data([self.target_point[0]], [self.target_point[1]])

        self.bot_radius = get_robot_radius(self.sim, 'Turtlebot3/base_link')
        self.logger.info(f"Buscando obstáculos da cena com raio de inflação: {self.bot_radius:.2f}m")
        
        self.obstacles_data, self.boundary_vertices, self.wall_polygons = get_environment_obstacles(
            self.sim, robot_radius=self.bot_radius
        )

        self.plot_robot_body = patches.Circle(
            (pos[0], pos[1]), radius=self.bot_radius,
            edgecolor='#34495E', facecolor='#ECF0F1', linewidth=2, zorder=5
        )
        self.ax.add_patch(self.plot_robot_body)
        
        self.plot_influence_zone = patches.Circle(
            (pos[0], pos[1]), radius=self.planner.rho_0,
            edgecolor='#E67E22', facecolor='#FAD7A1', alpha=0.08, 
            linestyle='--', linewidth=1.2, zorder=4
        )
        self.ax.add_patch(self.plot_influence_zone)
        
        dx = self.bot_radius * np.cos(pos[2]) * 1.8
        dy = self.bot_radius * np.sin(pos[2]) * 1.8
        self.plot_robot_dir, = self.ax.plot(
            [pos[0], pos[0] + dx], [pos[1], pos[1] + dy],
            color='#2980B9', linewidth=2.5, zorder=6
        )

    def define_plot_configs(self):
        plt.ion() 
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_aspect('equal')
        
        self.ax.set_xlim(-7, 7)
        self.ax.set_ylim(-7, 7)
        
        self.ax.set_title("Navegação por Campo Potencial", fontsize=14, 
                         fontweight='bold', color='#1E293B', pad=15)
        self.ax.set_xlabel("X (metros)", fontsize=10, color='#64748B')
        self.ax.set_ylabel("Y (metros)", fontsize=10, color='#64748B')
        self.ax.grid(True, linestyle='--', color='#CBD5E1', alpha=0.5, zorder=2)
        self.ax.set_facecolor('#F8FAFC')

        self.plot_robot_center, = self.ax.plot([], [], 'o', 
            color='#E74C3C', markersize=6, markeredgecolor='#C0392B', 
            markeredgewidth=1.5, zorder=6)
        
        self.plot_target_marker, = self.ax.plot(
            [self.target_point[0]], [self.target_point[1]], 
            '*', color='#2ECC71', markersize=18, markeredgecolor='#27AE60',
            markeredgewidth=2, zorder=7
        )
        
        self.plot_lidar, = self.ax.plot([], [], '.', 
            color='#94A3B8', markersize=2, alpha=0.4, zorder=3)
        
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], marker='*', color='w', markerfacecolor='#2ECC71', markersize=12, label='Objetivo'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#E74C3C', markersize=8, label='Robô'),
            patches.Patch(color='#F39C12', alpha=0.5, label='Intensidade do Campo')
        ]
        self.ax.legend(handles=legend_elements, loc='upper right', framealpha=0.9, edgecolor='#CBD5E1')
        
        self.fig.canvas.mpl_connect('button_press_event', self.on_map_click)

    def compute_potential_field(self, bounds=(-7, 7, -7, 7), resolution=0.05, lidar_data=None):
        """Calcula o campo potencial e retorna matrizes prontas para um mapa de calor"""
        x_min, x_max, y_min, y_max = bounds
        
        # Cria as grades X e Y usando a resolução
        x_coords = np.arange(x_min, x_max, resolution)
        y_coords = np.arange(y_min, y_max, resolution)
        X, Y = np.meshgrid(x_coords, y_coords)
        
        # Achata a grade para calcular os pontos de uma vez
        points = np.column_stack((X.ravel(), Y.ravel()))
        forces = np.zeros_like(points)
        goal_2d = self.target_point[:2]
        
        # Junta obstáculos estáticos com pontos do Lidar
        all_obs_pts = []
        if len(self.obstacles_data) > 0:
            for obs in self.obstacles_data:
                if 'corners' in obs:
                    all_obs_pts.extend(obs['corners'])
                    
        if lidar_data is not None and len(lidar_data) > 0:
            # Pega 1 a cada 3 pontos do Lidar para performance
            all_obs_pts.extend(lidar_data[::3, :2]) 
        
        obs_array = np.array(all_obs_pts) if all_obs_pts else None
        
        for i, pt in enumerate(points):
            # Força atrativa
            error = goal_2d - pt
            dist = np.linalg.norm(error)
            if dist > 1e-6:
                f_att = error / dist * min(self.planner.k_att * dist, 2.0)
            else:
                f_att = np.zeros(2)
            
            # Força repulsiva
            f_rep = np.zeros(2)
            if obs_array is not None:
                diff = pt - obs_array
                dists = np.linalg.norm(diff, axis=1)
                dists = np.maximum(dists, 0.05)
                
                mask = dists <= self.planner.rho_0
                if np.any(mask):
                    valid_diff = diff[mask]
                    valid_dists = dists[mask]
                    magnitudes = self.planner.k_rep * ((1.0 / valid_dists) - (1.0 / self.planner.rho_0)) / (valid_dists ** 2)
                    directions = valid_diff / valid_dists[:, None]
                    f_rep = np.sum(directions * magnitudes[:, None], axis=0)
            
            forces[i] = f_att + f_rep
        
        # Calcula a magnitude resultante para o mapa de calor e remodela para a grade 2D
        magnitudes_grid = np.linalg.norm(forces, axis=1).reshape(X.shape)
        return X, Y, magnitudes_grid

    def plot_result(self, ds, robot, plot_lidar=True):
        if plot_lidar and ds is not None and len(ds) > 0:
            self.plot_lidar.set_data(ds[:, 0], ds[:, 1])
        else:
            self.plot_lidar.set_data([], [])
        
        pos = robot.pose
        theta = pos[2]
        
        self.plot_robot_body.set_center((pos[0], pos[1]))
        self.plot_influence_zone.set_center((pos[0], pos[1]))
        self.plot_robot_center.set_data([pos[0]], [pos[1]])
        
        dx = self.bot_radius * np.cos(theta) * 1.8
        dy = self.bot_radius * np.sin(theta) * 1.8
        self.plot_robot_dir.set_data([pos[0], pos[0] + dx], [pos[1], pos[1] + dy])
        
        try:
            # 1. Atualizamos a chamada para passar o Lidar e receber dados de mapa de calor
            X, Y, Z = self.compute_potential_field(resolution=0.15, lidar_data=ds)
            
            # Limpa o mapa de calor anterior para não sobrecarregar a memória do Matplotlib
            if self.field_heatmap is not None:
                self.field_heatmap.remove()
                
            # Recorta os valores para que as proximidades extremas dos obstáculos não quebrem a visualização
            Z_clipped = np.clip(Z, 0, 3.0) 
            
            # 2. Desenha o mapa de calor
            self.field_heatmap = self.ax.pcolormesh(
                X, Y, Z_clipped, cmap='turbo', alpha=0.4, 
                zorder=1, vmin=0, vmax=3.0, shading='nearest'
            )
            
        except Exception as e:
            self.logger.debug(f"Erro ao atualizar mapa de calor: {e}")
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()



    def loop(self, t, actual_state=None):
        try:
            data_sensor = self.robot.get_sensor(sensor_name='LIDAR').update() 
            actual_pos = self.robot.pose 

            # 1. O Campo Potencial avalia atração do objetivo global e repulsão do Lidar
            target_imediato = self.planner.get_next_target(
                current_pos=actual_pos, 
                goal_pos=self.target_point, 
                lidar_points=data_sensor, 
                static_obstacles=self.obstacles_data
            )
            
            # 2. Verifica se já chegou no objetivo final para parar
            dist_to_global_goal = np.linalg.norm(self.target_point[:2] - actual_pos[:2])
            
            if dist_to_global_goal < 0.1:
                # Se chegou, o alvo é a própria posição (para o robô)
                self.control.set_point = np.array([actual_pos[0], actual_pos[1], actual_pos[2]])
            else:
                # 3. Alimenta o controlador automático apenas com a nova coordenada X, Y calculada
                # Assumindo que o controlador espera um array [X, Y, Theta]
                # O Theta pode ser a angulação apontando para o próprio target imediato
                dx = target_imediato[0] - actual_pos[0]
                dy = target_imediato[1] - actual_pos[1]
                theta_imediato = np.arctan2(dy, dx)
                
                self.control.set_point = np.array([target_imediato[0], target_imediato[1], theta_imediato])
            
            # 4. Atualiza o gráfico (Sem a linha de previsão, como você pediu originalmente)
            self.plot_result(ds=data_sensor, robot=self.robot, plot_lidar=self.show_lidar)
            
        except Exception as e:
            self.logger.error(f"Erro detectado no loop(): {e}")
    def stop(self):
        try:
            self.robot.stop()
            plt.ioff()
            plt.close(self.fig)
        except Exception as e:
            self.logger.error(f"Erro detectado in stop(): {e}")
    
def app():
    aplicacao = PathPlanning(show_lidar=True)
    aplicacao.run()

if __name__ == '__main__':
    app()