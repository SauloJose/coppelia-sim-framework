import numpy as np
import matplotlib.pyplot as plt 
import matplotlib.patches as patches

from brainbyte import BaseApp
from brainbyte.robots.movel.TurtleBot import *
from brainbyte.sensors.LDS_02 import *
from brainbyte.utils.environment import *

from brainbyte.planner.path.potential import PotentialFieldPlanner
from brainbyte.control.automatic import * 

class PathPlanning(BaseApp):
    def __init__(self, show_lidar=True):
        super().__init__(scene_file="mapa.ttt", sim_name="PathPlanning", sim_time=120)
        self.obstacles_data = []
        self.show_lidar = show_lidar
        self.bot_radius = 0.15 

        self.target_point = np.array([0.0, 0.0, 0.0])
        
        # Planejador limpo
        self.planner = PotentialFieldPlanner(
            k_att=2.0, 
            k_rep=0.5, 
            rho_0=0.25
        )

        # Controladores PID
        self.pid_w = PID_Controller(var=0.0, kp=0.8, ki=0.02, kd=0.1, dt=self.dt, set_point=0.0)
        self.pid_v = PID_Controller(var=0.0, kp=1, ki=0.01, kd=0.02, dt=self.dt, set_point=0.0)
        self.current_v = 0.0 

        # --- VARIÁVEIS PARA OS GRÁFICOS DE ERRO ---
        self.time_history = []
        self.error_v_history = []
        self.error_w_history = []
        self.start_time = None

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
        
        self.define_plot_configs()

    def on_map_click(self, event):
        # Ignora cliques que aconteçam na janela de erros
        if event.inaxes != self.ax:
            return
            
        if event.xdata is None or event.ydata is None:
            return
            
        x_clicado = event.xdata
        y_clicado = event.ydata
        
        self.target_point = np.array([x_clicado, y_clicado, 0.0])
        self.plot_target_marker.set_data([x_clicado], [y_clicado])
        self.logger.info(f"Novo objetivo em: X={x_clicado:.2f}, Y={y_clicado:.2f}")

    def post_start(self):
        super().post_start()
        
        pos = self.robot.pose
        self.target_point = pos 
        self.plot_target_marker.set_data([self.target_point[0]], [self.target_point[1]])

        self.bot_radius = get_robot_radius(self.sim, 'Turtlebot3/base_link')
        self.obstacles_data, _, _ = get_environment_obstacles(self.sim, robot_radius=self.bot_radius)

        # Região de Influência (rho_0)
        # Zorder=1.5 garante que fique acima da grade (1), mas abaixo do Lidar (2) e do Robô (5)
        rho_0 = getattr(self.planner, 'rho_0', 0.5) 
        self.plot_influence_region = patches.Circle(
            (pos[0], pos[1]), 
            radius=rho_0, 
            facecolor='red', 
            edgecolor='none',
            alpha=0.15, 
            zorder=1.5 
        )
        self.ax.add_patch(self.plot_influence_region)

        # Robô
        self.plot_robot_body = patches.Circle((pos[0], pos[1]), radius=self.bot_radius, edgecolor='#34495E', facecolor='#ECF0F1', linewidth=2, zorder=5)
        self.ax.add_patch(self.plot_robot_body)
        
        # Direção
        self.plot_robot_dir, = self.ax.plot([], [], color='black', linewidth=3, zorder=6)

    def define_plot_configs(self):
        plt.ion() 
        
        # --- JANELA 1: MAPA E LIDAR (Reduzida) ---
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.fig.canvas.manager.set_window_title('Mapa e Sensores')
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-7, 7)
        self.ax.set_ylim(-7, 7)
        self.ax.set_title("Visão Local e Vetores de Força", fontsize=12, fontweight='bold', pad=10)
        self.ax.grid(True, linestyle='--', alpha=0.5, zorder=1)

        self.plot_lidar, = self.ax.plot([], [], '.', color='#94A3B8', markersize=3, alpha=0.6, zorder=2)
        self.plot_target_marker, = self.ax.plot([self.target_point[0]], [self.target_point[1]], '*', color='#2ECC71', markersize=18, zorder=4)
        
        self.plot_f_att, = self.ax.plot([], [], color='green', linewidth=2.5, zorder=7)
        self.plot_f_rep, = self.ax.plot([], [], color='red', linewidth=2.5, zorder=7)
        self.plot_f_tot, = self.ax.plot([], [], color='blue', linewidth=3.5, zorder=8) 
        
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], marker='*', color='w', markerfacecolor='#2ECC71', markersize=10, label='Objetivo'),
            Line2D([0], [0], color='blue', lw=3.5, label='F. Resultante'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=10, alpha=0.3, label='Área de Influência')
        ]
        self.ax.legend(handles=legend_elements, loc='upper right', fontsize=8)
        self.fig.canvas.mpl_connect('button_press_event', self.on_map_click)

        # --- JANELA 2: GRÁFICOS DE ERRO DOS PIDs ---
        self.fig_err, (self.ax_err_v, self.ax_err_w) = plt.subplots(2, 1, figsize=(6, 6))
        self.fig_err.canvas.manager.set_window_title('Erros dos PIDs')
        self.fig_err.subplots_adjust(hspace=0.4)

        # Gráfico Erro Linear
        self.ax_err_v.set_title("Erro de Velocidade Linear ($v$)")
        self.ax_err_v.set_ylabel("Erro (m/s)")
        self.ax_err_v.grid(True, linestyle=':', alpha=0.7)
        self.line_err_v, = self.ax_err_v.plot([], [], 'b-', lw=2)

        # Gráfico Erro Angular
        self.ax_err_w.set_title("Erro de Velocidade Angular ($w$)")
        self.ax_err_w.set_ylabel("Erro (rad)")
        self.ax_err_w.set_xlabel("Tempo (s)")
        self.ax_err_w.grid(True, linestyle=':', alpha=0.7)
        self.line_err_w, = self.ax_err_w.plot([], [], 'r-', lw=2)

    def plot_result(self, ds, robot):
        # --- ATUALIZA JANELA 1 (MAPA) ---
        if self.show_lidar and ds is not None and len(ds) > 0:
            self.plot_lidar.set_data(ds[:, 0], ds[:, 1])
        else:
            self.plot_lidar.set_data([], [])
        
        pos = robot.pose
        x, y, theta = pos[0], pos[1], pos[2]
        
        # Atualiza a posição do Robô E da Região de Influência
        self.plot_robot_body.set_center((x, y))
        self.plot_influence_region.set_center((x, y))
        
        dx_dir = self.bot_radius * np.cos(theta)
        dy_dir = self.bot_radius * np.sin(theta)
        self.plot_robot_dir.set_data([x, x + dx_dir], [y, y + dy_dir])
        
        if hasattr(self.planner, 'f_att'):
            desired_len = 0.5
            
            mag_a = np.linalg.norm(self.planner.f_att)
            f_a = (self.planner.f_att / mag_a) * desired_len if mag_a > 1e-6 else np.zeros(2)
            self.plot_f_att.set_data([x, x + f_a[0]], [y, y + f_a[1]])
            
            mag_r = np.linalg.norm(self.planner.f_rep)
            f_r = (self.planner.f_rep / mag_r) * desired_len if mag_r > 1e-6 else np.zeros(2)
            self.plot_f_rep.set_data([x, x + f_r[0]], [y, y + f_r[1]])
            
            mag_t = np.linalg.norm(self.planner.f_total)
            f_t = (self.planner.f_total / mag_t) * desired_len if mag_t > 1e-6 else np.zeros(2)
            self.plot_f_tot.set_data([x, x + f_t[0]], [y, y + f_t[1]])
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # --- ATUALIZA JANELA 2 (ERROS) ---
        if len(self.time_history) > 0:
            self.line_err_v.set_data(self.time_history, self.error_v_history)
            self.line_err_w.set_data(self.time_history, self.error_w_history)

            # Cria uma "janela móvel" mostrando apenas os últimos 10 segundos
            current_time = self.time_history[-1]
            min_time = max(0, current_time - 10)
            
            self.ax_err_v.set_xlim(min_time, current_time + 1)
            self.ax_err_w.set_xlim(min_time, current_time + 1)

            # Ajusta o eixo Y dinamicamente
            self.ax_err_v.relim()
            self.ax_err_v.autoscale_view()
            self.ax_err_w.relim()
            self.ax_err_w.autoscale_view()

            self.fig_err.canvas.draw()
            self.fig_err.canvas.flush_events()

    def loop(self, t, actual_state=None):
        try:
            # Inicializa o tempo para o gráfico
            if self.start_time is None:
                self.start_time = t
            current_t = t - self.start_time

            lidar_sensor = self.robot.get_sensor(sensor_name='LIDAR')
            all_world_pts = lidar_sensor.update() 
            
            actual_pos = self.robot.pose 
            yaw = actual_pos[2] 

            # Filtragem por setores
            left, front, right, back = lidar_sensor.get_cloud_points_sectores()
            critical_local_pts = []

            for sector in [left, front, right, back]:
                if sector.size > 0:
                    distances = np.linalg.norm(sector, axis=1)
                    closest_idx = np.argmin(distances)
                    critical_local_pts.append(sector[closest_idx])

            if critical_local_pts:
                planner_lidar_pts = lidar_sensor._transform_to_world(np.array(critical_local_pts))
            else:
                planner_lidar_pts = np.empty((0, 3))

            f_total = self.planner.compute_force_vector(
                current_pos=actual_pos, 
                goal_pos=self.target_point, 
                lidar_points=planner_lidar_pts, 
                static_obstacles=self.obstacles_data
            )
            
            mag_total = np.linalg.norm(f_total)
            if mag_total < 1e-4:
                self.robot.set_wheel_velocity(0.0, 0.0)
                self.pid_w.reset()
                self.pid_v.reset()
                self.current_v = 0.0
                
                # Zera erros ao chegar
                self.error_v_history.append(0.0)
                self.error_w_history.append(0.0)
                self.time_history.append(current_t)
                
                self.plot_result(ds=all_world_pts, robot=self.robot)
                return

            # PID Angular
            target_yaw = np.arctan2(f_total[1], f_total[0])
            yaw_error = target_yaw - yaw
            yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error)) # wrap [-pi, pi]
            
            self.pid_w.set_setpoint(0.0)
            w = self.pid_w.run(y=-yaw_error, u_min=-2.84, u_max=2.84)
            
            # PID Linear
            alignment = max(0.0, np.cos(yaw_error))
            target_v = mag_total * alignment
            target_v = min(target_v, self.robot._v_max) 
            
            self.pid_v.set_setpoint(target_v)
            actual_v = self.robot.robot_velocity[0] 
            v = self.pid_v.run(y=actual_v, u_min=0.0, u_max=self.robot._v_max) 
            
            # --- SALVA OS ERROS PARA O GRÁFICO ---
            self.time_history.append(current_t)
            self.error_w_history.append(yaw_error)
            self.error_v_history.append(target_v - actual_v)
            
            # Evita que a lista cresça infinitamente na memória (guarda últimos 200 pontos)
            if len(self.time_history) > 200:
                self.time_history.pop(0)
                self.error_w_history.pop(0)
                self.error_v_history.pop(0)

            self.robot.set_wheel_velocity(v, w)
            self.plot_result(ds=all_world_pts, robot=self.robot)
            
        except Exception as e:
            self.logger.error(f"Erro detectado no loop(): {e}")

    def stop(self):
        try:
            self.robot.stop()
            plt.ioff()
            plt.close(self.fig)
            plt.close(self.fig_err)
        except Exception as e:
            self.logger.error(f"Erro detectado in stop(): {e}")
            
def app():
    aplicacao = PathPlanning(show_lidar=True)
    aplicacao.run()

if __name__ == '__main__':
    app()