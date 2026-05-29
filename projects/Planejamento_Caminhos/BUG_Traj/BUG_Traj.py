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
from brainbyte.planner.path.bugP import *

class BUG_Traj(BaseApp):
    def __init__(self, show_lidar=True):
        super().__init__(scene_file="mapa.ttt", sim_name="PathPlanning", sim_time=120)
        self.obstacles_data = []
        self.show_lidar = show_lidar
        self.bot_radius = 0.15 

        self.target_point = np.array([0.0, 0.0, 0.0])

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

        # =======================================================================================
        position = self.robot.pose
        self.target_point = np.array([position[0], position[1], position[2]])
        
        # Usando o controlador atualizado
        self.control = DifferentialController(pos_init=position,
                                      set_point=self.target_point,
                                      k_alpha=3,
                                      k_beta=-0.5,
                                      k_rho=0.5,   
                                      dt=self.dt)


        # Usando fallback seguro para os limites
        v_max_fallback = getattr(self.robot, '_v_max', 0.22)
        w_max_fallback = getattr(self.robot, '_w_max', 2.84)
        self.control.set_max_values(v_max=v_max_fallback, w_max=w_max_fallback)
            
        self.robot.add_control(control_name='AUTO_DIFF', control_instance=self.control)
        # =======================================================================================
        
        # Adicionando o planner atualizado
        # Configurações de plot
        self.define_plot_configs()
        self.command_lines()

    def on_map_click(self, event):
        """Função disparada automaticamente sempre que você clica dentro do gráfico."""
        if event.xdata is None or event.ydata is None:
            return
            
        x_clicado = event.xdata
        y_clicado = event.ydata
        
        self.target_point = np.array([x_clicado, y_clicado, 0.0])
        
        self.plot_target_marker.set_data([x_clicado], [y_clicado])
        
        if hasattr(self.control, 'set_point'):
            self.control.set_point = self.target_point
        elif hasattr(self.control, 'update_setpoint'):
            self.control.update_setpoint(self.target_point)

        self.planner.target_point = np.array([x_clicado, y_clicado])
        self.planner.start_point = None
        self.planner.current_state = self.planner.STATE_GO_TO_GOAL

        self.logger.info(f"Novo alvo definido via clique: X={x_clicado:.2f}, Y={y_clicado:.2f}")

    def command_lines(self):
        self.logger.warning("Aqui ainda será implementado uma lógica para entrar com variáveis para a simulação")

    def post_start(self):
        super().post_start()
        self.bridge.step() # Forçar atualização

        # =====================================================================================================================
        pos = self.robot.pose
        self.logger.info(f'Initial robot position: x={pos[0]:.2f}, y={pos[1]:.2f}, theta ={np.rad2deg(pos[2]):.2f}')
        self.target_point = np.array([pos[0], pos[1], pos[2]])
        self.planner = BugPlanner(
                        start_point=pos,
                        target_point=self.target_point, 
                        safety_distance=0.5)
        
        self.control.set_point = self.target_point
        self.planner.target_point = np.array([pos[0], pos[1]])
        self.plot_target_marker.set_data([self.target_point[0]], [self.target_point[1]])
        # =====================================================================================================================

        self.bot_radius = get_robot_radius(self.sim, 'Turtlebot3/base_link')

        self.plot_robot_body = patches.Circle(
            (pos[0], pos[1]), radius=self.bot_radius,
            edgecolor='r', facecolor='none', linewidth=2, label='Contorno do Robô', zorder=5
        )
        self.ax.add_patch(self.plot_robot_body)
        
        dx = self.bot_radius * np.cos(pos[2]) * 2
        dy = self.bot_radius * np.sin(pos[2]) * 2
        self.plot_robot_dir, = self.ax.plot(
            [pos[0], pos[0] + dx],
            [pos[1], pos[1] + dy],
            color='b', linewidth=2, zorder=6, label='Direção'
        )

        self.ax.legend(loc='upper right')

        self.obstacles_data, self.boundary_vertices, _ = get_environment_obstacles(
            self.sim, 
            robot_radius=self.bot_radius,
            wall_keywords=['cuboid'] 
        )
        
        for obs in self.obstacles_data:
            polygon_inflated = patches.Polygon(
                obs['corners'], closed=True, linewidth=1.2, 
                edgecolor='red', facecolor='red', alpha=0.15, linestyle='--', zorder=2
            )
            self.ax.add_patch(polygon_inflated)

            polygon_real = patches.Polygon(
                obs['corners_originals'], closed=True, linewidth=1.5, 
                edgecolor='#333333', facecolor='#666666', alpha=0.9, zorder=3
            )
            self.ax.add_patch(polygon_real)

        if self.boundary_vertices:
            boundary_np = np.array(self.boundary_vertices)
            
            polygon_boundary = patches.Polygon(
                boundary_np, closed=True, linewidth=4,
                edgecolor="#000000", facecolor='none', linestyle='-',
                label='Limite da Área Útil', zorder=3
            )
            self.ax.add_patch(polygon_boundary)
            
            self.ax.scatter(
                boundary_np[:, 0], boundary_np[:, 1],
                color='#00AA55', marker='s', s=50, 
                label='Vértices Discretização', zorder=4
            )

        self.ax.legend(loc='upper right')
            
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

        self.plot_robot_center, = self.ax.plot([], [], 'ro', markersize=4, zorder=6)
        
        self.plot_target_marker, = self.ax.plot(
            [self.target_point[0]], [self.target_point[1]], 
            'g.', markersize=12, label='Objetivo Atual', zorder=7
        )
        
        self.plot_lidar, = self.ax.plot([], [], 'r.', markersize=2, alpha=0.6, label='Lidar', zorder=3)
            
        self.ax.legend(loc='upper right')
        self.fig.canvas.mpl_connect('button_press_event', self.on_map_click)

    def loop(self, t, actual_state=None):
        try:
            # 1. Coleta dados dos sensores
            accumulated_points = self.robot.get_sensor(sensor_name='LIDAR').update() 
            left, front, right, back = self.robot.get_sensor(sensor_name='LIDAR').get_cloud_points_sectores() 
            actual_pos = self.robot.pose 

            # Validação segura dos arrays do sensor
            dst_front_min = np.min(np.hypot(front[:, 0], front[:, 1])) if (front is not None and front.size > 0) else float('inf')
            dst_right_min = np.min(np.hypot(right[:, 0], right[:, 1])) if (right is not None and right.size > 0) else float('inf')
            obstacle_detected = bool(dst_front_min < self.planner.safety_distance)

            # 2. Controlador de alvo LIVRE 
            v_goal, w_goal = self.control.get_control(actual_point=actual_pos, 
                                                      dt = self.dt)

            # 3. BugPlanner atualizado com passagem de mapa (obstacles_data)
            v_cmd, w_cmd, state_status = self.planner.update(
                actual_pos=actual_pos,
                obstacle_in_front=obstacle_detected,
                wall_distance=dst_right_min,
                v_goal=v_goal,
                w_goal=w_goal,
                obstacles_data=self.obstacles_data  # <-- NOVA LINHA INTEGRADA AQUI
            )

            # 4. Envia comandos e atualiza plot
            self.robot.set_wheel_velocity(linear_vel=v_cmd, angular_vel=w_cmd)
            self.plot_result(ds=accumulated_points, robot=self.robot, plot_lidar=self.show_lidar)
            
        except Exception as e:
            self.logger.error(f"Erro detectado in loop(): {e}")

    def plot_result(self, ds, robot, plot_lidar=True):
        if plot_lidar and ds is not None and len(ds) > 0:
            self.plot_lidar.set_data(ds[:, 0], ds[:, 1])
        else:
            self.plot_lidar.set_data([], [])
        
        pos = robot.pose
        theta = pos[2]
        
        self.plot_robot_body.set_center((pos[0], pos[1]))
        self.plot_robot_center.set_data([pos[0]], [pos[1]])
        
        dx = self.bot_radius * np.cos(theta) * 2
        dy = self.bot_radius * np.sin(theta) * 2
        self.plot_robot_dir.set_data([pos[0], pos[0] + dx], [pos[1], pos[1] + dy])
        
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
    aplicacao = BUG_Traj(show_lidar=False)
    aplicacao.run()