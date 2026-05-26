import numpy as np
import matplotlib.pyplot as plt 
import matplotlib.patches as patches
import matplotlib.transforms as transforms

from brainbyte import BaseApp
from brainbyte.robots.movel.TurtleBot import *
from brainbyte.control.automatic import * 
from brainbyte.sensors.LDS_02 import *
from brainbyte.utils.environment import *
from brainbyte.planner.cells.makeCells import * #Importando o módulo de células
from brainbyte.planner.path.astar import * #Importanto o Astar

class PathPlanning(BaseApp):
    def __init__(self, show_lidar=True):
        super().__init__(scene_file="mapa.ttt", sim_name="PathPlanning", sim_time=120)
        self.obstacles_data = []
        self.show_lidar = show_lidar
        self.bot_radius = 0.15 

        self.target_point = np.array([0.0, 0.0, 0.0])

        #Rota do A*
        self.waypoints = []
        self.current_waypoint_idx =0 

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
        
        self.control = DifferentialController(pos_init=position,
                                              set_point=self.target_point,
                                              k_alpha=1.2,
                                              k_beta=-0.05,
                                              k_rho=0.3,
                                              dt=self.dt)  

        self.control.set_max_values(v_max=self.robot._v_max*2/3, w_max=self.robot._w_max)
            
        self.robot.add_control(control_name='AUTO_DIFF', control_instance=self.control)
        
        # Grid inicializado como None (será construído dinamicamente pelas dimensões das paredes)
        self.grid_map = None

        self.define_plot_configs()
        self.command_lines()

    def on_map_click(self, event):
        if event.xdata is None or event.ydata is None:
            return
            
        x_clicado = event.xdata
        y_clicado = event.ydata
        
        self.target_point = np.array([x_clicado, y_clicado, 0.0])
        self.plot_target_marker.set_data([x_clicado], [y_clicado])
        
        if self.grid_map is not None:
            pos_atual = self.robot.pose
            
            self.logger.info("Invocando o módulo path_planner.py...")
            # 1. Pega o caminho completo gerado pelo seu A* original
            caminho_completo = astar(self.grid_map, pos_atual, self.target_point)
            
            if caminho_completo is not None:
                self.logger.info(f"Rota calculada! {len(caminho_completo)} pontos no total.")
                
                # 2. Puxa apenas os pontos de curva/vértices usando sua função
                vertices = simplify_path(caminho_completo)
                
                # 3. O robô vai seguir o caminho simplificado (menos pontos/mais fluido)
                self.waypoints = vertices.tolist()
                self.current_waypoint_idx = 0
                
                # 4. Plota o caminho completo (linha contínua ciano)
                self.plot_path.set_data(caminho_completo[:, 0], caminho_completo[:, 1])
                
                # 5. Plota os vértices (bolinhas amarelas para destacar onde o robô vira)
                if not hasattr(self, 'plot_vertices'):
                    # Cria o elemento dinamicamente se ele ainda não existir no plot
                    self.plot_vertices, = self.ax.plot([], [], 'yo', markersize=6, label='Turning Points', zorder=5)
                    self.ax.legend(loc='upper right')
                
                self.plot_vertices.set_data(vertices[:, 0], vertices[:, 1])
                
            else:
                self.logger.warning("Não foi possível encontrar um caminho válido.")
                self.plot_path.set_data([], [])
                if hasattr(self, 'plot_vertices'):
                    self.plot_vertices.set_data([], [])
                self.waypoints = []

    def command_lines(self):
        self.logger.warning("Aqui ainda será implementado uma lógica para entrar com variáveis para a simulação")

    def post_start(self):
        super().post_start()
        self.bridge.step() 

        pos = self.robot.pose
        self.logger.info(f'Initial robot position: x={pos[0]:.2f}, y={pos[1]:.2f}, theta ={np.rad2deg(pos[2]):.2f}')
        self.target_point = pos 
        self.control.set_point = self.target_point
        self.plot_target_marker.set_data([self.target_point[0]], [self.target_point[1]])

        self.bot_radius = get_robot_radius(self.sim, 'Turtlebot3/base_link')

        # Desenha o contorno do Robô
        self.plot_robot_body = patches.Circle(
            (pos[0], pos[1]), radius=self.bot_radius,
            edgecolor='r', facecolor='none', linewidth=2, label='Contorno do Robô', zorder=5
        )
        self.ax.add_patch(self.plot_robot_body)
        
        # Vetor de Direção do Robô
        dx = self.bot_radius * np.cos(pos[2])*2
        dy = self.bot_radius * np.sin(pos[2])*2
        self.plot_robot_dir, = self.ax.plot(
            [pos[0],pos[0]+dx], [pos[1],pos[1]+dy],
            color='b', linewidth=2, zorder=6, label='Direção'
        )

        # Captura os dados poligonais e as paredes do CoppeliaSim
        self.obstacles_data, self.boundary_vertices, _ = get_environment_obstacles(
            self.sim, 
            robot_radius=self.bot_radius,
            wall_keywords=['cuboid'] 
        )
        
        # PROCESSAMENTO E MAPEAMENTO DO GRID DE OCUPAÇÃO (DINÂMICO)
        if self.boundary_vertices and len(self.boundary_vertices) > 0:
            boundary_np = np.array(self.boundary_vertices)
            
            # Extrai os limites reais com base nos vértices das paredes
            x_min, y_min = np.min(boundary_np, axis=0)
            x_max, y_max = np.max(boundary_np, axis=0)
            
            self.logger.info(f"Limites dinâmicos detectados -> X: [{x_min:.2f} a {x_max:.2f}], Y: [{y_min:.2f} a {y_max:.2f}]")
        else:
            x_min, x_max, y_min, y_max = -7.0, 7.0, -7.0, 7.0
            self.logger.warning("Paredes não detectadas. Usando limites padrão (-7 a 7).")

        # Ajusta as janelas de exibição do Matplotlib para enquadrar perfeitamente as paredes com margem de 0.5m
        self.ax.set_xlim(x_min - 0.5, x_max + 0.5)
        self.ax.set_ylim(y_min - 0.5, y_max + 0.5)

        # Instancia e constrói a matriz de células
        self.grid_map = GridMap(x_min=x_min, x_max=x_max, y_min=y_min, y_max=y_max, cell_size=0.1)
        
        self.logger.info("Discretizando ambiente em Grid de Ocupação...")
        self.grid_map.build_grid(self.obstacles_data)
        self.logger.info(f"Grid gerado com sucesso! Dimensões: {self.grid_map.matrix.shape}")

        # Plota apenas a matriz de células (Ocupado = Escuro, Livre = Claro)
        self.ax.imshow(
            self.grid_map.matrix, 
            origin='lower', 
            extent=[self.grid_map.x_min, self.grid_map.x_max, self.grid_map.y_min, self.grid_map.y_max], 
            cmap='Greys', 
            alpha=0.35, 
            zorder=1
        )

        # Renderização visual APENAS da linha limite das Paredes Externas
        if self.boundary_vertices:
            boundary_np = np.array(self.boundary_vertices)
            polygon_boundary = patches.Polygon(
                boundary_np, closed=True, linewidth=4,
                edgecolor="#000000", facecolor='none', linestyle='-',
                label='Paredes do Mapa', zorder=3
            )
            self.ax.add_patch(polygon_boundary)

        self.ax.legend(loc='upper right')
            
    def define_plot_configs(self):
        plt.ion() 
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_aspect('equal')
        
        # Limites iniciais temporários (serão redefinidos dinamicamente no post_start)
        self.ax.set_xlim(-7, 7)
        self.ax.set_ylim(-7, 7)
        
        self.ax.set_title("Navegação e Planejamento de Caminhos", fontsize=14, pad=15)
        self.ax.set_xlabel("X (metros)")
        self.ax.set_ylabel("Y (metros)")
        self.ax.grid(True, linestyle='--', alpha=0.3, zorder=0)

        self.plot_robot_center, = self.ax.plot([], [], 'ro', markersize=4, zorder=6)
        self.plot_target_marker, = self.ax.plot(
            [self.target_point[0]], [self.target_point[1]], 
            'g.', markersize=12, label='Objetivo Atual', zorder=7
        )
        self.plot_lidar, = self.ax.plot([], [], 'r.', markersize=2, alpha=0.6, label='Lidar', zorder=3)
        
        self.plot_path, = self.ax.plot([], [], 'c-', linewidth=2.5, label='Caminho A*', zorder=4)

        self.ax.legend(loc='upper right')
        self.fig.canvas.mpl_connect('button_press_event', self.on_map_click)

    def loop(self, t, actual_state=None):
        try:
            data_sensor = self.robot.get_sensor(sensor_name='LIDAR').update() 
            actual_pos = self.robot.pose 

            # CONTROLADOR DE SEGUIMENTO DE CAMINHO (WAYPOINTS)
            if self.waypoints and self.current_waypoint_idx < len(self.waypoints):
                ponto_alvo = self.waypoints[self.current_waypoint_idx]
                
                self.control.set_point = np.array([ponto_alvo[0], ponto_alvo[1], 0.0])
                
                dist_ao_ponto = np.sqrt((actual_pos[0] - ponto_alvo[0])**2 + (actual_pos[1] - ponto_alvo[1])**2)
                
                if dist_ao_ponto < 0.15:
                    self.current_waypoint_idx += 1
                    if self.current_waypoint_idx >= len(self.waypoints):
                        self.logger.info("O robô chegou com sucesso ao destino final!")
            else:
                self.control.set_point = self.target_point

            # Calcula e envia os comandos físicos de velocidade para os motores
            v_cmd, w_cmd = self.robot.get_control('AUTO_DIFF').get_control(actual_point=actual_pos)
            self.robot.set_wheel_velocity(linear_vel=v_cmd, angular_vel=w_cmd)
            
            self.plot_result(ds=data_sensor, robot=self.robot, plot_lidar=self.show_lidar)
            
        except Exception as e:
            self.logger.error(f"Erro detectado no loop(): {e}")
    
    def plot_result(self, ds, robot, plot_lidar=True):
        if plot_lidar and ds is not None and len(ds) > 0:
            self.plot_lidar.set_data(ds[:, 0], ds[:, 1])
        else:
            self.plot_lidar.set_data([], [])
        
        pos = robot.pose
        theta = pos[2]
        
        self.plot_robot_body.set_center((pos[0], pos[1]))
        self.plot_robot_center.set_data([pos[0]], [pos[1]])
        
        dx = self.bot_radius * np.cos(theta)*2
        dy = self.bot_radius * np.sin(theta)*2
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
    aplicacao = PathPlanning(show_lidar=False)
    aplicacao.run()