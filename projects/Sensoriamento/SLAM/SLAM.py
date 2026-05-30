from brainbyte import BaseApp
from brainbyte.robots.movel.TurtleBot import *
from brainbyte.control.automatic import *   
from brainbyte.sensors.LDS_02 import *
from brainbyte.gui.auxF import get_key
from brainbyte.control.manual import *
from brainbyte.utils.environment import *
import matplotlib.pyplot as plt 

# handles
V_MAX = 0.5            # 0,5 m/s
W_MAX = np.deg2rad(20) # 20 graus/s


# Aqui está a definição da classe que representa a simulação
class SLAM(BaseApp):
    """
        Teste de locomoção de obstáculos do robotino
    """
    def __init__(self):
        """ Inicialização da aplicação"""
        super().__init__(scene_file="turtleBot.ttt", sim_name="turtleBot", sim_time=120)

    def setup(self):
        """Configura os recurso da simulação"""
        self.logger.info("Configuring Robot, Sensor and Controllers..")

        # Exemplo de adicionar um robô
        self.robot = TurtleBot(bridge=self.bridge,
                               robot_name='Turtlebot3', 
                               left_motor='left_motor', 
                               right_motor='right_motor',
                               base_link='base_link'
                              )
        
        # Sensores do robô
        self.Lidar = LDS_02(bridge=self.bridge, base_name= 'Turtlebot3')

        self.robot.add_sensor(sensor_name='LIDAR',sensor_instance=self.Lidar)

        # AJUSTE 3: Handshake com o CoppeliaSim (NOVO)
        monitor_paths = self.robot.get_monitor_paths()
        actuator_paths = self.robot.get_actuator_paths()
        self.bridge.initialize(monitor_paths, actuator_paths, self.sim)

        # Controladores do robô
        v_max = 0.2
        w_max = np.deg2rad(20)
        self.control = KeyboardController(v_max=v_max, w_max=w_max)

        #Buffers para os pontos calculados
        self.buffer = PointCloudAccumulator(max_point=100000)

        self.logger.info(f"Robot configurated for a manual controller W-A-S-D with v_max = {v_max} and w_max = {w_max}...")
        
        # Filtro passa baixa
        self.v_cmd = 0
        self.w_cmd = 0
        self.tau   = 0.3

        # Tempo para salvar os pontos no Point_Cloud 
        self.dP_cloud_time = 1 #segundos
        self.define_plot_configs()

        self.last_save_time = 0

    def post_start(self):
        """ É executado logo quando inicia a simulação"""
        super().post_start()
        
        pos = self.robot.pose
        self.theta = pos[2] #Orientação do robô

        self.logger.info(f'Initial robot position: x={pos[0]:.2f}, y={pos[1]:.2f}')

    
    def define_plot_configs(self):
        """ Configurações de plot """
        plt.ion() 
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)

        self.plot_rays, = self.ax.plot([], [], color='red', alpha=0.12, linewidth=0.5, zorder=2)
        
        self.plot_lidar, = self.ax.plot([], [], 'r.', markersize=1.5, label='Lidar Atual', zorder=4)
        
        self.plot_robot, = self.ax.plot([], [], 'go', markersize=8, label='Robô', zorder=5)
        self.plot_robot_dir, = self.ax.plot([], [], color='b', linewidth=2, zorder=6, label='Direção')

        # Legenda da nuvem preta
        self.ax.plot([], [], 'k.', markersize=1, label='Nuvem Acumulada')
        
        self.ax.legend(loc='upper right')
        self.ax.grid(True)

    def update_graphics(self, data_sensor, pos):
        """ Função exclusiva para processar e renderizar os gráficos """
        tempo_atual = self.simu_time()
        
        # 1. Atualiza a Nuvem de Pontos Acumulada (Preta) a cada 0.5s
        if tempo_atual - self.last_save_time >= self.dP_cloud_time:
            self.buffer.add(data_sensor)
            self.last_save_time = tempo_atual
            self.ax.plot(data_sensor[:, 0], data_sensor[:, 1], 'k.', markersize=1, zorder=1)

        # 2. OTIMIZAÇÃO CRÍTICA: Desenha todos os feixes de laser usando uma única linha com np.nan
        num_points = len(data_sensor)
        rays_x = np.empty(3 * num_points)
        rays_y = np.empty(3 * num_points)
        
        rays_x[0::3] = pos[0]               # Origem X (Robô)
        rays_x[1::3] = data_sensor[:, 0]    # Destino X (Lidar)
        rays_x[2::3] = np.nan               # Desconecta a linha
        
        rays_y[0::3] = pos[1]               # Origem Y (Robô)
        rays_y[1::3] = data_sensor[:, 1]    # Destino Y (Lidar)
        rays_y[2::3] = np.nan               # Desconecta a linha
        
        self.plot_rays.set_data(rays_x, rays_y)

        # 3. Atualiza os pontos instantâneos do Lidar (Vermelho)
        self.plot_lidar.set_data(data_sensor[:, 0], data_sensor[:, 1])
        
        # 4. Atualiza a posição do Robô (Verde)
        self.plot_robot.set_data([pos[0]], [pos[1]])
        
        # 5. Atualiza a linha de direção do Robô (Azul)
        dx = self.robot._L * np.cos(pos[2])
        dy = self.robot._L * np.sin(pos[2])
        self.plot_robot_dir.set_data([pos[0], pos[0] + dx], [pos[1], pos[1] + dy])

        # 6. Atualiza a tela do Matplotlib
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def loop(self, t, actual_state=None):
        """ Etapas do loop - Focado em controle e dados """
        try:
            # Controlador manual e Filtro Passa-Baixa
            v_target, w_target = self.control.get_command()
            alpha = self.dt / self.tau   
            self.v_cmd += alpha * (v_target - self.v_cmd)
            self.w_cmd += alpha * (w_target - self.w_cmd)

            # Atua nos motores
            self.robot.set_wheel_velocity(linear_vel=self.v_cmd, angular_vel=self.w_cmd)

            # Coleta dados de Telemetria (Sensores e Pose)
            data_sensor = self.robot.get_sensor(sensor_name='LIDAR').update() 
            pos = self.robot.pose

            # Delega toda a parte visual para a função especialista externa
            self.update_graphics(data_sensor, pos)

        except Exception as e:
            self.logger.error(f"Erro detected in loop(): {e}")

    def stop(self):
        """ Executado após a simulação terminar - parada segura """
        try:
            self.robot.stop()
            
            plt.ioff()
            self.logger.info(f"Simulação finalizada. Uso do Buffer: {self.buffer._total_count} pontos.")
            
            # Verifica se há pontos salvos para evitar erros
            if self.buffer._total_count > 0:
                self.logger.info("Gerando e salvando a imagem apenas com a nuvem de pontos...")
                
                # Extrai todos os pontos acumulados
                pontos = self.buffer.get_all() # ou .get_points() caso a biblioteca use esse nome
                cx = pontos[:, 0]
                cy = pontos[:, 1]
                
                # Cria uma NOVA figura exclusiva para salvar a nuvem
                fig_cloud, ax_cloud = plt.subplots(figsize=(8, 8))
                ax_cloud.set_aspect('equal')
                
                # Plota apenas os pontos acumulados em preto
                ax_cloud.plot(cx, cy, 'k.', markersize=1)
                
                # Formatações opcionais para a imagem ficar bonita
                ax_cloud.grid(True)
                ax_cloud.set_title("Nuvem de Pontos Acumulada")
                ax_cloud.set_xlabel("X (metros)")
                ax_cloud.set_ylabel("Y (metros)")
                
                # Define o caminho de salvamento na mesma pasta do módulo
                diretorio_modulo = os.path.dirname(os.path.abspath(__file__))
                caminho_final = os.path.join(diretorio_modulo, 'point_cloud.png')
                
                # Salva apenas esta figura limpa
                fig_cloud.savefig(caminho_final, dpi=300, bbox_inches='tight')
                self.logger.info(f"Nuvem de pontos salva com sucesso em: {caminho_final}")
                
            else:
                self.logger.warning("Nenhum ponto no buffer para gerar a imagem.")
            
            plt.show()
            
        except Exception as e:
            self.logger.error(f"Erro detectado in stop(): {e}")
    
def app():
    """
        Ponto de entrada para a simulação em main.py
    """
    aplicacao = SLAM()
    aplicacao.run()