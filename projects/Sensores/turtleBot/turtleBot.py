from brainbyte import BaseApp
from brainbyte.robots.movel.TurtleBot import *
from brainbyte.control.automatic import *   
from brainbyte.sensors.LDS_02 import *
from brainbyte.gui.auxF import get_key
from brainbyte.control.manual import *
import matplotlib.pyplot as plt 

# handles
V_MAX = 0.5            # 0,5 m/s
W_MAX = np.deg2rad(20) # 20 graus/s

def get_command():
    """
    Retorna (v, w) baseado nas teclas pressionadas:
        W / seta cima  → frente
        S / seta baixo → trás
        A / seta esq.  → gira no lugar (anti-horário)
        D / seta dir.  → gira no lugar (horário)
    Se nenhuma tecla for pressionada, retorna (0.0, 0.0).
    """
    v = 0.0
    w = 0.0

    # Movimento linear (frente/trás)
    if keyboard.is_pressed('w') or keyboard.is_pressed('up'):
        v += V_MAX
    if keyboard.is_pressed('s') or keyboard.is_pressed('down'):
        v -= V_MAX

    # Movimento angular (rotação)
    if keyboard.is_pressed('a') or keyboard.is_pressed('left'):
        w += W_MAX
    if keyboard.is_pressed('d') or keyboard.is_pressed('right'):
        w -= W_MAX

    return v, w
    

# Aqui está a definição da classe que representa a simulação
class turtleBot(BaseApp):
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

        # Variáveis computadas 
        self.dt = self.d_time()

        #Buffers para os pontos calculados
        self.buffer = PointCloudAccumulator(max_point=100000)

        self.logger.info(f"Robot configurated for a manual controller W-A-S-D with v_max = {v_max} and w_max = {w_max}...")
        
        # Filtro passa baixa
        self.v_cmd = 0
        self.w_cmd = 0
        self.tau   = 0.3

        self.define_plot_configs()

    def post_start(self):
        """ É executado logo quando inicia a simulação"""
        super().post_start()
        
        # AJUSTE 4: Lemos a pose inicial aqui, pois a ponte já terá os dados no cache
        pos = self.robot.pose
        self.logger.info(f'Initial robot position: x={pos[0]:.2f}, y={pos[1]:.2f}')

    
    def define_plot_configs(self):
        """ Configurações de plot """
        self.plot_counter = 0

        plt.ion() #Ativa modo interativo
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)

        self.plot_robot, = self.ax.plot([],[],'ro',label='Robô',zorder=5)
        self.plot_lidar, = self.ax.plot([],[],'b.',markersize=1, label='Lidar')

    def loop(self, t):
        """ Etapas do loop"""
        try:
            #Controlador manual
            v_target,w_target = self.control.get_command()
            
            ### Filtro passa baixa para o comando (tau.dv/dt + v = v_desjado) 
            alpha = self.dt / self.tau   # fator de mistura

            # Aproximação de euler (v[k+1] = v[k] + alpha * (v_max - v[comando])): 
            self.v_cmd += alpha * (v_target - self.v_cmd)
            self.w_cmd += alpha * (w_target - self.w_cmd)

            self.robot.set_wheel_velocity(linear_vel=self.v_cmd,angular_vel=self.w_cmd)

            #Puxa dados dos sensores e salva
            data_sensor = self.robot.get_sensor(sensor_name='LIDAR').update() #Puxando dados do LIDAR
            #self.buffer.add(data_sensor)

            # --- Atualiza plot do lidar com limite de FPS ---
            self.plot_counter += 1
            if self.plot_counter % 2 == 0:
                lx = data_sensor[:,0]
                ly = data_sensor[:,1]
                self.plot_lidar.set_data(lx, ly)
                
                pos = self.robot.pose
                self.plot_robot.set_data([pos[0]], [pos[1]])

                self.fig.canvas.draw()
                self.fig.canvas.flush_events()

        except Exception as e:
            # CHAVES DUPLAS AQUI:
            self.logger.error(f"Erro detected in loop(): {e}")

    def stop(self):
        """ Executado após a simulação terminar - parada segura"""
        try:
            self.robot.stop()

            plt.ioff()
            plt.close(self.fig)
            self.logger.debug(f"Buffer Usage: {self.buffer._total_count}")
            
        except Exception as e:
            # CHAVES DUPLAS AQUI:
            self.logger.error(f"Erro detected in stop(): {e}")
    
def app():
    """
        Ponto de entrada para a simulação em main.py
    """
    aplicacao = turtleBot()
    aplicacao.run()