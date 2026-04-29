import numpy as np
import traceback

from brainbyte import BaseApp             # A aplicação básica está aqui
from brainbyte.robots import * # Os robôs configurados estão nessa Pasta 
from brainbyte.control.automatic import * # Controle automático    
from brainbyte.control.manual import * # Controle Manual  
from brainbyte.core.bridge import SimulationBridge
from brainbyte.sensors.LDS_02 import *

"""
@GN0MIO: Este template serve como ponto de partida para a estruturação e desenvolvimento da sua simulação.

- setup(): Configuração inicial do cenário. Utilize este método para instanciar robôs e chamar o bridge.initialize().
          IMPORTANTE: Agora o initialize recebe 'self.sim' para destravar o modo síncrono.

- post_start(): Executado logo após o início real da simulação. 
                Ideal para capturar a primeira pose ou dados de sensores após o handshake.

- loop(): Núcleo de execução contínua. 
          A física avança automaticamente via self.bridge.step() (gerenciado pelo BaseApp).

- stop(): Rotina de encerramento para salvar logs, gráficos ou parar motores.
"""

class Controle_de_caminho(BaseApp):
    """
    Classe principal da simulação Controle_de_caminho.
    Gerencia o ciclo de vida da aplicação, integrando a lógica de controle com a cena do CoppeliaSim.
    """
    def __init__(self):
        """Inicializa os parâmetros base da aplicação, definindo a cena e o tempo total de simulação."""
        super().__init__(scene_file="Scene.ttt", sim_name="Controle_de_caminho", sim_time=180)

    def setup(self):
        """Configura os recursos iniciais da simulação (instanciação de robôs, sensores e controladores)."""
        try:
            self.logger.info("Configuring Robot, Sensor and Controllers...")

            # Exemplo de adicionar um robô
            self.robot = TurtleBot(bridge=self.bridge,
                               robot_name='Turtlebot3', 
                               left_motor='left_motor', 
                               right_motor='right_motor',
                               base_link='base_link'
                              )
            
            self.Lidar = LDS_02(bridge=self.bridge, base_name= 'Turtlebot3')

            self.robot.add_sensor(sensor_name='LIDAR',sensor_instance=self.Lidar)

            # Parte importante no projeto!!!
            self.handshake()

            #Controlador do robô
            # Informações iniciais 
            position = self.robot.pose

            pos_desejada = np.array([2,2,np.deg2rad(40)]) #(3,3,40º)
            self.dt = self.d_time()

            self.control = DifferentialController(pos_init=position,
                                                  set_point=pos_desejada,
                                                  k_alpha=0.8,
                                                  k_beta=-0.1,
                                                  k_rho=0.3,
                                                  dt = self.dt)  

            self.control._setup_output_filter(tau=0.05, dt=self.dt) #low_pass_filter
           
            self.robot.add_control(control_name='AUTO_DIFF',
                                   control_instance=self.control)
            
            self.logger.info("Handshake with CoppeliaSim: OK!")  

        except Exception as e:
            self.logger.error(f"Error detected in setup(): {e}")
            self.logger.error(traceback.format_exc())
            
    def post_start(self):
        """Executado uma única vez após o startSimulation(). Ideal para leituras iniciais."""
        try:
            return super().post_start()
        except Exception as e:
            self.logger.error(f"Error detected in post_start(): {e}")
    
    def handshake(self):
        """Necessário para iniciar a lógica de comunicação. Siga esse padrão e adicione
        cada robô novo no monitor_path e actuator_path"""
        try: 
            monitor_paths = self.robot.get_monitor_paths()
            actuator_paths = self.robot.get_actuator_paths()
            self.bridge.initialize(monitor_paths, actuator_paths, self.sim)
            self.logger.info("Handshake with CoppeliaSim: OK!") 
        except Exception as e:
            msg = traceback.format_exc()
            self.logger.error(f"Error in Handshake with CoppeliaSim ({e})! \n Traceback:\n{msg}") 
    
    def loop(self, t):
        """
        Núcleo de execução contínua. 
        Implemente aqui a lógica de controle principal, leitura de sensores e atualização de atuadores.
        """
        try:
            # Adicione a lógica do loop aqui (força ser um numpy array)
            actual_pos = self.robot.pose 

            #Controlador manual
            v_cmd,w_cmd = self.control.get_control(actual_point=actual_pos)
            
            self.robot.set_wheel_velocity(linear_vel=v_cmd,angular_vel=w_cmd)
        
        except Exception as e:
            self.logger.error(f"Error detected in loop(): {e}")

    def stop(self):
        """Rotina de encerramento para garantir a parada segura dos componentes e exportação de resultados."""
        try:
            # Descomente a linha abaixo quando tiver instanciado self.robot no setup()
            self.robot.stop()
        except Exception as e:
            self.logger.error(f"Error detected in stop(): {e}")

def app():
    """
    Ponto de entrada principal da simulação. 
    Instancia a classe e inicia o ciclo de vida (run) para integração com o gerenciador BRAINBYTE.
    """
    aplicacao = Controle_de_caminho()
    aplicacao.run()