from brainbyte import BaseApp              # A aplicação básica está aqui
from brainbyte.robots import * # Os robôs configurados estão nessa Pasta 
from brainbyte.control.automatic import * # Controle automático    
from brainbyte.control.manual import * # Controle Manual  
from brainbyte.core.bridge import SimulationBridge
import numpy as np
import traceback

"""
@GN0MIO: Este template serve como ponto de partida para a estruturação e desenvolvimento da sua simulação.

- setup(): Configuração inicial do cenário. Instancie seus robôs e sensores aqui.
- post_start(): Executado logo após o início real da simulação. Ideal para capturar poses iniciais.
- loop(t): Núcleo de execução contínua. Onde 't' é o tempo de simulação atual.
- stop(): Rotina de encerramento para salvar logs, gráficos ou parar motores.
"""

class algo(BaseApp):
    """
    Classe principal da simulação algo.
    Gerencia o ciclo de vida da aplicação, integrando a lógica de controle com a cena do CoppeliaSim.
    """
    def __init__(self):
        """Inicializa os parâmetros base da aplicação, definindo a cena e o tempo total de simulação."""
        super().__init__(scene_file="teste.ttt", sim_name="algo", sim_time=12)
        
        # Uma lista para guardar quantos robôs o usuário criar
        self.robots = [] 

    def setup(self):
        """Configura os recursos iniciais da simulação (instanciação de robôs, sensores e controladores)."""
        try:
            self.logger.info("Configuring Robot, Sensor and Controllers...")

            # 1. Instanciar os Robôs
            # meu_robo = Robotino(bridge=self.bridge, robot_name="robotino")
            # self.robots.append(meu_robo) # Registra o robô para o handshake automático
            
            # 2. Instanciar sensores
            # lidar = HokuyoSensorSim(self.bridge, f"/{{meu_robo.robot_name}}/fastHokuyo", True)
            # meu_robo.add_sensor("LIDAR", lidar)

            # 3. Faz o handshake com todos os robôs registrados
            self.handshake()

        except Exception as e:
            self.logger.error(f"Error detected in setup()!\nTraceback:\n{traceback.format_exc()}")
            
    def post_start(self):
        """Executado uma única vez após o startSimulation(). Ideal para leituras iniciais."""
        try:
            # Exemplo: pos = self.robots[0].pose
            return super().post_start()
        except Exception as e:
            self.logger.error(f"Error detected in post_start()!\nTraceback:\n{traceback.format_exc()}")

    def loop(self, t,actual_state=None):
        """
        Núcleo de execução contínua. 
        Implemente aqui a lógica de controle principal, leitura de sensores e atualização de atuadores.
        """
        try:
            # Adicione a lógica do loop aqui
            pass 
        except Exception as e:
            self.logger.error(f"Error detected in loop()!\nTraceback:\n{traceback.format_exc()}")
            
    def handshake(self):
        """Coleta caminhos de monitoramento de todos os robôs registrados e inicia a comunicação ZMQ."""
        try: 
            monitor_paths = []
            actuator_paths = []
            
            for robot in self.robots:
                monitor_paths.extend(robot.get_monitor_paths())
                actuator_paths.extend(robot.get_actuator_paths())
                
            self.bridge.initialize(monitor_paths, actuator_paths, self.sim)
            self.logger.info("Handshake with CoppeliaSim: OK!") 
        except Exception as e:
            self.logger.error(f"Error in Handshake with CoppeliaSim!\nTraceback:\n{traceback.format_exc()}") 

    def stop(self):
        """Rotina de encerramento para garantir a parada segura dos componentes e exportação de resultados."""
        try:
            # for robot in self.robots:
            #     robot.stop()
            pass
        except Exception as e:
            self.logger.error(f"Error detected in stop()!\nTraceback:\n{traceback.format_exc()}")

def app():
    """
    Ponto de entrada principal da simulação. 
    Instancia a classe e inicia o ciclo de vida (run) para integração com o gerenciador BRAINBYTE.
    """
    aplicacao = algo()
    aplicacao.run()