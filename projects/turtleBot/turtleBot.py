from brainbyte import BaseApp
from brainbyte.robots.movel.TurtleBot import *
from brainbyte.control.control import * # Classe principal    


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
        self.logger.info("Configurando o robô e os sensores")

        # Exemplo de adicionar um robô
        self.robot = TurtleBot(sim=self.sim,
                               robot_name='Turtlebot3', 
                               left_motor='left_motor', 
                               right_motor='right_motor'
                              )
        
        self.dt = self.d_time()
        pos = self.robot.pose

        # CHAVES DUPLAS AQUI:
        self.logger.info(f'Initial robot position: x={pos[0]:.2f}, y={pos[1]:.2f}')
        
        
        # Defina um controlador para adicionar aqui
        #self.robot.add_control(control_name="PID",
                               #control_instance=self.PID) 

    def post_start(self):
        """ É executado logo quando inicia a simulação"""
        return super().post_start()
    
    def loop(self, t):
        """ Etapas do loop"""
        try:
            self.robot.set_wheel_velocity(linear_vel=0.2, angular_vel=np.deg2rad(20))
        except Exception as e:
            # CHAVES DUPLAS AQUI:
            self.logger.error(f"Erro: {e}")

    def stop(self):
        """ Executado após a simulação terminar - parada segura"""
        try:
            self.robot.stop()
        except Exception as e:
            # CHAVES DUPLAS AQUI:
            self.logger.error(f"Erro detectado: {e}")

def app():
    """
        Ponto de entrada para a simulação em main.py
    """
    aplicacao = turtleBot()
    aplicacao.run()