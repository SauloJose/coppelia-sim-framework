from brainbyte import BaseApp
from brainbyte.robots.movel.Robotino import *



class RobotinoSimu(BaseApp):
    """
        Teste de locomoção de obstáculos do robotino
    """
    def __init__(self):
        """ Inicialização da aplicação"""
        super().__init__(scene_file="robotino.ttt",sim_name="robotino_exemple",sim_time=60)

    def setup(self):
        """Configura os recurso da simulação"""
        self.logger.info("Configurando o robô e os sensores")

        # Instancio o robô para abstrair comandos
        self.robot = Robotino(sim=self.sim, 
                            robot_name='robotino')
        
        pos = self.robot.pose
        self.logger.info(f'Initial robot position: x={pos[0]:.2f}, y={pos[1]:.2f}')

    def post_start(self):
        """ É executado logo quando inicia a simulação"""
        return super().post_start()
    
    def loop(self, t):
        """ Etapas do loop"""
        try:
            #Lógica de enviar velocidades
            sim_t = self.sim.getSimulationTime()
            if sim_t <= 10:
                self.robot.set_velocity_rot([5,5], 0)
            elif sim_t <= 30:
                self.robot.direct_cin(10,-10,10)
            else:
                self.robot.direct_cin(10,-10,-10)

            self.logger.debug(f"Velocidades da roda {self.robot._wheel_vels}")
        except Exception as e:
            self.logger.error(f"Erro: {e}")

    def stop(self):
        """ Executado após a simulação terminar - parada segura"""
        try:
            self.robot.stop()
        except Exception as e:
            self.logger.error(f"Erro detectado: {e}")


def app():
    """
        Ponto de entrada para a simulação em main.py
    """
    aplicacao = RobotinoSimu()
    aplicacao.run()
