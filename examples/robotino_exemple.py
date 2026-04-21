from brainbyte import BaseApp
from brainbyte.robots.movel.robotino import *



class RobotinoSimu(BaseApp):
    """
        Teste de locomoção de obstáculos do robotino
    """
    def __init__(self):
        """ Inicialização da aplicação"""
        super().__init__(scene_file="robotino.ttt",sim_time=60)

    def setup(self):
        """Configura os recurso da simulação"""
        self.robotino = Robotino(sim=self.sim, robot_name='robotino')

    def post_start(self):
        """ É executado logo quando inicia a simulação"""
        return super().post_start()
    
    def loop(self, t):
        """ Etapas do loop"""
        try:
            i = 1
        except Exception as e:
            self.logger.error(f"Erro: {e}")

    def stop(self):
        """ Executado após a simulação terminar - parada segura"""
        try:
            alg = 0 
        except Exception as e:
            self.logger.error(f"Erro detectado: {e}")


def app():
    """
        Ponto de entrada para a simulação em main.py
    """
    aplicacao = RobotinoSimu()
    aplicacao.run()
