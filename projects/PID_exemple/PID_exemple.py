from brainbyte import BaseApp
from brainbyte.robots.movel.Manta import Manta
from brainbyte.control.control import * 


# Classe principal    
class PIDSimu(BaseApp):
    """
        Teste de locomoção de obstáculos do robotino
    """
    def __init__(self):
        """ Inicialização da aplicação"""
        super().__init__(scene_file="PIDtest.ttt",sim_name="PID_exemple",sim_time=60)

    def setup(self):
        """Configura os recurso da simulação"""
        self.logger.info("Configurando o robô e os sensores")
        self.robot = Manta(sim=self.sim,
                           robot_name="Manta",
                           steer_name="steer_joint",
                           motor_name="motor_joint",
                           max_steer=np.deg2rad(10),
                           max_velocity=20)
        
        self.dt = self.d_time()
        pos = self.robot.pose
        self.logger.info(f'Initial robot position: x={pos[0]:.2f}, y={pos[1]:.2f}')
        
        
        #Definido para teste um controlador onoff
        self.OnOff = On_Off_Controller(var=1, 
                               set_point=0, 
                               u_max=0.17,      # ~10 graus em radianos para a direita
                               u_min=-0.17,     # ~10 graus em radianos para a esquerda
                               hysteresis=0.05) # Só muda a roda se o erro passar de 5cm
        
        #Definindo o PID 
        self.PID = PID_Controller(var=1,
                                  kp=4,
                                  kd=3,
                                  ki=0.1,
                                  set_point=0,
                                  dt =self.dt)
        
        self.robot.add_control(control_name="PID",
                               control_instance=self.PID) 


    def post_start(self):
        """ É executado logo quando inicia a simulação"""
        return super().post_start()
    
    def loop(self, t):
        """ Etapas do loop"""
        try:
            estado_atual = self.robot.pose 

            y_atual = estado_atual[1]

            u = self.robot.get_control("PID").run(y_atual)

            self.robot.set_velocity(velocity=30, steer = u)
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
    aplicacao = PIDSimu()
    aplicacao.run()
