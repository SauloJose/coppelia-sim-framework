from brainbyte import BaseApp
from brainbyte.robots.movel.Manta import Manta
from brainbyte.control.automatic import * 


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
        self.robot = Manta(bridge=self.bridge,
                           robot_name="Manta",
                           steer_name="steer_joint",
                           motor_name="motor_joint",
                           max_steer=np.deg2rad(10),
                           max_velocity=20)
        
        # AJUSTE 2: Handshake (Informa à Bridge quais dados manter em cache)
        monitor_paths = self.robot.get_monitor_paths()
        actuator_paths = self.robot.get_actuator_paths()
        self.bridge.initialize(monitor_paths, actuator_paths,self.sim)
        self.logger.info("Handshake com o CoppeliaSim concluído.")

        self.dt = self.d_time()

        # Definido para teste um controlador onoff
        self.OnOff = On_Off_Controller(var=1, 
                                       set_point=0, 
                                       u_max=0.17,      # ~10 graus em radianos para a direita
                                       u_min=-0.17,     # ~10 graus em radianos para a esquerda
                                       hysteresis=0.05) # Só muda a roda se o erro passar de 5cm
        
        # Definindo o PID 
        self.PID = PID_Controller(var=1,
                                  kp=4,
                                  kd=2,
                                  ki=0.2,
                                  set_point=-1,
                                  dt=self.dt)
        
        self.robot.add_control(control_name="PID",
                               control_instance=self.PID)


    def post_start(self):
        """ Executado logo após o primeiro frame da simulação """
        super().post_start()
        
        # AJUSTE 3: Lemos a pose inicial aqui, pois agora o cache da ponte está preenchido!
        pos = self.robot.pose
        self.logger.info(f'Initial robot position: x={pos[0]:.2f}, y={pos[1]:.2f}')

    
    def loop(self, t):
        """ Etapas do loop"""
        try:
            estado_atual = self.robot.pose 

            y_atual = estado_atual[1]

            u = self.robot.get_control("PID").run(y_atual)

            self.robot.set_velocity(velocity=10, steer = u)
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
