from brainbyte import BaseApp
from brainbyte.robots.movel.Robotino import *
from brainbyte.control.automatic import *


class RobotinoSimu(BaseApp):
    """
        Teste de locomoção de obstáculos do robotino
    """
    def __init__(self):
        """ Inicialização da aplicação"""
        super().__init__(scene_file="robotino.ttt",sim_name="robotino_exemple",sim_time=60)

    def setup(self):
        """Configura os recursos da simulação"""
        self.logger.info("Configurando o robô e os sensores")

        # Instancio o robô para abstrair comandos
        self.robot = Robotino(bridge=self.bridge, robot_name='robotino')
        
        # Lista de waypoints (x, y, theta)
        self.list_pos = np.array([
            [1.0,  1.0, np.deg2rad(40)],
            [1.0, -1.0, np.deg2rad(90)],
            [-1.0, 1.0, np.deg2rad(45)],
            [-1.0,-1.0, np.deg2rad(50)],
            [1.0,  1.0, np.deg2rad(180)],
            [0.0,  0.0, 0.0]
        ])

        self.it = 0

        self.control = OmnidirectionalController(set_point=self.list_pos[0],
                                                k_x=0.8,
                                                k_y=0.8, 
                                                k_theta=1,
                                                rho_tol=0.05,
                                                theta_tol=np.deg2rad(1))  
        
        self.robot.add_control(control_name='AUTO_OMNI', control_instance=self.control)
            
        monitor_paths = self.robot.get_monitor_paths()
        actuator_paths = self.robot.get_actuator_paths()
    
        # Envia a lista de inicialização para a bridge
        self.bridge.initialize(monitor_paths, actuator_paths, self.sim)
        self.logger.info("Handshake com a Bridge concluído!")

    def post_start(self):
        """ É executado logo quando inicia a simulação"""
        super().post_start()
        
        pos = self.robot.pose
        if pos is not None:
            self.logger.info(f'Posição inicial do robô: x={pos[0]:.2f}, y={pos[1]:.2f}, theta={np.rad2deg(pos[2]):.2f}º')
    
    def loop(self, t, actual_state=None):
        try:
            actual_pos = self.robot.pose 
            if actual_pos is None:
                return # Evita quebrar o loop se a bridge pular um frame

            controller = self.robot.get_control('AUTO_OMNI')
            
            v_cmd, w_cmd = controller.get_control(actual_point=actual_pos)

            if v_cmd[0] == 0.0 and v_cmd[1] == 0.0 and w_cmd == 0.0:
                if self.it < len(self.list_pos) - 1:
                    self.it += 1
                    novo_alvo = self.list_pos[self.it]
                    controller.set_SP(novo_alvo)
                    self.logger.info(f"Waypoint alcançado! Indo para waypoint {self.it}: {novo_alvo}")
                else:
                    pass 

            self.robot.set_velocity_rot(linear_vel=v_cmd, angular_vel=w_cmd)
            
        except Exception as e:
            self.logger.error(f"Erro detectado no loop: {e}")

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
