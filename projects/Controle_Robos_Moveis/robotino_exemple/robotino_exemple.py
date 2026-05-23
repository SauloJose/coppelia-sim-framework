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
        """Configura os recurso da simulação"""
        self.logger.info("Configurando o robô e os sensores")

        # Instancio o robô para abstrair comandos
        self.robot = Robotino(bridge=self.bridge, 
                            robot_name='robotino')
        
        position = self.robot.pose #(x,y,theta) -> Em versões futuras vou ajustar
        
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

        self.control = OmnidirectionalController(pos_init=position,
                                                set_point=self.list_pos[0],
                                                k_x=0.8,
                                                k_y=-0.1,
                                                k_theta=0.3,
                                                dt = self.dt)  
        
        self.control.set_max_values(v_max = self.robot._v_max, 
                                    w_max = self.robot._w_max)
        
        self.robot.add_control(control_name='AUTO_DIFF',
                                control_instance=self.control)
            

        # PEGA OS CAMINHOS DO PRÓPRIO ROBÔ (Chassi e Motores)
        monitor_paths = self.robot.get_monitor_paths()
        actuator_paths = self.robot.get_actuator_paths()
    
        # Envia a lista de inicialização para a bridge
        self.bridge.initialize(monitor_paths, actuator_paths,self.sim)
        self.logger.info("Handshake com a Bridge concluído!")

    def post_start(self):
        """ É executado logo quando inicia a simulação"""
        super().post_start()
        
        # A leitura da pose deve ficar aqui, após o cache da bridge ser preenchido!
        pos = self.robot.pose
        self.logger.info(f'Initial robot position: x={pos[0]:.2f}, y={pos[1]:.2f}')
    
    def loop(self, t,actual_state=None):
        try:
            actual_pos = self.robot.pose 


            #v_cmd, w_cmd = self.robot.get_control('AUTO_DIFF').get_control(actual_point=actual_pos)


            self.robot.set_velocity_rot(linear_vel=[0,0.5], angular_vel=0)

            
        except Exception as e:
            self.logger.error(f"Erro detectado: {e}")

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
