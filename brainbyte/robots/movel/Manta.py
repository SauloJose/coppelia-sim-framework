from brainbyte.robots.base.base_bot import *
from typing import Optional, Tuple, Union

class Manta(BaseBot):
    """Carro Ackerman"""

    def __init__(
        self,
        bridge,
        robot_name: str = "Manta",
        steer_name: str = "steer_joint",
        motor_name: str = "motor_joint",
        max_steer: float = np.deg2rad(10.0),
        max_velocity: float =20
    ):
        """
        Inicializa o robô e obtém os handles das juntas.

        Parâmetros
        ----------
        bridge : objeto para comunicar com o simulador
            Interface com o simulador.
        robot_name : str, opcional
            Nome do robô na cena, por padrão "Manta".
        steer_name : str, opcional
            Nome da junta de direção, por padrão "steer_joint".
        motor_name : str, opcional
            Nome da junta do motor, por padrão "motor_joint".
        max_steer : float, opcional
            Ângulo máximo de esterçamento em radianos, padrão 10 graus.
        max_velocity: float, opicional
            Velocidade máxima que o motor irá ter, por padrão 20 m/s.
        """

        super().__init__(bridge, robot_name) #Aqui já foi criado o handle e o nome

        # Mapeamento dos caminhos (Strings absolutas em vez de Handles numéricos)
        self.path_steer = steer_name if steer_name.startswith(('/', '.')) else f"/{self.robot_name}/{steer_name}"
        self.path_motor = motor_name if motor_name.startswith(('/', '.')) else f"/{self.robot_name}/{motor_name}"
        
        #Configuração do robô
        self.motor_torque = 60
        self.motor_velocity = 0
        self.steer = 0 

        # Limitadores
        self.max_steer = max_steer 
        self.max_velocity = max_velocity

        self.set_torque(60)
    # ==========================================
    # HANDSHAKE (INICIALIZAÇÃO BATCH)
    # ==========================================
    def get_actuator_paths(self):
        """Informa à Bridge quais juntas o Lua Script deve armazenar no cache."""
        paths = super().get_actuator_paths()
        paths.extend([self.path_steer, self.path_motor])
        return paths

    def get_monitor_paths(self):
        """
        Adiciona sufixos especiais para o Lua Script saber que queremos ler 
        a posição do volante e a velocidade do motor.
        """
        paths = super().get_monitor_paths()
        paths.extend([f"{self.path_steer}_jointpos", f"{self.path_motor}_jointvel"])
        return paths
    
    # CONTROLE E ATUAÇÃO
    def set_torque(self, torque):
        self.motor_torque = torque
        self.bridge.queue_command('forces', self.path_motor, self.motor_torque)

    # Propriedades
    @property
    def current_steer(self) -> float:
        """Ângulo atual da junta de direção (rad)."""
        val = self.bridge.get_sensor_data(f"{self.path_steer}_jointpos")
        if val is not None:
            self.steer = float(val)
        return self.steer

    @property
    def current_velocity(self) -> float:
        """Velocidade atual da junta do motor (unidade do simulador)."""
        val = self.bridge.get_sensor_data(f"{self.path_motor}_jointvel")
        if val is not None:
            self.motor_velocity = float(val)
        return self.motor_velocity

    # Métodos de controle básico
    def set_velocity(self, velocity: float, steer: float) -> None:
        # Forçando a conversão para float nativo do Python (Evita bug do CBOR)
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))
        velocity = float(np.clip(velocity, -self.max_velocity, self.max_velocity))

        self.motor_velocity = velocity
        self.steer = steer

        # Agrupa os comandos por categoria
        self.bridge.queue_command('positions', self.path_steer, steer)
        self.bridge.queue_command('velocities', self.path_motor, velocity)

    def set_steer(self, steer: float) -> None:
        """
        Controla apenas o ângulo da direção (mantém a velocidade atual).

        Parâmetros
        ----------
        steer : float
            Ângulo de esterçamento (limitado a ±self.max_steer).
        """
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))
        self.steer = steer 
        self.bridge.queue_command('positions', self.path_steer, steer)

    def set_motor_velocity(self, velocity:float) -> None:
        """
        Controla apenas a velocidade do motor (mantém o ângulo atual).

        Parâmetros
        ----------
        velocity : float
            Velocidade desejada (limitada a ±self.max_velocity).
        """
        velocity = float(np.clip(velocity, -self.max_velocity, self.max_velocity))
        self.motor_velocity = velocity
        self.bridge.queue_command('velocities', self.path_motor, velocity)

    def stop(self) -> None:
        """Para o motor e centraliza a direção."""
        self.sim.setJointTargetVelocity(self._motor_handle, 0.0)
        self.sim.setJointTargetPosition(self._steer_handle, 0.0)

    def __repr__(self) -> str:
        return (
            f"{self.__class__.__name__}(robot_name='{self.robot_name}', "
            f"max_velocity={self.max_velocity}, max_steer={np.rad2deg(self.max_steer):.1f}°)"
        )