import numpy as np
from brainbyte.robots.base.base_bot import *

class Robotino(BaseBot):
    """Controle específico para o robô "robotino" omnidirecional de 3 rodas."""

    def __init__(self, bridge, 
                 robot_name='robotino',
                 w0_handle = 'wheel0_joint',
                 w1_handle = 'wheel1_joint',
                 w2_handle = 'wheel2_joint'):
        super().__init__(bridge, robot_name)
        

        # ESTADOS INTERNOS PROTEGIDOS (Usarão getters/setters)
        self._robot_vel = np.zeros(3)     # [vx, vy, omega] local
        self._wheel_vels = np.zeros(3)    # [w0, w1, w2] rad/s das rodas
        

        # PARÂMETROS FÍSICOS
        self._L = 0.135   # Distância do centro do robô até a roda (m)
        self._R = 0.04    # Raio da roda (m)
        self.H = np.zeros((3, 3))      # Matriz de Cinemática Inversa
        self.H_inv = np.zeros((3, 3))  # Matriz de Cinemática Direta
        
        self._update_kinematic_matrices() # Monta as matrizes na inicialização

        # CONFIGURAÇÃO DE JUNTAS / HANDLES NO SIMULADOR
        self.w0_handle = w0_handle
        self.w1_handle = w1_handle
        self.w2_handle = w2_handle # Corrigido de 'wheel3_joint' para manter o padrão 0, 1, 2

        # Mapeando as juntas no simulador (CoppeliaSim/V-REP)
        self.joints = {
            'wheel0': f"/{self.robot_name}/{self.w0_handle}",
            'wheel1': f"/{self.robot_name}/{self.w1_handle}",
            'wheel2': f"/{self.robot_name}/{self.w2_handle}"
        }

    # Inicialização de BATCH
    def get_actuator_paths(self):
        """
        Sobrescreve o método da BaseBot para incluir as rodas do Robotino no Handshake.
        """
        # Pega o caminho raiz do robô + os caminhos das 3 rodas
        paths = [self.robot_path]
        paths.extend(self.joints.values())
        return paths
    

    #PROPRIEDADES INTEGRADAS
    @property
    def wheel_velocities(self):
        """Retorna as velocidades atuais das rodas [w0, w1, w2] em rad/s."""
        return self._wheel_vels

    @property
    def robot_velocity(self):
        """Retorna as velocidades locais do chassi [vx, vy, omega]."""
        return self._robot_vel

    @property
    def dimensions(self):
        """Retorna as dimensões (L, R) do robô."""
        return self._L, self._R

    @dimensions.setter
    def dimensions(self, values):
        """
        Atualiza as dimensões físicas do robô e recalcula as matrizes.
        Exemplo de uso: robo.dimensions = (0.135, 0.04)
        """
        L, R = values
        self._L = L
        self._R = R
        self._update_kinematic_matrices()

    
    # CÁLCULOS CINEMÁTICOS
    def _update_kinematic_matrices(self):
        """
        Atualiza as matrizes de cinemática baseando-se nos ângulos específicos do Robotino.
        A matriz abaixo reflete a disposição geométrica informada no seu código original.
        """
        c = np.cos(np.deg2rad(30))
        s = np.sin(np.deg2rad(30))
        
        # Matriz de Cinemática Inversa (H)
        self.H = (1.0 / self._R) * np.array([
            [-c,  s, self._L],
            [ 0, -1, self._L],
            [ c,  s, self._L]
        ])
        
        # Matriz de Cinemática Direta (H_inv)
        self.H_inv = np.linalg.pinv(self.H)

    def set_velocity_rot(self, linear_vel, angular_vel):
        """
        Cinemática Inversa: Aplica as velocidades desejadas do robô para as rodas no simulador.

        :param linear_vel: Lista ou array [vx, vy] com as velocidades lineares desejadas (m/s).
        :param angular_vel: Escalar com a velocidade angular desejada (rad/s).
        """
        vx, vy = linear_vel
        omega = angular_vel
        self._robot_vel = np.array([vx, vy, omega])

        # Multiplicação de matrizes correta usando '@' (ou .dot())
        V_chassi = np.array([vx, vy, omega])
        self._wheel_vels = self.H @ V_chassi

        # Enfileira as velocidades no "carrinho de compras" da Bridge
        self.bridge.queue_command('velocities', self.joints['wheel0'], float(self._wheel_vels[0]))
        self.bridge.queue_command('velocities', self.joints['wheel1'], float(self._wheel_vels[1]))
        self.bridge.queue_command('velocities', self.joints['wheel2'], float(self._wheel_vels[2]))

    def direct_cin(self, w0, w1, w2):
        """
        MODO 2 (Piloto de Baixo Nível): Controle pelas rodas usando Cinemática Direta.
        Você fornece a velocidade de cada roda diretamente e ele atualiza o simulador
        e o estado interno do chassi.
        
        :param w0, w1, w2: Velocidades angulares das rodas (rad/s).
        :return: Array [vx, vy, omega] representando a velocidade resultante do robô.
        """
        # 1. Salva as velocidades angulares
        self._wheel_vels = np.array([float(w0), float(w1), float(w2)])
        
        # Enfileira as velocidades 
        self.bridge.queue_command('velocities', self.joints['wheel0'], float(self._wheel_vels[0]))
        self.bridge.queue_command('velocities', self.joints['wheel1'], float(self._wheel_vels[1]))
        self.bridge.queue_command('velocities', self.joints['wheel2'], float(self._wheel_vels[2]))
        
        # Atualiza o estado interno (Direta) multiplicando a pseudo-inversa pelas velocidades das rodas
        self._robot_vel = self.H_inv @ self._wheel_vels
        
        return self._robot_vel
    
    
    # CONTROLES GERAIS
    def set_wheels_handles(self, w0='wheel0_joint', w1='wheel1_joint', w2='wheel2_joint'):
        """Atualiza o nome dos identificadores gerando novas strings absolutas."""
        self.w0_handle = w0
        self.w1_handle = w1
        self.w2_handle = w2 
        
        self.joints['wheel0'] = f'/{self.robot_name}/{self.w0_handle}'
        self.joints['wheel1'] = f'/{self.robot_name}/{self.w1_handle}'
        self.joints['wheel2'] = f'/{self.robot_name}/{self.w2_handle}'
        
    def set_velocities(self, w0, w1, w2):
        """
        Alias para direct_cin: Seta as velocidades angulares das rodas.
        Mantido apenas por retrocompatibilidade caso outros scripts já o utilizem.
        """
        return self.direct_cin(w0, w1, w2)

    def stop(self):
        """Zera a velocidade de todos os motores, parando o robô."""
        super().stop()
        try:
            self.direct_cin(0.0, 0.0, 0.0)
        except Exception as e:
            print(f"[Robotino] Erro ao parar motores: {e}")