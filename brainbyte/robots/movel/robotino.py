import numpy as np
from brainbyte.robots.base.base_bot import *

class Robotino(BaseBot):
    """Controle específico para o robô "robotino" omnidirecional de 3 rodas."""

    def __init__(self, sim, robot_name='robotino'):
        super().__init__(sim, robot_name)
        
        # ==========================================
        # ESTADOS INTERNOS PROTEGIDOS (Usarão getters/setters)
        # ==========================================
        self._robot_vel = np.zeros(3)     # [vx, vy, omega] local
        self._wheel_vels = np.zeros(3)    # [w0, w1, w2] rad/s das rodas
        
        # ==========================================
        # PARÂMETROS FÍSICOS
        # ==========================================
        self._L = 0.135   # Distância do centro do robô até a roda (m)
        self._R = 0.04    # Raio da roda (m)
        self.H = np.zeros((3, 3))      # Matriz de Cinemática Inversa
        self.H_inv = np.zeros((3, 3))  # Matriz de Cinemática Direta
        
        self._update_kinematic_matrices() # Monta as matrizes na inicialização

        # ==========================================
        # CONFIGURAÇÃO DE JUNTAS / HANDLES NO SIMULADOR
        # ==========================================
        self.w0_handle = 'wheel0_joint'
        self.w1_handle = 'wheel1_joint'
        self.w2_handle = 'wheel2_joint' # Corrigido de 'wheel3_joint' para manter o padrão 0, 1, 2

        # Mapeando as juntas no simulador (CoppeliaSim/V-REP)
        self.joints = {
            'wheel0': self.sim.getObject(f'/{self.robot_name}/{self.w0_handle}'),
            'wheel1': self.sim.getObject(f'/{self.robot_name}/{self.w1_handle}'),
            'wheel2': self.sim.getObject(f'/{self.robot_name}/{self.w2_handle}')
        }

        # Verificação de handles válidos
        for name, handle in self.joints.items():
            if handle == -1:
                raise ValueError(f"Junta '{name}' não encontrada no robô '{robot_name}'. Verifique a hierarquia no simulador.")

    # ==========================================
    # PROPRIEDADES
    # ==========================================
    @property
    def pose(self):
        """
        Retorna a pose real do robô [x, y, theta] diretamente do simulador.
        Utiliza o método get_pose() herdado da classe BaseBot.
        """
        # pos = [x, y, z] e ori = [alpha, beta, gamma]
        pos, ori = self.get_pose() 
        
        # Para um robô planar, queremos X, Y e a rotação no eixo Z (gamma)
        x, y = pos[0], pos[1]
        theta = ori[2] 
        
        return np.array([x, y, theta])

    @pose.setter
    def pose(self, nova_pose):
        """
        Teleporta o robô para uma nova pose [x, y, theta] no CoppeliaSim.
        """
        if len(nova_pose) != 3:
            raise ValueError("A pose deve conter 3 elementos: [x, y, theta].")
        
        x, y, theta = nova_pose
        
        # Pegamos a pose atual para não alterar a altura (Z) 
        # e as rotações de inclinação (alpha, beta) acidentalmente.
        pos_atual, ori_atual = self.get_pose()
        
        z = pos_atual[2]
        alpha = ori_atual[0]
        beta = ori_atual[1]
        
        # Envia os comandos diretamente para a API do CoppeliaSim
        # (Assumindo a sintaxe padrão da API ZeroMQ/B0 do CoppeliaSim)
        self.sim.setObjectPosition(self.robot_handle, self.sim.handle_world, [x, y, z])
        self.sim.setObjectOrientation(self.robot_handle, self.sim.handle_world, [alpha, beta, theta])

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

    # ==========================================
    # CÁLCULOS CINEMÁTICOS
    # ==========================================
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

        # Setando velocidades nas juntas do simulador
        self.sim.setJointTargetVelocity(self.joints['wheel0'], self._wheel_vels[0])
        self.sim.setJointTargetVelocity(self.joints['wheel1'], self._wheel_vels[1])
        self.sim.setJointTargetVelocity(self.joints['wheel2'], self._wheel_vels[2])

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
        
        # 2. Converte para lista nativa do Python (Evita erro CBOR do CoppeliaSim)
        vels_nativas = self._wheel_vels.tolist()

        # Aplica a velocidade informada diretamente nos motores do simulador
        self.sim.setJointTargetVelocity(self.joints['wheel0'], vels_nativas[0])
        self.sim.setJointTargetVelocity(self.joints['wheel1'], vels_nativas[1])
        self.sim.setJointTargetVelocity(self.joints['wheel2'], vels_nativas[2])
        
        # Atualiza o estado interno (Direta) multiplicando a pseudo-inversa pelas velocidades das rodas
        self._robot_vel = self.H_inv @ self._wheel_vels
        
        return self._robot_vel
    
    # ==========================================
    # CONTROLES GERAIS
    # ==========================================
    def set_wheels_handles(self, w0='wheel0_joint', w1='wheel1_joint', w2='wheel2_joint'):
        """Atualiza o nome dos handles e remapeia as juntas."""
        self.w0_handle = w0
        self.w1_handle = w1
        self.w2_handle = w2 
        
        self.joints['wheel0'] = self.sim.getObject(f'/{self.robot_name}/{self.w0_handle}')
        self.joints['wheel1'] = self.sim.getObject(f'/{self.robot_name}/{self.w1_handle}')
        self.joints['wheel2'] = self.sim.getObject(f'/{self.robot_name}/{self.w2_handle}')

    def set_velocities(self, w0, w1, w2):
        """
        Alias para direct_cin: Seta as velocidades angulares das rodas.
        Mantido apenas por retrocompatibilidade caso outros scripts já o utilizem.
        """
        return self.direct_cin(w0, w1, w2)

    def stop(self):
        """Zera a velocidade de todos os motores, parando o robô."""
        self.direct_cin(0.0, 0.0, 0.0)