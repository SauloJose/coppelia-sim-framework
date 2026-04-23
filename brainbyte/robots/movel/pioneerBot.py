import numpy as np
from brainbyte.robots.base.base_bot import BaseBot

class PioneerBot(BaseBot):
    """Controle específico para robô diferencial Pioneer P3DX."""

    def __init__(self, sim, 
                 robot_name='PioneerP3DX', 
                 left_motor='leftMotor', 
                 right_motor='rightMotor'
                 ):
        super().__init__(sim, robot_name)

        
        # ESTADOS INTERNOS
        
        self._robot_vel = np.zeros(2)     # [v, omega] (linear em X, angular em Z)
        self._wheel_vels = np.zeros(2)    # [wl, wr] em rad/s (roda esquerda, roda direita)
        
        
        # PARÂMETROS FÍSICOS
        
        self._R = 0.0975  # Raio da roda (m)
        self._L = 0.381   # Distância entre as rodas (Wheelbase) (m)
        
        self.H = np.zeros((2, 2))      # Matriz de Cinemática Inversa
        self.H_inv = np.zeros((2, 2))  # Matriz de Cinemática Direta
        
        self._update_kinematic_matrices()

        
        # CONFIGURAÇÃO DE JUNTAS / HANDLES NO SIMULADOR
        
        # Lógica inteligente para o caminho:
        # Se começar com '/' ou './', o usuário passou o caminho exato. Usa direto.
        # Caso contrário, assume que é filho direto do robô e monta a string padrão.
        path_left = left_motor if left_motor.startswith(('/', '.')) else f'/{robot_name}/{left_motor}'
        path_right = right_motor if right_motor.startswith(('/', '.')) else f'/{robot_name}/{right_motor}'

        self.joints = {
            'left_wheel': self.sim.getObject(path_left),
            'right_wheel': self.sim.getObject(path_right)
        }

        # Verificação de handles
        for name, handle in self.joints.items():
            if handle == -1:
                raise ValueError(f"Junta '{name}' não encontrada no robô '{robot_name}'. Caminho buscado: {self.joints}")
            
    
    # PROPRIEDADES INTEGRADAS AO COPPELIASIM
    @property
    def dimensions(self):
        """Retorna (Wheelbase, Raio da Roda)."""
        return self._L, self._R

    @dimensions.setter
    def dimensions(self, values):
        """Atualiza dimensões e recalcula as matrizes cinemáticas automaticamente."""
        self._L, self._R = values
        self._update_kinematic_matrices()

    @property
    def wheel_velocities(self):
        """Retorna as velocidades atuais exigidas nas rodas [wl, wr]."""
        return self._wheel_vels

    @property
    def robot_velocity(self):
        """Retorna as velocidades locais do chassi [v, omega]."""
        return self._robot_vel

    
    # CÁLCULOS CINEMÁTICOS (MATRIZES)
    def _update_kinematic_matrices(self):
        """
        Constrói as matrizes de cinemática baseadas na modelagem diferencial.
        H mapeia [v, omega] para [wl, wr].
        H_inv mapeia [wl, wr] para [v, omega].
        """
        r = self._R
        L = self._L
        
        # Cinemática Inversa
        # wl = v/r - (omega * L) / (2r)
        # wr = v/r + (omega * L) / (2r)
        self.H = np.array([
            [1.0 / r, -L / (2.0 * r)],
            [1.0 / r,  L / (2.0 * r)]
        ])
        
        # Cinemática Direta
        # v = (r/2)*wl + (r/2)*wr
        # omega = -(r/L)*wl + (r/L)*wr
        self.H_inv = np.array([
            [ r / 2.0,  r / 2.0],
            [-r / L,    r / L  ]
        ])

    def set_wheel_velocity(self, linear_vel, angular_vel):
        """
        Cinemática Inversa: Define velocidades linear (m/s) e angular (rad/s) do chassi,
        calcula as velocidades necessárias nas rodas e as aplica no simulador.
        """
        self._robot_vel = np.array([linear_vel, angular_vel])
        
        # Multiplicação matricial: [wl, wr]^T = H @ [v, omega]^T
        self._wheel_vels = self.H @ self._robot_vel

        # Aplica no simulador
        self.sim.setJointTargetVelocity(self.joints['left_wheel'], self._wheel_vels[0])
        self.sim.setJointTargetVelocity(self.joints['right_wheel'], self._wheel_vels[1])

    def direct_cin(self, wl, wr):
        """
        MODO 2 (Piloto de Baixo Nível): Controle pelas rodas usando Cinemática Direta.
        Você fornece a velocidade de cada roda (rad/s) diretamente.
        """
        self._wheel_vels = np.array([wl, wr])
        
        # Aplica a velocidade informada diretamente nos motores do simulador
        self.sim.setJointTargetVelocity(self.joints['left_wheel'], self._wheel_vels[0])
        self.sim.setJointTargetVelocity(self.joints['right_wheel'], self._wheel_vels[1])
        
        # Atualiza o estado interno do robô (Direta) para sabermos a que velocidade o chassi está indo
        self._robot_vel = self.H_inv @ self._wheel_vels
        
        return self._robot_vel

    
    # CONTROLES GERAIS
    
    def stop(self):
        """Para as rodas."""
        self.set_wheel_velocity(0.0, 0.0)