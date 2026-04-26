import numpy as np
from brainbyte.robots.base.base_bot import BaseBot

class TurtleBot(BaseBot):
    """Controle específico para robô diferencial Turtle Bot."""

    def __init__(self, bridge, 
                 robot_name = 'Turtlebot3', 
                 left_motor = 'left_Motor', 
                 right_motor= 'right_Motor',
                 base_link  = 'base_link'
                 ):
        super().__init__(bridge, robot_name)
        
        #New handle 
        self.robot_path = f'/{robot_name}/{base_link}'
    
        # ESTADOS INTERNO
        self._robot_vel = np.zeros(2)     # [v, omega] (linear em X, angular em Z)
        self._wheel_vels = np.zeros(2)    # [wl, wr] em rad/s (roda esquerda, roda direita)
        
        
        # PARÂMETROS FÍSICOS
        self._R = 0.0066                             # Raio da roda (m)
        self._L = 0.287                              # Distância entre as rodas (Wheelbase) (m)
        self._mass = 1.8                             # em kg
        self._wheel_and_motor_mass = 0.111
        self._chassi_moment_of_inertia = 1.46e-2     # Momento de inertia do chassi
        self._wheel_mof_about_the_diameter = 1.12e-5 #
        self._wheel_mof_about_the_axis= 2.07e-5

        
        # Matrizes de cinemática
        self.H = np.zeros((2, 2))      # Matriz de Cinemática Inversa
        self.H_inv = np.zeros((2, 2))  # Matriz de Cinemática Direta
        self._update_kinematic_matrices()

        
        # CONFIGURAÇÃO DE JUNTAS / HANDLES NO SIMULADOR
        self.path_left = left_motor if left_motor.startswith(('/', '.')) else f'/{robot_name}/{left_motor}'
        self.path_right = right_motor if right_motor.startswith(('/', '.')) else f'/{robot_name}/{right_motor}'

        self.joints = {
            'left_wheel': self.path_left,
            'right_wheel': self.path_right
        }

    
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
    def inertial_dimensions(self):
        """Retorna (Wheelbase, Raio da Roda)."""
        return self._mass, self._chassi_moment_of_inertia

    @inertial_dimensions.setter
    def inertial_dimensions(self, values):
        """Atualiza dimensões e recalcula as matrizes cinemáticas automaticamente."""
        self._mass, self._chassi_moment_of_inertia = values
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

        # Envia comados para o buffer da Bridge
        self.bridge.queue_command('velocities', self.joints['left_wheel'], float(self._wheel_vels[0]))
        self.bridge.queue_command('velocities', self.joints['right_wheel'], float(self._wheel_vels[1]))
    
    def direct_cin(self, wl, wr):
        """
        MODO 2 (Piloto de Baixo Nível): Controle pelas rodas usando Cinemática Direta.
        Você fornece a velocidade de cada roda (rad/s) diretamente.
        """
        self._wheel_vels = np.array([wl, wr])
        
        # ENFILEIRA NO BUFFER DA BRIDGE
        self.bridge.queue_command('velocities', self.joints['left_wheel'], float(self._wheel_vels[0]))
        self.bridge.queue_command('velocities', self.joints['right_wheel'], float(self._wheel_vels[1]))

        # Atualiza o estado interno do robô (Direta) para sabermos a que velocidade o chassi está indo
        self._robot_vel = self.H_inv @ self._wheel_vels
        
        return self._robot_vel

    
    # CONTROLES GERAIS
    def stop(self):
        """Para as rodas."""
        super().stop()
        try: 
            self.set_wheel_velocity(0.0, 0.0)
        except Exception as e:
            print(f"[TurtleBot] Erro ao parar motores: {e}")