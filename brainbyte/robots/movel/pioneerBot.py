import numpy as np
from brainbyte.robots.base.base_bot import BaseBot

class PioneerBot(BaseBot):
    """Controlador para robôs móveis de acionamento diferencial (Differential Drive).

    Esta classe abstrai a interface de controle de robôs com tração diferencial no
    simulador CoppeliaSim. Ela encapsula as propriedades físicas do robô (raio da roda,
    distância entre rodas, massa e inércia) e gerencia automaticamente as matrizes
    de transformação cinemática.

    A classe oferece duas abordagens de navegação:
    - Alto Nível (Cinemática Inversa): O usuário comanda as velocidades linear (v) 
      e angular (omega) do chassi, e a classe calcula as velocidades das rodas.
    - Baixo Nível (Cinemática Direta): O usuário comanda as velocidades individuais 
      das rodas (wl, wr), e a classe deduz o impacto na velocidade do chassi.

    Por ser fundamentada no modelo diferencial padrão, sua estrutura é totalmente
    reutilizável para plataformas como TurtleBot, Pioneer P3-DX ou qualquer outro
    robô de duas rodas motrizes independentes.

    Atributos:
        brigde (objeto): Instância da interface de comunicação em batch com o simulador
        dimensions (tuple): Propriedade para acessar/alterar a distância entre rodas e o raio.
        inertial_dimensions (tuple): Propriedade para acessar/alterar a massa e momento de inércia.
        robot_velocity (np.ndarray): Velocidade atual do chassi no formato [v, omega].
        wheel_velocities (np.ndarray): Velocidade atual das rodas no formato [wl, wr].
    """
    def __init__(self, bridge, 
                 robot_name='PioneerP3DX', 
                 left_motor='leftMotor', 
                 right_motor='rightMotor'
                 ):
        super().__init__(bridge, robot_name)

        
        # ESTADOS INTERNOS
        self._robot_vel = np.zeros(2)     # [v, omega] (linear em X, angular em Z)
        self._wheel_vels = np.zeros(2)    # [wl, wr] em rad/s (roda esquerda, roda direita)
        
        
        # PARÂMETROS FÍSICOS
        self._mass = 17   # kg
        self._R = 0.0975  # Raio da roda (m)
        self._L = 0.381   # Distância entre as rodas (Wheelbase) (m)
        
        # LIMITES FÍSICOS 
        self._v_max = 1.6 #m/s
        self._w_max = 5.2 #rad/s

        # Cinemática
        self.H = np.zeros((2, 2))      # Matriz de Cinemática Inversa
        self.H_inv = np.zeros((2, 2))  # Matriz de Cinemática Direta
        
        self._update_kinematic_matrices()

        # CONFIGURAÇÃO DE JUNTAS (Caminhos em String)
        self.path_left = left_motor if left_motor.startswith(('/', '.')) else f'/{robot_name}/{left_motor}'
        self.path_right = right_motor if right_motor.startswith(('/', '.')) else f'/{robot_name}/{right_motor}'

        self.joints = {
            'left_wheel': self.path_left,
            'right_wheel': self.path_right
        }

    # ==========================================
    # HANDSHAKE (INICIALIZAÇÃO BATCH)
    # ==========================================
    def get_actuator_paths(self):
        """
        Informa à Bridge quais motores o Lua Script deve armazenar no cache.
        """
        paths = super().get_actuator_paths()
        paths.extend([self.joints['left_wheel'], self.joints['right_wheel']])
        return paths
    
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
        #Saturação do chassi 
        v_cmd = np.clip(linear_vel, -self._v_max, self._v_max)
        w_cmd = np.clip(angular_vel, -self._w_max, self._w_max)

        # Cinemática inversa (wl, wr)
        wl, wr = self.H @ np.array([v_cmd, w_cmd])
        
        # Respeito os limites das rodas (wheel_max = v_max/r)
        wheel_max = self._v_max / self._R
        max_w = max(abs(wl), abs(wr))
        if max_w > wheel_max:
            scale = wheel_max / max_w
            wl *= scale
            wr *= scale
            v_cmd, w_cmd = self.H_inv @ np.array([wl, wr])

        # Atualiza estado interno 
        self._robot_vel = np.array([v_cmd, w_cmd])
        self._wheel_vels = np.array([wl, wr])

        # Enfileira na Bridge (convertendo para float nativo)
        self.bridge.queue_command('velocities', self.joints['left_wheel'], float(self._wheel_vels[0]))
        self.bridge.queue_command('velocities', self.joints['right_wheel'], float(self._wheel_vels[1]))

    def direct_cin(self, wl, wr):
        """
        Comando direto das rodas com saturação realista.
        """
        wheel_max = self._v_max / self._R

        #  Saturação com fator de escala (preserva relação wl/wr) 
        max_w = max(abs(wl), abs(wr))
        if max_w > wheel_max:
            scale = wheel_max / max_w
            wl *= scale
            wr *= scale

        #  Atualiza estado interno 
        self._wheel_vels = np.array([wl, wr])
        self._robot_vel = self.H_inv @ self._wheel_vels

        #  Envia ao simulador 
        self.bridge.queue_command('velocities', self.joints['left_wheel'], float(wl))
        self.bridge.queue_command('velocities', self.joints['right_wheel'], float(wr))

        return self._robot_vel

    
    # CONTROLES GERAI
    def stop(self):
        """Para as rodas."""
        super().stop()
        try:
            self.set_wheel_velocity(0.0, 0.0)
        except Exception as e:
            print(f"[PionnerBot] Erro ao parar motores: {e}")

    