"""Exemplo: Teste de Trajetória Lissajous com Pioneer P3DX.

Exemplo de uso do framework CoppeliaSim que demonstra:
- Herança de BaseApp
- Cinemática diferencial
- Geração de trajetória Lissajous em tempo real
- Captura e visualização de trajetória
"""

from brainbyte import BaseApp, Plot2D
from brainbyte.robots.movel.PioneerBot import PioneerBot 
import numpy as np



class LocomocaoTeste(BaseApp):
    """Teste de trajetória Lissajous com robô Pioneer P3DX com malha aberta

    Exemplo de um teste que herda `BaseApp`. Implementa:
    - `setup()`: captura de handles do robô
    - `post_start()`: captura da pose inicial no instante t=0
    - `loop(t)`: geração de velocidades e captura de trajetória
    - `stop()`: finalização e plotagem dos resultados
    """

    def __init__(self):
        super().__init__(scene_file="locomocao.ttt",
                         sim_name="malha_aberta_exemplo",
                         sim_time=60.0)

    def setup(self):
        """Configura recursos necessários antes da simulação começar."""
        self.logger.info("Configurando o robô para o teste...")
        
        self.robot = PioneerBot(bridge=self.bridge, 
                                robot_name='Pioneer_p3dx',
                                left_motor="/Pioneer_p3dx_leftMotor",
                                right_motor="/Pioneer_p3dx_rightMotor")

        # Instanciar usando a self.bridge
        self.robot = PioneerBot(bridge=self.bridge, 
                                robot_name='Pioneer_p3dx',
                                left_motor="/Pioneer_p3dx_leftMotor",
                                right_motor="/Pioneer_p3dx_rightMotor")

        # Handshake (Avisa ao Lua o que precisa ser lido em cada ciclo)
        monitor_paths = self.robot.get_monitor_paths()
        actuator_paths = self.robot.get_actuator_paths()
        self.bridge.initialize(monitor_paths, actuator_paths, self.sim)
        self.logger.info("Handshake concluído com sucesso.")
        
        # Limites dinâmicos da simulação
        self.v_max = 1.0     # Velocidade linear máxima (m/s)
        self.w_max = 2.0     # Velocidade angular máxima (rad/s)

        # Variáveis de controle
        self.v = 0.0         # Velocidade linear (m/s)
        self.w = 0.0         # Velocidade angular (rad/s)

        # Arrays vazios para as trajetórias (serão preenchidos no post_start)
        self.traj_real = []      
        self.traj_reference = []

    def post_start(self):
        """
        AJUSTE 3: Executado após o primeiro step da Bridge.
        Garante que a leitura da pose traga dados válidos do CoppeliaSim.
        """
        pos = self.robot.pose
        self.logger.info(f'Pose inicial do robô: x={pos[0]:.2f}, y={pos[1]:.2f}, theta={np.rad2deg(pos[2]):.2f}°')
        
        ponto_inicial = [pos[0], pos[1], 0.0]
        self.traj_real.append(ponto_inicial.copy())      
        self.traj_reference.append(ponto_inicial.copy())

    def generate_trajectory(self, t):
        """
        Gera velocidades de referência baseado em trajetória Lissajous.
        
        A trajetória Lissajous é um padrão matemático que combina sinusoides
        em frequências diferentes, criando curvas interessantes.
        
        Args:
            t: Tempo atual da simulação (segundos)
        """
        # Parâmetros da trajetória Lissajous
        a = 2.0              # Amplitude eixo X (metros)
        b = 2.0              # Amplitude eixo Y (metros)
        w1 = np.deg2rad(20)  # Frequência eixo X (rad/s)
        w2 = np.deg2rad(10)  # Frequência eixo Y (rad/s)

        # Posição desejada (referência)
        x_ref = a * np.sin(w1 * t)
        y_ref = b * np.sin(w2 * t)

        # Primeira derivada: velocidades (vx, vy)
        vx = a * w1 * np.cos(w1 * t)
        vy = b * w2 * np.cos(w2 * t)

        # Segunda derivada: acelerações (ax, ay)
        ax = -a * w1 * w1 * np.sin(w1 * t)
        ay = -b * w2 * w2 * np.sin(w2 * t)

        # Magnitude da velocidade
        v_magnitude_sq = vx**2 + vy**2
        
        # Proteção contra divisão por zero
        if v_magnitude_sq < 1e-6:
            v_magnitude = 0.0
        else:
            v_magnitude = np.sqrt(v_magnitude_sq)

        # Velocidade linear (escalar)
        self.v = np.clip(v_magnitude, -self.v_max, self.v_max)

        # Velocidade angular: w = (vx*ay - vy*ax) / (vx^2 + vy^2)
        if v_magnitude_sq > 1e-6:
            self.w = (vx * ay - vy * ax) / v_magnitude_sq
        else:
            self.w = 0.0
        
        # Limitar velocidade angular a valores realistas
        self.w = np.clip(self.w, -self.w_max, self.w_max)

        # Armazenar referência para comparação posterior
        self.ref_pos = np.array([x_ref, y_ref, 0.0])
    
    def loop(self, t):
        """Executado a cada passo de simulação."""
        # Gerar velocidades de referência baseado na trajetória Lissajous
        t = self.sim.getSimulationTime()
        self.generate_trajectory(t)

        # Registrar posição de referência
        self.traj_reference.append(self.ref_pos.copy())

        # Enviar comandos de velocidade para os motores
        try:
            self.robot.set_wheel_velocity(self.v, self.w)

            wl, wr = self.robot.wheel_velocities
            self.logger.debug(f"Chassi: v={self.v:.3f}, w={self.w:.3f} | Rodas: wl={wl:.4f}, wr={wr:.4f}")
        except Exception as e:
            self.logger.error("Falha ao aplicar velocidades nas juntas.")

        # Obter e registrar posição real do robô
        pos_real = self.robot.pose
        self.traj_real.append([pos_real[0], pos_real[1], 0.0])
        
    def stop(self):
        """Executado após a simulação terminar."""
        self.logger.info("Parando e finalizando simulação...")
        
        # Parar os motores (segurança)
        try:
            self.robot.stop()
        except Exception as e:
            self.logger.warning(f"Erro ao parar motores: {e}")

        # Lê a posição e orientação finais usando a propriedade simplificada
        pos_final = self.robot.pose
        self.logger.info(f"Posição/Orientação Final: x={pos_final[0]:.2f}, y={pos_final[1]:.2f}, theta={np.rad2deg(pos_final[2]):.2f}°")

        # Plotar trajetória real 
        self.logger.info(f"Total de pontos registrados: {len(self.traj_real)}")
        Plot2D(
            self.traj_real, 
            'X (m)', 
            'Y (m)', 
            tamanho_janela=(10, 8),
            title="Trajetória Real do Robô (Simulação CoppeliaSim)"
        )


def app():
    """Ponto de entrada esperado por `main.py`.

    Cria a instância do teste e inicia sua execução.
    """
    teste = LocomocaoTeste()
    teste.run()


if __name__ == "__main__":
    app()
