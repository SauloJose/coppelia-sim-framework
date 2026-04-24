"""Exemplo: Teste de Trajetória Lissajous com Pioneer P3DX.

Exemplo de uso do framework CoppeliaSim que demonstra:
- Herança de BaseApp
- Cinemática diferencial
- Geração de trajetória Lissajous em tempo real
- Captura e visualização de trajetória
"""

from brainbyte import BaseApp, Plot2D
from brainbyte.robots.movel.PioneerBot import PioneerBot 
from brainbyte.control.automatic import PID_Controller
import numpy as np



class LocomocaoMF(BaseApp):
    """Teste de trajetória Lissajous com robô Pioneer P3DX com malha aberta

    Exemplo de um teste que herda `BaseApp`. Implementa:
    - `setup()`: captura de handles do robô
    - `loop(t)`: geração de velocidades e captura de trajetória
    - `stop()`: finalização e plotagem dos resultados
    """

    def __init__(self):
        super().__init__(scene_file="locomocao.ttt",
                         sim_name="malha_fechada_exemplo", 
                         sim_time=60.0)

    def setup(self):
        """Configura recursos necessários antes da simulação começar."""
        self.logger.info("Configurando o robô para o teste...")
        
        # Instanciando abstração para o robô
        self.robot = PioneerBot(sim=self.sim, 
                                robot_name='Pioneer_p3dx',
                                left_motor="/Pioneer_p3dx_leftMotor",
                                right_motor="/Pioneer_p3dx_rightMotor")

        # Posição inicial do robô
        pos = self.robot.pose
        self.logger.info(f'Pose inicial do robô: x={pos[0]:.2f}, y={pos[1]:.2f}, theta={np.rad2deg(pos[2]):.2f}°')
        
        # Limites dinâmicos da simulação
        self.v_max = 5     # Velocidade linear máxima (m/s)
        self.w_max = 2.0     # Velocidade angular máxima (rad/s)

        # Passo da simulação:
        self.dt = self.d_time()

        
        # Variáveis de controle
        self.pid_v = PID_Controller(
            var=0.0, kp=2, ki=0.0, kd=0.1, dt=self.dt, set_point=0.0
        )
        self.pid_w = PID_Controller(
            var=0.0, kp=2, ki=0.1, kd=0.1, dt=self.dt, set_point=0.0
        )

        # Variáveis de controle Feedforward
        self.v_ff = 0.0      
        self.w_ff = 0.0      
        self.theta_ref = 0.0

        # Trajetórias: real (da simulação) e referência (Lissajous)
        # Salvamos [x, y, 0.0] para manter o formato 3D caso o Plot2D exija
        ponto_inicial = [pos[0], pos[1], 0.0]
        self.traj_real = [ponto_inicial.copy()]      
        self.traj_reference = [ponto_inicial.copy()]


    def generate_trajectory(self, t):
        """Gera velocidades de referência (Feedforward) e a pose ideal."""
        a = 2.0              
        b = 2.0              
        w1 = np.deg2rad(20)  
        w2 = np.deg2rad(10)  

        # Posição desejada (referência)
        x_ref = a * np.sin(w1 * t)
        y_ref = b * np.sin(w2 * t)

        # Primeira derivada: velocidades (vx, vy)
        vx = a * w1 * np.cos(w1 * t)
        vy = b * w2 * np.cos(w2 * t)

        # Segunda derivada: acelerações (ax, ay)
        ax = -a * w1 * w1 * np.sin(w1 * t)
        ay = -b * w2 * w2 * np.sin(w2 * t)

        v_magnitude_sq = vx**2 + vy**2
        
        if v_magnitude_sq < 1e-6:
            v_magnitude = 0.0
        else:
            v_magnitude = np.sqrt(v_magnitude_sq)

        # Velocidade nominal (Feedforward)
        self.v_ff = v_magnitude

        # Velocidade angular nominal (Feedforward)
        if v_magnitude_sq > 1e-6:
            self.w_ff = (vx * ay - vy * ax) / v_magnitude_sq
        else:
            self.w_ff = 0.0
        
        # Ângulo de referência da trajetória geométrica
        self.theta_ref = np.arctan2(vy, vx)

        # Armazenar referência para plotagem
        self.ref_pos = np.array([x_ref, y_ref, 0.0])

    def loop(self, t):
        """Executado a cada passo de simulação."""
        
        # Obter tempo atual
        sim_time = self.simu_time()

        # Gerar velocidades de referência baseado na trajetória Lissajous
        self.generate_trajectory(sim_time)
        self.traj_reference.append(self.ref_pos.copy())

        # Obter a pose real
        pos_real = self.robot.pose
        x_real, y_real, theta_real = pos_real[0], pos_real[1], pos_real[2]
        self.traj_real.append([x_real, y_real, 0.0])

        #Calcular os Erros para o PID
        erro_x_global = self.ref_pos[0] - x_real
        erro_y_global = self.ref_pos[1] - y_real

        # Projeta o erro global no eixo da frente do robô (Erro longitudinal)
        erro_longitudinal = (erro_x_global * np.cos(theta_real)) + (erro_y_global * np.sin(theta_real))

        # Calcula o erro de orientação (diferença entre a rota ideal e a rota atual)
        erro_theta = self.theta_ref - theta_real

        # Normaliza o ângulo para manter entre -pi e pi
        erro_theta = np.arctan2(np.sin(erro_theta), np.cos(erro_theta))

        #  Computar PIDs usando a sua classe
        v_pid = self.pid_v.run(y=-erro_longitudinal)
        w_pid = self.pid_w.run(y=-erro_theta)

        # Comando Final: Nominal (Feedforward) + Correção (Feedback)
        v_cmd = self.v_ff + v_pid
        w_cmd = self.w_ff + w_pid

        # Limitar velocidades aos parâmetros dinâmicos do robô
        v_cmd = np.clip(v_cmd, -self.v_max, self.v_max)
        w_cmd = np.clip(w_cmd, -self.w_max, self.w_max)

        # Enviar comandos de velocidade para os motores
        try:
            self.robot.set_wheel_velocity(v_cmd, w_cmd)
        except Exception as e:
            self.logger.error("Falha ao aplicar velocidades nas juntas.")

        # Obter e registrar posição real do robô
        pos_real = self.robot.pose
        self.traj_real.append([pos_real[0], pos_real[1], 0.0])
        
    def stop(self):
        """Executado após a simulação terminar."""
        self.logger.info("Parando e finalizando simulação...")
        try:
            self.robot.stop()
        except Exception as e:
            self.logger.warning(f"Erro ao parar motores: {e}")

        self.logger.info(f"Total de pontos registrados: {len(self.traj_real)}")
        
        Plot2D(
            self.traj_real, 
            'X (m)', 
            'Y (m)', 
            tamanho_janela=(10, 8),
            title="Trajetória Real vs Referência (Controle PID)"
        )


def app():
    """Ponto de entrada esperado por `main.py`.

    Cria a instância do teste e inicia sua execução.
    """
    teste = LocomocaoMF()
    teste.run()


if __name__ == "__main__":
    app()
