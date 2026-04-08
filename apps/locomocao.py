from core.base_app import BaseApp
from core.utils import Plot2D
import traceback
import logging
import matplotlib.pyplot as plt
import numpy as np
from robots.sensors.HokuyoSensor import HokuyoSensorSim
import time
import math

logger = logging.getLogger(__name__)

class TesteEvasaoObstaculos(BaseApp):
    """Teste de evasão de obstáculos.

    Exemplo mínimo de um teste que herda `BaseApp`. A responsabilidade da classe é
    implementar `setup()` (captura de handles/sensores/atuadores) e `loop(t)`
    (lógica de controle executada a cada passo de simulação).
    """

    def __init__(self):
        # Indica à classe base qual cena carregar e por quanto tempo executar
        # auto_diagnostic: se True, após startSimulation executa um pulso curto
        # nas rodas para validar que os comandos são aplicados.
        self.auto_diagnostic = False
        super().__init__(scene_file="locomocao.ttt", sim_time=60.0)
        self._first_exec = True #flag

    def setup(self):
        """Configura recursos necessários antes da simulação começar.

        Ideal para buscar handles (objetos, sensores, motores) através do client/sim.
        """
        logger.info("Configurando o robô para o teste...")
        
        self.robotname = 'Pioneer_p3dx'

        # Handles do robô e rodas
        self.robotHandle = self.sim.getObject('/' + self.robotname)
        self.l_wheel = self.sim.getObject('/' + self.robotname + '/'+self.robotname+'_leftMotor')
        self.r_wheel = self.sim.getObject('/' + self.robotname + '/'+self.robotname+'_rightMotor')

        # Verifica se os handles foram obtidos corretamente
        bad = []
        for name, h in (('robot', self.robotHandle), ('leftWheel', self.l_wheel), ('rightWheel', self.r_wheel)):
            if h == -1:
                bad.append(name)

        if bad:
            logger.error(f"Handles inválidos: {bad}. Verifique nomes dos objetos na cena e paths usados.")
            raise RuntimeError(f"Handles inválidos: {bad}")
        else:
            logger.debug(f"Handles: robot={self.robotHandle}, left={self.l_wheel}, right={self.r_wheel}")
        
        # Posição inicial do robô
        pos = self.sim.getObjectPosition(self.robotHandle, self.sim.handle_world)
        logger.info(f'Posição inicial do robô: {pos}')
        
        # Parâmetros do Pioneer P3DX
        self.L = 0.331       # Distância entre eixos (metros)
        self.r = 0.09751     # Raio das rodas (metros)
        self.v_max = 1.0     # Velocidade linear máxima (m/s)
        self.w_max = 2.0     # Velocidade angular máxima (rad/s)

        # Velocidades atuais
        self.v = 0.0         # Velocidade linear (m/s)
        self.w = 0.0         # Velocidade angular (rad/s)
        self.wl = 0.0        # Velocidade angular roda esquerda (rad/s)
        self.wr = 0.0        # Velocidade angular roda direita (rad/s)

        # Trajetórias: real (da simulação) e referência (Lissajous)
        pos_inicial = self.sim.getObjectPosition(self.robotHandle, self.sim.handle_world)
        self.traj_real = [pos_inicial.copy()]      # Trajetória real do robô
        self.traj_reference = [pos_inicial.copy()] # Trajetória de referência desejada

    def GerTraj(self, t):
        """
        Gera velocidades de referência baseado em trajetória Lissajous.
        
        A trajetória Lissajous é um padrão matemático que combinasinusoides
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

        # Cinemática inversa: converter (v, w) para velocidades das rodas
        # Para robô diferencial: wl = v/r - (w*L)/(2r), wr = v/r + (w*L)/(2r)
        self.wl = self.v / self.r - (self.w * self.L) / (2 * self.r)
        self.wr = self.v / self.r + (self.w * self.L) / (2 * self.r)

        # Armazenar referência para comparação posterior
        self.ref_pos = np.array([x_ref, y_ref, 0.0])
    
    def loop(self, t):
        """Executado a cada passo de simulação."""
        
        # Obter tempo atual e passo de simulação
        dt = self.sim.getSimulationTimeStep()
        sim_time = self.sim.getSimulationTime()

        # Gerar velocidades de referência baseado na trajetória Lissajous
        self.GerTraj(sim_time)

        # Registrar posição de referência
        self.traj_reference.append(self.ref_pos.copy())

        # Enviar comandos de velocidade para os motores
        try:
            self.sim.setJointTargetVelocity(self.l_wheel, self.wl)
            self.sim.setJointTargetVelocity(self.r_wheel, self.wr)
            logger.debug(f"[{sim_time:.3f}s] Velocidades: wl={self.wl:.4f}, wr={self.wr:.4f} (v={self.v:.3f}, w={self.w:.3f})")
        except Exception as e:
            logger.exception("Falha ao aplicar velocidades nas juntas.")

        # Obter e registrar posição real do robô
        pos_real = self.sim.getObjectPosition(self.robotHandle, self.sim.handle_world)
        self.traj_real.append(np.array(pos_real))
        
    def stop(self):
        """Executado após a simulação terminar."""
        logger.info("Parando e finalizando simulação...")
        
        # Parar os motores (segurança)
        try:
            self.sim.setJointTargetVelocity(self.l_wheel, 0)
            self.sim.setJointTargetVelocity(self.r_wheel, 0)
        except Exception as e:
            logger.warning(f"Erro ao parar motores: {e}")

        # Obter posição final real
        pos_final = self.sim.getObjectPosition(self.robotHandle, self.sim.handle_world)
        ori_final = self.sim.getObjectOrientation(self.robotHandle, self.sim.handle_world)
        
        logger.info(f"Posição final: {pos_final}")
        logger.info(f"Orientação final: {np.rad2deg(ori_final)}")

        # Plotar trajetória real (obtida da simulação)
        logger.info(f"Total de pontos registrados: {len(self.traj_real)}")
        Plot2D(
            self.traj_real, 
            'X (m)', 
            'Y (m)', 
            tamanho_janela=(10, 8),
            title="Trajetória Real do Robô (Simulação CoppeliaSim)"
        )
        

def app():
    """Ponto de entrada esperado por `main.py`.

    Cria a instância do teste e inicia sua execução via `BaseApp.run()`.
    """
    teste = TesteEvasaoObstaculos()
    teste.run()