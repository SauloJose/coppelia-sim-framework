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
        super().__init__(scene_file="locomocao.ttt", sim_time=30.0)
        self._first_exec = True #flag

    def setup(self):
        """Configura recursos necessários antes da simulação começar.

        Ideal para buscar handles (objetos, sensores, motores) através do client/sim.
        """
        logger.info("Configurando o robô para o teste...")
        # Exemplo (comentar/descomentar conforme integração com CoppeliaSim):
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
        
        # Dados do Pioneer
        self.L = 0.331  # Metros
        self.r = 0.09751 # Metros

        # Configuração inicial
        self.q = np.array([0,0,0])

        # velocidades
        self.v = 1 
        self.w = np.deg2rad(-100)

        # Modelo cinemático do Pioneer P3DX
        self.wl = self.v / self.r - (self.w * self.L) / (2 * self.r)
        self.wr = self.v / self.r + (self.w * self.L) / (2 * self.r)

        self.u = np.array([self.wr, self.wl])

        self.traj = [self.q.copy()]

    def loop(self, t):

        # Enviando velocidades para os motores (log para diagnóstico)
        logger.debug(f"Comando de velocidade: wl={self.wl:.3f}, wr={self.wr:.3f}")
        try:
            self.sim.setJointTargetVelocity(self.l_wheel, self.wl)
            self.sim.setJointTargetVelocity(self.r_wheel, self.wr)
        except Exception:
            logger.exception("Falha ao aplicar velocidades nas juntas.")
        
        # Configuração inicial
        dt = self.sim.getSimulationTimeStep()

        # Calculando posição
        self.q = self.sim.getObjectPosition(self.robotHandle, self.sim.handle_world)

        self.traj.append(self.q.copy())
        
    def stop(self):
        logger.debug("Parando o robô")
        self.sim.setJointTargetVelocity(self.l_wheel, 0)
        self.sim.setJointTargetVelocity(self.r_wheel, 0)

        logger.debug(f'CALC Pos: {self.sim_time}, {self.q[:2]}, {np.rad2deg(self.q[2])}')
      
        pos = self.sim.getObjectPosition(self.robotHandle, self.sim.handle_world)
        logger.debug(f'SIM Pos: {pos}')

        ori = self.sim.getObjectOrientation(self.robotHandle, self.sim.handle_world)
        logger.debug(f'SIM Ori: {np.rad2deg(ori)}')

        Plot2D(self.traj, 'X (m)', 'Y (m)', title="Trajetória do Robô")
        

def app():
    """Ponto de entrada esperado por `main.py`.

    Cria a instância do teste e inicia sua execução via `BaseApp.run()`.
    """
    teste = TesteEvasaoObstaculos()
    teste.run()