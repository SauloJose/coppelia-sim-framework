from core.base_app import BaseApp
import traceback
import logging
import matplotlib.pyplot as plt
import numpy as np
from robots.sensors.HokuyoSensor import HokuyoSensorSim
import time
import math

logger = logging.getLogger(__name__)

"""
Plots the laser scan data.
"""
def draw_laser_data(laser_data, max_sensor_range=5, show=False, save_path=None):
    """Plota os dados do laser.

    Por padrão não bloqueia com uma GUI; se `show=True` tenta exibir a janela
    (pode falhar em ambientes sem GUI). Se `save_path` for fornecido, salva a
    imagem no caminho indicado.
    """

    fig = plt.figure(figsize=(6, 6), dpi=100)
    ax = fig.add_subplot(111, aspect='equal')

    # Combine angle and distance data for plotting
    for ang, dist in laser_data:
        # Filter out readings that are at the maximum range
        if (max_sensor_range - dist) > 0.1:
            x = dist * np.cos(ang)
            y = dist * np.sin(ang)
            c = 'r' if ang >= 0 else 'b'
            ax.plot(x, y, 'o', color=c)

    # Plot the sensor's origin
    ax.plot(0, 0, 'k>', markersize=10)

    ax.grid(True)
    ax.set_xlim([-max_sensor_range, max_sensor_range])
    ax.set_ylim([-max_sensor_range, max_sensor_range])

    if save_path:
        fig.savefig(save_path)
        plt.close(fig)
        logger.info(f"Laser plot salvo em: {save_path}")
        return

    if show:
        try:
            plt.show(block=False)
            plt.pause(0.1)
        except Exception:
            # Ambiente sem GUI
            logger.warning("Falha ao mostrar figura (sem GUI). Salvando em 'laser_plot.png'.")
            fig.savefig('laser_plot.png')
            plt.close(fig)
    else:
        # Default: save to a timestamped file to avoid blocking
        timestamp = int(time.time())
        filename = f'laser_plot_{timestamp}.png'
        fig.savefig(filename)
        plt.close(fig)
        logger.info(f"Laser plot salvo em: {filename}")

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
        super().__init__(scene_file="house.ttt", sim_time=60.0)
        self._first_exec = True #flag

    def setup(self):
        """Configura recursos necessários antes da simulação começar.

        Ideal para buscar handles (objetos, sensores, motores) através do client/sim.
        """
        logger.info("Configurando o robô para o teste...")
        # Exemplo (comentar/descomentar conforme integração com CoppeliaSim):
        self.robotname = 'PioneerP3DX'

        # Handles do robô e rodas
        self.robotHandle = self.sim.getObject('/' + self.robotname)
        self.l_wheel = self.sim.getObject('/' + self.robotname + '/leftMotor')
        self.r_wheel = self.sim.getObject('/' + self.robotname + '/rightMotor')

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
        
        # Instância do sensor (não tentamos ler os dados aqui, pois a simulação ainda não começou)
        self.hokuyo_sensor = HokuyoSensorSim(self.sim, "/" + self.robotname + "/fastHokuyo")

        # Posição inicial do robô
        pos = self.sim.getObjectPosition(self.robotHandle, self.sim.handle_world)
        logger.info(f'📍 Posição inicial do robô: {pos}')
        
        # Dados do Pioneer
        self.L = 0.381  # Metros
        self.r = 0.0975 # Metros

    def post_start(self):
        """Executado logo após `startSimulation()` (veja `BaseApp.post_start`).

        Se `self.auto_diagnostic` estiver True, executa um pulso de velocidade
        curto para validar atuadores e handles.
        """
        if getattr(self, 'auto_diagnostic', False):
            logger.info("Executando diagnóstico automático: pulso nas rodas.")
            try:
                # velocidade de teste e duração em segundos
                self.diagnostic_pulse(duration=1.0, speed=0.6)
            except Exception:
                logger.exception("Falha no diagnóstico automático")

    def diagnostic_pulse(self, duration=1.0, speed=0.6):
        """Envia um pulso de velocidade nas rodas para testar atuação.

        - duration: tempo (s) que o pulso dura
        - speed: velocidade linear de referência (m/s) — convertida para wl/wr
        """
        if any(h == -1 for h in (self.l_wheel, self.r_wheel)):
            logger.error("Cannot run diagnostic: invalid wheel handles")
            return

        # calcula wl/wr correspondentes a v=speed e w=0
        v = float(speed)
        w = 0.0
        wl = v / self.r - (w * self.L) / (2 * self.r)
        wr = v / self.r + (w * self.L) / (2 * self.r)

        logger.debug(f"Diagnostic pulse: wl={wl:.3f}, wr={wr:.3f}, duration={duration}s")

        start = self.sim.getSimulationTime()
        try:
            while (self.sim.getSimulationTime() - start) < duration:
                self.sim.setJointTargetVelocity(self.l_wheel, wl)
                self.sim.setJointTargetVelocity(self.r_wheel, wr)
                # step para avançar a simulação durante o pulso
                self.sim.step()

        finally:
            # Zeramos as velocidades ao final
            try:
                self.sim.setJointTargetVelocity(self.l_wheel, 0)
                self.sim.setJointTargetVelocity(self.r_wheel, 0)
            except Exception:
                logger.exception("Falha ao zerar velocidades após diagnóstico")
            logger.info("Diagnóstico concluído: pulso finalizado")

    def loop(self, t):
        # Fazendo leitura do LIDAR
        try:
            laser_data = np.asarray(self.hokuyo_sensor.getSensorData())
            #print(laser_data)
            if self._first_exec is True:
                draw_laser_data(laser_data, 5,True)
            self._first_exec = False 
        except Exception:
            logger.exception("Erro lendo dados do sensor no loop.")
            return

        n = len(laser_data)
        
        # Se a câmera ainda não darenderizou (primeiros instantes), apenas aguarda
        if n == 0: return

        # Pega os índices baseados no tamanho do array de pontos (geralmente 684)
        frente = int(n / 2)
        lado_direito = int(n * 1 / 4)
        lado_esquerdo = int(n * 3 / 4)

        # Lendo as distâncias exatas (a coluna 1 contém as distâncias, a 0 contém os ângulos)
        dist_frente = laser_data[frente, 1]
        dist_esq = laser_data[lado_esquerdo, 1]
        dist_dir = laser_data[lado_direito, 1]

        # Log dos dados do sensor (use nível DEBUG para não poluir o console em produção)
        logger.debug(f"[{t:.2f}s] Sensor -> Esq: {dist_esq:.2f}m | Frente: {dist_frente:.2f}m | Dir: {dist_dir:.2f}m")

        # === LÓGICA DE DESVIO DE OBSTÁCULOS ===
        v = 0.0
        w = 0.0
        
        # Consideramos 0.8 metros como uma distância perigosa
        if dist_frente > 0.8:
            # Caminho livre! Vai reto.
            v = 0.4
            w = 0.0
        else:
            # Obstáculo na frente! Analisa qual lado tem mais espaço
            v = -1 
            if dist_esq > dist_dir:
                # Esquerda está mais livre, gira pra esquerda (positivo)
                w = np.deg2rad(180)
            else:
                # Direita está mais livre, gira pra direita (negativo)
                w = np.deg2rad(-180)

        # Modelo cinemático do Pioneer P3DX
        wl = v / self.r - (w * self.L) / (2 * self.r)
        wr = v / self.r + (w * self.L) / (2 * self.r)

        # Enviando velocidades para os motores (log para diagnóstico)
        logger.debug(f"Comando de velocidade: wl={wl:.3f}, wr={wr:.3f}")
        try:
            self.sim.setJointTargetVelocity(self.l_wheel, wl)
            self.sim.setJointTargetVelocity(self.r_wheel, wr)
        except Exception:
            logger.exception("Falha ao aplicar velocidades nas juntas.")
        

def app():
    """Ponto de entrada esperado por `main.py`.

    Cria a instância do teste e inicia sua execução via `BaseApp.run()`.
    """
    teste = TesteEvasaoObstaculos()
    teste.run()