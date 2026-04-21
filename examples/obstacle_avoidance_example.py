"""Exemplo: Evasão de Obstáculos com Sensor LIDAR.

Exemplo de uso do framework CoppeliaSim que demonstra:
- Herança de BaseApp
- Integração com sensor LIDAR (Hokuyo)
- Lógica simples de desvio de obstáculos
- Validação robusta de dados de sensor
"""

from brainbyte import BaseApp, setup_logger
from brainbyte.sensors import HokuyoSensorSim
import matplotlib.pyplot as plt
import numpy as np
import time

def draw_laser_data(laser_data, max_sensor_range=5, save_path=None):
    """Plota cinemática do laser em um gráfico 2D.
    
    Args:
        laser_data: Array com dados [ângulo, distância]
        max_sensor_range: Alcance máximo do sensor
        save_path: Caminho para salvar a imagem. Se None, usa timestamp.
    """
    fig = plt.figure(figsize=(6, 6), dpi=100)
    ax = fig.add_subplot(111, aspect='equal')

    # Plotar pontos do sensor
    for ang, dist in laser_data:
        if (max_sensor_range - dist) > 0.1:  # Filtrar leituras no máximo alcance
            x = dist * np.cos(ang)
            y = dist * np.sin(ang)
            cor = 'r' if ang >= 0 else 'b'
            ax.plot(x, y, 'o', color=cor, markersize=3)

    # Marcar origem do sensor
    ax.plot(0, 0, 'k>', markersize=10, label='Sensor')
    ax.grid(True, alpha=0.3)
    ax.set_xlim([-max_sensor_range, max_sensor_range])
    ax.set_ylim([-max_sensor_range, max_sensor_range])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend()

    # Salvar com timestamp se não especificado
    if save_path is None:
        save_path = f'laser_plot_{int(time.time())}.png'
    
    fig.savefig(save_path, dpi=100)
    plt.close(fig)

class ObstacleAvoidanceTester(BaseApp):
    """Teste de evasão de obstáculos com sensor LIDAR.

    Implementa lógica simples de desvio baseada em distâncias
    medidas em três direções (frente, esquerda, direita).
    """

    def __init__(self):
        super().__init__(scene_file="house.ttt", sim_time=60.0)
        
        # Constantes da lógica de navegação
        self.DIST_SEGURA = 0.8              # Distância considerada segura (metros)
        self.ANGULO_GIRO = np.deg2rad(180)  # Ângulo para girar em obstáculo (radianos)
        self.VEL_LINEAR = 0.4               # Velocidade linear ao avançar (m/s)
        self.VEL_RECUA = -1.0               # Velocidade ao recuar (m/s)

    def setup(self):
        """Configura recursos necessários antes da simulação começar."""
        self.logger.info("Configurando o robô para o teste...")
        
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
            self.logger.error(f"Handles inválidos: {bad}. Verifique nomes dos objetos na cena e paths usados.")
            raise RuntimeError(f"Handles inválidos: {bad}")
        else:
            self.logger.debug(f"Handles: robot={self.robotHandle}, left={self.l_wheel}, right={self.r_wheel}")
        
        # Posição inicial do robô
        pos = self.sim.getObjectPosition(self.robotHandle, self.sim.handle_world)
        self.logger.info(f'Posição inicial do robô: {pos}')
        
        # Parâmetros do Pioneer P3DX
        self.L = 0.381   # Distância entre eixos (metros)
        self.r = 0.0975  # Raio das rodas (metros)
        
        # Instância do sensor
        self.hokuyo_sensor = HokuyoSensorSim(self.sim, "/" + self.robotname + "/fastHokuyo")
        
        # Pré-calcular índices do sensor (economiza operações no loop)
        # Típico: 684 pontos, então frente=342, esq=513, dir=171
        self.sensor_n_points = 684
        self.idx_frente = self.sensor_n_points // 2
        self.idx_esq = (3 * self.sensor_n_points) // 4
        self.idx_dir = self.sensor_n_points // 4
        self.logger.debug(f"Índices do sensor pré-calculados: frente={self.idx_frente}, esq={self.idx_esq}, dir={self.idx_dir}")

    def post_start(self):
        """Executado logo após a simulação iniciar - captura primeira imagem do sensor LIDAR."""
        self.logger.info("Capturando primeira imagem do sensor LIDAR...")
        try:
            laser_data = np.asarray(self.hokuyo_sensor.getSensorData())
            
            # Validar forma do array (deve ser 2D: [pontos, 2])
            if laser_data is None or laser_data.size == 0:
                self.logger.warning("Sensor retornou dados vazios")
                return
            
            if laser_data.ndim == 0:
                self.logger.error(f"Sensor retornou valor escalar: {laser_data}")
                return
                
            if laser_data.ndim != 2 or laser_data.shape[1] < 2:
                self.logger.error(f"Formato incorreto do sensor: shape={laser_data.shape}, esperado (N, 2)")
                return
            
            self.logger.info(f"Primeira imagem capturada com {laser_data.shape[0]} pontos")
            draw_laser_data(laser_data, max_sensor_range=5)
            
        except Exception as e:
            self.logger.error(f"Erro ao capturar dados iniciais do sensor: {e}")

    def loop(self, t):
        """Executado a cada passo da simulação."""
        # Ler dados do LIDAR UMA VEZ por ciclo de simulação
        try:
            laser_data = np.asarray(self.hokuyo_sensor.getSensorData())
        except Exception as e:
            self.logger.error(f"Erro ao ler sensor LIDAR: {e}")
            return

        # Validações de segurança: dados vazios ou formato inválido
        if laser_data is None or laser_data.size == 0:
            return
        
        if laser_data.ndim == 0:
            self.logger.error(f"Sensor retornou valor escalar ao invés de array: {laser_data}")
            return
        
        if laser_data.ndim != 2 or laser_data.shape[1] < 2:
            self.logger.error(f"Formato do sensor inválido: shape={laser_data.shape}, esperado (N, 2)")
            return
        
        # Atualizar índices dinamicamente se o tamanho mudar (raro, mas seguro)
        n_atual = laser_data.shape[0]
        if n_atual != self.sensor_n_points:
            self.sensor_n_points = n_atual
            self.idx_frente = self.sensor_n_points // 2
            self.idx_esq = (3 * self.sensor_n_points) // 4
            self.idx_dir = self.sensor_n_points // 4
            self.logger.debug(f"Tamanho do sensor alterado para {self.sensor_n_points} pontos")

        # Extrair distâncias nas três direções (coluna 1 = distância, coluna 0 = ângulo)
        dist_frente = laser_data[self.idx_frente, 1]
        dist_esq = laser_data[self.idx_esq, 1]
        dist_dir = laser_data[self.idx_dir, 1]

        self.logger.debug(f"LIDAR [{t:.2f}s] Esq: {dist_esq:.2f}m | Frente: {dist_frente:.2f}m | Dir: {dist_dir:.2f}m")

        # === LÓGICA DE DESVIO DE OBSTÁCULOS ===
        if dist_frente > self.DIST_SEGURA:
            # Caminho livre: avança reto
            v = self.VEL_LINEAR
            w = 0.0
        else:
            # Obstáculo detectado: recua e gira para o lado mais livre
            v = self.VEL_RECUA
            w = self.ANGULO_GIRO if dist_esq > dist_dir else -self.ANGULO_GIRO

        # Cinemática inversa: converter velocidade linear (v) e angular (w) 
        # para velocidades das rodas esquerda (wl) e direita (wr)
        wl = (v / self.r) - (w * self.L) / (2 * self.r)
        wr = (v / self.r) + (w * self.L) / (2 * self.r)

        # Aplicar velocidades nos motores
        try:
            self.sim.setJointTargetVelocity(self.l_wheel, wl)
            self.sim.setJointTargetVelocity(self.r_wheel, wr)
        except Exception as e:
            self.logger.error(f"Erro ao aplicar velocidades nos motores: {e}")
    
    def stop(self):
        """Executado após a simulação terminar - parada segura."""
        self.logger.info("Parando motores e finalizando simulação...")
        try:
            self.sim.setJointTargetVelocity(self.l_wheel, 0)
            self.sim.setJointTargetVelocity(self.r_wheel, 0)
            self.logger.info("Simulação finalizada com sucesso")
        except Exception as e:
            self.logger.error(f"Erro ao parar motores: {e}")
        

def app():
    """Ponto de entrada esperado por `main.py`.

    Cria a instância do teste e inicia sua execução.
    """
    teste = ObstacleAvoidanceTester()
    teste.run()


if __name__ == "__main__":
    app()
