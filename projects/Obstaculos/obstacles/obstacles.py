"""Exemplo: Evasão de Obstáculos com Sensor LIDAR.

Exemplo de uso do framework CoppeliaSim que demonstra:
- Herança de BaseApp
- Integração com sensor LIDAR (Hokuyo)
- Lógica simples de desvio de obstáculos
- Validação robusta de dados de sensor
"""

from brainbyte import BaseApp
from brainbyte.sensors import HokuyoSensorSim
from brainbyte.robots.movel.PioneerBot import *
import matplotlib.pyplot as plt
import numpy as np
import time
import traceback
import os 

def draw_laser_data(laser_data, max_sensor_range=5, save_path=None):
    """Plota cinemática do laser em um gráfico 2D."""
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

    # Tratamento de caminho seguro
    if save_path is None:
        # Pega a pasta atual onde o script está rodando
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Monta o caminho para a pasta Figures
        figures_dir = os.path.join(current_dir, '/projects/Obstaculos/obstacles/Figures')
        
        # Garante que a pasta existe (se não existir, ele cria)
        os.makedirs(figures_dir, exist_ok=True)
        
        # Cria o nome do arquivo com timestamp para não perder o histórico
        nome_arquivo = f'laser_plot_{int(time.time())}.png'
        
        # Caminho final completo
        save_path = os.path.join(figures_dir, nome_arquivo)
    
    fig.savefig(save_path, dpi=100)
    plt.close(fig)

class ObstacleAvoidanceTester(BaseApp):
    """Teste de evasão de obstáculos com sensor LIDAR.

    Implementa lógica simples de desvio baseada em distâncias
    medidas em três direções (frente, esquerda, direita).
    """

    def __init__(self):
        super().__init__(scene_file="house.ttt", 
                         sim_name="obstacles",
                         sim_time=60.0)
        
        # Constantes da lógica de navegação
        self.DIST_SEGURA = 0.8              # Distância considerada segura (metros)
        self.ANGULO_GIRO = np.deg2rad(180)  # Ângulo para girar em obstáculo (radianos)
        self.VEL_LINEAR = 0.4               # Velocidade linear ao avançar (m/s)
        self.VEL_RECUA = -1.0               # Velocidade ao recuar (m/s)

    def setup(self):
        """Configura recursos necessários antes da simulação começar."""
        self.logger.info("Configurando o robô para o teste...")
        
        # Instancia o robô usando a nova classe 
        self.robot = PioneerBot(bridge=self.bridge, robot_name='PioneerP3DX')

        # Usa a propriedade integrada para ver a pose inciial
        self.logger.info(f'Pose inicial do robô (x,y,theta): {self.robot.pose}')

        # Instancia o sensor (usando o nome dinâmico do robô)
        self.hokuyo_sensor = HokuyoSensorSim(self.bridge, 
                                             f"/{self.robot.robot_name}/fastHokuyo",True)


        self.robot.add_sensor("LIDAR",self.hokuyo_sensor)

        # Junta os caminhos de monitoramento do robô e do sensor num só pacote
        monitor_paths = self.robot.get_monitor_paths()
        actuator_paths = self.robot.get_actuator_paths()
        self.bridge.initialize(monitor_paths, actuator_paths, self.sim)

        # TESTE DE DEBUG:
        print("--- CHAVES NO CACHE DA BRIDGE ---")
        # Tenta rodar um step manual para forçar a atualização
        self.bridge.step() 
        print(self.bridge.latest_state.keys())
        print("---------------------------------")

        # Pré-calcular índices do sensor (economiza operações no loop)
        self.sensor_n_points = 684
        self.idx_frente = self.sensor_n_points // 2
        self.idx_esq = (3 * self.sensor_n_points) // 4
        self.idx_dir = self.sensor_n_points // 4
        self.logger.debug(f"Índices do sensor pré-calculados: frente={self.idx_frente}, esq={self.idx_esq}, dir={self.idx_dir}")

    def post_start(self):
        """Executado logo após a simulação iniciar - captura primeira imagem do sensor LIDAR."""
        
        # AJUSTE 4: Usa a propriedade integrada para ver a pose inicial aqui (cache já preenchido)
        self.logger.info(f'Pose inicial do robô (x,y,theta): {self.robot.pose}')
        
        self.logger.info("Capturando primeira imagem do sensor LIDAR...")
        try:
            # AJUSTE 5: Usa o novo método nativo update() para ler do cache (Zero-Lag)
            laser_data = np.asarray(self.hokuyo_sensor.update())
            
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
        """Executado a cada passo da simulação com lógica de evasão."""
        try:
            # 1. Puxar dados do cache da Bridge (Zero-Lag)
            # O .update() retorna o que a Bridge capturou no último sim.step()
            raw_data = self.hokuyo_sensor.update()
            
            if raw_data is None:
                # Se a Bridge ainda não recebeu nada do Coppelia, saímos cedo
                return

            # Converter para Numpy para processamento matemático
            laser_data = np.asarray(raw_data)

            # 2. VALIDAÇÃO CRÍTICA (Evita o IndexError)
            # Verificamos se o array é 2D [pontos, 2] e se tem dados
            if laser_data.ndim != 2 or laser_data.shape[0] == 0:
                self.logger.warning(f"LIDAR vazio ou malformado. Shape recebido: {laser_data.shape}")
                return # Interrompe a execução deste loop específico

            # 3. Atualizar índices baseados no tamanho real do scan
            # (Alguns sensores mudam a resolução dinamicamente)
            n_points = laser_data.shape[0]
            idx_frente = n_points // 2
            idx_esq = (3 * n_points) // 4
            idx_dir = n_points // 4

            # 4. Extração de distâncias (Coluna 0: Ângulo, Coluna 1: Distância)
            dist_frente = laser_data[idx_frente, 1]
            dist_esq = laser_data[idx_esq, 1]
            dist_dir = laser_data[idx_dir, 1]

            # Log de depuração para monitorar as leituras no terminal
            self.logger.debug(f"Distâncias -> Esq: {dist_esq:.2f}m | Frente: {dist_frente:.2f}m | Dir: {dist_dir:.2f}m")

            # 5. LÓGICA DE DECISÃO (Evasão de Obstáculos)
            if dist_frente > self.DIST_SEGURA:
                # Caminho livre à frente
                v = self.VEL_LINEAR
                w = 0.0
            else:
                # Obstáculo detectado: decide para onde girar
                self.logger.info("Obstáculo detectado! Iniciando manobra...")
                v = self.VEL_RECUA
                # Se tiver mais espaço na esquerda, gira para a esquerda, senão direita
                w = self.ANGULO_GIRO if dist_esq > dist_dir else -self.ANGULO_GIRO

            # 6. ENVIAR COMANDOS (Enfileira no buffer da Bridge)
            self.robot.set_wheel_velocity(linear_vel=v, angular_vel=w)

        except Exception as e:
            self.logger.error(f"Falha catastrófica no loop de controle: {e}")
            # Opcional: self.stop_simulation() se o erro for persistente
    def stop(self):
        """Executado após a simulação terminar - parada segura."""
        self.logger.info("Parando motores e finalizando simulação...")
        try:
            self.robot.stop()
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
    try:
        app()
    except Exception as e:
        print("\n" + "="*50)
        print(" ERRO FATAL CAPTURADO:")
        traceback.print_exc()
        print("="*50 + "\n")
        input("Pressione ENTER para sair...") # Pausa a tela para você conseguir ler