from core.base_app import BaseApp
import traceback
import matplotlib.pyplot as plt
import numpy as np
from robots.sensors.HukuyoSensor import * 
import time
import math

"""
Plots the laser scan data.
"""
def draw_laser_data(laser_data, max_sensor_range=5):
    
    fig = plt.figure(figsize=(6,6), dpi=100)
    ax = fig.add_subplot(111, aspect='equal')
                  
    # Combine angle and distance data for plotting
    for ang, dist in laser_data:
        # Filter out readings that are at the maximum range, as they
        # likely indicate no object was detected by that beam.
        if (max_sensor_range - dist) > 0.1:
            x = dist * np.cos(ang)
            y = dist * np.sin(ang)
            # Use different colors for different quadrants for clarity
            c = 'r'
            if ang < 0:    
                c = 'b'
            ax.plot(x, y, 'o', color=c)

    # Plot the sensor's origin
    ax.plot(0, 0, 'k>', markersize=10)
        
    ax.grid(True)
    ax.set_xlim([-max_sensor_range, max_sensor_range])
    ax.set_ylim([-max_sensor_range, max_sensor_range])
    plt.show()

class TesteEvasaoObstaculos(BaseApp):
    """Teste de evasão de obstáculos.

    Exemplo mínimo de um teste que herda `BaseApp`. A responsabilidade da classe é
    implementar `setup()` (captura de handles/sensores/atuadores) e `loop(t)`
    (lógica de controle executada a cada passo de simulação).
    """

    def __init__(self):
        # Indica à classe base qual cena carregar e por quanto tempo executar
        super().__init__(scene_file="labirinto.ttt", sim_time=20.0)

    def setup(self):
        """Configura recursos necessários antes da simulação começar.

        Ideal para buscar handles (objetos, sensores, motores) através do client/sim.
        """
        print("🔧 Configurando o robô para o teste...")
        # Exemplo (comentar/descomentar conforme integração com CoppeliaSim):
        self.robotname = 'Pioneer_p3dx'
        # The new API uses sim.getObject to get handles. The path starts with '/'
        self.robotHandle = self.sim.getObject('/' + self.robotname)
        
        # Handle para as juntas das RODAS
        self.l_wheel = self.sim.getObject('/' + self.robotname + '/Pioneer_p3dx_leftMotor')
        self.r_wheel = self.sim.getObject('/' + self.robotname + '/Pioneer_p3dx_rightMotor')
        
        # Parar a simulação se estiver executando
        initial_sim_state = self.sim.getSimulationState()
        if initial_sim_state != 0:
            self.sim.stopSimulation()
            time.sleep(1)
        
        #Criando istância do sensor
        self.hokuyo_sensor = HokuyoSensorSim(self.sim, "/"+self.robotname+"/fastHokuyo")
        self.initial_laser_data = self.hokuyo_sensor.getSensorData()
        draw_laser_data(self.initial_laser_data)

        # Posição inicial do robô
        pos = self.sim.getObjectPosition(self.robotHandle, self.sim.handle_world)
        print(f'Initial Robot Position: {pos}')
        
        # Dados do Pioneer
        self.L = 0.381  # Metros
        self.r = 0.0975 # Metros

    def loop(self, t):
        """Lógica executada repetidamente pela `BaseApp` até `sim_time`.

        Recebe o tempo de simulação `t` em segundos. Deve ser não-blocking e rápida.
        """
        print(f"Simulation time: {self.sim_time:.2f} [s]")
        
        # Fazendo leitura do laser
        laser_data = self.hokuyo_sensor.getSensorData()

        # Velocidade básica (linear, angular)
        v = 0
        w = np.deg2rad(0)

        frente = int(len(laser_data) / 2)
        lado_direito = int(len(laser_data) * 1 / 4)
        lado_esquerdo = int(len(laser_data) * 3 / 4)

        # Lógica de desvio de obstáculo
        if laser_data[frente, 1] > 2:
            v = .5
            w = 0
        elif laser_data[lado_direito, 1] > 2:
            v = 0
            w = np.deg2rad(-30)
        elif laser_data[lado_esquerdo, 1] > 2:
            v = 0
            w = np.deg2rad(30)

        # Modelo cinemático
        wl = v / self.r - (w * self.L) / (2 * self.r)
        wr = v / self.r + (w * self.L) / (2 * self.r)

        # Enviando velocidades (não precisa mais de opmode)
        self.sim.setJointTargetVelocity(self.l_wheel, wl)
        self.sim.setJointTargetVelocity(self.r_wheel, wr)

        self.sim.step()

        # Parando o robô
        print("Stopping robot...")
        self.sim.setJointTargetVelocity(self.r_wheel, 0)
        self.sim.setJointTargetVelocity(self.l_wheel, 0)


def app():
    """Ponto de entrada esperado por `main.py`.

    Cria a instância do teste e inicia sua execução via `BaseApp.run()`.
    """
    teste = TesteEvasaoObstaculos()
    teste.run()