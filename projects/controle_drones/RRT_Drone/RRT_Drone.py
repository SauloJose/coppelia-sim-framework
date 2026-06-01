from brainbyte import BaseApp              
from brainbyte.robots.vant.quadcopter import * 
from brainbyte.control.automatic import * 
from brainbyte.control.manual import * 
from brainbyte.core.bridge import SimulationBridge
import numpy as np
import traceback
import re 

class Drone_Basico(BaseApp):
    """
    Classe principal da simulação Drone_Basico.
    Gerencia o ciclo de vida da aplicação com troca de alvos via input em bloco [X, Y, Z].
    """
    def __init__(self):
        super().__init__(scene_file="Drone.ttt", sim_name="Drone_Basico", sim_time=360)
        self.robots = []
        
        # Inicializados como None; serão configurados no post_start
        self.target_atual = None 
        self.target_final = None 
        
        # Configuração da suavização
        self.velocidade_max_alvo = 0.6  # m/s
        self.last_t = 0.0

    def setup(self):
        try:
            self.logger.info("Configuring Quadcopter and Handshake...")
            self.drone = Quadcopter(bridge=self.bridge, robot_name="Quadcopter")
            self.robots.append(self.drone) 
            self.handshake()
            self.bridge
        except Exception as e:
            self.logger.error(f"Error detected in setup()!\nTraceback:\n{traceback.format_exc()}")
            
    def post_start(self):
        try:
            super().post_start()
            self.logger.info("Simulation started! Synchronizing initial positions...")
            
            # 1. Captura a posição de nascimento do drone direto da telemetria inicial
            drone_xyz = [0.0, 0.0, 0.0]
            actual_state = self.bridge.step()
            
            if actual_state:
                for k, v in actual_state.items():
                    if k.endswith("_pos") and "target" not in k:
                        drone_xyz = [round(x, 2) for x in v]
                        break
            
            # 2. Inicializa os alvos exatamente onde o drone está parado
            self.target_atual = drone_xyz.copy()
            self.target_final = drone_xyz.copy()
            
            self.logger.info(f"[TELEMETRIA INICIAL] O drone iniciará em: {drone_xyz}")
            
            # 3. Solicita o primeiro input logo na decolagem
            self.solicitar_nova_posicao_em_bloco(drone_xyz)
            
            return super().post_start()
        except Exception as e:
            self.logger.error(f"Error detected in post_start()!\nTraceback:\n{traceback.format_exc()}")

    def loop(self, t, actual_state=None):
        try:
            # Garante proteção caso o loop rode antes do post_start concluir por algum motivo assíncrono
            if self.target_atual is None:
                return

            # 1. Atualiza o tempo da simulação
            dt = self.dt
            self.last_t = t

            # 2. --- LÓGICA DE INTERPOLAÇÃO DO ALVO ---
            direcao = np.array(self.target_final) - np.array(self.target_atual)
            distancia_para_alvo_final = np.linalg.norm(direcao)

            if distancia_para_alvo_final > 0.01: 
                passo_maximo = self.velocidade_max_alvo * dt
                
                if passo_maximo >= distancia_para_alvo_final:
                    self.target_atual = self.target_final.copy()
                else:
                    vetor_unitario = direcao / distancia_para_alvo_final
                    novo_target = np.array(self.target_atual) + vetor_unitario * passo_maximo
                    self.target_atual = novo_target.tolist()

            # 3. Garante o envio do alvo INTERPOLADO para a física do drone
            self.drone.move_to(self.target_atual)

            # 4. Extrai a posição real do drone usando a função nativa do objeto
            pose, ang = self.drone.get_pose()
            drone_xyz = [round(x, 2) for x in pose] # Arredonda para o log e telemetria ficarem limpos

            # 5. Verifica se o drone atingiu o alvo final com estabilização
            if self.drone.has_reached_target(actual_state, tolerance=0.1):
                if distancia_para_alvo_final <= 0.10: 
                    self.logger.info(f"[SUCESSO]: O drone CHEGOU ao alvo final {self.target_final}!")
                    self.logger.info(f"[TELEMETRIA ATUAL] O drone está estabilizado em: {drone_xyz}")
                    self.solicitar_nova_posicao_em_bloco(drone_xyz)

        except Exception as e:
            self.logger.error(f"Error detected in loop()!\nTraceback:\n{traceback.format_exc()}")

    def solicitar_nova_posicao_em_bloco(self, posicao_atual):
        """Bloqueia o console e processa a string recebida no formato [X, Y, Z] ou X, Y, Z"""

        self.logger.info(f"AGUARDANDO INPUT -> Posição Atual do Drone: {posicao_atual}")
        self.logger.info("Digite a nova coordenada no formato [X, Y, Z] ou X, Y, Z (aceita valores negativos):")

        try:
            entrada = input()
            numeros = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", entrada)
            
            if len(numeros) == 3:
                x = float(numeros[0])
                y = float(numeros[1])
                z = float(numeros[2])
                
                self.target_final = [x, y, z]
                self.logger.info(f"Novo alvo final recebido: {self.target_final}. Iniciando deslocamento suave.")
            else:
                self.logger.warning(f"[AVISO]: Formato inválido! Você digitou: '{entrada}'. Mantendo o alvo anterior: {self.target_final}")
                
        except Exception as e:
            self.logger.error(f"Erro ao processar o input: {e}. Mantendo o alvo anterior.")


    def handshake(self):
        try: 
            monitor_paths = []
            actuator_paths = []
            for robot in self.robots:
                monitor_paths.extend(robot.get_monitor_paths())
                actuator_paths.extend(robot.get_actuator_paths())
            
            self.bridge.initialize(monitor_paths, actuator_paths, self.sim)
            self.logger.info("Handshake with CoppeliaSim: OK!") 
        except Exception as e:
            self.logger.error(f"Error in Handshake with CoppeliaSim!\nTraceback:\n{traceback.format_exc()}") 

    def stop(self):
        try:
            self.logger.info("Stopping simulation...")
            for robot in self.robots:
                robot.stop()
        except Exception as e:
            self.logger.error(f"Error detected in stop()!\nTraceback:\n{traceback.format_exc()}")

def app():
    aplicacao = Drone_Basico()
    aplicacao.run()