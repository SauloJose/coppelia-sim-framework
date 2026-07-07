import os
import numpy as np
import matplotlib.pyplot as plt 
from brainbyte import BaseApp
from brainbyte.robots.movel.TurtleBot import TurtleBot
from brainbyte.control.RL import DDQNAgent
from brainbyte.sensors.LDS_02 import LDS_02
from brainbyte.robots.dummy.dummy import Dummy

class RLRobotic(BaseApp):
    """
    Treinamento de locomoção com obstáculos do TurtleBot usando DDQN
    com visualização interativa do progresso.
    """
    def __init__(self):
        super().__init__(scene_file="Arena6x6.ttt", sim_name="turtleBot", sim_time=28800)
        
        # Listas para armazenar o histórico de plotagem
        self.plot_episodes = []
        self.plot_rewards = []
        self.plot_distances = []

    def setup(self):
        self.logger.info("Configuring Robot, Sensor and Controllers..")

        # Configuração do Robô, Alvo e Lidar
        self.robot = TurtleBot(
            bridge=self.bridge, robot_name='Turtlebot3', 
            left_motor='left_motor', right_motor='right_motor', base_link='base_link'
        )
        self.target = Dummy(bridge=self.bridge, obj_name='target')
        self.Lidar = LDS_02(bridge=self.bridge, base_name='Turtlebot3')
        self.robot.add_sensor(sensor_name='LIDAR', sensor_instance=self.Lidar)

        # Handshake=> Faltou colocar o do dummy
        monitor_paths = self.robot.get_monitor_paths()
        monitor_paths.extend(self.target.get_monitor_paths())
        
        actuator_paths = self.robot.get_actuator_paths()
        self.bridge.initialize(monitor_paths, actuator_paths, self.sim)

        # Configurações RL (18 dimensões: 16 LiDAR + Distância + Ângulo)
        self.agent = DDQNAgent(state_dim=18, num_actions=5)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.save_dir = os.path.join(script_dir, "Models", "TurtleBot_DDQN")

        self.collision_margin = 0.20
        self.arrival_margin = 0.30
        self.max_steps = 1500
        self.steps_att_q = 1000 # atualiza a rede target a cada 1000 passos
        
        self.epsilon = 1.0
        self.episode_count = 0
        self.global_step = 0
        self.current_episode_reward = 0.0

        # Tenta carregar checkpoints antigos
        res = self.agent.load_checkpoint(folder=self.save_dir)
        if res["status"] == "SUCCESS":
            self.logger.info(res["message"])
            self.epsilon, self.episode_count, self.global_step = res["data"]
            
        self.start_pos = [-2.5, -1.5, 0.05]
        self.start_orient = [0.0, 0.0, 0.0]

        # Inicializa a janela de gráficos
        self.define_plot_configs()

    def post_start(self):
        super().post_start()
        self.logger.info(f'Initial robot position: x={self.robot.pose[0]:.2f}, y={self.robot.pose[1]:.2f}')
        self.logger.info(f'Initial target position: x={self.target.pose[0]:.2f}, y={self.target.pose[1]:.2f}')
        self.start_episode()


    def define_plot_configs(self):
        """ Inicializa a figura interativa do Matplotlib com 2 subplots """
        plt.ion()  # Ativa o modo interativo
        self.fig, (self.ax_reward, self.ax_dist) = plt.subplots(2, 1, figsize=(8, 8))
        self.fig.suptitle('Progresso de Treinamento - DDQN', fontsize=14)

        # Gráfico de Recompensa
        self.ax_reward.set_title('Recompensa Acumulada por Episódio')
        self.ax_reward.set_ylabel('Recompensa')
        self.ax_reward.grid(True, linestyle='--', alpha=0.6)
        self.line_reward, = self.ax_reward.plot([], [], 'g-', label='Recompensa', linewidth=2)
        self.ax_reward.legend(loc='lower right')

        # Gráfico de Distância
        self.ax_dist.set_title('Distância Final ao Alvo por Episódio')
        self.ax_dist.set_xlabel('Episódios')
        self.ax_dist.set_ylabel('Distância (m)')
        self.ax_dist.grid(True, linestyle='--', alpha=0.6)
        self.line_dist, = self.ax_dist.plot([], [], 'b-', label='Distância Final', linewidth=2)
        
        # Linha verde de referência indicando sucesso (arrival_margin)
        self.ax_dist.axhline(y=self.arrival_margin, color='r', linestyle=':', label='Margem de Sucesso')
        self.ax_dist.legend(loc='upper right')

        plt.tight_layout()

    def update_plots(self):
        """ Atualiza os dados das linhas e redesenha o canvas """
        # Atualiza dados das linhas
        self.line_reward.set_data(self.plot_episodes, self.plot_rewards)
        self.line_dist.set_data(self.plot_episodes, self.plot_distances)

        # Ajusta os limites dos eixos dinamicamente
        self.ax_reward.relim()
        self.ax_reward.autoscale_view()
        self.ax_dist.relim()
        self.ax_dist.autoscale_view()

        # Atualiza a interface
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    # =====================================================================

    def get_continuous_state(self):
        rx, ry, r_theta = self.robot.pose[:3]
        tx, ty = self.target.pose[:2]

        #print(f"target={tx},{ty}")
        #print(f"bot={rx},{ry}")
        
        dist_target = np.hypot(tx - rx, ty - ry)
        angle_target = np.arctan2(ty - ry, tx - rx)
        rel_angle = np.arctan2(np.sin(angle_target - r_theta), np.cos(angle_target - r_theta))
        
        distances, angles = self.Lidar.get_distances()
        
        # Inicializa com valor máximo (3.0m significa sem obstáculo)
        rl_lidar = np.full(16, 3.0) 
        min_sonar = 3.0
        
        if distances.size > 0:
            # 1. Mascara apenas a visão frontal (-90 a 90 graus)
            front_mask = (angles >= -np.pi/2) & (angles <= np.pi/2)
            front_dists = distances[front_mask]
            front_angles = angles[front_mask]
            
            if front_dists.size > 0:
                min_sonar = np.min(front_dists)
                
                # 2. Cria 16 setores de ângulos entre -90 e 90 graus
                bins = np.linspace(-np.pi/2, np.pi/2, 17)
                
                # 3. Puxa a menor distância dentro de cada um dos 16 setores
                for i in range(16):
                    sector_mask = (front_angles >= bins[i]) & (front_angles < bins[i+1])
                    if np.any(sector_mask):
                        # Se há leituras no setor, pega o obstáculo mais próximo nele
                        rl_lidar[i] = np.min(front_dists[sector_mask])

        # Normalizações
        norm_lidar = np.clip(rl_lidar, 0, 3.0) / 3.0
        norm_dist = min(dist_target / 10.0, 1.0)
        norm_angle = rel_angle / np.pi
        
        state = np.concatenate([norm_lidar, [norm_dist, norm_angle]], dtype=np.float32)
        
        return state, dist_target, rel_angle, min_sonar

    def calculate_reward(self, current_dist, min_sonar, rel_angle):
        done = False
        reward = 0.0

        REWARD_SUCCESS = 100.0
        PENALTY_COLLISION = -50.0   # Dobramos a punição para garantir que bater seja a pior coisa possível
        PENALTY_TIME_STEP = -0.01    # Reduzido em 10x para não massacrar o robô a cada passo
        WEIGHT_PROGRESS = 2.0        # Aumentado para valorizar mais o fato de ele andar para frente
        WEIGHT_OBSTACLE = 0.1        # Ajustado proporcionalmente à nova escala de recompensas
        OBSTACLE_THRESHOLD = 0.6     # Mantido igual

        if current_dist <= self.arrival_margin:
            reward = REWARD_SUCCESS
            done = True
            self.logger.info(f"[Ep {self.episode_count+1}] SUCESSO! Alvo alcançado. (Distância final: {current_dist:.2f}m)")
            
        elif min_sonar <= self.collision_margin:
            reward = PENALTY_COLLISION
            done = True
            # Atualizado: Mostra a distância do obstáculo E a distância real que faltava para o alvo
            self.logger.info(f"[Ep {self.episode_count+1}] FALHA! Colisão a {min_sonar:.2f}m do obstáculo. Faltavam {current_dist:.2f}m para o alvo.")
            
        elif self.step_count >= self.max_steps:
            reward = 0.0
            done = True
            self.logger.info(f"[Ep {self.episode_count+1}] TEMPO ESGOTADO. Robô parou a {current_dist:.2f}m do alvo.")
            
        else:
            prox_penalty = 0.0
            if min_sonar < OBSTACLE_THRESHOLD:
                prox_penalty = WEIGHT_OBSTACLE * ((OBSTACLE_THRESHOLD - min_sonar) / OBSTACLE_THRESHOLD)
            
            progress_reward = WEIGHT_PROGRESS * (self.last_dist - current_dist) * np.cos(rel_angle)
            reward = PENALTY_TIME_STEP + progress_reward - prox_penalty
            
        return reward, done
    
    def start_episode(self):
        self.step_count = 0
        self.current_state, self.last_dist, _, _ = self.get_continuous_state()
        self.epsilon = max(0.02, self.epsilon * 0.996)
        self.logger.info(f"==> Iniciando Episódio {self.episode_count + 1} | Epsilon: {self.epsilon:.3f}")

    def loop(self, t, actual_state=None):
        try:
            self.step_count += 1
            self.global_step += 1
            
            state, current_dist, rel_angle, min_sonar = self.get_continuous_state()
            
            action_idx = self.agent.select_action(state, epsilon=self.epsilon)
            v_cmd, w_cmd = self.agent.get_velocities(action_idx)
            self.robot.set_wheel_velocity(linear_vel=v_cmd, angular_vel=w_cmd)
            
            reward, done = self.calculate_reward(current_dist, min_sonar, rel_angle)
                
            self.current_episode_reward += reward
            self.last_dist = current_dist
            
            self.agent.memory.push(self.current_state, action_idx, reward, state, done)
            
            update_res = self.agent.update()
            if update_res and update_res.get("status") == "ERROR":
                 self.logger.error(f"Erro no update: {update_res.get('message')}")
                 
            self.current_state = state
            
            if self.global_step % self.steps_att_q == 0:
                self.agent.hard_update_target()

            if done:
                self.episode_count += 1
                self.logger.info(f"Fim do Ep {self.episode_count} | Recompensa: {self.current_episode_reward:.2f}")
                
                # =====================================================
                # ALIMENTA OS DADOS DOS GRÁFICOS E ATUALIZA A JANELA
                # =====================================================
                self.plot_episodes.append(self.episode_count)
                self.plot_rewards.append(self.current_episode_reward)
                self.plot_distances.append(current_dist)
                
                self.update_plots()
                # =====================================================

                self.current_episode_reward = 0.0
                
                if self.episode_count % 10 == 0:
                    res = self.agent.save_checkpoint(self.epsilon, self.episode_count, self.global_step, self.save_dir)
                    if res["status"] != "SUCCESS":
                         self.logger.error(f"Failed to save checkpoint: {res.get('message')}")
                
                self.robot.stop()

                h_robot = self.sim.getObject('/Turtlebot3')
                self.sim.setObjectPosition(h_robot, self.sim.handle_world, self.start_pos)
                self.sim.setObjectOrientation(h_robot, self.sim.handle_world, self.start_orient)
                
                self.start_episode()

        except Exception as e:
            self.logger.error(f"Erro no loop RL: {e}")

    def stop(self):
        try:
            # Verifica se o robô foi criado antes de parar
            if hasattr(self, 'robot'):
                self.robot.stop()
            
            # Verifica se o agente foi criado antes de salvar
            if hasattr(self, 'agent'):
                self.agent.save_checkpoint(self.epsilon, self.episode_count, self.global_step, self.save_dir)
            
            # Desativa o modo interativo e mostra o gráfico final de forma estática
            if hasattr(self, 'fig'):
                plt.ioff()
                self.logger.info("Simulação finalizada. Feche a janela de gráficos para encerrar o script.")
                plt.show()
            
        except Exception as e:
            self.logger.error(f"Erro em stop(): {e}")

def app():
    aplicacao = RLRobotic()
    aplicacao.run()