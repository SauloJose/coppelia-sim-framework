import os
import numpy as np
import random
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

# =====================================================================
# 1. REDE NEURAL (Com Inicialização Kaiming para ReLU)
# =====================================================================
class DDQNNetwork(nn.Module):
    def __init__(self, input_dim=10, num_actions=5):
        super(DDQNNetwork, self).__init__()
        self.fc1 = nn.Linear(input_dim, 128)
        self.fc2 = nn.Linear(128, 64)
        self.q_out = nn.Linear(64, num_actions)

        # Inicialização de pesos Otimizada para ReLU
        nn.init.kaiming_uniform_(self.fc1.weight, nonlinearity='relu')
        nn.init.kaiming_uniform_(self.fc2.weight, nonlinearity='relu')
        nn.init.xavier_uniform_(self.q_out.weight)

    def forward(self, state):
        x = F.relu(self.fc1(state))
        x = F.relu(self.fc2(x))
        return self.q_out(x)

# =====================================================================
# 2. REPLAY BUFFER OTIMIZADO (Pré-alocação em Memória)
# =====================================================================
class ReplayBuffer:
    def __init__(self, state_dim, capacity=100000): 
        self.capacity = capacity
        self.ptr = 0
        self.size = 0
        self.state_dim = state_dim
        
        # Pré-alocação contígua na memória (Muito mais rápido que Deque)
        self.state = np.zeros((capacity, state_dim), dtype=np.float32)
        self.action = np.zeros((capacity, 1), dtype=np.int64)
        self.reward = np.zeros((capacity, 1), dtype=np.float32)
        self.next_state = np.zeros((capacity, state_dim), dtype=np.float32)
        self.done = np.zeros((capacity, 1), dtype=np.float32)

    def push(self, state, action, reward, next_state, done):
        self.state[self.ptr] = state
        self.action[self.ptr] = action
        self.reward[self.ptr] = reward
        self.next_state[self.ptr] = next_state
        self.done[self.ptr] = done

        self.ptr = (self.ptr + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size):
        # Amostragem ultra-rápida via índices aleatórios
        ind = np.random.randint(0, self.size, size=batch_size)
        
        return (
            self.state[ind],
            self.action[ind],
            self.reward[ind],
            self.next_state[ind],
            self.done[ind]
        )

    def __len__(self):
        return self.size

    # -----------------------------------------------------------------
    # MÉTODOS CORRIGIDOS PARA SALVAR E CARREGAR O BUFFER
    # -----------------------------------------------------------------
    def state_dict(self):
        """ Salva apenas os dados preenchidos para economizar espaço em disco """
        return {
            'state': self.state[:self.size],
            'action': self.action[:self.size],
            'reward': self.reward[:self.size],
            'next_state': self.next_state[:self.size],
            'done': self.done[:self.size],
            'ptr': self.ptr,
            'size': self.size,
            'capacity': self.capacity,
            'state_dim': self.state_dim
        }

    def load_state_dict(self, state_dict):
        """ Restaura os dados para a matriz pré-alocada """
        self.capacity = state_dict.get('capacity', self.capacity)
        self.size = state_dict['size']
        self.ptr = state_dict['ptr']
        self.state_dim = state_dict.get('state_dim', self.state_dim)

        # Recria as matrizes cheias de zeros com a capacidade total
        self.state = np.zeros((self.capacity, self.state_dim), dtype=np.float32)
        self.action = np.zeros((self.capacity, 1), dtype=np.int64)
        self.reward = np.zeros((self.capacity, 1), dtype=np.float32)
        self.next_state = np.zeros((self.capacity, self.state_dim), dtype=np.float32)
        self.done = np.zeros((self.capacity, 1), dtype=np.float32)

        # Injeta os dados salvos de volta no início das matrizes
        if self.size > 0:
            self.state[:self.size] = state_dict['state']
            self.action[:self.size] = state_dict['action']
            self.reward[:self.size] = state_dict['reward']
            self.next_state[:self.size] = state_dict['next_state']
            self.done[:self.size] = state_dict['done']


# =====================================================================
# 3. AGENTE DDQN
# =====================================================================
class DDQNAgent:
    def __init__(self, state_dim=10, num_actions=5):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.num_actions = num_actions
        self.state_dim = state_dim

        self.policy_net = DDQNNetwork(state_dim, num_actions).to(self.device)
        self.target_net = DDQNNetwork(state_dim, num_actions).to(self.device)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()

        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=1e-3)
        self.memory = ReplayBuffer(state_dim=self.state_dim, capacity=100000)

        self.batch_size = 256 #ter mais informação
        self.gamma = 0.99
        
        self.action_space = {
            0: (0.15, 0.0),    # FRENTE: Avança de forma segura e constante
            1: (0.0, 1.2),     # ESQUERDA: Giro no próprio eixo (yaw)
            2: (0.0, -1.2),    # DIREITA: Giro no próprio eixo (yaw)
            3: (0.15, 0.6),    # FRENTE-ESQUERDA: Curva suave, velocidade angular reduzida pela metade
            4: (0.15, -0.6)    # FRENTE-DIREITA: Curva suave, velocidade angular reduzida pela metade
        }

    def select_action(self, state, epsilon=0.1):
        if random.random() < epsilon:
            return random.randint(0, self.num_actions - 1)
        
        state_tensor = torch.as_tensor(state, dtype=torch.float32, device=self.device).unsqueeze(0)
        with torch.no_grad():
            q_values = self.policy_net(state_tensor)
            return q_values.argmax().item()

    def get_velocities(self, action_id):
        return self.action_space[action_id]

    def update(self):
        if len(self.memory) < self.batch_size:
            return {"status": "SKIPPED", "message": "Memória insuficiente para formar um batch."}

        states, actions, rewards, next_states, dones = self.memory.sample(self.batch_size)

        states = torch.as_tensor(states, device=self.device)
        actions = torch.as_tensor(actions, device=self.device)
        rewards = torch.as_tensor(rewards, device=self.device)
        next_states = torch.as_tensor(next_states, device=self.device)
        dones = torch.as_tensor(dones, device=self.device)

        current_q = self.policy_net(states).gather(1, actions)

        with torch.no_grad():
            next_actions = self.policy_net(next_states).argmax(1, keepdim=True)
            next_q = self.target_net(next_states).gather(1, next_actions)
            target_q = rewards + (self.gamma * next_q * (1 - dones))

        loss = F.mse_loss(current_q, target_q)

        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.policy_net.parameters(), max_norm=1.0)
        self.optimizer.step()

        return {"status": "SUCCESS", "loss": loss.item()}

    def hard_update_target(self):
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()
        return {"status": "SUCCESS", "message": "Target Network atualizada com sucesso."}

    # =================================================================
    # MÉTODOS DE SALVAMENTO E CARREGAMENTO
    # =================================================================
    
    def save_models(self, folder="models/dqn", filename="ddqn_pioneer", episode=None):
        try:
            os.makedirs(folder, exist_ok=True)
            filepath_latest = os.path.join(folder, f"{filename}_policy.pth")
            torch.save(self.policy_net.state_dict(), filepath_latest)
            
            msg = f"Pesos salvos em: {filepath_latest}"
            
            if episode is not None:
                filepath_versioned = os.path.join(folder, f"{filename}_policy_ep{episode}.pth")
                torch.save(self.policy_net.state_dict(), filepath_versioned)
                msg += f" | Backup de episódio salvo: {filepath_versioned}"
                
            return {"status": "SUCCESS", "message": msg}
        except Exception as e:
            return {"status": "ERROR", "message": f"Falha ao salvar pesos: {str(e)}"}

    def load_models(self, folder="models/dqn", filename="ddqn_pioneer"):
        """ Use este método quando quiser testar a rede em um NOVO CENÁRIO (Sem carregar a memória antiga) """
        filepath = os.path.join(folder, f"{filename}_policy.pth")
        if os.path.exists(filepath):
            try:
                state_dict = torch.load(filepath, map_location=self.device, weights_only=True)
                self.policy_net.load_state_dict(state_dict)
                self.target_net.load_state_dict(self.policy_net.state_dict())
                self.target_net.eval()
                return {"status": "SUCCESS", "message": f"Pesos carregados com sucesso de: {filepath}"}
            except Exception as e:
                return {"status": "ERROR", "message": f"Arquivo corrompido ou erro na leitura: {str(e)}"}
        else:
            return {"status": "NOT_FOUND", "message": f"Arquivo não encontrado em: {filepath}. Iniciando do zero."}

    def save_checkpoint(self, epsilon, episode, step=0, folder="models/dqn", filename="ddqn_checkpoint.pth"):
        try:
            os.makedirs(folder, exist_ok=True)
            filepath = os.path.join(folder, filename)
            
            checkpoint = {
                'policy_net_state_dict': self.policy_net.state_dict(),
                'optimizer_state_dict': self.optimizer.state_dict(),
                'epsilon': epsilon,
                'episode': episode,
                'step': step,
                'replay_buffer': self.memory.state_dict() # Agora isso vai funcionar perfeitamente!
            }
            
            torch.save(checkpoint, filepath)
            return {"status": "SUCCESS", "message": f"Checkpoint completo salvo em: {filepath}"}
        except Exception as e:
            return {"status": "ERROR", "message": f"Falha ao salvar checkpoint: {str(e)}"}

    def load_checkpoint(self, folder="models/dqn", filename="ddqn_checkpoint.pth"):
        """ Use este método quando quiser CONTINUAR o treinamento de onde parou no MESMO CENÁRIO """
        filepath = os.path.join(folder, filename)
        if os.path.exists(filepath):
            try:
                checkpoint = torch.load(filepath, map_location=self.device, weights_only=False)
                
                self.policy_net.load_state_dict(checkpoint['policy_net_state_dict'])
                self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
                self.target_net.load_state_dict(self.policy_net.state_dict())
                self.target_net.eval()
                
                msg = f"Checkpoint carregado de: {filepath}."
                if 'replay_buffer' in checkpoint:
                    self.memory.load_state_dict(checkpoint['replay_buffer'])
                    msg += f" Buffer restaurado ({len(self.memory)} transições)."
                    
                return {
                    "status": "SUCCESS", 
                    "message": msg,
                    "data": (checkpoint.get('epsilon'), checkpoint.get('episode'), checkpoint.get('step', 0))
                }
            except Exception as e:
                return {
                    "status": "ERROR", 
                    "message": f"Erro ao ler checkpoint: {str(e)}",
                    "data": (None, None, 0)
                }
        else:
            return {
                "status": "NOT_FOUND", 
                "message": f"Nenhum checkpoint encontrado em: {filepath}",
                "data": (None, None, 0)
            }