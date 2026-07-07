import numpy as np

class PotentialFieldPlanner:
    def __init__(self, k_att=1.2, k_rep=0.8, rho_0=0.3, dead_zone=0.1):
        self.k_att = k_att
        self.k_rep = k_rep
        self.rho_0 = rho_0              # Região de influência dos obstáculos (m)
        self.dead_zone = dead_zone      # Zona morta / Tolerância de chegada ao objetivo (m)
        
        # Variável para suavização
        self.last_force = np.zeros(2)
        
        # Variáveis de estado para armazenar as forças (essencial para o Plotly/Matplotlib)
        self.f_att = np.zeros(2)
        self.f_rep = np.zeros(2)
        self.f_total = np.zeros(2)

    def compute_attractive_force(self, current_pos, goal_pos):
        error = goal_pos - current_pos
        dist = np.linalg.norm(error)
        
        if dist < 1e-6:
            return np.zeros(2)
        
        mag = min(self.k_att * dist, 2.0)
        return mag * (error / dist)

    def compute_repulsive_force(self, current_pos, goal_pos, lidar_points, static_obstacles):
        obs_arrays = []

        if lidar_points is not None and len(lidar_points) > 0:
            obs_arrays.append(lidar_points[:, :2])

        if static_obstacles:
            for obs in static_obstacles:
                if 'corners' in obs and len(obs['corners']) > 0:
                    obs_arrays.append(np.array(obs['corners']))

        if not obs_arrays:
            return np.zeros(2)

        all_pts = np.vstack(obs_arrays)
        diff = current_pos - all_pts
        dists = np.linalg.norm(diff, axis=1)

        # Qualquer leitura menor que 0.15m (raio do robô) é considerada ruído do sensor (raio vazio = 0.0)
        valid_points_mask = dists > 0.15

        # Utiliza o rho_0 como limite superior da região de influência e o valid_points_mask como limite inferior
        mask = (dists <= self.rho_0) & valid_points_mask
        
        if not np.any(mask):
            return np.zeros(2)

        valid_diff = diff[mask]
        valid_dists = dists[mask]

        dist_to_goal = np.linalg.norm(goal_pos - current_pos)
        gnron_factor = min(dist_to_goal, 1.0)

        # Cálculo da magnitude original mantido
        rep_magnitudes = self.k_rep * ((1.0 / valid_dists) - (1.0 / self.rho_0)) / (valid_dists ** 2)
        rep_magnitudes *= gnron_factor

        directions = valid_diff / valid_dists[:, None]
        f_rep = np.sum(directions * rep_magnitudes[:, None], axis=0)
        
        # Impede que a força atinja valores exorbitantes que quebrem a estabilidade do PID.
        f_rep_norm = np.linalg.norm(f_rep)
        max_rep_force = 5.0  # Ajuste esse valor dependendo de quão agressiva você quer a repulsão
        
        if f_rep_norm > max_rep_force:
            f_rep = (f_rep / f_rep_norm) * max_rep_force

        return f_rep
    
    def compute_force_vector(self, current_pos, goal_pos, lidar_points, static_obstacles):
        """
        Calcula e retorna apenas o vetor de força resultante.
        A conversão para velocidades é responsabilidade dos controladores PID externos.
        """
        curr_2d = np.array(current_pos[:2])
        goal_2d = np.array(goal_pos[:2])
        
        dist_goal = np.linalg.norm(goal_2d - curr_2d)
        
        # ALTERAÇÃO: Agora utiliza a zona morta configurável para zerar tudo e parar
        if dist_goal < self.dead_zone:
            self.f_att = np.zeros(2)
            self.f_rep = np.zeros(2)
            self.f_total = np.zeros(2)
            self.last_force = np.zeros(2)
            return self.f_total
            
        f_att = self.compute_attractive_force(curr_2d, goal_2d)
        f_rep = self.compute_repulsive_force(curr_2d, goal_2d, lidar_points, static_obstacles)
        
        f_total = f_att + f_rep
        
        # Suavização da força (Filtro passa-baixa mantido para evitar saltos bruscos no plot e no PID)
        alpha_smooth = 0.3
        f_total = alpha_smooth * self.last_force + (1.0 - alpha_smooth) * f_total
        self.last_force = f_total.copy()
        
        # Atualiza os atributos internos para leitura pela interface gráfica
        self.f_att = f_att
        self.f_rep = f_rep
        self.f_total = f_total
        
        return f_total