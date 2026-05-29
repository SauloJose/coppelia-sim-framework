import numpy as np

class PotentialFieldPlanner:
    def __init__(self, k_att=1.2, k_rep=0.8, rho_0=1.0, step_size=0.15):
        self.k_att = k_att
        self.k_rep = k_rep
        self.rho_0 = rho_0
        self.step_size = step_size
        self.last_force = np.zeros(2)

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
        dists = np.maximum(dists, 0.05)

        mask = dists <= self.rho_0
        if not np.any(mask):
            return np.zeros(2)

        valid_diff = diff[mask]
        valid_dists = dists[mask]

        dist_to_goal = np.linalg.norm(goal_pos - current_pos)
        gnron_factor = min(dist_to_goal, 1.0)

        # A matemática está correta aqui:
        rep_magnitudes = self.k_rep * ((1.0 / valid_dists) - (1.0 / self.rho_0)) / (valid_dists ** 2)
        rep_magnitudes *= gnron_factor

        directions = valid_diff / valid_dists[:, None]
        f_rep = np.sum(directions * rep_magnitudes[:, None], axis=0)
        
        return f_rep

    def get_next_target(self, current_pos, goal_pos, lidar_points, static_obstacles):
        curr_2d = np.array(current_pos[:2])
        goal_2d = np.array(goal_pos[:2])
        
        dist_goal = np.linalg.norm(goal_2d - curr_2d)
        if dist_goal < 0.05:
            return np.array([goal_2d[0], goal_2d[1], 0.0])
            
        f_att = self.compute_attractive_force(curr_2d, goal_2d)
        f_rep = self.compute_repulsive_force(curr_2d, goal_2d, lidar_points, static_obstacles)
        
        f_total = f_att + f_rep
        
        alpha_smooth = 0.3
        f_total = alpha_smooth * self.last_force + (1.0 - alpha_smooth) * f_total
        self.last_force = f_total.copy()
        
        mag_total = np.linalg.norm(f_total)
        if mag_total > self.step_size:
            f_total = (f_total / mag_total) * self.step_size
            
        next_pos_2d = curr_2d + f_total
        return np.array([next_pos_2d[0], next_pos_2d[1], 0.0])