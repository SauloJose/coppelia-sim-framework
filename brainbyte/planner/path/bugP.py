import numpy as np
from numba import njit 

class BugPlanner:
    def __init__(self, target_point, 
                 safety_distance=0.5, 
                 hit_tolerance=0.15,
                 final_tolerance=0.05):
        
        self.target_point = np.array(target_point)[:2]
        self.start_point = None 
        
        self.safety_distance = safety_distance
        self.hit_tolerance = hit_tolerance
        self.position_tolerance = final_tolerance

        self.STATE_GO_TO_GOAL = "GO_TO_GOAL"
        self.STATE_WALL_FOLLOWING = "WALL_FOLLOWING"
        self.STATE_ARRIVED_ON_DESTINY = "ARRIVED"
        self.current_state = self.STATE_GO_TO_GOAL 

        self.hit_point = None 
        self._distance_to_goal_at_hit = float('inf') # Unificado com underline

    @njit 
    def _distance_to_m_line(self, pos):
        if self.start_point is None: 
            return 0.0
        x0, y0 = pos[0], pos[1]
        x1, y1 = self.start_point[0], self.start_point[1]
        x2, y2 = self.target_point[0], self.target_point[1]
        
        num = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
        den = np.hypot(y2 - y1, x2 - x1)
        return num / den if den != 0 else 0.0

    @njit 
    def _distance_to_target(self,pos):
        if self.start_point is None:
            return 0.0
        x0, y0 = pos[0], pos[1]
        x1, y1 = self.start_point[0], self.start_point[1]
        x2, y2 = self.target_point[0], self.target_point[1]

        return np.hypot(y2-y1, x2-x1)

    def update(self, actual_pos, obstacle_in_front, wall_distance, v_goal, w_goal):
        dist_to_goal = np.hypot(self.target_point[0] - actual_pos[0], self.target_point[1] - actual_pos[1])
        
        # 1. Condição de parada global
        if dist_to_goal < self.position_tolerance:
            return 0.0, 0.0, self.STATE_ARRIVED_ON_DESTINY
        
        # 2. Inicialização da M-Line
        if self.start_point is None:
            self.start_point = np.array([actual_pos[0], actual_pos[1]])

        # ==========================================
        # ESTADO: INDO DIRETO PARA O OBJETIVO
        # ==========================================
        if self.current_state == self.STATE_GO_TO_GOAL:
            if obstacle_in_front:
                self.current_state = self.STATE_WALL_FOLLOWING
                self.hit_point = np.array([actual_pos[0], actual_pos[1]])
                self._distance_to_goal_at_hit = dist_to_goal # CORRIGIDO: Atributo correto
                return 0.0, 0.0, self.current_state
            else:
                return v_goal, w_goal, self.current_state

        # ==========================================
        # ESTADO: SEGUINDO PAREDE (WALL FOLLOWING)
        # ==========================================
        elif self.current_state == self.STATE_WALL_FOLLOWING:
            dist_to_m_line = self._distance_to_m_line(actual_pos)
            dist_to_hit = np.hypot(actual_pos[0] - self.hit_point[0], actual_pos[1] - self.hit_point[1])
            
            # Condição rigorosa de saída (Bug2): Cruzou a M-line mais perto do alvo do que quando colidiu
            # O dist_to_hit > 0.4 garante que ele não saia imediatamente no milissegundo pós-colisão
            if dist_to_m_line < self.hit_tolerance and dist_to_hit > 0.4:
                if dist_to_goal < (self._distance_to_goal_at_hit - 0.1): # Margem de segurança de 10cm
                    self.current_state = self.STATE_GO_TO_GOAL
                    return v_goal, w_goal, self.current_state

            # Comportamento de Desvio Cinematáco
            if obstacle_in_front:
                v_cmd = 0.0   
                w_cmd = 0.2   # Otimizado: Giro rápido para a esquerda para livrar a quina frontal
            else:
                if wall_distance != float('inf'):
                    v_cmd = 0.12  # Velocidade linear segura de cruzeiro
                    erro_d = wall_distance - self.safety_distance
                    w_cmd = -erro_d * 2.0  # Ganho P aumentado para correções mais firmes
                else:
                    # Parede sumiu (encontrou uma quina de fuga para a direita)
                    v_cmd = 0.08  # Reduz a frente um pouco para fechar bem a curva
                    w_cmd = -0.2  # Força curva fechada para a direita para recuperar o objeto

            return v_cmd, w_cmd, self.current_state

        return 0.0, 0.0, self.current_state