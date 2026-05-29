import numpy as np
from shapely.geometry import LineString, Polygon

import numpy as np
from shapely.geometry import LineString, Polygon, Point

class BugPlanner:
    def __init__(self, start_point, target_point, safety_distance=0.5, final_tolerance=0.05):
        self.start_point = np.array(start_point)[:2]
        self.target_point = np.array(target_point)[:2]
        self.safety_distance = safety_distance
        self.position_tolerance = final_tolerance

        # BUG2: Linha imaginária fixa do ponto de partida ao alvo (m-line)
        self.m_line = LineString([self.start_point, self.target_point])
        
        # Variável fundamental do BUG2: Distância ao alvo no momento do impacto
        self.hit_dist_to_goal = float('inf')  

        self.STATE_GO_TO_GOAL = "GO_TO_GOAL"
        self.STATE_WALL_FOLLOWING = "WALL_FOLLOWING"
        self.STATE_TURN_CORNER = "TURN_CORNER"
        self.current_state = self.STATE_GO_TO_GOAL 

        self.obstacles_data = [] 

    def _is_path_clear(self, pos):
        """
        Verifica se a linha reta entre o robô e o alvo cruza o interior de algum obstáculo.
        """
        if not self.obstacles_data:
            return True 

        linha_para_alvo = LineString([pos[:2], self.target_point])
        
        for obs in self.obstacles_data:
            poly = Polygon(obs['corners'])
            intersecao = poly.intersection(linha_para_alvo)
            
            # Se a linha penetra mais de 2cm no polígono, o caminho não está livre.
            if intersecao.length > 0.02:
                return False
                
        return True

    def _distance_to_target(self, pos):
        return np.hypot(self.target_point[0] - pos[0], self.target_point[1] - pos[1])
        
    def _distance_to_m_line(self, pos):
        """Calcula a menor distância ortogonal do robô até a m-line"""
        return Point(pos[:2]).distance(self.m_line)

    def update(self, actual_pos, obstacle_in_front, wall_distance, v_goal, w_goal, obstacles_data):
        self.obstacles_data = obstacles_data 
        dist_to_goal = self._distance_to_target(actual_pos)

        # 1. Condição de parada global
        if dist_to_goal < self.position_tolerance:
            return 0.0, 0.0, "ARRIVED"

        # ==========================================
        # ESTADO: INDO DIRETO PARA O OBJETIVO
        # ==========================================
        if self.current_state == self.STATE_GO_TO_GOAL:
            if obstacle_in_front:
                self.current_state = self.STATE_WALL_FOLLOWING
                # BUG2: Registra a distância ao alvo EXATAMENTE no ponto de impacto
                self.hit_dist_to_goal = dist_to_goal
                return 0.0, 0.2, self.current_state
            else:
                return v_goal, w_goal, self.current_state

        # ==========================================
        # ESTADO: SEGUINDO PAREDE
        # ==========================================
        elif self.current_state == self.STATE_WALL_FOLLOWING:
            
            # NOVA CONDIÇÃO DE SAÍDA (Lógica Clássica do BUG2):
            dist_to_m_line = self._distance_to_m_line(actual_pos)
            
            # Condição 1: Robô cruzou a linha inicial (tolerância de 5cm para o erro de odometria)
            on_m_line = dist_to_m_line < 0.05 
            
            # Condição 2: Está mais perto do alvo do que quando bateu (Histerese de 5cm para evitar loop infinito na mesma quina)
            closer_to_goal = dist_to_goal < (self.hit_dist_to_goal - 0.05) 

            # BUG 2 Exige que cruze a m-line mais perto do alvo. O _is_path_clear atua como fail-safe final.
            if on_m_line and closer_to_goal and self._is_path_clear(actual_pos):
                self.current_state = self.STATE_GO_TO_GOAL
                return v_goal, w_goal, self.current_state

            # Lógica reativa de contorno de parede
            if obstacle_in_front:
                return 0.0, 0.1, self.current_state
            elif wall_distance == float('inf') or wall_distance > (self.safety_distance * 1.5):
                self.current_state = self.STATE_TURN_CORNER
                return 0.1, -0.05, self.current_state
            else:
                v_cmd = 0.08
                erro_d = wall_distance - self.safety_distance
                w_cmd = -erro_d * 2.5 
                return v_cmd, w_cmd, self.current_state

        # ==========================================
        # ESTADO: TURN_CORNER (Contorno de Quina Externa)
        # ==========================================
        elif self.current_state == self.STATE_TURN_CORNER:
            if obstacle_in_front or wall_distance < (self.safety_distance * 1.2):
                self.current_state = self.STATE_WALL_FOLLOWING
                return 0.0, 0.0, self.current_state
            else:
                return 0.08, -0.2, self.current_state

        return 0.0, 0.0, self.current_state