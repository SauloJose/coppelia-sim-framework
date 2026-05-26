import numpy as np
from shapely.geometry import LineString, Polygon

class BugPlanner:
    def __init__(self, target_point, safety_distance=0.5, final_tolerance=0.05):
        self.target_point = np.array(target_point)[:2]
        self.safety_distance = safety_distance
        self.position_tolerance = final_tolerance

        self.STATE_GO_TO_GOAL = "GO_TO_GOAL"
        self.STATE_WALL_FOLLOWING = "WALL_FOLLOWING"
        self.STATE_TURN_CORNER = "TURN_CORNER"
        self.current_state = self.STATE_GO_TO_GOAL 

        # Nova variável para receber o mapa global (polígonos inflados)
        self.obstacles_data = [] 

    def _is_path_clear(self, pos):
        """
        Equivalente ao `polyxpoly` e `isempty()` do MATLAB.
        Verifica se a linha reta entre o robô e o alvo cruza o interior de algum obstáculo.
        """
        if not self.obstacles_data:
            return True # Sem mapa, confia apenas nos sensores

        # CORREÇÃO AQUI: Força o uso apenas de X e Y do robô (pos[:2])
        linha_para_alvo = LineString([pos[:2], self.target_point])
        
        for obs in self.obstacles_data:
            poly = Polygon(obs['corners'])
            
            # Calculamos a interseção geométrica. 
            # Se a linha penetra mais de 2cm no polígono inflado, o caminho não está livre.
            intersecao = poly.intersection(linha_para_alvo)
            if intersecao.length > 0.02:
                return False
                
        return True

    def _distance_to_target(self, pos):
        return np.hypot(self.target_point[0] - pos[0], self.target_point[1] - pos[1])

    def update(self, actual_pos, obstacle_in_front, wall_distance, v_goal, w_goal, obstacles_data):
        # Atualiza a percepção do mapa
        self.obstacles_data = obstacles_data 
        dist_to_goal = self._distance_to_target(actual_pos)

        # 1. Condição de parada global
        if dist_to_goal < self.position_tolerance:
            return 0.0, 0.0, "ARRIVED"

        # ==========================================
        # ESTADO: INDO DIRETO PARA O OBJETIVO
        # ==========================================
        if self.current_state == self.STATE_GO_TO_GOAL:
            # Equivalente ao `inpolygon` do MATLAB, mas usando sensor real
            if obstacle_in_front:
                self.current_state = self.STATE_WALL_FOLLOWING
                # Para e gira imediatamente para iniciar o contorno
                return 0.0, 0.2, self.current_state
            else:
                return v_goal, w_goal, self.current_state

        # ==========================================
        # ESTADO: SEGUINDO PAREDE
        # ==========================================
        elif self.current_state == self.STATE_WALL_FOLLOWING:
            
            # NOVA CONDIÇÃO DE SAÍDA INSPIRADA NO MATLAB: Linha de Visão Direta Livre
            # Se houver um caminho reto para o alvo sem bater em nada, abandone a parede imediatamente.
            if self._is_path_clear(actual_pos):
                self.current_state = self.STATE_GO_TO_GOAL
                return v_goal, w_goal, self.current_state

            # Lógica reativa de contorno (Mantida via Lidar)
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