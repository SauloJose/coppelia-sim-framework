import math
import random
import numpy as np
from shapely.geometry import LineString, Polygon

class Node:
    """Classe que representa um ponto (nó) na árvore do RRT"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None # Guarda de qual nó ele veio (para traçar a rota de volta)

class RRTPlanner:
    def __init__(self, bounds, obstacles, step_size=0.5, max_iter=2000, goal_sample_rate=0.1):
        """
        bounds: tupla (x_min, x_max, y_min, y_max)
        obstacles: lista de polígonos (listas de vértices)
        step_size: tamanho máximo do galho a cada iteração
        max_iter: limite de tentativas para encontrar o alvo
        goal_sample_rate: % de chance de amostrar o objetivo diretamente (viés)
        """
        self.x_min, self.x_max, self.y_min, self.y_max = bounds
        self.obstacles = [Polygon(obs) for obs in obstacles if len(obs) >= 3]
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_sample_rate = goal_sample_rate
        self.nodes = []

    def find_path(self, start_arr, goal_arr):
        start_node = Node(start_arr[0], start_arr[1])
        goal_node = Node(goal_arr[0], goal_arr[1])
        self.nodes = [start_node]

        for i in range(self.max_iter):
            # 1. Sorteia um ponto no mapa
            rand_node = self._get_random_node(goal_node)
            
            # 2. Acha o nó da árvore mais perto do ponto sorteado
            nearest_node = self._get_nearest_node(self.nodes, rand_node)
            
            # 3. Dá um "passo" em direção ao ponto sorteado
            new_node = self._steer(nearest_node, rand_node)

            # 4. Verifica se o passo esbarra em um obstáculo
            if not self._check_collision(nearest_node, new_node):
                self.nodes.append(new_node)

                # 5. Verifica se chegou perto do objetivo
                if self._calc_dist(new_node, goal_node) <= self.step_size:
                    if not self._check_collision(new_node, goal_node):
                        goal_node.parent = new_node
                        self.nodes.append(goal_node)
                        return self._extract_path(goal_node)
        
        return None # Falhou em encontrar após max_iter tentativas

    def _get_random_node(self, goal_node):
        """Sorteia um ponto com um pequeno viés (tendência) para ir direto ao alvo"""
        if random.random() < self.goal_sample_rate:
            return Node(goal_node.x, goal_node.y)
        return Node(random.uniform(self.x_min, self.x_max),
                    random.uniform(self.y_min, self.y_max))

    def _get_nearest_node(self, node_list, target_node):
        """Encontra o galho mais próximo usando distância euclidiana"""
        dists = [self._calc_dist(node, target_node) for node in node_list]
        min_idx = dists.index(min(dists))
        return node_list[min_idx]

    def _steer(self, from_node, to_node):
        """Cresce a árvore de from_node em direção a to_node no tamanho do step_size"""
        new_node = Node(from_node.x, from_node.y)
        dist, theta = self._calc_dist_and_angle(from_node, to_node)

        new_node.x += min(self.step_size, dist) * math.cos(theta)
        new_node.y += min(self.step_size, dist) * math.sin(theta)
        new_node.parent = from_node
        return new_node

    def _check_collision(self, node1, node2):
        """Cria uma linha reta entre os nós e verifica se corta algum polígono do Shapely"""
        line = LineString([(node1.x, node1.y), (node2.x, node2.y)])
        for obs in self.obstacles:
            if line.intersects(obs):
                return True # Bateu
        return False # Livre

    def _calc_dist(self, node1, node2):
        return math.hypot(node1.x - node2.x, node1.y - node2.y)

    def _calc_dist_and_angle(self, node1, node2):
        dx = node2.x - node1.x
        dy = node2.y - node1.y
        dist = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return dist, theta

    def _extract_path(self, goal_node):
        """Volta pelo 'parent' de cada nó do objetivo até o início para traçar a rota final"""
        path = []
        current = goal_node
        while current is not None:
            path.append([current.x, current.y])
            current = current.parent
        return path[::-1] # Inverte a lista para ficar: Início -> Fim