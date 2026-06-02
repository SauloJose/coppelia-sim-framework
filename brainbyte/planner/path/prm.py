import numpy as np
from sklearn.neighbors import NearestNeighbors
from matplotlib.path import Path
import heapq

# Bibliotecas de grid mantidas conforme solicitado
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
 
class PRMPlanner:
    def __init__(self, num_samples, k_neighbors, bounds, obstacles):
        """
        :param num_samples: Quantidade de pontos aleatórios a serem gerados.
        :param k_neighbors: Número de vizinhos mais próximos para conectar (k-NN).
        :param bounds: Tupla (x_min, x_max, y_min, y_max) definindo os limites do mapa.
        :param obstacles: Lista de polígonos, onde cada polígono é uma lista/array de vértices (x, y).
        """
        self.num_samples = num_samples
        self.k_neighbors = k_neighbors
        self.bounds = bounds
        self.obstacles = [Path(obs) for obs in obstacles] # Usa matplotlib Path para colisões
        self.nodes = []
        self.graph = {} # Dicionário de adjacência: {node_idx: [(neighbor_idx, cost), ...]}

    def _is_point_free(self, point):
        """Verifica se um ponto está fora de todos os obstáculos."""
        for obs in self.obstacles:
            if obs.contains_point(point):
                return False
        return True

    def _is_edge_free(self, p1, p2, steps=10):
        """Verifica se a linha reta entre dois pontos cruza algum obstáculo."""
        for t in np.linspace(0, 1, steps):
            pt = p1 + t * (p2 - p1)
            if not self._is_point_free(pt):
                return False
        return True

    def build_roadmap(self, start, goal):
        """Gera os nós aleatórios e constrói o grafo conectando vizinhos próximos."""
        x_min, x_max, y_min, y_max = self.bounds
        
        # 1. Amostragem de pontos (Sampling)
        self.nodes = [np.array(start), np.array(goal)]
        while len(self.nodes) < self.num_samples + 2:
            rand_pt = np.array([np.random.uniform(x_min, x_max), 
                                np.random.uniform(y_min, y_max)])
            if self._is_point_free(rand_pt):
                self.nodes.append(rand_pt)
                
        self.nodes = np.array(self.nodes)
        self.graph = {i: [] for i in range(len(self.nodes))}

        # 2. Conectando os nós usando k-Nearest Neighbors
        nbrs = NearestNeighbors(n_neighbors=self.k_neighbors, algorithm='kd_tree').fit(self.nodes)
        distances, indices = nbrs.kneighbors(self.nodes)

        for i in range(len(self.nodes)):
            for j, dist in zip(indices[i], distances[i]):
                if i != j and self._is_edge_free(self.nodes[i], self.nodes[j]):
                    self.graph[i].append((j, dist))
                    # Adiciona a aresta de volta (grafo não direcionado)
                    if (i, dist) not in self.graph[j]:
                        self.graph[j].append((i, dist))

    def heuristic(self, p1, p2):
        """Calcula a distância Euclidiana (Heurística para o A*)."""
        return np.linalg.norm(p1 - p2)

    def find_path(self):
        """
        Executa a busca A* sobre o grafo gerado usando heapq.
        Retorna a lista de waypoints (coordenadas 2D) se achar caminho, senão None.
        Nota: O nó 0 sempre é o Início, e o nó 1 sempre é o Objetivo.
        """
        start_idx = 0
        goal_idx = 1
        
        # Fila de prioridade armazena tuplas: (f_score, current_node)
        open_set = []
        heapq.heappush(open_set, (0.0, start_idx))
        
        # Dicionários de custo e caminho
        came_from = {}
        g_score = {i: float('inf') for i in range(len(self.nodes))}
        g_score[start_idx] = 0.0

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_idx:
                # Reconstrói o caminho de trás para frente
                path = []
                while current in came_from:
                    path.append(self.nodes[current])
                    current = came_from[current]
                path.append(self.nodes[start_idx])
                return np.array(path[::-1]) # Inverte a lista

            for neighbor, cost in self.graph[current]:
                tentative_g = g_score[current] + cost

                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    
                    # f_score = g_score + heurística (distância direta até o alvo)
                    f_score = tentative_g + self.heuristic(self.nodes[neighbor], self.nodes[goal_idx])
                    heapq.heappush(open_set, (f_score, neighbor))

        # Retorna None caso a fila esvazie e não encontre o alvo
        return None
