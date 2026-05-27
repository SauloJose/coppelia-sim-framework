import numpy as np
from numba import cuda
import math

# Kernel CUDA para expansão paralela de nós vizinhos
@cuda.jit
def gpu_expand_nodes_kernel(matrix, g_score, f_score, came_from_row, came_from_col, 
                            open_mask, closed_mask, goal_row, goal_col, changed):
    # Obtém a coordenada da célula que esta Thread vai processar
    col, row = cuda.grid(2)
    
    # Verifica se a thread está dentro dos limites da matriz
    if row >= matrix.shape[0] or col >= matrix.shape[1]:
        return
        
    # Se a célula atual não está na lista aberta (fronteira), não faz nada
    if not open_mask[row, col]:
        return
        
    # Retira da lista aberta atual e joga na fechada
    open_mask[row, col] = False
    closed_mask[row, col] = True
    
    current_g = g_score[row, col]
    
    # Varre os 8 vizinhos (ortogonais e diagonais)
    for dr in range(-1, 2):
        for dc in range(-1, 2):
            if dr == 0 and dc == 0:
                continue
                
            n_row = row + dr
            n_col = col + dc
            
            # Valida limites do grid e colisão com obstáculo (1)
            if (0 <= n_row < matrix.shape[0]) and (0 <= n_col < matrix.shape[1]):
                if matrix[n_row, n_col] == 1 or closed_mask[n_row, n_col]:
                    continue
                    
                # Peso do movimento: 1.414 para diagonal, 1.0 para reto
                weight = 1.41421356 if (dr != 0 and dc != 0) else 1.0
                tentative_g = current_g + weight
                
                # Se achou um caminho melhor para o vizinho
                if tentative_g < g_score[n_row, n_col]:
                    g_score[n_row, n_col] = tentative_g
                    
                    # Heurística Euclidiana calculada na GPU
                    h = math.sqrt((n_row - goal_row)**2 + (n_col - goal_col)**2)
                    f_score[n_row, n_col] = tentative_g + h
                    
                    # Registra o nó pai para reconstrução posterior
                    came_from_row[n_row, n_col] = row
                    came_from_col[n_row, n_col] = col
                    
                    # Coloca o vizinho na lista aberta para a próxima rodada
                    open_mask[n_row, n_col] = True
                    changed[0] = True # Avisa a CPU que o mapa ainda está expandindo