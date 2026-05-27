from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import numpy as np 

def astar(grid_map, start_world, goal_world):
    start_row, start_col = grid_map.world_to_grid(start_world[0], start_world[1])
    goal_row, goal_col = grid_map.world_to_grid(goal_world[0], goal_world[1])
    
    # O pacote 'pathfinding' inverte a lógica: 1 é livre, 0 é obstáculo
    inverse_matrix = np.where(grid_map.matrix == 1, 0, 1)
    
    # Instancia o grid deles
    grid = Grid(matrix=inverse_matrix.tolist())
    
    start_node = grid.node(start_col, start_row)
    end_node = grid.node(goal_col, goal_row)
    
    # Configura o localizador permitindo andar em diagonais
    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
    path, runs = finder.find_path(start_node, end_node, grid)
    
    if not path:
        return None
        
    # Converte de volta para metros
    world_path = []
    for node in path:
        cx, cy = grid_map.grid_to_world(node.y, node.x)
        world_path.append([cx, cy])
        
    return np.array(world_path)