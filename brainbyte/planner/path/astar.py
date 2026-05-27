import numpy as np 
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

def simplify_path(full_path):
    """
    Filters the A* path keeping only the start, end, and turning points.
    """
    if full_path is None or len(full_path) <= 2:
        return full_path

    simplified_path = [full_path[0]]

    for i in range(1, len(full_path) - 1):
        v1 = full_path[i] - full_path[i-1]
        v2 = full_path[i+1] - full_path[i]
        
        # 2D cross product to detect direction changes
        cross_product = v1[0] * v2[1] - v1[1] * v2[0]
        
        if not np.isclose(cross_product, 0.0, atol=1e-5):
            simplified_path.append(full_path[i])

    simplified_path.append(full_path[-1])
    return np.array(simplified_path)


def astar(grid_map, start_world, goal_world):
    """
    Executes A* search and returns the simplified path in world coordinates (meters).
    """
    if grid_map is None:
        return None

    start_row, start_col = grid_map.world_to_grid(start_world[0], start_world[1])
    goal_row, goal_col = grid_map.world_to_grid(goal_world[0], goal_world[1])
    
    # Validação preventiva: verifica se os pontos estão dentro das dimensões do mapa
    rows, cols = grid_map.matrix.shape
    if not (0 <= start_row < rows and 0 <= start_col < cols) or \
       not (0 <= goal_row < rows and 0 <= goal_col < cols):
        print("WARNING: Start or Goal is outside the grid bounds!")
        return None
    
    # Validação de obstáculo usando sua lógica (1 = bloqueado)
    if grid_map.matrix[start_row, start_col] == 1:
        print("WARNING: Start position is blocked by an obstacle!")
        return None
        
    if grid_map.matrix[goal_row, goal_col] == 1:
        print("WARNING: Destination is blocked by an obstacle!")
        return None
    
    inverse_matrix = 1 - grid_map.matrix
    grid = Grid(matrix=inverse_matrix.tolist())
    
    start_node = grid.node(start_col, start_row)
    end_node = grid.node(goal_col, goal_row)
    
    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
    path, _ = finder.find_path(start_node, end_node, grid)
    
    if not path or len(path) == 0:
        print("WARNING: No path found!")
        return None
        
    world_path = []
    for node in path:
        # Retorna para o formato do seu grid_map (linha=node.y, coluna=node.x)
        cx, cy = grid_map.grid_to_world(node.y, node.x)
        world_path.append([cx, cy])
        
    return np.array(world_path)
