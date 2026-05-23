import numpy as np

def get_environment_obstacles(sim, keywords=['block', 'pillar', 'wall', 'cuboid', 'parede', 'pilar']):
    """
    Busca todas as formas geométricas estáticas da cena do CoppeliaSim.
    
    Parâmetros:
        sim: Instância de conexão com a API do CoppeliaSim (self.sim).
        keywords: Lista de palavras-chave para filtrar os objetos pelo nome.
        
    Retorna:
        Uma lista de dicionários contendo a posição, dimensões e rotação de cada objeto:
        [{'x': float, 'y': float, 'w': float, 'h': float, 'angle': float}, ...]
    """
    obstacles_data = []
    
    try:
        # Pega todas as formas (shapes) presentes na cena
        all_shapes = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type)
        
        for handle in all_shapes:
            alias = sim.getObjectAlias(handle, -1).lower()
            
            # Ignora componentes do robô
            if 'turtlebot' in alias or 'robot' in alias:
                continue
            
            # Filtra pelos nomes definidos nas palavras-chave
            if any(kw in alias for kw in keywords):
                pos = sim.getObjectPosition(handle, -1)
                ori = sim.getObjectOrientation(handle, -1)
                angle_deg = np.rad2deg(ori[2]) # Rotação no eixo Z
                
                # Valores padrão de fallback
                width, height = 0.5, 0.5 
                
                try:
                    geom_info = sim.getShapeGeomInfo(handle)
                    if geom_info:
                        dims = geom_info[-1] if isinstance(geom_info[-1], (list, tuple)) else geom_info
                        width = dims[0]  
                        height = dims[1] 
                except Exception:
                    if 'wall' in alias or 'parede' in alias:
                        width, height = 2.0, 0.1
                
                obstacles_data.append({
                    'x': pos[0],
                    'y': pos[1],
                    'w': width,
                    'h': height,
                    'angle': angle_deg
                })
                
    except Exception as e:
        print(f"[ERROR] Erro ao mapear cenário no módulo externo: {e}")
        
    return obstacles_data

def get_obb_corners(obs):
    # Converte o ângulo e calcula seno/cosseno
    theta = np.radians(obs['angle'])
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    
    # Metades das dimensões
    hw = obs['w'] / 2.0
    hh = obs['h'] / 2.0
    
    # Matriz com os 4 cantos locais (Formato 4x2)
    locais = np.array([
        [-hw, -hh],
        [ hw, -hh],
        [ hw,  hh],
        [-hw,  hh]
    ])
    
    # Matriz de rotação 2D
    rot_matrix = np.array([
        [cos_t, -sin_t],
        [sin_t,  cos_t]
    ])
    
    # Multiplicação de matrizes para rotacionar (@) e soma do vetor de translação
    # Usamos rot_matrix.T (transposta) para alinhar as dimensões na multiplicação
    centro = np.array([obs['x'], obs['y']])
    corners = (locais @ rot_matrix.T) + centro
    
    # Retorna como uma lista de tuplas para manter a mesma estrutura do seu código original
    return corners.tolist()