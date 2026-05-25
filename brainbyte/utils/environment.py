import numpy as np

def get_environment_obstacles(sim, botname=['turtlebot', 'robot'], keywords=['block', 'pillar', 'wall', 'cuboid', 'parede', 'pilar']):
    """
    Busca todas as formas geométricas estáticas da cena do CoppeliaSim.
    
    Retorna:
        Uma lista de dicionários contendo a posição, dimensões e rotação (em radianos) de cada objeto.
    """
    obstacles_data = []
    
    try:
        # Pega todas as formas (shapes) presentes na cena
        all_shapes = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type)
        
        for handle in all_shapes:
            # Pega o alias limpo (options=0) e converte para minúsculo
            alias = sim.getObjectAlias(handle, 0).lower()
            
            # Ignora componentes do robô usando busca por substring
            if any(bot in alias for bot in botname):
                continue
            
            # Filtra pelos nomes definidos nas palavras-chave
            if any(kw in alias for kw in keywords):
                pos = sim.getObjectPosition(handle, -1)
                ori = sim.getObjectOrientation(handle, -1)
                angle_rad = ori[2] # Mantém em radianos para evitar conversões redundantes
                
                # Valores padrão de fallback
                width, height = 0.5, 0.5 
                
                try:

                    # Retorna (size, pose), onde size é [sizeX, sizeY, sizeZ]
                    size, _ = sim.getShapeBB(handle)
                    if size:
                        width = size[0]  # Dimensão X
                        height = size[1] # Dimensão Y
                except Exception:
                    if 'wall' in alias or 'parede' in alias:
                        width, height = 2.0, 0.1
                
                obstacles_data.append({
                    'x': pos[0],
                    'y': pos[1],
                    'w': width,
                    'h': height,
                    'angle': angle_rad # Salvando diretamente em radianos
                })
                
    except Exception as e:
        print(f"[ERROR] Erro ao mapear cenário no módulo externo: {e}")
        
    return obstacles_data

def get_obb_corners(obs):
    # OBS: O ângulo agora já chega diretamente em radianos
    theta = obs['angle']
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    
    hw = obs['w'] / 2.0
    hh = obs['h'] / 2.0
    
    locais = np.array([
        [-hw, -hh],
        [ hw, -hh],
        [ hw,  hh],
        [-hw,  hh]
    ])
    
    rot_matrix = np.array([
        [cos_t, -sin_t],
        [sin_t,  cos_t]
    ])
    
    centro = np.array([obs['x'], obs['y']])
    corners = (locais @ rot_matrix.T) + centro
    
    return corners.tolist()


def get_robot_radius(sim, base_shape_name='Turtlebot3/base_link'):
    """
    Busca o handle da carcaça principal do robô e calcula o raio do círculo
    que engloba perfeitamente o robô (Bounding Circle).
    """
    try:
        # Pega o ID (handle) apenas da forma geométrica da base do robô
        robot_handle = sim.getObject(f'/{base_shape_name}')
        
        # Pega as dimensões X, Y, Z da Bounding Box dessa peça
        size, _ = sim.getShapeBB(robot_handle)
        
        if size:
            w = size[0]
            h = size[1]
            
            # Calcula o raio do círculo circunscrito para cobrir as quinas
            # R = sqrt((w/2)^2 + (h/2)^2)
            radius = np.sqrt((w / 2.0)**2 + (h / 2.0)**2)
            
            print(f"[INFO] Dimensões do robô obtidas: w={w:.3f}m, h={h:.3f}m -> Raio Seguro={radius:.3f}m")
            return radius
            
    except Exception as e:
        print(f"[ERROR] Erro ao obter dimensões do robô: {e}")
        
    # Fallback genérico caso a leitura falhe (ex: 15cm para um Turtlebot padrão)
    return 0.15