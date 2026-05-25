import numpy as np
from shapely.geometry import Polygon as ShapelyPolygon

def get_environment_obstacles(sim, robot_radius=0.0, botname=['turtlebot', 'robot'], keywords=['block', 'pillar', 'wall', 'cuboid', 'parede', 'pilar']):
    """
    Busca todas as formas geométricas estáticas da cena do CoppeliaSim V4.1.
    Se 'robot_radius' for maior que zero, realiza a Soma de Minkowski para inflar os obstáculos.
    
    Retorna:
        Uma lista de dicionários contendo a posição, dimensões, rotação, vértices originais
         e os vértices inflados (C-Space).
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
                angle_rad = ori[2] # Mantém em radianos
                
                # Valores padrão de fallback
                width, height = 0.5, 0.5 
                
                try:
                    # Suporte para CoppeliaSim V4.1 (Verificação manual de limites)
                    def read_param(param_id):
                        val = sim.getObjectFloatParameter(handle, param_id)
                        return val[1] if isinstance(val, (tuple, list)) else val

                    max_x = read_param(sim.objfloatparam_objbbox_max_x)
                    min_x = read_param(sim.objfloatparam_objbbox_min_x)
                    max_y = read_param(sim.objfloatparam_objbbox_max_y)
                    min_y = read_param(sim.objfloatparam_objbbox_min_y)

                    width = max_x - min_x
                    height = max_y - min_y
                except Exception:
                    if 'wall' in alias or 'parede' in alias:
                        width, height = 2.0, 0.1
                
                # Monta a estrutura temporária para extrair os cantos originais
                raw_obs = {
                    'x': pos[0], 'y': pos[1],
                    'w': width, 'h': height,
                    'angle': angle_rad
                }
                
                # 1. Calcula os vértices originais rotacionados
                original_corners = get_obb_corners(raw_obs)
                inflated_corners = original_corners # Por padrão, se não inflar, são iguais
                
                # 2. SE o raio do robô for passado, aplica a Soma de Minkowski usando Shapely
                if robot_radius > 0.0:
                    try:
                        shapely_poly = ShapelyPolygon(original_corners)
                        # join_style=2 mantém as quinas o mais retas/quadradas possível
                        inflated_poly = shapely_poly.buffer(robot_radius, join_style=2)
                        inflated_corners = list(inflated_poly.exterior.coords)
                    except Exception as e:
                        print(f"[WARNING] Erro ao inflar obstáculo {alias}: {e}")
                
                # Guarda o dicionário completo com geometria pura e geometria C-Space
                obstacles_data.append({
                    'x': pos[0],
                    'y': pos[1],
                    'w': width,
                    'h': height,
                    'angle': angle_rad,
                    'corners_originals': original_corners,
                    'corners': inflated_corners
                })
                
    except Exception as e:
        print(f"[ERROR] Erro ao mapear cenário no módulo externo: {e}")
        
    return obstacles_data

def get_obb_corners(obs):
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
    Busca as dimensões da carcaça no V4.1 e calcula o raio do círculo circunscrito.
    """
    try:
        robot_handle = sim.getObject(f'/{base_shape_name}')
        
        def read_param(param_id):
            val = sim.getObjectFloatParameter(robot_handle, param_id)
            return val[1] if isinstance(val, (tuple, list)) else val

        max_x = read_param(sim.objfloatparam_objbbox_max_x)
        min_x = read_param(sim.objfloatparam_objbbox_min_x)
        max_y = read_param(sim.objfloatparam_objbbox_max_y)
        min_y = read_param(sim.objfloatparam_objbbox_min_y)

        w = max_x - min_x
        h = max_y - min_y
        
        radius = np.sqrt((w / 2.0)**2 + (h / 2.0)**2)
        print(f"[INFO] Dimensões do robô (V4.1): w={w:.3f}m, h={h:.3f}m -> Raio Seguro={radius:.3f}m")
        return radius
            
    except Exception as e:
        print(f"[ERROR] Erro ao obter dimensões do robô: {e}")
        
    return 0.15