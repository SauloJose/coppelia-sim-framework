import numpy as np
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.ops import unary_union

def get_environment_obstacles(sim, robot_radius=0.0, 
                              botname=['turtlebot', 'robot'], 
                              keywords=['block', 'pillar', 'wall', 'parede', 'pilar'],
                              wall_keywords=['cuboid']):
    """
    Busca todas as formas geométricas da cena do CoppeliaSim V4.1.
    Une os cuboids/blocos de parede para calcular o retângulo interno útil (com recuo do raio).
    Infla os obstáculos internos (com expansão do raio).
    
    Retorna:
        obstacles_data (list): Lista de dicionários dos obstáculos internos inflados.
        boundary_vertices (list): Vértices [[x,y], ...] da área interna útil pronta para discretização.
    """
    obstacles_data = []
    wall_polygons = []
    boundary_vertices = []
    
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
            if any(kw in alias for kw in keywords) or any(kw in alias for kw in wall_keywords) :
                pos = sim.getObjectPosition(handle, -1)
                ori = sim.getObjectOrientation(handle, -1)
                angle_rad = ori[2] # Mantém em radianos
                
                # Valores padrão de fallback
                width, height = 0.5, 0.5 
                
                try:
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
                
                is_wall = any(w_kw in alias for w_kw in wall_keywords)
                
                if is_wall:
                    wall_polygons.append(ShapelyPolygon(original_corners))
                else:
                    inflated_corners = original_corners
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
                        'corners': inflated_corners,
                        'name': alias
                    })

        if wall_polygons:
            try:
                # Une os 4 cuboids em uma única "moldura" geométrica
                combined_walls = unary_union(wall_polygons)
                
                # O "buraco" interno (interiors) representa o espaço útil da arena
                if len(combined_walls.interiors) > 0:
                    interior_poly = ShapelyPolygon(combined_walls.interiors[0])
                    
                    # Aplica o buffer NEGATIVO para encolher a área útil (afastar o robô das paredes)
                    if robot_radius > 0.0:
                        interior_poly = interior_poly.buffer(-robot_radius, join_style=2)
                    
                    boundary_vertices = list(interior_poly.exterior.coords)
                else:
                    print("[WARNING] Não foi possível detectar o vão interno das paredes. Verifique se os cuboids se sobrepõem nas quinas.")
            except Exception as e:
                print(f"[ERROR] Erro ao processar geometria das paredes: {e}")      
    except Exception as e:
        print(f"[ERROR] Erro ao mapear cenário no módulo externo: {e}")
        
    return obstacles_data, boundary_vertices

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
        return radius
            
    except Exception as e:
        return -1
        
    return 0.15