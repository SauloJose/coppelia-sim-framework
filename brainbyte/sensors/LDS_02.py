import math
import numpy as np
from brainbyte.sensors.base.base_sensor import *

class LDS_02(BaseSensor):
    """
    Simulates an LDS_02 LiDAR sensor in CoppeliaSim.

    This class interfaces with a vision sensor that provides point cloud data,
    typically attached to a robot. It reads the point cloud in the sensor's local
    frame and optionally transforms it to world coordinates.
    """
    _lidar_handle_template = "/{}/{}"                
    _lidar_base_handle_template = "/{}/{}"           #ex:  base_name/base_link
    _lidar_point_cloud_handle_template = "/{}/{}/{}"    #ex:  base_name/scan_joint/point_cloud
    
    def __init__(self,
                 bridge,
                 base_name,         # robot name
                 point_cloud_name   ='point_cloud',
                 base_link_name     = 'base_link',
                 scan_joint_name    = 'scan_joint',
                 scan_name          = 'laser_joint',
                 is_range_data      = False):
        
        lidar_path = f"/{base_name}/{scan_joint_name}"

        super().__init__(bridge, sensor_path=lidar_path)

        self._base_name = base_name
        self._is_range_data = is_range_data

        # Constrói as strings de caminho absoluto
        self._lidar_path = self._lidar_handle_template.format(base_name, scan_joint_name) 
        self._point_cloud_path = self._lidar_point_cloud_handle_template.format(base_name, scan_joint_name, point_cloud_name)
        self._base_path = self._lidar_base_handle_template.format(base_name, base_link_name)
        
        #Debug
        print(f"[DEBUG] [LDS_02]: point LIDAR handle: '{self._lidar_path}'")
        print(f"[DEBUG] [LDS_02]: point BASE handle: '{self._base_path}'")
        print(f"[DEBUG] [LDS_02]: cloud point handle: '{self._point_cloud_path}'")

        # last frame 
        self._last_points_local = np.empty((0, 3))
        self._last_points_world = np.empty((0, 3))
    
    def get_monitor_paths(self) -> list:
        """ Estende o método da BaseSensor pedindo a matriz e a nuvem de pontos """
        return [
            f"{self.sensor_path}_matrix",      # Para transformar pra World
            f"{self._point_cloud_path}_ptcloud" # Os pontos brutos
        ]
    
    def _read_lidar(self):
        """Read all points of the PointCloud in the local frame sensor directly from Bridge cache."""
        
        raw_points = self.bridge.get_sensor_data(f"{self._point_cloud_path}_ptcloud_bin")
        
        if raw_points is None or len(raw_points) == 0:
            self._last_points_local = np.empty((0, 3))
            return self._last_points_local

        # Transforma a lista 1D de floats nativa do C++ em matriz (N, 3)
        points = raw_points.reshape(-1, 3)
        self._last_points_local = points
        return points
    
    def _transform_to_world(self, points):
        """Convert points from sensor local frame to world frame using homogeneous matrix."""
        if points.size == 0:
            self._last_points_world = points
            return points
        
        m = self.bridge.get_sensor_data(f"{self._lidar_path}_matrix")
        if m is None:
            return points # Fallback de segurança

        h_matrix = np.array(m).reshape(3, 4)
        h_matrix = np.vstack([h_matrix, [0, 0, 0, 1]])

        ones = np.ones((points.shape[0], 1))  
        points_homo = np.hstack([points, ones]) 

        points_world_homo = (h_matrix @ points_homo.T).T 
        
        self._last_points_world = points_world_homo[:, :3]
        return self._last_points_world
    
    def update(self):
        """ Read the point cloud and transform to world coordinates (Zero-lag). """
        local_pts = self._read_lidar()
        world_pts = self._transform_to_world(local_pts)
        return world_pts
    
    def read_lidar_local(self):
        """ Get the point cloud in the robot coordinates. """
        return self._read_lidar()
    
    def get_cloud_points(self, world_coordinates=True):
        return self._last_points_world if world_coordinates else self._last_points_local
    
    def get_cloud_points_sectores(self):
        """
        Returns the local point cloud divided into real geometric sectors.
        """
        local_pts = self._read_lidar()
        
        if local_pts is None or local_pts.size == 0:
            empty = np.empty((0, 3))
            return empty, empty, empty, empty

        x = local_pts[:, 0]
        y = local_pts[:, 1]
        angles = np.arctan2(y, x)

        mask_front = (angles > np.deg2rad(-30)) & (angles < np.deg2rad(30))
        mask_left  = (angles >= np.deg2rad(30)) & (angles <= np.deg2rad(150))
        mask_right = (angles >= np.deg2rad(-150)) & (angles <= np.deg2rad(-30))
        mask_back  = (angles > np.deg2rad(150)) | (angles < np.deg2rad(-150))

        front = local_pts[mask_front]
        left  = local_pts[mask_left]
        right = local_pts[mask_right]
        back  = local_pts[mask_back]

        return left, front, right, back

    def get_distances(self):
        """
        Converte a nuvem de pontos local em Arrays de distâncias e ângulos,
        ordenados do menor para o maior ângulo.
        
        Retorna:
            distances (np.ndarray): Distâncias em metros.
            angles (np.ndarray): Ângulos correspondentes em radianos [-pi a pi].
        """
        local_pts = self._read_lidar()
        if local_pts is None or local_pts.size == 0:
            return np.array([]), np.array([])
            
        x = local_pts[:, 0]
        y = local_pts[:, 1]
        
        # np.hypot calcula sqrt(x^2 + y^2) (A distância direta 2D do sensor até o ponto)
        distances = np.hypot(x, y)
        angles = np.arctan2(y, x)
        
        # Ordena os pontos pelo ângulo para que o acesso por índice seja previsível
        # índice 0 = direita/trás (-180º) | índice do meio = frente (0º) | último = esquerda/trás (+180º)
        sorted_indices = np.argsort(angles)
        
        return distances[sorted_indices], angles[sorted_indices]

    def get_sector_min_distances(self, max_range=3.0):
        """
        Retorna a menor distância detectada nos 4 setores geométricos padrões (Esq, Fren, Dir, Trás).
        Excelente para usar em lógicas de colisão ou estados básicos de RL.
        """
        left, front, right, back = self.get_cloud_points_sectores()
        
        def min_dist(pts):
            if pts.size == 0:
                return max_range # Se não há pontos, retorna distância livre máxima
            return np.min(np.hypot(pts[:, 0], pts[:, 1]))
            
        return {
            'left': min_dist(left),
            'front': min_dist(front),
            'right': min_dist(right),
            'back': min_dist(back)
        }

    def get_rl_distances(self, num_sectors=8, max_range=3.0):
        """
        Divide toda a leitura de 360º do LiDAR em 'num_sectors' fatias iguais 
        e retorna a MENOR distância dentro de cada fatia. 
        
        Este método já retorna o array (ex: 8 valores contínuos) perfeito para 
        concatenar direto no estado (state) da sua Rede Neural.
        """
        distances, angles = self.get_distances()
        if distances.size == 0:
            return np.full(num_sectors, max_range)
            
        # Cria as fronteiras (bins) de ângulos. Ex para 8 setores: de -180 a 180 em 8 passos.
        bins = np.linspace(-np.pi, np.pi, num_sectors + 1)
        
        rl_state = []
        for i in range(num_sectors):
            # Encontra todos os pontos que o ângulo cai dentro dessa "fatia"
            mask = (angles >= bins[i]) & (angles < bins[i+1])
            if np.any(mask):
                rl_state.append(np.min(distances[mask]))
            else:
                rl_state.append(max_range) # Sem obstáculo na fatia
                
        return np.array(rl_state)

    @property
    def is_range_data(self):
        return self._is_range_data

    @is_range_data.setter
    def is_range_data(self, value):
        self._is_range_data = value


#class to cache data of the LiDar Sensor:
# Vox grid (transformo os pontos em coordenadas de grade e só adiciono se estiverem próximos)
class PointCloudAccumulator:
    def __init__(self, max_point = 100000, voxel_size = 0.005):
        self._cache_list = []
        self._total_count = 0 

        self.max_points = max_point
        self.voxel_size = voxel_size
        self._occupied_voxels = set()

    def _get_voxel_coords(self, points):
        # Transforma coordenadas reais em índices inteiros de grade
        return np.floor(points / self.voxel_size).astype(int)
    
    def add(self, points: np.ndarray):
        if points.size == 0:
            return 0
        
        # Aplico a lógica do VOXEL
        voxels = self._get_voxel_coords(points)

        unique_mask = []
        for i,v in enumerate(voxels):
            v_tuple = (v[0], v[1], v[2])
            if v_tuple not in self._occupied_voxels:
                self._occupied_voxels.add(v_tuple)
                unique_mask.append(i)

        if not unique_mask:
            return 
        
        new_pts =points[unique_mask]
        self._cache_list.append(new_pts)
        self._total_count += len(new_pts)

        if self.max_points and self._total_count > self.max_points:
            self._prune_cache()

    def get_all(self) -> np.ndarray:
        if not self._cache_list:
            return np.empty((0, 3))
        return np.vstack(self._cache_list)

    def _prune_cache(self):
        """ Consolida e remove pontos antigos para evitar estouro de RAM """
        full_cloud = self.get_all()
        pruned_cloud = full_cloud[-self.max_points:]
        
        self._cache_list = [pruned_cloud]
        self._total_count = len(pruned_cloud)
        
        # Reconstrói o set de voxels para refletir apenas os pontos que ficaram
        voxels = self._get_voxel_coords(pruned_cloud)
        self._occupied_voxels = set(tuple(v) for v in voxels)

    def clear(self):
        self._cache = np.empty((0, 3))
        self._occupied_voxels.clear()

    def _rebuild_voxel_set(self):
        # Sincroniza o set com os pontos que sobraram no cache
        voxels = self._get_voxel_coords(self._cache)
        self._occupied_voxels = set(tuple(v) for v in voxels)


    @property
    def count(self):
        return self._total_count
    