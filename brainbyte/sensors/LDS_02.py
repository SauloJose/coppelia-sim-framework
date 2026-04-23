import math
import numpy as np

class LDS_02(object):
    """
    Simulates an LDS_02 LiDAR sensor in CoppeliaSim.

    This class interfaces with a vision sensor that provides point cloud data,
    typically attached to a robot. It reads the point cloud in the sensor's local
    frame and optionally transforms it to world coordinates.

    Args:
        sim: CoppeliaSim simulation API object.
        base_name (str): Full path to the vision sensor object in the scene.
        max_cache_points (int, optional): Max total points to keep in the
            accumulated cache. None means no limit.
    """
    _lidar_handle_template = "/{}/{}"                
    _lidar_base_handle_template = "/{}/{}"           #ex:  base_name/base_link
    _lidar_point_cloud_handle_template = "/{}/{}/{}"    #ex:  base_name/scan_joint/point_cloud
    
    def __init__(self,
                 sim,
                 base_name,         # robot name
                 point_cloud_name   ='point_cloud',
                 base_link_name     = 'base_link',
                 scan_joint_name    = 'scan_joint',
                 scan_name          = 'laser_joint',
                 is_range_data      = False):
        
        self._sim = sim
        self._base_name = base_name
        self._point_cloud_name = point_cloud_name
        self._base_link_name = base_link_name
        self._scan_joint_name = scan_joint_name
        self._is_range_data = is_range_data

        lidar_path = self._lidar_handle_template.format(base_name,scan_joint_name) 
        point_cloud_path = self._lidar_point_cloud_handle_template.format(base_name, scan_joint_name, point_cloud_name)
        base_path = self._lidar_base_handle_template.format(base_name, base_link_name)
        
        #Debug
        print(f"[DEBUG] [LDS_02]: point LIDAR handle: '{lidar_path}'")
        print(f"[DEBUG] [LDS_02]: point BASE handle: '{base_path}'")
        print(f"[DEBUG] [LDS_02]: cloud point handle: '{point_cloud_path}'")

        # handles from Coppelia
        self._lidar_handle = self._sim.getObject(lidar_path)
        self._point_cloud_handle = None
        self._point_cloud_path = point_cloud_path

        # last frame 
        self._last_points_local = np.empty((0, 3))
        self._last_points_world = np.empty((0, 3))
        
    def _get_point_cloud_handle(self):
        if self._point_cloud_handle is None:
                point_cloud_handle = self._point_cloud_handle =self._sim.getObject(self._point_cloud_path)
        return self._point_cloud_handle
    
    def _read_lidar(self):
        """read all points of the PointCloud in the local frame sensor."""
        pc_handle = self._get_point_cloud_handle()
        res = self._sim.getPointCloudPoints(pc_handle)
        if not res:
            return np.array([]).reshape(0, 3)

        # Transforma a lista flat em matriz (N, 3)
        points = np.array(res).reshape(-1, 3)
        self._last_points_local = points
        return points
    
    def _transform_to_world(self, points):
        """Convert points from sensor local frame to world frame using homogeneous matrix."""
        if points.size == 0:
            return points
        
        # Get the sensor's transformation matrix (3x4) and convert to 4x4
        m = self._sim.getObjectMatrix(self._lidar_handle, -1)
        h_matrix = np.array(m).reshape(3, 4)
        h_matrix = np.vstack([h_matrix, [0, 0, 0, 1]])

        # Convert points to homogeneous coordinates
        ones = np.ones((points.shape[0], 1))
        points_homo = np.hstack([points, ones])

        # Transform: P_world = H @ P_local
        points_world_homo = (h_matrix @ points_homo.T).T
        
        self._last_points_world = points_world_homo[:, :3]
        return self._last_points_world
    
    def update(self):
        """
        Read the point cloud and transform to world coordinates.

        Returns:
            np.ndarray: World-frame points with shape (N, 3).
        """
        local_pts = self._read_lidar()
        world_pts = self._transform_to_world(local_pts)

        return world_pts
    
    def get_cloud_points(self, world_coordinates=True):
        """
        Return the last captured point cloud.

        Args:
            world_coordinates (bool): If True, returns points in world frame;
                                      if False, in sensor's local frame.

        Returns:
            np.ndarray: Point cloud array (N, 3) or empty array if no data.
        """
        return self._last_points_world if world_coordinates else self._last_points_local
    
    # Getters e Setters
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
    