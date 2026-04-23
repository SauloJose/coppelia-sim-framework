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
class PointCloudAccumulator:
    def __init__(self, max_point = None):
        self._cache = np.empty((0,3))
        self.max_points = max_point

    def add(self, points: np.ndarray):
        if points.size == 0:
            return 0
        self._cache = np.vstack([self._cache, points])
        if self.max_points is not None and self._cache.shape[0] > self.max_points:
            self._cache = self._cache[-self.max_points:]

    def get_all(self) -> np.ndarray:
        return self._cache

    def clear(self):
        self._cache = np.empty((0, 3))

    @property
    def count(self):
        return self._cache.shape[0]
    