import math
import numpy as np
from brainbyte.sensors.base.base_sensor import *

class HokuyoSensorSim(BaseSensor):
    """
    Simulates a Hokuyo laser sensor in CoppeliaSim using vision sensors.

    This class provides an interface to interact with a simulated Hokuyo sensor,
    typically attached to a robot in CoppeliaSim. It manages the underlying vision
    sensors and provides methods to retrieve sensor data in either range or point format.

    Attributes:
        _bridge: The simulation API object used to interact with CoppeliaSim.
        _base_name (str): The name of the base object to which the Hokuyo sensor is attached.
        _is_range_data (bool): Determines if sensor data is returned as range values (True) or 3D points (False).
        _base_obj: The handle of the base object in the simulation.
        _vision_sensors_obj (list): Handles of the vision sensors used to simulate the Hokuyo sensor.

    Args:
        bridge: The simulation API object.
        base_name (str): The name of the base object (must contain 'fastHokuyo').
        is_range_data (bool, optional): If True, sensor data is returned as range values. Defaults to False.

    Raises:
        ValueError: If 'fastHokuyo' is not in the base_name, or if the base object or vision sensors are not found.

    Methods:
        get_is_range_data() -> bool:
            Returns whether sensor data is returned as range values.

        set_is_range_data(is_range_data: bool) -> None:
            Sets whether sensor data should be returned as range values.

        getSensorData():
            Retrieves sensor data from the vision sensors.
            Returns either a list of range values or a list of 3D points, depending on _is_range_data.
    """
    _vision_sensor_name_template = "{}/fastHokuyo_body/fastHokuyo_joint{}/fastHokuyo_sensor{}"
    
    _angle_min = -120 * math.pi / 180
    _angle_max = 120 * math.pi / 180
    _angle_increment = (240 / 684) * math.pi / 180 # angle: 240 deg, pts: 684

    def __init__(self, bridge, base_name, is_range_data=True):
        self._base_obj_path = base_name if base_name.startswith(('/', '.')) else f"/{base_name}"
        
        # 2. AJUSTE: Inicializa a classe pai BaseSensor
        super().__init__(bridge, sensor_path=self._base_obj_path)

        self._base_name = base_name
        self._is_range_data = is_range_data

        if "fastHokuyo" not in base_name:
            raise ValueError(f"ERR: fastHokuyo must be in the base object name. Ex: `/PioneerP3DX/fastHokuyo`")

        self._vision_sensor_1 = self._vision_sensor_name_template.format(self._base_obj_path, 1, 1)
        self._vision_sensor_2 = self._vision_sensor_name_template.format(self._base_obj_path, 2, 2)
        
        self._vision_sensors = [self._vision_sensor_1, self._vision_sensor_2]


    def get_monitor_paths(self):
        """ Declara no Handshake quais informações o Lua deve pré-calcular """
        paths = [f"{self._base_obj_path}_matrix"]
        for vs in self._vision_sensors:
            paths.append(f"{vs}_vision") # Pedido do pacote auxiliar da câmera
            paths.append(f"{vs}_matrix") # Pedido da matriz do sensor
        return paths
    
    def get_is_range_data(self) -> bool:
        return self._is_range_data

    def set_is_range_data(self, is_range_data: bool) -> None:
        self._is_range_data = is_range_data

    def _to_4x4(self, matrix_12):
        """Converte a matriz 3x4 do Coppelia para 4x4 homogênea."""
        m = np.array(matrix_12).reshape(3, 4)
        return np.vstack((m, [0, 0, 0, 1]))

    def update(self):
        """
        Gera os dados do Lidar usando vetorização do NumPy em frações de milissegundo.
        """
        sensor_data = []
        angle = self._angle_min

        # 1. Pega a matriz da base e calcula a inversa
        baseM = self.get_bridge_data('_matrix')

        if baseM is None:
            return np.array([])
            
        H_base = self._to_4x4(baseM)
        H_base_inv = np.linalg.inv(H_base)

        # 2. Processa cada uma das câmeras que formam o Hokuyo
        for vision_sensor in self._vision_sensors:
            u = self.bridge.get_sensor_data(f"{vision_sensor}_vision_bin")
            sensorM = self.bridge.get_sensor_data(f"{vision_sensor}_matrix")
            
            if u is None or u is False or len(u) < 2 or sensorM is None:
                # Se não houver pontos capturados, apenas avança o relógio de ângulos
                angle += 342 * self._angle_increment 
                continue

            # 'u' é o array 1D contendo [count_x, count_y, p1x, p1y, p1z, d1, p2x...]
            count_x = int(u[0])
            count_y = int(u[1])
            num_points = count_x * count_y
            
            points_data = np.asarray(u[2:])
            
            if len(points_data) < num_points * 4:
                continue

            # VETORIZAÇÃO: Transforma os milhares de floats em uma matriz Nx4 instantaneamente
            pts = points_data[:num_points * 4].reshape(-1, 4)
            
            # Gera o array de ângulos simultaneamente
            angles = angle + (np.arange(num_points) + 1) * self._angle_increment
            
            if self._is_range_data:
                # Se quer só o range, junta os ângulos com a coluna 3 (distância)
                distances = pts[:, 3]
                data = np.column_stack((angles, distances))
                sensor_data.append(data)
            else:
                # Se quer os pontos 3D relativos ao robô (Base)
                H_sensor = self._to_4x4(sensorM)
                H_rel = H_base_inv @ H_sensor # Matriz final de transformação
                
                # Extrai xyz e adiciona '1' para multiplicar pela matriz 4x4
                pts_3d = pts[:, 0:3]
                pts_homo = np.hstack((pts_3d, np.ones((pts_3d.shape[0], 1))))
                
                # Mágica do NumPy: Multiplica centenas de pontos pela matriz de uma vez
                transformed_pts = (H_rel @ pts_homo.T).T[:, 0:3]
                sensor_data.append(transformed_pts)
            
            # Atualiza o ângulo base para o próximo sensor
            angle += num_points * self._angle_increment

        if not sensor_data:
            return np.array([])
            
        return np.vstack(sensor_data)
    
    # Opcional: Alias para retrocompatibilidade se scripts antigos usarem getSensorData
    def getSensorData(self):
        return self.update()
