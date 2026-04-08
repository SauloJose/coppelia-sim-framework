import math
import numpy as np

class HokuyoSensorSim(object):
    """
    Simulates a Hokuyo laser sensor in CoppeliaSim using vision sensors.

    This class provides an interface to interact with a simulated Hokuyo sensor,
    typically attached to a robot in CoppeliaSim. It manages the underlying vision
    sensors and provides methods to retrieve sensor data in either range or point format.

    Attributes:
        _sim: The simulation API object used to interact with CoppeliaSim.
        _base_name (str): The name of the base object to which the Hokuyo sensor is attached.
        _is_range_data (bool): Determines if sensor data is returned as range values (True) or 3D points (False).
        _base_obj: The handle of the base object in the simulation.
        _vision_sensors_obj (list): Handles of the vision sensors used to simulate the Hokuyo sensor.

    Args:
        sim: The simulation API object.
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

    _sim = None

    _base_name = ""
    _vision_sensor_name_template = "{}/fastHokuyo_body/fastHokuyo_joint{}/fastHokuyo_sensor{}"

    # _vision_sensors_obj will be initialized in __init__
    _base_obj = None
    _is_range_data = False
    
    _angle_min=-120*math.pi/180
    _angle_max=120*math.pi/180
    _angle_increment=(240/684)*math.pi/180 # angle: 240 deg, pts: 684

    def __init__(self, sim, base_name, is_range_data=True):
        self._sim = sim
        self._base_name = base_name
        self._is_range_data = is_range_data

        if "fastHokuyo" not in base_name:
            raise ValueError(
                f"ERR: fastHokuyo must be in the base object name. Ex: `/PioneerP3DX/fastHokuyo`"
            )

        self._base_obj = sim.getObject(base_name)
        if self._base_obj == -1:
            raise ValueError(
                f"ERR: base_obj ({self._base_obj}) is not a valid name in the simulation"
            )

        self._vision_sensors_obj = [
            sim.getObject(self._vision_sensor_name_template.format(self._base_name, 1,1)),
            sim.getObject(self._vision_sensor_name_template.format(self._base_name, 2,2)),
        ]

        if any(obj == -1 for obj in self._vision_sensors_obj):
            raise ValueError(
                f"ERR: the _vision_sensors_obj names are not valid in the simulation"
            )

    def get_is_range_data(self) -> bool:
        return self._is_range_data

    def set_is_range_data(self, is_range_data: bool) -> None:
        self._is_range_data = is_range_data

    def getSensorData(self):
        
        angle = self._angle_min
        sensor_data = []
        
        try:
            for vision_sensor in self._vision_sensors_obj:
                r, t, u = self._sim.readVisionSensor(vision_sensor)
                if u:
                    sensorM = self._sim.getObjectMatrix(vision_sensor)
                    relRefM = self._sim.getObjectMatrix(self._base_obj)
                    relRefM = self._sim.getMatrixInverse(relRefM)
                    relRefM = self._sim.multiplyMatrices(relRefM, sensorM)

                    p = [0, 0, 0]
                    p = self._sim.multiplyVector(sensorM, p)
                    t = [p[0], p[1], p[2], 0, 0, 0]
                    for j in range(int(u[1])):
                        for k in range(int(u[0])):
                            w = 2 + 4 * (j * int(u[0]) + k)
                            v = [u[w], u[w + 1], u[w + 2], u[w + 3]]
                            angle = angle + self._angle_increment
                            if self._is_range_data:
                                sensor_data.append([angle, v[3]])
                            else:
                                p = self._sim.multiplyVector(relRefM, v)
                                sensor_data.append([p[0], p[1], p[2]])
        except Exception as e:
            return  -1
                 
        return np.array(sensor_data)