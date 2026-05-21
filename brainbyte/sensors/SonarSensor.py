import numpy as np
from brainbyte.sensors.base.base_sensor import *

class UltrasonicSensor(BaseSensor):
    """
    Simula um sensor de proximidade ultrassônico (ou infravermelho) no CoppeliaSim.

    Esta classe fornece uma interface para interagir com um Proximity Sensor padrão.
    Ela gerencia a comunicação através do bridge e processa os dados detectados, 
    podendo retornar a distância direta até o obstáculo ou as coordenadas 3D do ponto de colisão.

    Atributos:
        _bridge: O objeto da API de simulação usado para interagir com o CoppeliaSim.
        _sensor_path (str): O caminho absoluto do objeto do sensor no simulador.
        _return_point_data (bool): Define se o sensor retorna as coordenadas 3D (True) ou apenas a distância (False).

    Args:
        bridge: O objeto da API de simulação.
        sensor_name (str): O nome ou caminho do sensor de proximidade no CoppeliaSim.
        return_point_data (bool, opcional): Se True, retorna o ponto 3D. Padrão é False (retorna distância em metros).
    """
    def __init__(self, bridge, sensor_name, return_point_data=False):
        self._sensor_path = sensor_name if sensor_name.startswith(('/', '.')) else f"/{sensor_name}"
        
        # Inicializa a classe pai BaseSensor
        super().__init__(bridge, sensor_path=self._sensor_path)

        self._sensor_name = sensor_name
        self._return_point_data = return_point_data


    def get_monitor_paths(self):
        """ 
        Declara no Handshake quais informações o script Lua deve pré-calcular.
        Espera-se que o lado Lua leia o sensor (via sim.readProximitySensor)
        e empacote o resultado na chave terminada em '_proximity'.
        """
        paths = [f"{self._sensor_path}_proximity"]
        return paths
    
    def get_return_point_data(self) -> bool:
        return self._return_point_data

    def set_return_point_data(self, return_point_data: bool) -> None:
        self._return_point_data = return_point_data

    def update(self):
        """
        Lê os dados do sensor de proximidade vindos do bridge.
        
        O script Lua no CoppeliaSim deve enviar um array com o seguinte formato:
        [estado_da_leitura (0 ou 1), distancia, ponto_x, ponto_y, ponto_z]
        """
        # Solicita os dados que o Lua preparou no handshake
        data = self.bridge.get_sensor_data(f"{self._sensor_path}_proximity")

        # Se não houver dados válidos, assumimos que não há obstáculo no alcance
        if data is None or len(data) < 2:
            return np.array([]) if self._return_point_data else np.inf
            
        # Extrai o status da detecção (1 = detectou, 0 = não detectou)
        detection_state = int(data[0])

        if detection_state == 0:
            # Nada detectado
            return np.array([]) if self._return_point_data else np.inf

        # Se houver detecção, extrai as informações
        distance = float(data[1])

        if self._return_point_data:
            # Verifica se as coordenadas 3D (x, y, z) foram passadas pelo Lua
            if len(data) >= 5:
                pts_3d = np.array(data[2:5])
                return pts_3d
            else:
                return np.array([])
        else:
            # Retorna apenas a distância (Range Mode)
            return distance
    
    # Alias para retrocompatibilidade
    def getSensorData(self):
        return self.update()