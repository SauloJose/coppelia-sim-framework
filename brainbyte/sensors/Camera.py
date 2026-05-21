import cv2
import numpy as np
from brainbyte.sensors.base.base_sensor import *

class CameraSensor(BaseSensor):
    """
    Simula uma Câmera RGB (Vision Sensor) no CoppeliaSim.

    Esta classe captura imagens do simulador e as converte automaticamente
    para matrizes tridimensionais do NumPy (Altura, Largura, Canais), prontas
    para serem usadas com OpenCV, TensorFlow, PyTorch, etc.

    Args:
        bridge: O objeto da API de simulação.
        sensor_name (str): O nome do Vision Sensor no CoppeliaSim.
    """
    def __init__(self, bridge, sensor_name):
        self._sensor_path = sensor_name if sensor_name.startswith(('/', '.')) else f"/{sensor_name}"
        
        # Inicializa a classe pai BaseSensor
        super().__init__(bridge, sensor_path=self._sensor_path)

    def get_monitor_paths(self):
        """ 
        Garante que o Lua monitore esse sensor especificamente para extração de imagem.
        """
        return [f"{self._sensor_path}_camera"]

    def update(self):
        """
        Lê os dados binários da imagem vindos do bridge e reconstrói a matriz RGB.
        
        Retorna:
            np.ndarray: Imagem no formato (Height, Width, 3) do tipo uint8.
                        Retorna um array vazio se não houver dados.
        """
        # Puxa os dados empacotados pelo Lua
        img_buffer = self.bridge.get_sensor_data(f"{self._sensor_path}_camera_img")
        resolution = self.bridge.get_sensor_data(f"{self._sensor_path}_camera_res")

        if img_buffer is None or resolution is None or len(resolution) < 2:
            return np.array([])

        width = int(resolution[0])
        height = int(resolution[1])

        # 1. Converte o buffer de bytes diretamente para um array 1D super rápido
        img_np = np.frombuffer(img_buffer, dtype=np.uint8)
        
        # 2. Transforma o vetor 1D na matriz da imagem (Altura, Largura, 3 Canais RGB)
        img_np = img_np.reshape((height, width, 3))
        
        # 3. O CoppeliaSim envia imagens de cabeça para baixo (origem no canto inferior esquerdo). 
        # Flip vertical para corrigir a orientação.
        img_np = np.flipud(img_np)

        return img_np

    # Alias para retrocompatibilidade
    def getSensorData(self):
        return self.update()