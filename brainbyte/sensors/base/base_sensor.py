from abc import ABC, abstractmethod

class BaseSensor(ABC):
    """
    Classe base abstrata para qualquer sensor no CoppeliaSim (Arquitetura Batch Dataflow).
    
    Fornece:
    - Referência à Bridge (para leitura instantânea sem lag).
    - Caminho base do sensor na cena.
    - Métodos padronizados para o Handshake (get_monitor_paths).
    - Método utilitário para buscar dados no cache.
    """

    def __init__(self, bridge, sensor_path: str):
        """
        Args:
            bridge: Instância da SimulationBridge.
            sensor_path: Caminho absoluto do sensor na cena (ex: '/Turtlebot3/Lidar').
        """
        self.bridge = bridge
        
        # Garante que o caminho é absoluto para o CoppeliaSim
        self.sensor_path = sensor_path if sensor_path.startswith(('/', '.')) else f"/{sensor_path}"

    def get_monitor_paths(self) -> list:
        """
        Declara quais chaves o script Lua deve colocar no cache a cada frame.
        Por padrão, pede a posição do próprio sensor. 
        Classes filhas podem estender isso com sufixos (ex: '_ptcloud', '_vision').
        """
        return [self.sensor_path]

    def get_bridge_data(self, suffix: str = ""):
        """
        Busca a leitura mais recente do sensor diretamente na memória do Python,
        sem acionar a rede.
        
        Args:
            suffix: Sufixo usado no monitoramento (ex: '_matrix', '_vision').
        """
        path_to_fetch = f"{self.sensor_path}{suffix}"
        return self.bridge.get_sensor_data(path_to_fetch)

    @abstractmethod
    def update(self):
        """
        Método que deve ser chamado no loop principal para processar os dados do sensor.
        Toda classe filha (LiDAR, Câmera, Proximidade) OBRIGATORIAMENTE deve implementar.
        """
        pass