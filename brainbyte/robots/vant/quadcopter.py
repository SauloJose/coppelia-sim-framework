import numpy as np
from brainbyte.robots.base.base_bot import BaseBot
from typing import Union

class Quadcopter(BaseBot):
    """
    Controle do Quadricóptero totalmente integrado à arquitetura BaseBot.
    Controla o drone manipulando a posição tridimensional de um objeto Target.
    """

    def __init__(self, bridge, robot_name: str = 'Quadcopter', target_name: str = 'base/target'):
        """
        Args:
            bridge: Instância da SimulationBridge.
            robot_name: Nome do nó do drone no CoppeliaSim (ex: 'Quadcopter').
            target_name: Nome do objeto Dummy de alvo no CoppeliaSim (ex: 'base/target').
        """
        # Inicializa a classe base (configura self.bridge, self.robot_name e self.robot_path)
        super().__init__(bridge, robot_name)
        
        # Caminho absoluto para o objeto alvo
        self.target_name = target_name
        self.target_path = f"/{robot_name}/{target_name}"
        
        # Estados internos para armazenar os últimos comandos enviados ao alvo
        self._target_pos = np.zeros(3)
        self._target_ori = np.zeros(3)

    # ==========================================
    # MÉTODOS OBRIGATÓRIOS DA BASEBOT (OVERRIDE)
    # ==========================================

    def get_monitor_paths(self) -> list:
        """Estende a BaseBot para monitorar tanto o corpo do drone quanto o Target."""
        # Pega os caminhos padrão do robô (/[robot_name]_pos e /[robot_name]_ori)
        paths = super().get_monitor_paths()
        
        # Adiciona o monitoramento do objeto Target para feedback visual/físico se necessário
        paths.extend([f"{self.target_path}_pos", f"{self.target_path}_ori"])
        return paths

    def get_actuator_paths(self) -> list:
        """Estende a BaseBot para incluir o Target na lista de atuadores válidos do Handshake."""
        paths = super().get_actuator_paths()
        if self.target_path not in paths:
            paths.append(self.target_path)
        return paths

    def stop(self):
        """Implementação obrigatória do método abstrato. Para os controladores e estabiliza o alvo."""
        # 1. Garante que qualquer controlador associado via 'add_control' seja interrompido
        super().stop()
        
        # 2. Rotina específica do drone: Força o target a manter a última posição conhecida para evitar desvios
        try:
            self.move_to(self._target_pos)
        except Exception as e:
            print(f"[Quadcopter] Erro ao estabilizar target no stop: {e}")

    # ==========================================
    # MÉTODOS ESPECÍFICOS DE MOVIMENTAÇÃO (3D)
    # ==========================================

    @property
    def drone_pose_3d(self) -> tuple:
        """Retorna a posição e orientação completas em 3D do corpo do drone (não truncadas em 2D)."""
        return self.get_pose()

    def move_to(self, position: Union[list, tuple, np.ndarray]):
        """Desloca o objeto alvo (Target) para uma coordenada [X, Y, Z] específica."""
        if len(position) != 3:
            raise ValueError("A posição fornecida deve conter exatamente 3 coordenadas: [x, y, z].")
        
        self._target_pos = np.array(position, dtype=float)
        
        # Encapsula os dados para o interpretador Lua executar no bloco 'if msg.teleports'
        dados_teleporte = {'pos': self._target_pos.tolist()}
        self.bridge.queue_command('teleports', self.target_path, dados_teleporte)

    def set_target_orientation(self, orientation: Union[list, tuple, np.ndarray]):
        """Altera os ângulos de rotação [alpha, beta, gamma] do objeto alvo."""
        if len(orientation) != 3:
            raise ValueError("A orientação deve conter exatamente 3 elementos: [alpha, beta, gamma].")
            
        self._target_ori = np.array(orientation, dtype=float)
        
        # Encapsula os dados para o interpretador Lua executar no bloco 'if msg.teleports'
        dados_teleporte = {'ori': self._target_ori.tolist()}
        self.bridge.queue_command('teleports', self.target_path, dados_teleporte)

    def has_reached_target(self, actual_state: dict, tolerance: float = 0.5) -> bool:
        """
        Verifica se a distância tridimensional entre o drone e o Target é menor que a tolerância.
        
        Args:
            actual_state: O dicionário contendo o estado atual retornado pela Bridge.
            tolerance: Distância máxima em metros considerada como "chegou" (padrão: 15cm).
        Returns:
            bool: True se o drone chegou no alvo, False caso contrário.
        """
        if not actual_state:
            return False
            
        drone_pos = None
        for chave, valor in actual_state.items():
            if chave.endswith("_pos") and "target" not in chave:
                drone_pos = np.array(valor, dtype=float)
                break
                
        if drone_pos is None:
            return False
            
        distancia = np.linalg.norm(drone_pos - self._target_pos)
        
        return distancia <= tolerance