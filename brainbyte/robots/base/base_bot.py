from abc import ABC, abstractmethod
from brainbyte.utils import *
from brainbyte.utils.math import * 

# Import all sensors
from brainbyte.sensors import * 


class BaseBot(ABC):
    """Abstract base class for any robot in CoppeliaSim (Batch Dataflow Architecture).
    
    Provides:
    - Access to synchronous Bridge for communication
    - Path to the robot root object (String Path hierarchy)
    - Sensor and controller management
    - Methods to declare which endpoints Lua should monitor
    """

    def __init__(self, bridge, robot_name):
        """Initialize robot instance.
        
        Args:
            bridge: SimulationBridge instance for batch communication.
            robot_name: Name of robot root object in scene (e.g., 'Turtlebot3').
        """
        self.bridge = bridge
        self.robot_name = robot_name

        self.robot_path = f'/{robot_name}'

        self._sensores = {}   # Sensors attached to this robot
        self._control = {}    # Controllers attached to this robot

    # PROPERTIES
    @property
    def pose(self):
        """Return robot pose [x, y, theta] directly from simulator.
        
        Uses get_pose() method from BaseBot class.
        """
        # pos = [x, y, z] and ori = [alpha, beta, gamma]
        pos, ori = self.get_pose() 
        
        # Protection for first frame (before bridge receives data)
        if pos is None or ori is None:
            return np.array([0.0, 0.0, 0.0])
        
        # For a planar robot, we want X, Y and rotation around Z axis (gamma)
        x, y = pos[0], pos[1]
        theta = ori[2] 
        
        return np.array([x, y, theta])

    @pose.setter
    def pose(self, new_pose):
        """Teleport robot to a new pose [x, y, theta] in CoppeliaSim."""
        if len(new_pose) != 3:
            raise ValueError("Pose must contain 3 elements: [x, y, theta].")
        
        x, y, theta = new_pose
        
        # Get current pose to not alter height (Z)
        # and inclination rotations (alpha, beta) accidentally
        pos_atual, ori_atual = self.get_pose()
        if pos_atual is None:
            z, alpha, beta = 0.0, 0.0, 0.0
        else:
            z = pos_atual[2]
            alpha = ori_atual[0]
            beta = ori_atual[1]

        # Queue teleport command to Bridge
        dados_teleporte = {'pos': [float(x), float(y), float(z)], 
                           'ori': [float(alpha), float(beta), float(theta)]}
        self.bridge.queue_command('teleports', self.robot_path, dados_teleporte)

    def get_pose(self):
        """
        Lê a posição (x,y,z) e orientação (alpha,beta,gamma) absolutas diretamente 
        do cache da Bridge, sem travar a rede.
        """
        # Esperamos que o Lua envie esses dados com os sufixos _pos e _ori
        pos = self.bridge.get_sensor_data(f"{self.robot_path}_pos")
        ori = self.bridge.get_sensor_data(f"{self.robot_path}_ori")
        return pos, ori
    
    # Gerenciamento de sensores
    def add_sensor(self, sensor_name, sensor_instance):
        """ 
        Adiciona um sensor ao dicionário de sensores do robô.
        
        Args:
            sensor_name (str): Nome fácil para buscar depois (ex: 'camera_rgb').
            sensor_instance: O objeto da classe do sensor instanciado.
        """
        self._sensores[sensor_name] = sensor_instance

    def get_sensor(self, sensor_name):
        """ Recupera um sensor pelo nome. """
        if sensor_name not in self._sensores:
            raise KeyError(f"Sensor '{sensor_name}' não está associado a este robô.")
        return self._sensores[sensor_name]
    
    def show_sensors(self):
        """ Retorna um string com os sensores do robô"""
        text = f"SENSORES DE {self.robot_name}\n{{}}"

        for key, value in self._sensores.items():
            name = getattr(value, '__name__', str(value))
            text += f"\n     {key} : {name}"
        text += "\n}}"
        return text 
    
    # Gerenciamento de controladores
    def add_control(self, control_name, control_instance):
        """ 
        Adiciona um algorítmo de controle ao dicionário de controles do robô.
        
        Args:
            control_name (str): Nome fácil para buscar depois (ex: 'camera_rgb').
            control_instance: O objeto da classe do sensor instanciado.
        """
        self._control[control_name] = control_instance

    def get_control(self, control_name):
        """ Recupera um sensor pelo nome. """
        if control_name not in self._control:
            raise KeyError(f"Controle '{control_name}' não está associado a este robô.")
        return self._control[control_name] 
    
    def show_controls(self):
        """ Retorna um string com os sensores do robô"""
        text = f"Controladores de {self.robot_name}\n{{}}"

        for key, value in self._control.items():
            name = getattr(value, '__name__', str(value))
            text += f"\n     {key} : {name}"
        text += "\n}}"
        return text 
    
    # ==========================================
    # MÉTODOS DE HANDSHAKE (INICIALIZAÇÃO BATCH)
    # ==========================================
    def get_monitor_paths(self):
        """
        Coleta e retorna uma lista com os caminhos absolutos que este robô 
        e seus sensores precisam que o Lua monitore em tempo real.
        """
        # O robô sempre pede para monitorar sua própria posição e orientação
        paths = [f"{self.robot_path}_pos", f"{self.robot_path}_ori"]
        
        # Repassa o pedido para os sensores associados
        for sensor in self._sensores.values():
            if hasattr(sensor, 'get_monitor_paths'):
                paths.extend(sensor.get_monitor_paths())
        return paths

    def get_actuator_paths(self):
        """
        Coleta e retorna os caminhos dos motores/atuadores deste robô.
        A classe base retorna apenas o seu path para o teleporte (se necessário),
        as classes filhas devem estender este método (com super()) 
        incluindo os paths das rodas.
        """
        return [self.robot_path]
    
    @abstractmethod
    def stop(self):
        """
        Para todos os controladores associados ao robô.
        Classes filhas devem sobrescrever este método para parar os motores (atuadores específicos),
        mas DEVEM chamar super().stop() para garantir a parada dos controles genéricos.
        """
        # Itera sobre todos os controladores registrados
        for name, control_instance in self._control.items():
            # Verifica se o controlador possui um método 'stop' e se ele é executável (callable)
            if hasattr(control_instance, 'stop') and callable(control_instance.stop):
                try:
                    control_instance.stop()
                except Exception as e:
                    print(f"[BaseBot] Erro ao parar o controlador '{name}': {e}")
                    