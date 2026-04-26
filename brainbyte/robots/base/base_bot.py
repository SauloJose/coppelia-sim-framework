from abc import ABC, abstractmethod
from brainbyte.utils import *
from brainbyte.utils.math import * 

# Importando todos os sensores
from brainbyte.sensors import * 


class BaseBot(ABC):
    """
    Classe base abstrata para qualquer robô no CoppeliaSim (Arquitetura Batch Dataflow).
    Fornece:
    - Acesso à Bridge de comunicação síncrona
    - Caminho do objeto raiz do robô (String Path)
    - Gerenciamento de sensores e controladores
    - Métodos para declarar quais endpoints o Lua deve monitorar
    """

    def __init__(self, bridge, robot_name):
        """
        Args:
            bridge: Instância de SimulationBridge para comunicação em lote.
            robot_name: Nome do objeto raiz do robô na cena (ex: 'Turtlebot3').
        """
        self.bridge = bridge
        self.robot_name = robot_name

        self.robot_path = f'/{robot_name}'

        self._sensores =   {} #sensores associados ao robô
        self._control  =   {} #Guardo os controles do robô

    # PROPRIEDADES
    @property
    def pose(self):
        """
        Retorna a pose real do robô [x, y, theta] diretamente do simulador.
        Utiliza o método get_pose() herdado da classe BaseBot.
        """
        # pos = [x, y, z] e ori = [alpha, beta, gamma]
        pos, ori = self.get_pose() 
        
        # Proteção para o primeiro frame (antes da bridge receber os dados)
        if pos is None or ori is None:
            return np.array([0.0, 0.0, 0.0])
        
        # Para um robô planar, queremos X, Y e a rotação no eixo Z (gamma)
        x, y = pos[0], pos[1]
        theta = ori[2] 
        
        return np.array([x, y, theta])

    @pose.setter
    def pose(self, new_pose):
        """
        Teleporta o robô para uma nova pose [x, y, theta] no CoppeliaSim.
        """
        if len(new_pose) != 3:
            raise ValueError("A pose deve conter 3 elementos: [x, y, theta].")
        
        x, y, theta = new_pose
        
        # Pegamos a pose atual para não alterar a altura (Z) 
        # e as rotações de inclinação (alpha, beta) acidentalmente.
        pos_atual, ori_atual = self.get_pose()
        if pos_atual is None:
            z, alpha, beta = 0.0, 0.0, 0.0
        else:
            z = pos_atual[2]
            alpha = ori_atual[0]
            beta = ori_atual[1]

        # Enfileira na Bridge um comando da categoria 'teleports'
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
                    