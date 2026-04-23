from abc import ABC, abstractmethod
from brainbyte.utils import *
from brainbyte.utils.math import * 

# Importando todos os sensores
from brainbyte.sensors import * 


class BaseBot(ABC):
    """
    Classe base abstrata para qualquer robô no CoppeliaSim.
    Fornece apenas:
    - Acesso ao cliente da API
    - Handle do objeto raiz do robô
    - Handle para guardar sensores do robô
    - Métodos comuns como obter pose e parada de emergência (opcional)
    """

    def __init__(self, sim, robot_name):
        """
        Args:
            sim: Objeto cliente da API do CoppeliaSim (ex.: retornado por RemoteAPIClient).
            robot_name: Nome do objeto raiz do robô na cena.
        """
        self.sim = sim
        self.robot_name = robot_name
        self.robot_handle = self.sim.getObject(f'/{robot_name}')

        if self.robot_handle == -1:
            raise ValueError(f"Robô '{robot_name}' não encontrado na cena.")
        
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
        
        # Para um robô planar, queremos X, Y e a rotação no eixo Z (gamma)
        x, y = pos[0], pos[1]
        theta = ori[2] 
        
        return np.array([x, y, theta])

    @pose.setter
    def pose(self, nova_pose):
        """
        Teleporta o robô para uma nova pose [x, y, theta] no CoppeliaSim.
        """
        if len(nova_pose) != 3:
            raise ValueError("A pose deve conter 3 elementos: [x, y, theta].")
        
        x, y, theta = nova_pose
        
        # Pegamos a pose atual para não alterar a altura (Z) 
        # e as rotações de inclinação (alpha, beta) acidentalmente.
        pos_atual, ori_atual = self.get_pose()
        
        z = pos_atual[2]
        alpha = ori_atual[0]
        beta = ori_atual[1]
        
        # Envia os comandos diretamente para a API do CoppeliaSim
        # (Assumindo a sintaxe padrão da API ZeroMQ/B0 do CoppeliaSim)
        self.sim.setObjectPosition(self.robot_handle, self.sim.handle_world, [x, y, z])
        self.sim.setObjectOrientation(self.robot_handle, self.sim.handle_world, [alpha, beta, theta])

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

    # Gerenciamento de posição no simulador
    def get_pose(self, relative_to=None):
        """
        Retorna a posição (x,y,z) e orientação (alpha,beta,gamma) do robô.
        """
        ref = relative_to if relative_to is not None else self.sim.handle_world
        pos = self.sim.getObjectPosition(self.robot_handle, ref)
        ori = self.sim.getObjectOrientation(self.robot_handle, ref)
        return pos, ori
    
    @abstractmethod
    def stop(self):
        """
        Para todos os atuadores conhecidos do robô.
        Deve ser sobrescrito pela subclasse se houver atuadores.
        Por padrão, não faz nada.
        """
        pass