from abc import ABC, abstractmethod
import numpy as np

class BaseBot(ABC):
    """
    Classe base abstrata para qualquer robô no CoppeliaSim.
    Fornece apenas:
    - Acesso ao cliente da API
    - Handle do objeto raiz do robô
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

    def get_pose(self, relative_to=None):
        """
        Retorna a posição (x,y,z) e orientação (alpha,beta,gamma) do robô.
        """
        ref = relative_to if relative_to is not None else self.sim.handle_world
        pos = self.sim.getObjectPosition(self.robot_handle, ref)
        ori = self.sim.getObjectOrientation(self.robot_handle, ref)
        return pos, ori

    def stop(self):
        """
        Para todos os atuadores conhecidos do robô.
        Deve ser sobrescrito pela subclasse se houver atuadores.
        Por padrão, não faz nada.
        """
        pass