import numpy as np
from brainbyte.robots.base.base_bot import BaseBot


# Class to a Dummy in the simulation
class Dummy(BaseBot):
    def __init__(self, bridge, obj_name=None):
        """
        sim: The CoppeliaSim API instance object ('sim')
        robot_path: The base path of the Pioneer model in the scene hierarchy
        """
        super().__init__(bridge=bridge, robot_name=obj_name)

        self.name = obj_name

    def stop(self):
        """
        Implementação obrigatória do método abstrato.
        Como o Dummy é apenas um alvo inanimado, não precisamos fazer nada ao parar.
        """
        pass
    