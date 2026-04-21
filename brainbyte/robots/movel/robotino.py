from brainbyte.robots.base.base_bot import *

class Robotino(BaseBot):
    """Controle específico para o robô "robotino"."""

    def __init__(self, sim, robot_name='PioneerP3DX'):
        super().__init__(sim, robot_name)

        # Mapeamento de juntas (pode ser dinâmico)
        self.joints = {
            'left_wheel': self.sim.getObject(f'/{robot_name}/leftMotor'),
            'right_wheel': self.sim.getObject(f'/{robot_name}/rightMotor')
        }

        # Verificação de handles
        for name, handle in self.joints.items():
            if handle == -1:
                raise ValueError(f"Junta '{name}' não encontrada no robô '{robot_name}'.")

        # Parâmetros cinemáticos (específicos do modelo)
        self.wheel_radius = 0.0975   # m
        self.wheel_base = 0.381      # m

    def set_wheel_velocity(self, linear_vel, angular_vel):
        """Define velocidades linear (m/s) e angular (rad/s) usando cinemática diferencial."""
        v = linear_vel
        w = angular_vel
        r = self.wheel_radius
        L = self.wheel_base

        wl = (v / r) - (w * L) / (2 * r)
        wr = (v / r) + (w * L) / (2 * r)

        self.sim.setJointTargetVelocity(self.joints['left_wheel'], wl)
        self.sim.setJointTargetVelocity(self.joints['right_wheel'], wr)

    def stop(self):
        """Para as rodas."""
        self.set_wheel_velocity(0.0, 0.0)