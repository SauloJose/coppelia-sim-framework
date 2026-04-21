from brainbyte.robots.base.base_bot import *

class YouBot(BaseBot):
    """Controle para robô omnidirecional com quatro rodas mecanum."""

    def __init__(self, sim, robot_name='youBot'):
        super().__init__(sim, robot_name)

        # Mapeamento das rodas
        self.wheels = {}
        for wheel in ['rollingJoint_fl', 'rollingJoint_fr', 'rollingJoint_rl', 'rollingJoint_rr']:
            handle = self.sim.getObject(f'/{robot_name}/{wheel}')
            if handle == -1:
                raise ValueError(f"Roda '{wheel}' não encontrada.")
            self.wheels[wheel] = handle

        # Parâmetros físicos (exemplo)
        self.wheel_radius = 0.05
        self.length_x = 0.3   # distância entre rodas dianteiras/traseiras
        self.length_y = 0.2   # distância entre rodas esquerda/direita

    def set_velocity(self, vx, vy, omega):
        """
        Define velocidades no plano (vx, vy) em m/s e rotação omega (rad/s).
        Implementação da cinemática inversa para rodas mecanum.
        """
        # Fórmula típica para mecanum (ajustar conforme orientação)
        w_fl = (vx - vy - omega * (self.length_x + self.length_y)) / self.wheel_radius
        w_fr = (vx + vy + omega * (self.length_x + self.length_y)) / self.wheel_radius
        w_rl = (vx + vy - omega * (self.length_x + self.length_y)) / self.wheel_radius
        w_rr = (vx - vy + omega * (self.length_x + self.length_y)) / self.wheel_radius

        self.sim.setJointTargetVelocity(self.wheels['rollingJoint_fl'], w_fl)
        self.sim.setJointTargetVelocity(self.wheels['rollingJoint_fr'], w_fr)
        self.sim.setJointTargetVelocity(self.wheels['rollingJoint_rl'], w_rl)
        self.sim.setJointTargetVelocity(self.wheels['rollingJoint_rr'], w_rr)

    def stop(self):
        for handle in self.wheels.values():
            self.sim.setJointTargetVelocity(handle, 0.0)