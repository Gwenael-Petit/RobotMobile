from abc import ABC, abstractmethod
from math import cos, sin
class Moteur(ABC):

    @abstractmethod
    def commander(self, **kwargs):
        pass

    @abstractmethod
    def mettre_a_jour(self, robot, dt):
        pass


class MoteurDifferentiel(Moteur):
    def __init__(self, v=0.0, omega=0.0):
        self.v = v
        self.omega = omega

    def commander(self, v, omega) -> None:
        if not isinstance(v, (int, float)) or not isinstance(omega, (int, float)):
            raise TypeError("Les vitesses doivent être des nombres")
        self.v = float(v)
        self.omega = float(omega)

    def mettre_a_jour(self, robot, dt) -> None:
        # Update the robot's position and orientation based on the current velocities
        robot.x += self.v * cos(robot.orientation) * dt
        robot.y += self.v * sin(robot.orientation) * dt
        robot.orientation += self.omega * dt

    
class MoteurOmnidirectionnel(Moteur):
    def __init__(self, vx=0.0, vy=0.0, omega=0.0):
        if not isinstance(vx, (int, float)) or not isinstance(vy, (int, float)) or not isinstance(omega, (int, float)):
            raise TypeError("Les vitesses doivent être des nombres")
        self.vx = vx # vitesse avant
        self.vy = vy # vitesse latrale
        self.omega = omega
    
    def commander(self, vx, vy, omega) -> None:
        self.vx = vx
        self.vy = vy
        self.omega = omega

    def mettre_a_jour(self, robot, dt) -> None:
        # Update the robot's position and orientation based on the current velocities
        c = cos(robot.orientation)
        s = sin(robot.orientation)
        robot.x += self.vx * c * dt - self.vy * s * dt
        robot.y += self.vx * s * dt + self.vy * c * dt
        robot.orientation += self.omega * dt