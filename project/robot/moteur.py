from abc import ABC, abstractmethod
from math import cos, sin
class Moteur(ABC):

    @abstractmethod
    def commander(self, *args):
        pass

    @abstractmethod
    def mettre_a_jour(self, robot, dt):
        pass


class MoteurDifferentiel(Moteur):
    def __init__(self, v=0.0, omega=0.0):
        self.v = v
        self.omega = omega

    def commander(self, v, omega):
        self.v = v
        self.omega = omega

    def mettre_a_jour(self, robot, dt):
        # Update the robot's position and orientation based on the current velocities
        robot.x += self.v * cos(robot.orientation) * dt
        robot.y += self.v * sin(robot.orientation) * dt
        robot.orientation += self.omega * dt

    
class MoteurOmnidirectionnel(Moteur):
    def __init__(self, vx=0.0, vy=0.0, omega=0.0):
        self.vx = vx # vitesse avant
        self.vy = vy # vitesse latrale
        self.omega = omega
    
    def commander(self, vx, vy, omega):
        self.vx = vx
        self.vy = vy
        self.omega = omega

    def mettre_a_jour(self, robot, dt):
        # Update the robot's position and orientation based on the current velocities
        robot.x += self.vx * cos(robot.orientation) * dt - self.vy * sin(robot.orientation) * dt
        robot.y += self.vx * sin(robot.orientation) * dt + self.vy * cos(robot.orientation) * dt
        robot.orientation += self.omega * dt