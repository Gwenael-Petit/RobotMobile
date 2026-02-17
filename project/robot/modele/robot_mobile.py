
from math import cos, sin
from .moteur import *


class RobotMobile:

    _nb_robots = 0

    def __init__(self, moteur=None):
        self.__x = 0
        self.__y = 0
        self.__rotation = 0
        self.moteur = moteur
        RobotMobile._nb_robots += 1
        

    def move_forward(self, distance):
        self.__x += distance * cos(self.__rotation)
        self.__y += distance * sin(self.__rotation)
    def rotate(self, angle):
        self.__rotation += angle
        self.__rotation = self.__rotation % (2 * 3.141592653589793)

    def commander(self, **kwargs):
        if self.moteur is not None:
            self.moteur.commander(**kwargs)
    def mettre_a_jour(self, dt):
        if self.moteur is not None:
            self.moteur.mettre_a_jour(self, dt)

    @classmethod
    def nombre_robots(cls) -> int:
        """
        Retourne le nombre total de robots crees.
        """
        return cls._nb_robots
    
    @staticmethod
    def moteur_valide(moteur):
        return isinstance(moteur, Moteur)
            
    #Getters and setters
    @property
    def x(self):
        return self.__x
    
    @property
    def y(self):
        return self.__y
    
    @property
    def orientation(self):
        return self.__rotation
    
    @x.setter
    def x(self, value):
        self.__x = value
    @y.setter
    def y(self, value):
        self.__y = value
    
    @orientation.setter
    def orientation(self, value):
        self.__rotation = value

    def afficher(self):
        print(f"Robot position: (x={self.x}, y={self.y}, orientation={self.orientation})")

    def __str__(self):
        return str((self.x, self.y, self.orientation))