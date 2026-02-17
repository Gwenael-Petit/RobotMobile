from abc import ABC, abstractmethod
import math
import pygame

class Obstacle(ABC):
    """
    Classe abstraite représentant un obstacle.
    Tous les obstacles doivent implémenter collision() et dessiner().
    """

    @abstractmethod
    def collision(self, x: float, y: float, rayon_robot: float) -> bool:
        """
        Retourne True si le robot (x, y, rayon_robot) entre en collision.
        """
        pass

    @abstractmethod
    def dessiner(self, vue):
        """
        Permet à l’obstacle de s’afficher via une vue.
        """
        pass
