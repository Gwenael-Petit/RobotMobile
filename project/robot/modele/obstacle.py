from abc import ABC, abstractmethod
import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from project.robot.modele.robot_mobile import RobotMobile

class Obstacle(ABC):
    """
    Classe abstraite représentant un obstacle.
    Tous les obstacles doivent implémenter collision() et bounding_box().
    """

    @abstractmethod
    def collision(self, robot) -> bool:
        """
        Retourne True si le robot entre en collision avec cet obstacle.
        """
        pass

    @abstractmethod
    def bounding_box(self) -> tuple[float, float, float, float]:
        """
        Retourne (x_min, y_min, x_max, y_max) de l'obstacle.
        """
        pass

class ObstacleCirculaire(Obstacle):
    """Obstacle de forme circulaire."""

    def __init__(self, x: float, y: float, rayon: float, couleur: tuple = (100, 100, 100)):
        if rayon <= 0:
            raise ValueError(f"Le rayon doit être strictement positif, reçu : {rayon}")
        self.x = x
        self.y = y
        self.rayon = rayon
        self.couleur = couleur

    def collision(self, robot: "RobotMobile") -> bool:
        """
        Collision cercle-cercle : d <= r_obstacle + r_robot
        """
        distance = math.hypot(robot.x - self.x, robot.y - self.y)
        return distance <= (self.rayon + robot.rayon)

    def bounding_box(self) -> tuple[float, float, float, float]:
        """
        Retourne (x_min, y_min, x_max, y_max) de l'obstacle.
        """
        return (
            self.x - self.rayon,
            self.y - self.rayon,
            self.x + self.rayon,
            self.y + self.rayon,
        )

    def __repr__(self) -> str:
        return f"ObstacleCirculaire(x={self.x}, y={self.y}, rayon={self.rayon})"

class ObstacleRectangulaire(Obstacle):
    """Obstacle de forme rectangulaire aligné avec les axes."""

    def __init__(self, x: float, y: float, largeur: float, hauteur: float, 
                 couleur: tuple = (80, 80, 80)):
        if largeur <= 0 or hauteur <= 0:
            raise ValueError("Largeur et hauteur doivent être strictement positives")
        self.x = x  # centre du rectangle
        self.y = y  # centre du rectangle
        self.largeur = largeur
        self.hauteur = hauteur
        self.couleur = couleur

    def collision(self, robot: "RobotMobile") -> bool:
        """
        Collision rectangle-cercle (approximation par point le plus proche).
        """
        # Trouver le point du rectangle le plus proche du centre du robot
        closest_x = max(self.x - self.largeur / 2, 
                       min(robot.x, self.x + self.largeur / 2))
        closest_y = max(self.y - self.hauteur / 2, 
                       min(robot.y, self.y + self.hauteur / 2))
        
        # Distance entre ce point et le centre du robot
        distance = math.hypot(robot.x - closest_x, robot.y - closest_y)
        
        return distance <= robot.rayon

    def bounding_box(self) -> tuple[float, float, float, float]:
        """Retourne (x_min, y_min, x_max, y_max) de l'obstacle."""
        return (
            self.x - self.largeur / 2,
            self.y - self.hauteur / 2,
            self.x + self.largeur / 2,
            self.y + self.hauteur / 2,
        )

    def __repr__(self) -> str:
        return (f"ObstacleRectangulaire(x={self.x}, y={self.y}, "
                f"largeur={self.largeur}, hauteur={self.hauteur})")