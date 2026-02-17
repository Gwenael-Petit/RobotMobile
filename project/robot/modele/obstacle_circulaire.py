import math

import pygame
from project.robot.modele.obstacle import Obstacle


class ObstacleCirculaire(Obstacle):
    def __init__(self, x, y, rayon):
        self.x = x
        self.y = y
        self.rayon = rayon

    def collision(self, x_robot, y_robot, rayon_robot):
        """
        Collision cercle-cercle :
        d <= r_obstacle + r_robot
        """
        dx = x_robot - self.x
        dy = y_robot - self.y
        distance = math.sqrt(dx**2 + dy**2)

        return distance <= (self.rayon + rayon_robot)

    def dessiner(self, vue):
        if hasattr(vue, "convertir_coordonnees"):
            px, py = vue.convertir_coordonnees(self.x, self.y)
            pygame.draw.circle(
                vue.screen,
                (100, 100, 100),
                (px, py),
                int(self.rayon * vue.scale),
            )
