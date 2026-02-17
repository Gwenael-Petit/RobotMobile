import math
import pygame

class VueTerminal:
    def dessiner_robot(self, robot):
        print(f"Robot position: (x={robot.x}, y={robot.y}, orientation={robot.orientation})")

class VuePygame:
    def __init__(self, largeur=800, hauteur=600, scale=50):
        pygame.init()
        self.screen = pygame.display.set_mode((largeur, hauteur))

        pygame.display.set_caption("Simulation Robot Mobile")
        self.largeur = largeur
        self.hauteur = hauteur
        self.scale = scale # metres -> pixels
        self.clock = pygame.time.Clock()

    def convertir_coordonnees(self, x, y):
        px = int(self.largeur / 2 + (x * self.scale))
        py = int(self.hauteur / 2 - (y * self.scale))
        return px, py
    
    def dessiner_robot(self, robot):
        x, y = self.convertir_coordonnees(robot.x, robot.y)
        r = 30

        # dessiner un cercle representant le robot (pygame.draw.circle)
        pygame.draw.circle(self.screen, (0, 100, 255), (x, y), r)

        x_dir = x + int(r * math.cos(robot.orientation))
        y_dir = y - int(r * math.sin(robot.orientation))

        # dessiner un trait representant l'orientation du robot (pygame.draw.line)
        pygame.draw.line(self.screen, (255, 0, 0), (x, y), (x_dir, y_dir), 2)

    def tick(self, fps=60):
        self.clock.tick(fps)