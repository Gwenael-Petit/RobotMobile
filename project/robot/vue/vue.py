import math
import pygame

from project.robot.modele.environnement import Environnement
from project.robot.modele.robot_mobile import RobotMobile
from project.robot.modele.obstacle import ObstacleCirculaire, ObstacleRectangulaire


class VueTerminal:
    """Vue texte pour débogage ou simulation headless."""

    def dessiner(self, environnement: "Environnement") -> None:
        for robot in environnement.robots:
            self.dessiner_robot(robot)

    def dessiner_robot(self, robot: "RobotMobile") -> None:
        print(f"Robot: (x={robot.x:.2f}, y={robot.y:.2f}, "
              f"orientation={math.degrees(robot.orientation):.1f}°)")


class VuePygame:
    """Vue graphique pygame."""

    def __init__(self, largeur: int = 800, hauteur: int = 600, scale: int = 50):
        pygame.init()
        self.screen = pygame.display.set_mode((largeur, hauteur))
        pygame.display.set_caption("Simulation Robot Mobile")
        self.largeur = largeur
        self.hauteur = hauteur
        self.scale = scale  # mètres -> pixels
        self.clock = pygame.time.Clock()

    def convertir_coordonnees(self, x: float, y: float) -> tuple[int, int]:
        px = round(self.largeur / 2 + x * self.scale)
        py = round(self.hauteur / 2 - y * self.scale)
        return px, py

    def dessiner(self, environnement) -> None:
        self.screen.fill((255, 255, 255))
        for obstacle in environnement.obstacles:
            self.dessiner_obstacle(obstacle)
        for robot in environnement.robots:
            self.dessiner_robot(robot)
        pygame.display.flip()

    def dessiner_robot(self, robot: "RobotMobile") -> None:
        x, y = self.convertir_coordonnees(robot.x, robot.y)
        r = round(robot.rayon * self.scale)
        angle = robot.orientation
        
        # === 1. TRAJECTOIRE avec dégradé ===
        #if len(robot.trajectoire) > 1:
        #    points = [self.convertir_coordonnees(px, py) for px, py in robot.trajectoire]
         #   for i in range(len(points) - 1):
          #      alpha = int(255 * (i / len(points)))  # fade progressif
           #     couleur = (alpha // 3, alpha // 2, alpha)
            #    pygame.draw.line(self.screen, couleur, points[i], points[i + 1], 2)
        
        # === 2. OMBRE PORTÉE douce ===
        shadow_surface = pygame.Surface((r * 3, r * 3), pygame.SRCALPHA)
        pygame.draw.circle(shadow_surface, (0, 0, 0, 40), (r * 1.5, r * 1.5), r)
        self.screen.blit(shadow_surface, (x - r * 1.5 + 4, y - r * 1.5 + 4))
        
        # === 3. CHÂSSIS PRINCIPAL (forme de capsule) ===
        # Calculer les points pour une forme allongée
        length = r * 1.4
        width = r * 1.2
        
        # Points de la capsule orientée
        front_x = x + round(length * 0.3 * math.cos(angle))
        front_y = y - round(length * 0.3 * math.sin(angle))
        back_x = x - round(length * 0.3 * math.cos(angle))
        back_y = y + round(length * 0.3 * math.sin(angle))
        
        # Corps principal avec dégradé (simulé par plusieurs cercles)
        for i in range(5):
            offset = i * 0.2
            shade = 255 - i * 30
            pygame.draw.ellipse(self.screen, (40 + i * 10, 100 + i * 10, shade),
                            (x - width + offset, y - r + offset, 
                                width * 2 - offset * 2, r * 2 - offset * 2))
        
        # Contour principal
        pygame.draw.ellipse(self.screen, (20, 40, 80), 
                        (x - width, y - r, width * 2, r * 2), 3)
        
        # === 4. ROUES DÉTAILLÉES ===
        roue_width = r // 3
        roue_height = r // 2
        roue_offset = r * 0.85
        
        for side in [-1, 1]:  # gauche et droite
            roue_x = x + round(roue_offset * math.cos(angle + side * math.pi/2))
            roue_y = y - round(roue_offset * math.sin(angle + side * math.pi/2))
            
            # Roue avec profondeur
            pygame.draw.rect(self.screen, (30, 30, 30),
                            (roue_x - roue_width, roue_y - roue_height, 
                            roue_width * 2, roue_height * 2))
            pygame.draw.rect(self.screen, (60, 60, 60),
                            (roue_x - roue_width + 2, roue_y - roue_height + 2, 
                            roue_width * 2 - 4, roue_height * 2 - 4))
            pygame.draw.rect(self.screen, (0, 0, 0),
                            (roue_x - roue_width, roue_y - roue_height, 
                            roue_width * 2, roue_height * 2), 2)
        
        # === 5. CAPTEURS / LIDARS (trois points) ===
        sensor_positions = [
            (angle, r * 0.9, (255, 100, 100)),           # avant
            (angle + math.pi/6, r * 0.7, (100, 255, 100)),   # avant-gauche
            (angle - math.pi/6, r * 0.7, (100, 255, 100)),   # avant-droite
        ]
        
        for sensor_angle, sensor_dist, sensor_color in sensor_positions:
            sx = x + round(sensor_dist * math.cos(sensor_angle))
            sy = y - round(sensor_dist * math.sin(sensor_angle))
            
            # Effet de lueur
            glow_surface = pygame.Surface((20, 20), pygame.SRCALPHA)
            pygame.draw.circle(glow_surface, (*sensor_color, 80), (10, 10), 8)
            self.screen.blit(glow_surface, (sx - 10, sy - 10))
            
            # Capteur central
            pygame.draw.circle(self.screen, sensor_color, (sx, sy), 4)
            pygame.draw.circle(self.screen, (0, 0, 0), (sx, sy), 4, 1)
        
        # === 6. INDICATEUR DE DIRECTION (flèche stylisée) ===
        arrow_length = r * 0.6
        arrow_x = x + round(arrow_length * math.cos(angle))
        arrow_y = y - round(arrow_length * math.sin(angle))
        
        # Pointe de la flèche
        arrow_points = [
            (arrow_x, arrow_y),
            (arrow_x - round(r * 0.3 * math.cos(angle - math.pi/6)),
            arrow_y + round(r * 0.3 * math.sin(angle - math.pi/6))),
            (arrow_x - round(r * 0.3 * math.cos(angle + math.pi/6)),
            arrow_y + round(r * 0.3 * math.sin(angle + math.pi/6)))
        ]
        pygame.draw.polygon(self.screen, (255, 50, 50), arrow_points)
        pygame.draw.polygon(self.screen, (150, 0, 0), arrow_points, 2)
        
        # === 7. VITESSE VISUELLE (particules) ===
        if hasattr(robot.moteur, 'v') and abs(robot.moteur.v) > 0.1:
            vitesse = robot.moteur.v
            nb_particules = min(5, int(abs(vitesse) * 3))
            
            for i in range(nb_particules):
                offset_dist = -r * (1 + i * 0.3)
                particle_x = x + round(offset_dist * math.cos(angle))
                particle_y = y - round(offset_dist * math.sin(angle))
                alpha = 255 - i * 50
                
                particle_surface = pygame.Surface((6, 6), pygame.SRCALPHA)
                pygame.draw.circle(particle_surface, (150, 200, 255, alpha), (3, 3), 3)
                self.screen.blit(particle_surface, (particle_x - 3, particle_y - 3))

    def dessiner_obstacle(self, obstacle) -> None:
        if isinstance(obstacle, ObstacleCirculaire):
            px, py = self.convertir_coordonnees(obstacle.x, obstacle.y)
            pygame.draw.circle(self.screen, obstacle.couleur,
                               (px, py), round(obstacle.rayon * self.scale))
           
        elif isinstance(obstacle, ObstacleRectangulaire):
            # Convertir le coin supérieur gauche
            px_centre, py_centre = self.convertir_coordonnees(obstacle.x, obstacle.y)
            largeur_px = round(obstacle.largeur * self.scale)
            hauteur_px = round(obstacle.hauteur * self.scale)
            # Dessiner depuis le coin supérieur gauche
            pygame.draw.rect(
                self.screen,
                obstacle.couleur,
                (px_centre - largeur_px // 2, py_centre - hauteur_px // 2, 
                largeur_px, hauteur_px)
            )

    def gerer_evenements(self) -> bool:
        """Retourne False si l'utilisateur ferme la fenêtre."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
        return True

    def limiter_fps(self, fps: int = 60) -> None:
        """Limite la boucle principale à `fps` images par seconde."""
        self.clock.tick(fps)

    def fermer(self) -> None:
        pygame.quit()