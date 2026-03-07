import math
import pygame

from project.robot.modele.environnement import Environnement
from project.robot.modele.robot_mobile import RobotMobile, EtatRobot
from project.robot.modele.obstacle import ObstacleCirculaire, ObstacleRectangulaire


class VueTerminal:
    """Vue texte pour débogage ou simulation headless."""

    def dessiner(self, environnement: "Environnement") -> None:
        for robot in environnement.robots:
            self.dessiner_robot(robot)

    def dessiner_robot(self, robot: "RobotMobile") -> None:
        print(f"Robot #{robot.id}: (x={robot.x:.2f}, y={robot.y:.2f}, "
              f"orientation={math.degrees(robot.orientation):.1f}°, "
              f"état={robot.etat.name}, énergie={robot.energie_restante:.0f} J)")


class VuePygame:
    """Vue graphique pygame."""

    # Palette de couleurs
    COULEUR_FOND    = (245, 245, 240)
    COULEUR_PAQUET  = (255, 180, 0)     
    COULEUR_DEPOT   = (0, 200, 100)     
    COULEUR_TEXTE   = (40, 40, 40)

    def __init__(self, largeur: int = 800, hauteur: int = 800, scale: int = 50):
        pygame.init()
        self.screen = pygame.display.set_mode((largeur, hauteur))
        pygame.display.set_caption("Simulation Robot Mobile")
        self.largeur = largeur
        self.hauteur = hauteur
        self.scale = scale
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("monospace", 14)

    def convertir_coordonnees(self, x: float, y: float) -> tuple[int, int]:
        px = round(self.largeur / 2 + x * self.scale)
        py = round(self.hauteur / 2 - y * self.scale)
        return px, py

    # ------------------------------------------------------------------
    # Dessin principal
    # ------------------------------------------------------------------

    def dessiner(self, environnement: "Environnement") -> None:
        self.screen.fill(self.COULEUR_FOND)
        self._dessiner_grille(environnement)
        self._dessiner_paquet(environnement)
        self._dessiner_depot(environnement)
        for obstacle in environnement.obstacles:
            self.dessiner_obstacle(obstacle)
        for robot in environnement.robots:
            self.dessiner_robot(robot)
            self._dessiner_hud(robot)
        pygame.display.flip()

    # ------------------------------------------------------------------
    # Paquet & Dépôt
    # ------------------------------------------------------------------

    def _dessiner_paquet(self, environnement: "Environnement") -> None:
        """Dessine le paquet à récupérer (carré orange avec 'P')."""
        px, py = self.convertir_coordonnees(*environnement.position_paquet)
        taille = round(0.4 * self.scale)

        # Vérifie si un robot a déjà récupéré le paquet
        paquet_pris = any(r.etat == EtatRobot.CHARGE for r in environnement.robots)
        couleur = (180, 130, 0) if paquet_pris else self.COULEUR_PAQUET

        rect = pygame.Rect(px - taille, py - taille, taille * 2, taille * 2)
        pygame.draw.rect(self.screen, couleur, rect, border_radius=4)
        pygame.draw.rect(self.screen, (120, 80, 0), rect, 2, border_radius=4)

        label = self.font.render("P", True, (80, 40, 0))
        self.screen.blit(label, (px - label.get_width() // 2,
                                  py - label.get_height() // 2))

    def _dessiner_depot(self, environnement: "Environnement") -> None:
        """Dessine le point de dépôt (cercle vert avec 'D')."""
        px, py = self.convertir_coordonnees(*environnement.position_depot)
        rayon = round(0.5 * self.scale)

        pygame.draw.circle(self.screen, self.COULEUR_DEPOT, (px, py), rayon)
        pygame.draw.circle(self.screen, (0, 120, 60), (px, py), rayon, 2)

        label = self.font.render("D", True, (0, 60, 30))
        self.screen.blit(label, (px - label.get_width() // 2,
                                  py - label.get_height() // 2))

    # ------------------------------------------------------------------
    # HUD (état + énergie du robot)
    # ------------------------------------------------------------------

    def _dessiner_hud(self, robot: "RobotMobile") -> None:
        """Affiche l'état et la barre d'énergie au-dessus du robot."""
        px, py = self.convertir_coordonnees(robot.x, robot.y)
        r = round(robot.rayon * self.scale)

        # État textuel
        etat_label = self.font.render(robot.etat.name, True, self.COULEUR_TEXTE)
        self.screen.blit(etat_label, (px - etat_label.get_width() // 2, py - r - 28))

        # Barre d'énergie
        barre_w = r * 2
        barre_h = 6
        barre_x = px - r
        barre_y = py - r - 14

        ratio = max(0.0, robot.energie_restante / robot.autonomie)
        couleur_barre = (
            int(255 * (1 - ratio)),
            int(200 * ratio),
            0
        )

        pygame.draw.rect(self.screen, (180, 180, 180),
                         (barre_x, barre_y, barre_w, barre_h))
        pygame.draw.rect(self.screen, couleur_barre,
                         (barre_x, barre_y, round(barre_w * ratio), barre_h))
        pygame.draw.rect(self.screen, (80, 80, 80),
                         (barre_x, barre_y, barre_w, barre_h), 1)

    # ------------------------------------------------------------------
    # Grille de fond
    # ------------------------------------------------------------------

    def _dessiner_grille(self, environnement: "Environnement") -> None:
        """Dessine une grille de fond légère."""
        couleur_grille = (220, 220, 215)
        for x in range(-environnement.largeur // 2, environnement.largeur // 2 + 1):
            px, _ = self.convertir_coordonnees(x, 0)
            pygame.draw.line(self.screen, couleur_grille, (px, 0), (px, self.hauteur))
        for y in range(-environnement.hauteur // 2, environnement.hauteur // 2 + 1):
            _, py = self.convertir_coordonnees(0, y)
            pygame.draw.line(self.screen, couleur_grille, (0, py), (self.largeur, py))

    # ------------------------------------------------------------------
    # Robot
    # ------------------------------------------------------------------

    def dessiner_robot(self, robot: "RobotMobile") -> None:
        x, y = self.convertir_coordonnees(robot.x, robot.y)
        r = round(robot.rayon * self.scale)
        angle = robot.orientation

        # Trajectoire
        if len(robot.trajectoire) > 1:
            points = [self.convertir_coordonnees(px, py) for px, py in robot.trajectoire]
            for i in range(len(points) - 1):
                alpha = int(180 * (i / len(points)))
                couleur = (alpha // 3, alpha // 2, min(alpha + 80, 255))
                pygame.draw.line(self.screen, couleur, points[i], points[i + 1], 2)

        # Ombre
        shadow_surface = pygame.Surface((r * 3, r * 3), pygame.SRCALPHA)
        pygame.draw.circle(shadow_surface, (0, 0, 0, 40), (r * 1.5, r * 1.5), r)
        self.screen.blit(shadow_surface, (x - r * 1.5 + 4, y - r * 1.5 + 4))

        # Corps
        width = r * 1.2
        for i in range(5):
            offset = i * 0.2
            shade = 255 - i * 30
            pygame.draw.ellipse(self.screen, (40 + i * 10, 100 + i * 10, shade),
                                (x - width + offset, y - r + offset,
                                 width * 2 - offset * 2, r * 2 - offset * 2))
        pygame.draw.ellipse(self.screen, (20, 40, 80),
                            (x - width, y - r, width * 2, r * 2), 3)

        # Roues
        roue_width  = r // 3
        roue_height = r // 2
        roue_offset = r * 0.85
        for side in [-1, 1]:
            roue_x = x + round(roue_offset * math.cos(angle + side * math.pi / 2))
            roue_y = y - round(roue_offset * math.sin(angle + side * math.pi / 2))
            pygame.draw.rect(self.screen, (30, 30, 30),
                             (roue_x - roue_width, roue_y - roue_height,
                              roue_width * 2, roue_height * 2))
            pygame.draw.rect(self.screen, (60, 60, 60),
                             (roue_x - roue_width + 2, roue_y - roue_height + 2,
                              roue_width * 2 - 4, roue_height * 2 - 4))

        # Capteurs
        for s_angle, s_dist, s_color in [
            (angle,                r * 0.9, (255, 100, 100)),
            (angle + math.pi / 6, r * 0.7, (100, 255, 100)),
            (angle - math.pi / 6, r * 0.7, (100, 255, 100)),
        ]:
            sx = x + round(s_dist * math.cos(s_angle))
            sy = y - round(s_dist * math.sin(s_angle))
            glow = pygame.Surface((20, 20), pygame.SRCALPHA)
            pygame.draw.circle(glow, (*s_color, 80), (10, 10), 8)
            self.screen.blit(glow, (sx - 10, sy - 10))
            pygame.draw.circle(self.screen, s_color, (sx, sy), 4)

        # Flèche de direction
        arrow_length = r * 0.6
        arrow_x = x + round(arrow_length * math.cos(angle))
        arrow_y = y - round(arrow_length * math.sin(angle))
        arrow_points = [
            (arrow_x, arrow_y),
            (arrow_x - round(r * 0.3 * math.cos(angle - math.pi / 6)),
             arrow_y + round(r * 0.3 * math.sin(angle - math.pi / 6))),
            (arrow_x - round(r * 0.3 * math.cos(angle + math.pi / 6)),
             arrow_y + round(r * 0.3 * math.sin(angle + math.pi / 6))),
        ]
        pygame.draw.polygon(self.screen, (255, 50, 50), arrow_points)
        pygame.draw.polygon(self.screen, (150, 0, 0), arrow_points, 2)

        # Particules de vitesse
        if hasattr(robot.moteur, 'v') and abs(robot.moteur.v) > 0.1:
            for i in range(min(5, int(abs(robot.moteur.v) * 3))):
                offset_dist = -r * (1 + i * 0.3)
                particle_x = x + round(offset_dist * math.cos(angle))
                particle_y = y - round(offset_dist * math.sin(angle))
                alpha = 255 - i * 50
                ps = pygame.Surface((6, 6), pygame.SRCALPHA)
                pygame.draw.circle(ps, (150, 200, 255, alpha), (3, 3), 3)
                self.screen.blit(ps, (particle_x - 3, particle_y - 3))

    # ------------------------------------------------------------------
    # Obstacles
    # ------------------------------------------------------------------

    def dessiner_obstacle(self, obstacle) -> None:
        if isinstance(obstacle, ObstacleCirculaire):
            px, py = self.convertir_coordonnees(obstacle.x, obstacle.y)
            pygame.draw.circle(self.screen, obstacle.couleur,
                               (px, py), round(obstacle.rayon * self.scale))
        elif isinstance(obstacle, ObstacleRectangulaire):
            px, py = self.convertir_coordonnees(obstacle.x, obstacle.y)
            lw = round(obstacle.largeur * self.scale)
            lh = round(obstacle.hauteur * self.scale)
            pygame.draw.rect(self.screen, obstacle.couleur,
                             (px - lw // 2, py - lh // 2, lw, lh))

    # ------------------------------------------------------------------
    # Utilitaires pygame
    # ------------------------------------------------------------------

    def gerer_evenements(self) -> bool:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
        return True

    def limiter_fps(self, fps: int = 60) -> None:
        self.clock.tick(fps)

    def fermer(self) -> None:
        pygame.quit()