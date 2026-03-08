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
        print(f"Robot: (x={robot.x:.2f}, y={robot.y:.2f}, "
              f"orientation={math.degrees(robot.orientation):.1f}°, "
              f"état={robot.etat.name})")


class VuePygame:
    """Vue graphique pygame."""

    # ── Couleurs ─────────────────────────────────────────────────────────
    FOND          = (245, 245, 240)
    GRILLE        = (220, 220, 215)
    CHEMIN        = (80, 160, 255)      # bleu clair — chemin A*
    WAYPOINT      = (50, 120, 220)      # bleu foncé — points du chemin
    PAQUET        = (255, 180, 0)       # orange
    DEPOT         = (0, 200, 100)       # vert
    TEXTE         = (30, 30, 30)
    HUD_FOND      = (20, 20, 20, 180)   # fond semi-transparent HUD

    # Couleurs état robot
    ETAT_COULEURS = {
        "EN_ATTENTE"  : (150, 150, 150),
        "VERS_PAQUET" : (80,  160, 255),
        "CHARGE"      : (255, 180,   0),
        "LIVRE"       : (0,   200, 100),
        "EN_PANNE"    : (220,  50,  50),
        "VERS_BASE"   : (220, 100, 220),
        "EN_RECHARGE" : (100, 220, 255),
    }

    def __init__(self, largeur: int = 800, hauteur: int = 800, scale: int = 50):
        pygame.init()
        self.screen = pygame.display.set_mode((largeur, hauteur))
        pygame.display.set_caption("Simulation Robot Mobile")
        self.largeur = largeur
        self.hauteur = hauteur
        self.scale   = scale
        self.clock   = pygame.time.Clock()
        self.font_sm = pygame.font.SysFont("monospace", 13)
        self.font_md = pygame.font.SysFont("monospace", 15, bold=True)

    # ------------------------------------------------------------------
    # Conversion coordonnées
    # ------------------------------------------------------------------

    def convertir_coordonnees(self, x: float, y: float) -> tuple[int, int]:
        px = round(self.largeur / 2 + x * self.scale)
        py = round(self.hauteur / 2 - y * self.scale)
        return px, py

    # ------------------------------------------------------------------
    # Dessin principal
    # ------------------------------------------------------------------

    def dessiner(self, environnement: "Environnement") -> None:
        self.screen.fill(self.FOND)
        self._dessiner_grille(environnement)
        self._dessiner_paquet(environnement)
        self._dessiner_depot(environnement)

        for obstacle in environnement.obstacles:
            self.dessiner_obstacle(obstacle)

        for robot in environnement.robots:
            self._dessiner_chemin(robot)       # ← chemin A* SOUS le robot
            self.dessiner_robot(robot)
            self._dessiner_barre_energie(robot)
            self._dessiner_etat(robot)

        self._dessiner_hud(environnement)      # ← métriques en bas à gauche

        pygame.display.flip()

    # ------------------------------------------------------------------
    # 1. Chemin A* ← NOUVEAU
    # ------------------------------------------------------------------

    def _dessiner_chemin(self, robot: "RobotMobile") -> None:
        """Trace le chemin A* restant à parcourir."""
        if not hasattr(robot, '_pid') or robot._pid is None:
            return
        chemin = robot._pid.chemin
        if not chemin or len(chemin) < 2:
            return

        idx = robot._pid._index_waypoint

        # Segment robot → prochain waypoint
        if idx < len(chemin):
            px_robot, py_robot = self.convertir_coordonnees(robot.x, robot.y)
            px_next,  py_next  = self.convertir_coordonnees(*chemin[idx])
            pygame.draw.line(self.screen, self.CHEMIN,
                             (px_robot, py_robot), (px_next, py_next), 2)

        # Reste du chemin
        for i in range(max(idx, 1), len(chemin)):
            p1 = self.convertir_coordonnees(*chemin[i - 1])
            p2 = self.convertir_coordonnees(*chemin[i])
            pygame.draw.line(self.screen, self.CHEMIN, p1, p2, 2)

        # Waypoints
        for i in range(idx, len(chemin)):
            px, py = self.convertir_coordonnees(*chemin[i])
            pygame.draw.circle(self.screen, self.WAYPOINT, (px, py), 3)

    # ------------------------------------------------------------------
    # 2. Barre d'énergie + état ← NOUVEAU
    # ------------------------------------------------------------------

    def _dessiner_barre_energie(self, robot: "RobotMobile") -> None:
        """Barre d'énergie colorée au-dessus du robot."""
        px, py = self.convertir_coordonnees(robot.x, robot.y)
        r      = round(robot.rayon * self.scale)

        barre_w = r * 2
        barre_h = 6
        bx      = px - r
        by      = py - r - 14

        ratio  = max(0.0, robot.energie_restante / robot.autonomie)
        couleur = (int(255 * (1 - ratio)), int(200 * ratio), 0)

        # Fond gris
        pygame.draw.rect(self.screen, (180, 180, 180), (bx, by, barre_w, barre_h))
        # Niveau d'énergie
        pygame.draw.rect(self.screen, couleur, (bx, by, round(barre_w * ratio), barre_h))
        # Contour
        pygame.draw.rect(self.screen, (80, 80, 80), (bx, by, barre_w, barre_h), 1)

    def _dessiner_etat(self, robot: "RobotMobile") -> None:
        """Texte d'état coloré au-dessus de la barre d'énergie."""
        px, py  = self.convertir_coordonnees(robot.x, robot.y)
        r       = round(robot.rayon * self.scale)
        couleur = self.ETAT_COULEURS.get(robot.etat.name, self.TEXTE)

        label = self.font_sm.render(robot.etat.name, True, couleur)
        self.screen.blit(label, (px - label.get_width() // 2, py - r - 28))

    # ------------------------------------------------------------------
    # 3. HUD métriques ← NOUVEAU
    # ------------------------------------------------------------------

    def _dessiner_hud(self, environnement: "Environnement") -> None:
        if not environnement.robots:
            return

        padding = 8
        lh      = 18
        x0      = 10
        y0      = 10   # en haut à gauche

        for i, robot in enumerate(environnement.robots):
            couleur = getattr(robot, 'couleur', (220, 220, 220))
            label   = getattr(robot, 'label',   f"Robot {i+1}")
            lignes  = [
                f"{label}",
                f"Etat    : {robot.etat.name}",
                f"Vitesse : {robot.vitesse_max:.1f} m/s",       # ← ici
                f"Charge  : {robot.capacite_charge} kg",        # ← ici
                f"Autonom : {robot.autonomie:.0f} J",           # ← ici
                f"Energie : {robot.energie_restante:.0f}/{robot.autonomie:.0f} J",
                f"Colis   : {robot.colis_livres}/{robot.colis_a_livrer}",
                f"Cout    : {robot.calculer_cout():.1f}",
                f"Temps   : {robot.temps_mission:.1f} s",
            ]

            w = 220
            h = padding * 2 + lh * len(lignes)

            surf = pygame.Surface((w, h), pygame.SRCALPHA)
            surf.fill((20, 20, 20, 170))
            self.screen.blit(surf, (x0, y0))
            pygame.draw.rect(self.screen, couleur, (x0, y0, w, h), 2)

            for j, ligne in enumerate(lignes):
                c     = couleur if j == 0 else (220, 220, 220)
                texte = self.font_sm.render(ligne, True, c)
                self.screen.blit(texte, (x0 + padding, y0 + padding + j * lh))

            y0 += h + 8   # empile les HUD verticalement

    # ------------------------------------------------------------------
    # Paquet & Dépôt
    # ------------------------------------------------------------------

    def _dessiner_paquet(self, environnement: "Environnement") -> None:
        for i, pos in enumerate(environnement.positions_colis):
            px, py  = self.convertir_coordonnees(*pos)
            taille  = round(0.4 * self.scale)
            
            # Grisé si déjà livré (on estime via le robot le plus avancé)
            max_livres = max((r.colis_livres for r in environnement.robots), default=0)
            deja_livre = max_livres > i
            
            couleur = (180, 180, 180) if deja_livre else self.PAQUET
            contour = (100, 100, 100) if deja_livre else (120, 80, 0)

            rect = pygame.Rect(px - taille, py - taille, taille * 2, taille * 2)
            pygame.draw.rect(self.screen, couleur, rect, border_radius=4)
            pygame.draw.rect(self.screen, contour, rect, 2, border_radius=4)
            label = self.font_sm.render(str(i + 1), True, (80, 40, 0))
            self.screen.blit(label, (px - label.get_width() // 2,
                                    py - label.get_height() // 2))

    def _dessiner_depot(self, environnement: "Environnement") -> None:
        px, py = self.convertir_coordonnees(*environnement.position_depot)
        rayon  = round(0.5 * self.scale)
        pygame.draw.circle(self.screen, self.DEPOT, (px, py), rayon)
        pygame.draw.circle(self.screen, (0, 120, 60), (px, py), rayon, 2)
        label = self.font_sm.render("D", True, (0, 60, 30))
        self.screen.blit(label, (px - label.get_width() // 2,
                                  py - label.get_height() // 2))

    # ------------------------------------------------------------------
    # Grille de fond
    # ------------------------------------------------------------------

    def _dessiner_grille(self, environnement: "Environnement") -> None:
        for x in range(-environnement.largeur // 2, environnement.largeur // 2 + 1):
            px, _ = self.convertir_coordonnees(x, 0)
            pygame.draw.line(self.screen, self.GRILLE, (px, 0), (px, self.hauteur))
        for y in range(-environnement.hauteur // 2, environnement.hauteur // 2 + 1):
            _, py = self.convertir_coordonnees(0, y)
            pygame.draw.line(self.screen, self.GRILLE, (0, py), (self.largeur, py))

    # ------------------------------------------------------------------
    # Robot (inchangé — ton rendu original)
    # ------------------------------------------------------------------

    def dessiner_robot(self, robot: "RobotMobile") -> None:
        x, y  = self.convertir_coordonnees(robot.x, robot.y)
        r     = round(robot.rayon * self.scale)
        angle = robot.orientation

        # Trajectoire
        if len(robot.trajectoire) > 1:
            points = [self.convertir_coordonnees(px, py)
                      for px, py in robot.trajectoire]
            for i in range(len(points) - 1):
                alpha  = int(180 * (i / len(points)))
                couleur = (alpha // 3, alpha // 2, min(alpha + 80, 255))
                pygame.draw.line(self.screen, couleur,
                                 points[i], points[i + 1], 2)

        # Ombre
        shadow = pygame.Surface((r * 3, r * 3), pygame.SRCALPHA)
        pygame.draw.circle(shadow, (0, 0, 0, 40), (r * 1.5, r * 1.5), r)
        self.screen.blit(shadow, (x - r * 1.5 + 4, y - r * 1.5 + 4))

        # Corps — utilise robot.couleur si défini
        couleur_base = getattr(robot, 'couleur', (40, 100, 200))
        width = r * 1.2
        for i in range(5):
            offset = i * 0.2
            r_val  = max(0, min(255, couleur_base[0] - i * 20))
            g_val  = max(0, min(255, couleur_base[1] - i * 20))
            b_val  = max(0, min(255, couleur_base[2] - i * 20))
            pygame.draw.ellipse(self.screen, (r_val, g_val, b_val),
                                (x - width + offset, y - r + offset,
                                 width * 2 - offset * 2, r * 2 - offset * 2))
        contour = tuple(max(0, c - 60) for c in couleur_base)
        pygame.draw.ellipse(self.screen, contour,
                            (x - width, y - r, width * 2, r * 2), 3)

        # Roues
        roue_w = r // 3
        roue_h = r // 2
        for side in [-1, 1]:
            rx = x + round(r * 0.85 * math.cos(angle + side * math.pi / 2))
            ry = y - round(r * 0.85 * math.sin(angle + side * math.pi / 2))
            pygame.draw.rect(self.screen, (30, 30, 30),
                             (rx - roue_w, ry - roue_h, roue_w * 2, roue_h * 2))
            pygame.draw.rect(self.screen, (60, 60, 60),
                             (rx - roue_w + 2, ry - roue_h + 2,
                              roue_w * 2 - 4, roue_h * 2 - 4))

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

        # Flèche direction
        al = r * 0.6
        ax = x + round(al * math.cos(angle))
        ay = y - round(al * math.sin(angle))
        arrow = [
            (ax, ay),
            (ax - round(r * 0.3 * math.cos(angle - math.pi / 6)),
             ay + round(r * 0.3 * math.sin(angle - math.pi / 6))),
            (ax - round(r * 0.3 * math.cos(angle + math.pi / 6)),
             ay + round(r * 0.3 * math.sin(angle + math.pi / 6))),
        ]
        pygame.draw.polygon(self.screen, (255, 50, 50), arrow)
        pygame.draw.polygon(self.screen, (150, 0, 0), arrow, 2)

        # Particules de vitesse
        if hasattr(robot.moteur, 'v') and abs(robot.moteur.v) > 0.1:
            for i in range(min(5, int(abs(robot.moteur.v) * 3))):
                od  = -r * (1 + i * 0.3)
                ppx = x + round(od * math.cos(angle))
                ppy = y - round(od * math.sin(angle))
                ps  = pygame.Surface((6, 6), pygame.SRCALPHA)
                pygame.draw.circle(ps, (150, 200, 255, 255 - i * 50), (3, 3), 3)
                self.screen.blit(ps, (ppx - 3, ppy - 3))

    # ------------------------------------------------------------------
    # Obstacles (inchangé)
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
    # Utilitaires pygame (inchangé)
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