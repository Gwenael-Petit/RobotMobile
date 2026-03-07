import heapq


class Environnement:
    def __init__(self, largeur=15, hauteur=15, position_paquet: tuple[float, float] = (6.0, 6.0),
                 position_depot: tuple[float, float] = (-6.0, -6.0)):
        self.largeur = largeur      # en mètres
        self.hauteur = hauteur      # en mètres
        self.robots = []          
        self.obstacles = []

        # Attributs pour la mission
        self.position_paquet = position_paquet   # (x, y) du paquet à récupérer
        self.position_depot  = position_depot    # (x, y) du point de livraison

        # Grille de navigation pour A*
        self._resolution = 0.5
        self._grille_bloquee: set[tuple[int, int]] = set()

    # ------------------------------------------------------------------
    # Gestion robots & obstacles
    # ------------------------------------------------------------------

    def ajouter_robot(self, robot) -> None:
        self.robots.append(robot)   # ✅ Ajoute à la liste

    def ajouter_obstacle(self, obstacle) -> None:
        self.obstacles.append(obstacle)

    # ------------------------------------------------------------------
    # Grille de navigation (pour A*)
    # ------------------------------------------------------------------

    def _monde_vers_grille(self, x: float, y: float) -> tuple[int, int]:
        """Convertit des coordonnées monde (m) en indices de grille."""
        gx = round((x + self.largeur / 2) / self._resolution)
        gy = round((y + self.hauteur / 2) / self._resolution)
        return gx, gy

    def _grille_vers_monde(self, gx: int, gy: int) -> tuple[float, float]:
        """Convertit des indices de grille en coordonnées monde (m)."""
        x = gx * self._resolution - self.largeur / 2
        y = gy * self._resolution - self.hauteur / 2
        return x, y

    def _mettre_a_jour_grille(self) -> None:
        """Recalcule les cellules bloquées par les obstacles."""
        self._grille_bloquee = set()
        cols = round(self.largeur / self._resolution)
        rows = round(self.hauteur / self._resolution)

        for gx in range(cols + 1):
            for gy in range(rows + 1):
                x, y = self._grille_vers_monde(gx, gy)
                for obs in self.obstacles:
                    # On grossit légèrement l'obstacle (marge de 0.4 m)
                    class _FakeRobot:
                        pass
                    fake = _FakeRobot()
                    fake.x, fake.y, fake.rayon = x, y, 0.4
                    if obs.collision(fake):
                        self._grille_bloquee.add((gx, gy))
                        break

    def _cellule_hors_limites(self, gx: int, gy: int) -> bool:
        cols = round(self.largeur / self._resolution)
        rows = round(self.hauteur / self._resolution)
        return not (0 <= gx <= cols and 0 <= gy <= rows)

    def _cellule_bloquee(self, gx: int, gy: int) -> bool:
        return self._cellule_hors_limites(gx, gy) or (gx, gy) in self._grille_bloquee
    
    # ------------------------------------------------------------------
    # Pathfinding — A*
    # ------------------------------------------------------------------

    def calculer_chemin(self,
                        depart: tuple[float, float],
                        arrivee: tuple[float, float]) -> list[tuple[float, float]] | None:
        """
        Calcule le chemin le plus court entre deux points (coordonnées monde)
        en évitant les obstacles via A*.

        Returns:
            Liste de points (x, y) en coordonnées monde, ou None si inaccessible.
        """
        g_depart  = self._monde_vers_grille(*depart)
        g_arrivee = self._monde_vers_grille(*arrivee)

        if self._cellule_bloquee(*g_arrivee):
            return None

        def h(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        open_set = []
        heapq.heappush(open_set, (0, g_depart))
        came_from: dict = {}
        g_score: dict = {g_depart: 0}

        while open_set:
            _, courant = heapq.heappop(open_set)

            if courant == g_arrivee:
                # Reconstruction du chemin en coordonnées monde
                chemin = []
                while courant in came_from:
                    chemin.append(self._grille_vers_monde(*courant))
                    courant = came_from[courant]
                chemin.append(self._grille_vers_monde(*courant))
                chemin.reverse()
                return chemin

            gx, gy = courant
            for voisin in [(gx+1,gy),(gx-1,gy),(gx,gy+1),(gx,gy-1)]:
                if self._cellule_bloquee(*voisin):
                    continue
                tentative_g = g_score[courant] + 1
                if tentative_g < g_score.get(voisin, float('inf')):
                    came_from[voisin] = courant
                    g_score[voisin] = tentative_g
                    f = tentative_g + h(voisin, g_arrivee)
                    heapq.heappush(open_set, (f, voisin))

        return None  # Pas de chemin

    # ------------------------------------------------------------------
    # Mise à jour
    # ------------------------------------------------------------------

    def mise_a_jour(self, dt) -> None:
        """
        Met à jour tous les robots.
        - Si le robot est en mode autonome (etat != EN_ATTENTE via reset()),
          il navigue seul vers le paquet puis le dépôt.
        - Sinon (contrôle manuel), comportement original avec détection collision.
        """
        from project.robot.modele.robot_mobile import EtatRobot  
        for robot in self.robots:
            if robot.etat != EtatRobot.EN_ATTENTE:
                #  Mode autonome 
                robot.effectuer_pas_autonome(self, dt)
            else:
                #  Mode manuel (comportement original)
                etat_sauvegarde = robot.get_etat()
                robot.mettre_a_jour(dt)
                if self.collision_limites(robot) or self.collision_obstacles(robot):
                    robot.set_etat(etat_sauvegarde)

    # ------------------------------------------------------------------
    # Simulation complète d'un robot
    # ------------------------------------------------------------------

    def simuler_robot(self, robot,
                      position_depart: tuple[float, float] | None = None,
                      max_pas: int = 20_000) -> dict:
        """
        Lance une simulation complète pour un robot autonome.

        Args:
            robot           : RobotMobile configuré avec ses gènes
            position_depart : (x, y) de départ, défaut = position_depot
            max_pas         : garde-fou contre les boucles infinies

        Returns:
            dict métriques : succes, pas_effectues, energie_consommee, cout, etat_final
        """
        from project.robot.modele.robot_mobile import EtatRobot

        depart = position_depart or self.position_depot
        robot.reset(*depart)
        dt = 0.1  # pas de temps fixe pour la simulation headless

        for _ in range(max_pas):
            continuer = robot.effectuer_pas_autonome(self, dt)
            if not continuer:
                break

        return robot.metriques()

    # ------------------------------------------------------------------
    # Détection de collision 
    # ------------------------------------------------------------------

    def collision_limites(self, robot) -> bool:
        x, y = robot.x, robot.y
        return (
            x - robot.rayon < -self.largeur / 2 or
            x + robot.rayon >  self.largeur / 2 or
            y - robot.rayon < -self.hauteur / 2 or
            y + robot.rayon >  self.hauteur / 2
        )

    def collision_obstacles(self, robot) -> bool:
        for obstacle in self.obstacles:
            if obstacle.collision(robot):
                return True
        return False
