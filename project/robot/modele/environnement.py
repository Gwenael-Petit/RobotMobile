from math import hypot


class Environnement:
    def __init__(self, largeur=15, hauteur=15, position_paquet: tuple[float, float] = (6.0, 6.0),
                 position_depot: tuple[float, float] = (-6.0, -6.0)):
        self.largeur = largeur      # en mètres
        self.hauteur = hauteur      # en mètres
        self.robots = []          
        self.obstacles = []

        # Attributs pour la mission
        self.position_paquet = position_paquet   # (x, y) du paquet à récupérer
        self.position_depot  = position_depot

    # ------------------------------------------------------------------
    # Gestion robots & obstacles
    # ------------------------------------------------------------------

    def ajouter_robot(self, robot) -> None:
        self.robots.append(robot)   # ✅ Ajoute à la liste

    def ajouter_obstacle(self, obstacle) -> None:
        self.obstacles.append(obstacle)

    # ------------------------------------------------------------------
    # Mise à jour
    # ------------------------------------------------------------------

    def mise_a_jour(self, dt: float) -> None:
        """
        Met à jour tous les robots avec détection de collision.
        Utilisé en mode manuel (contrôle clavier).
        """
        for robot in self.robots:
            etat_sauvegarde = robot.get_etat()
            robot.mettre_a_jour(dt)
            if self.collision_limites(robot) or self.collision_obstacles(robot):
                robot.set_etat(etat_sauvegarde)

    # ------------------------------------------------------------------
    # Simulation autonome (appelée par Environnement en mode autonome)
    # ------------------------------------------------------------------

    def mise_a_jour_autonome(self, dt: float) -> None:
        """
        Met à jour tous les robots en mode autonome.
        Chaque robot doit avoir un ControleurPID attaché (_pid).
        Gère les transitions d'état VERS_PAQUET → CHARGE → LIVRE.
        """
        from .robot_mobile import EtatRobot

        for robot in self.robots:
            if not hasattr(robot, '_pid') or robot._pid is None:
                continue
            if robot.etat in (EtatRobot.LIVRE, EtatRobot.EN_PANNE):
                continue

            # Recharge en cours : pas de déplacement
            if robot.etat == EtatRobot.EN_RECHARGE:
                termine = robot.recharger(dt)
                if termine:
                    robot.etat = robot._etat_avant_recharge
                    robot._pid.set_chemin([])
                continue

            pid = robot._pid

            # Détection du besoin de recharge
            if (robot.besoin_recharge() and
                    robot.etat not in (EtatRobot.VERS_BASE, EtatRobot.EN_RECHARGE)):
                robot._etat_avant_recharge = robot.etat
                robot.etat = EtatRobot.VERS_BASE
                robot._pid.set_chemin([])

            if pid.est_arrive():
                self._recalculer_chemin(robot)

            commande = pid.calculer_commande(robot.x, robot.y, robot.orientation)
            robot.commander(**commande)

            etat_sauvegarde = robot.get_etat()
            robot.mettre_a_jour(dt)

            if self.collision_limites(robot) or self.collision_obstacles(robot):
                # Annule le mouvement
                robot.set_etat(etat_sauvegarde)
                robot.commander(v=0.0, omega=0.0)
                # Recalcule le chemin depuis la position actuelle
                self._recalculer_chemin(robot)

            self._verifier_transitions(robot)

    def _cible_courante(self, robot) -> tuple[float, float]:
        from .robot_mobile import EtatRobot
        if robot.etat == EtatRobot.VERS_PAQUET:
            return self.position_paquet
        return self.position_depot

    def _recalculer_chemin(self, robot) -> None:
        """Recalcule le chemin A* vers la cible courante du robot."""
        if not hasattr(robot, '_planificateur') or robot._planificateur is None:
            return
        cible  = self._cible_courante(robot)
        chemin = robot._planificateur.trouver_chemin((robot.x, robot.y), cible)
        robot._pid.set_chemin(chemin)

    def _verifier_transitions(self, robot) -> None:
        """Vérifie si le robot a atteint le paquet ou le dépôt."""
        from .robot_mobile import EtatRobot

        if robot.etat == EtatRobot.VERS_PAQUET:
            px, py = self.position_paquet
            if hypot(robot.x - px, robot.y - py) < robot.rayon + 0.4:
                robot.etat = EtatRobot.CHARGE
                robot._pid.set_chemin([])   # Force recalcul vers dépôt

        elif robot.etat == EtatRobot.CHARGE:
            dx, dy = self.position_depot
            if hypot(robot.x - dx, robot.y - dy) < robot.rayon + 0.4:
                robot.etat = EtatRobot.LIVRE
                robot.commander(v=0.0, omega=0.0)

        elif robot.etat == EtatRobot.VERS_BASE:
            dx, dy = self.position_depot
            if hypot(robot.x - dx, robot.y - dy) < robot.rayon + 0.4:
                robot.etat = EtatRobot.EN_RECHARGE
                robot.commander(v=0.0, omega=0.0)

    # ------------------------------------------------------------------
    # Simulation complète d'un robot
    # ------------------------------------------------------------------

    def simuler_robot(self, robot,
                      planificateur,
                      position_depart: tuple[float, float] | None = None,
                      dt: float = 0.05,
                      max_steps: int = 20_000) -> dict:
        """
        Lance une simulation complète pour un robot autonome.

        Args:
            robot           : RobotMobile avec ses gènes
            planificateur   : PlanificateurAStar pré-construit
            position_depart : (x, y) de départ, défaut = position_depot
            dt              : pas de temps fixe
            max_steps       : garde-fou contre les boucles infinies

        Returns:
            dict métriques : succes, pas_effectues, energie_consommee, cout, etat_final
        """
        from .robot_mobile import EtatRobot
        from project.robot.controleur.controleur_pid import ControleurPID

        depart = position_depart or self.position_depot
        robot.reset(*depart)

        self.robots.append(robot)

        # Attache planificateur et PID au robot
        pid = ControleurPID(v_max=robot.vitesse_max)
        robot._pid          = pid
        robot._planificateur = planificateur

        # Calcul du premier chemin
        chemin = planificateur.trouver_chemin(depart, self.position_paquet)
        pid.set_chemin(chemin)

        nb_steps = 0
        for _ in range(max_steps):
            if robot.etat in (EtatRobot.LIVRE, EtatRobot.EN_PANNE):
                break
            self.mise_a_jour_autonome(dt)
            nb_steps += 1

        print(f"  [DEBUG] steps={nb_steps}/{max_steps}, etat={robot.etat.name}, "
            f"distance={robot.distance_parcourue:.1f}, "
            f"cout={robot.calculer_cout():.1f}")

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
