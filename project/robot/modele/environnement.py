from math import hypot


class Environnement:
    def __init__(self, largeur=15, hauteur=15,
             positions_colis: list[tuple[float, float]] | None = None,
             position_depot: tuple[float, float] = (-6.0, -6.0)):
        self.largeur = largeur
        self.hauteur = hauteur
        self.robots = []
        self.obstacles = []

        # Attributs pour la mission
        self.position_depot = position_depot
        self.positions_colis = positions_colis or [
            ( 6.0,  6.0),
            (-4.0,  5.0),
            ( 5.0, -3.0),
            ( 0.0,  6.0),
            (-5.0,  2.0),
        ]
        # position_paquet pointe vers le prochain colis à récupérer
        self.position_paquet = self.positions_colis[0]

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

    def mise_a_jour_autonome_robot(self, robot, dt: float) -> None:
        """Met à jour un robot unique avec sa propre position de colis."""
        from .robot_mobile import EtatRobot

        if not hasattr(robot, '_pid') or robot._pid is None:
            return
        if robot.etat in (EtatRobot.LIVRE, EtatRobot.EN_PANNE):
            return

        if robot.etat == EtatRobot.EN_RECHARGE:
            termine = robot.recharger(dt)
            if termine:
                robot.etat = robot._etat_avant_recharge
                robot._pid.set_chemin([])
            return

        if (robot.besoin_recharge() and
                robot.etat not in (EtatRobot.VERS_BASE, EtatRobot.EN_RECHARGE)):
            robot._etat_avant_recharge = robot.etat
            robot.etat = EtatRobot.VERS_BASE
            robot._pid.set_chemin([])

        if robot._pid.est_arrive():
            self._recalculer_chemin_robot(robot)

        commande = robot._pid.calculer_commande(robot.x, robot.y, robot.orientation)
        robot.commander(**commande)

        etat_sauvegarde = robot.get_etat()
        robot.mettre_a_jour(dt)

        if self.collision_limites(robot) or self.collision_obstacles(robot):
            robot.set_etat(etat_sauvegarde)
            robot.commander(v=0.0, omega=0.0)
            self._recalculer_chemin_robot(robot)

        self._verifier_transitions_robot(robot)

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
                robot.colis_livres += robot.capacite_charge
                if robot.colis_livres >= robot.colis_a_livrer:
                    robot.etat = EtatRobot.LIVRE
                    robot.commander(v=0.0, omega=0.0)
                else:
                    # Prochain colis = index basé sur colis déjà récupérés
                    index = min(robot.colis_livres, len(self.positions_colis) - 1)
                    self.position_paquet = self.positions_colis[index]
                    robot.etat = EtatRobot.VERS_PAQUET
                    robot._pid.set_chemin([])

        elif robot.etat == EtatRobot.VERS_BASE:
            dx, dy = self.position_depot
            if hypot(robot.x - dx, robot.y - dy) < robot.rayon + 0.4:
                robot.etat = EtatRobot.EN_RECHARGE
                robot.commander(v=0.0, omega=0.0)

    def _recalculer_chemin_robot(self, robot) -> None:
        """Recalcule vers la cible propre au robot."""
        if not hasattr(robot, '_planificateur') or robot._planificateur is None:
            return
        cible  = self._cible_courante_robot(robot)
        chemin = robot._planificateur.trouver_chemin((robot.x, robot.y), cible)
        robot._pid.set_chemin(chemin)

    def _cible_courante_robot(self, robot) -> tuple[float, float]:
        """Cible selon l'état du robot et son index de colis."""
        from .robot_mobile import EtatRobot
        if robot.etat == EtatRobot.VERS_PAQUET:
            index = min(robot.index_colis_courant, len(self.positions_colis) - 1)
            return self.positions_colis[index]
        return self.position_depot

    def _verifier_transitions_robot(self, robot) -> None:
        """Transitions d'état pour un robot individuel."""
        from .robot_mobile import EtatRobot

        if robot.etat == EtatRobot.VERS_PAQUET:
            px, py = self.positions_colis[robot.index_colis_courant]
            if hypot(robot.x - px, robot.y - py) < robot.rayon + 0.4:
                robot.colis_en_cours += 1
                robot.index_colis_courant += 1

                if (robot.colis_en_cours >= robot.capacite_charge or
                        robot.index_colis_courant >= robot.colis_a_livrer):
                    # Chargé à max ou plus de colis à ramasser → retour base
                    robot.etat = EtatRobot.CHARGE
                    robot._pid.set_chemin([])
                else:
                    # Encore de la place → prochain colis directement
                    robot.etat = EtatRobot.VERS_PAQUET
                    self._recalculer_chemin_robot(robot)  # recalcul immédiat

        elif robot.etat == EtatRobot.CHARGE:
            dx, dy = self.position_depot
            if hypot(robot.x - dx, robot.y - dy) < robot.rayon + 0.4:
                robot.colis_livres += robot.colis_en_cours
                robot.colis_en_cours = 0
                if robot.colis_livres >= robot.colis_a_livrer:
                    robot.etat = EtatRobot.LIVRE
                    robot.commander(v=0.0, omega=0.0)
                else:
                    robot.etat = EtatRobot.VERS_PAQUET
                    self._recalculer_chemin_robot(robot)  # recalcul immédiat

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
                      max_steps: int = 3000) -> dict:
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
        self.position_paquet = self.positions_colis[0]

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
