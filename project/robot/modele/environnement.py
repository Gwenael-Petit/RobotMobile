class Environnement:
    def __init__(self, largeur=15, hauteur=15):
        self.largeur = largeur      # en mètres
        self.hauteur = hauteur      # en mètres
        self.robots = []            # ✅ Liste au lieu d'un seul robot
        self.obstacles = []

    def ajouter_robot(self, robot) -> None:
        self.robots.append(robot)   # ✅ Ajoute à la liste

    def ajouter_obstacle(self, obstacle) -> None:
        self.obstacles.append(obstacle)

    def collision_limites(self, robot) -> bool:  # ✅ Prend le robot en paramètre
        """
        Vérifie si un robot sort du monde.
        """
        x, y = robot.x, robot.y

        return (
            x - robot.rayon < -self.largeur / 2 or
            x + robot.rayon >  self.largeur / 2 or
            y - robot.rayon < -self.hauteur / 2 or
            y + robot.rayon >  self.hauteur / 2
        )

    def collision_obstacles(self, robot) -> bool:  # ✅ Prend le robot en paramètre
        """
        Teste la collision d'un robot avec tous les obstacles.
        """
        for obstacle in self.obstacles:
            if obstacle.collision(robot):
                return True
        return False

    def mise_a_jour(self, dt) -> None:
        """
        Met à jour tous les robots avec détection de collision.
        1. Sauvegarde position
        2. Robot calcule mouvement
        3. Vérifie collision
        4. Annule si nécessaire
        """
        for robot in self.robots:  # ✅ Itère sur tous les robots
            # Sauvegarde
            etat_sauvegarde = robot.get_etat()  # ✅ Utilise get_etat si disponible
            
            # Mise à jour cinématique
            robot.mettre_a_jour(dt)
            
            # Vérification collision
            if self.collision_limites(robot) or self.collision_obstacles(robot):
                # Annulation
                robot.set_etat(etat_sauvegarde)  # ✅ Utilise set_etat si disponible
                # Ou si pas encore implémenté :
                # robot.x, robot.y, robot.orientation = etat_sauvegarde
