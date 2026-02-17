class Environnement:
    def __init__(self, largeur=15, hauteur=15):
        self.largeur = largeur      # en mètres
        self.hauteur = hauteur      # en mètres
        self.robot = None
        self.obstacles = []
        self.rayon_robot = 0.3      # rayon robot en mètres

    def ajouter_robot(self, robot):
        self.robot = robot

    def ajouter_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def collision_limites(self):
        """
        Vérifie si le robot sort du monde.
        """
        if self.robot is None:
            return False

        x, y = self.robot.x, self.robot.y

        return (
            x - self.rayon_robot < -self.largeur / 2 or
            x + self.rayon_robot >  self.largeur / 2 or
            y - self.rayon_robot < -self.hauteur / 2 or
            y + self.rayon_robot >  self.hauteur / 2
        )

    def collision_obstacles(self):
        """
        Teste la collision avec tous les obstacles (polymorphisme).
        """
        for obstacle in self.obstacles:
            if obstacle.collision(self.robot.x, self.robot.y, self.rayon_robot):
                return True
        return False

    def mise_a_jour(self, dt):
        """
        1. Sauvegarde position
        2. Robot calcule mouvement
        3. Vérifie collision
        4. Annule si nécessaire
        """
        if self.robot is None:
            return

        # 1️⃣ sauvegarde
        ancienne_x = self.robot.x
        ancienne_y = self.robot.y
        ancienne_orientation = self.robot.orientation

        # 2️⃣ mise à jour cinématique
        self.robot.mettre_a_jour(dt)

        # 3️⃣ vérification collision
        if self.collision_limites() or self.collision_obstacles():
            # 4️⃣ annulation
            self.robot.x = ancienne_x
            self.robot.y = ancienne_y
            self.robot.orientation = ancienne_orientation

    def dessiner(self, vue):
        """
        Dessine obstacles + robot
        """
        for obstacle in self.obstacles:
            obstacle.dessiner(vue)

        if self.robot is not None:
            vue.dessiner_robot(self.robot)
