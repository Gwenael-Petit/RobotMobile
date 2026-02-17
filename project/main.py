import math
from project.robot.modele.environnement import Environnement
from project.robot.modele.obstacle import Obstacle
from project.robot.modele.obstacle_circulaire import ObstacleCirculaire
from project.robot.modele.robot_mobile import RobotMobile
from project.robot.modele.moteur import *
from project.robot.controleur.controleur import *
from project.robot.vue.vue import *
import pygame

robot = RobotMobile(...)

"""
robot.display_robot()
robot.move_forward(1.0)
robot.display_robot()
robot.rotate(0.7854) # Rotate 45 degrees in radians = 0.7854
robot.display_robot()
robot.move_forward(3.0)
robot.display_robot()


moteur_diff = MoteurDifferentiel()
robot = RobotMobile(moteur=moteur_diff)
dt = 1.0 # pas de temps (s)
robot.afficher()
# On doit nommer les arguments (v = ..., omega = ...) car on utilise **kwargs !
robot.commander(v = 3.0, omega = math.pi/2) # avance en tournant
robot.mettre_a_jour(dt)
robot.afficher()
robot.commander(v = 1.0, omega = 0.0) # avance tout droit
robot.mettre_a_jour(dt)
robot.afficher()

moteur_omni = MoteurOmnidirectionnel()
robot = RobotMobile(moteur=moteur_omni)
robot.afficher()
robot.commander(vx = 3.0, vy = 1.0, omega = 0.0) # avance en tournant
robot.mettre_a_jour(dt)
robot.afficher()
print(robot)




robot = RobotMobile(moteur=MoteurDifferentiel())
controleur = ControleurClavierPygame()
vue = VuePygame()
dt = 1.0 / 60.0 # 1 second passe entre chaque mise a jour
running = True
while running:
    # 1️⃣ Gestion des événements (OBLIGATOIRE)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    vue.screen.fill((255, 255, 255))  # fond blanc
    
    commande = controleur.lire_commande()
    robot.commander(**commande)
    robot.mettre_a_jour(dt)
    
    vue.dessiner_robot(robot)
    pygame.display.flip()
    vue.tick(60)

pygame.quit()
"""



def main():
    # =========================
    # INITIALISATION
    # =========================
    pygame.init()

    vue = VuePygame(largeur=800, hauteur=600, scale=60)
    controleur = ControleurClavierPygame()

    moteur = MoteurDifferentiel()
    robot = RobotMobile(moteur=moteur)

    env = Environnement(largeur=10, hauteur=8)
    env.ajouter_robot(robot)

    # Obstacles
    env.ajouter_obstacle(ObstacleCirculaire(2, 1, 0.8))
    env.ajouter_obstacle(ObstacleCirculaire(-2, -1, 1.0))
    env.ajouter_obstacle(ObstacleCirculaire(0, -2, 0.6))

    running = True

    # =========================
    # BOUCLE PRINCIPALE
    # =========================
    while running:
        dt = vue.clock.tick(60) / 1000.0  # delta temps en secondes

        # --- Gestion des événements ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # --- Lecture commande clavier ---
        commande = controleur.lire_commande()
        robot.commander(**commande)

        # --- Mise à jour simulation ---
        env.mise_a_jour(dt)

        # --- Affichage ---
        vue.screen.fill((255, 255, 255))  # fond blanc
        env.dessiner(vue)
        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
