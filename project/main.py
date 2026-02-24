import math
import sys
from project.robot.modele.environnement import Environnement
from project.robot.modele.obstacle import Obstacle, ObstacleCirculaire, ObstacleRectangulaire
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
    # Créer le modèle
    env = Environnement(largeur=15, hauteur=15)
    
    moteur = MoteurDifferentiel()
    robot = RobotMobile(moteur=moteur, rayon=0.5)
    robot.x = 0
    robot.y = 0
    env.ajouter_robot(robot)
    
    env.ajouter_obstacle(ObstacleCirculaire(3, 3, 1.0))
    env.ajouter_obstacle(ObstacleCirculaire(-2, 4, 0.5))
    env.ajouter_obstacle(ObstacleRectangulaire(-4, -2, 2, 3))
    env.ajouter_obstacle(ObstacleRectangulaire(0, -4, 5, 1))

    
    # Créer la vue
    vue = VuePygame(largeur=800, hauteur=800, scale=50)
    
    # Créer le contrôleur
    controleur = ControleurClavierPygame()
    
    # Boucle principale
    running = True
    dt = 0.016  # ~60 FPS
    
    while running:
        # Gestion des événements
        running = vue.gerer_evenements()
        
        # Lecture des commandes
        commandes = controleur.lire_commande()
        
        # Application des commandes à tous les robots
        for robot in env.robots:
            robot.commander(**commandes)
        
        # Mise à jour de la physique (modèle)
        env.mise_a_jour(dt)
        
        # Affichage (vue)
        vue.dessiner(env)  # ✅ C'est ici le changement
        
        # Limiter le framerate
        vue.limiter_fps(60)
    
    # Fermeture
    vue.fermer()


if __name__ == "__main__":
    main()

