import pygame
from project.robot.modele.environnement import Environnement
from project.robot.modele.obstacle import ObstacleCirculaire, ObstacleRectangulaire
from project.robot.modele.robot_mobile import RobotMobile, EtatRobot
from project.robot.modele.moteur import MoteurDifferentiel
from project.robot.controleur.controleur import ControleurClavierPygame
from project.robot.vue.vue import VuePygame
from project.robot.modele.grille_occupation import GrilleOccupation
from project.robot.controleur.planificateur_a_star import PlanificateurAStar
from project.robot.controleur.controleur_pid import ControleurPID

def main_autonome():
    #  Environnement
    env = Environnement(
        largeur=15, hauteur=15,
        position_paquet=(5.0, 5.0),
        position_depot=(-5.0, -5.0),
    )

    env.ajouter_obstacle(ObstacleCirculaire(3, 3, 1.0))
    env.ajouter_obstacle(ObstacleCirculaire(-2, 4, 0.5))
    env.ajouter_obstacle(ObstacleRectangulaire(-4, -2, 2, 3))
    env.ajouter_obstacle(ObstacleRectangulaire(0, -4, 5, 1))

    #  Grille + Planificateur 
    grille        = GrilleOccupation.construct(env, resolution=0.25)
    planificateur = PlanificateurAStar(grille)

    #  Robot 
    robot = RobotMobile(
        moteur=MoteurDifferentiel(),
        rayon=0.5,
        vitesse_max=3.0,
        capacite_charge=10.0,
        autonomie=8000.0,
    )
    env.ajouter_robot(robot)
    robot.reset(*env.position_depot)

    #  Attache le PID et le planificateur au robot 
    pid = ControleurPID(v_max=robot.vitesse_max)
    robot._pid           = pid
    robot._planificateur = planificateur

    # Premier chemin vers le paquet
    chemin = planificateur.trouver_chemin(env.position_depot, env.position_paquet)
    pid.set_chemin(chemin)

    #  Vue 
    vue = VuePygame(largeur=800, hauteur=800, scale=50)

    running = True
    dt = 0.05

    while running:
        running = vue.gerer_evenements()
        env.mise_a_jour_autonome(dt)   # ← ici le changement
        vue.dessiner(env)
        vue.limiter_fps(60)

        if robot.etat in (EtatRobot.LIVRE, EtatRobot.EN_PANNE):
            print("\n── Simulation terminée ──")
            m = robot.metriques()
            print(f"  Succès         : {m['succes']}")
            print(f"  Distance       : {m['distance_parcourue']:.1f} m")
            print(f"  Énergie conso  : {m['energie_consommee']:.1f} J")
            print(f"  Coût final     : {m['cout']:.2f}")
            pygame.time.wait(3000)
            running = False

    vue.fermer()

def main_manuel():
    """
    Mode manuel (contrôle clavier), comportement original conservé.
    """
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

    vue = VuePygame(largeur=800, hauteur=800, scale=50)
    controleur = ControleurClavierPygame()

    running = True
    dt = 0.016

    while running:
        running = vue.gerer_evenements()
        commandes = controleur.lire_commande()
        for r in env.robots:
            r.commander(**commandes)
        env.mise_a_jour(dt)
        vue.dessiner(env)
        vue.limiter_fps(60)

    vue.fermer()


if __name__ == "__main__":
    # Change ici pour basculer entre les deux modes
    main_autonome()
    # main_manuel()