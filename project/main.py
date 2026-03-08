import pygame
from project.robot.modele.environnement import Environnement
from project.robot.modele.obstacle import ObstacleCirculaire, ObstacleRectangulaire
from project.robot.modele.robot_mobile import RobotMobile, EtatRobot
from project.robot.modele.moteur import MoteurDifferentiel
from project.robot.controleur.controleur import ControleurClavierPygame
from project.robot.vue.vue import VuePygame


def main_autonome():
    """
    Mode autonome : le robot navigue seul vers le paquet puis le dépôt.
    Utile pour valider la Phase 1 avant l'algorithme génétique.
    """
    # ── Environnement ────────────────────────────────────────────────────
    env = Environnement(
        largeur=15, hauteur=15,
        position_paquet=(5.0, 5.0),
        position_depot=(-5.0, -5.0),
    )

    env.ajouter_obstacle(ObstacleCirculaire(3, 3, 1.0))
    env.ajouter_obstacle(ObstacleCirculaire(-2, 4, 0.5))
    env.ajouter_obstacle(ObstacleRectangulaire(-4, -2, 2, 3))
    env.ajouter_obstacle(ObstacleRectangulaire(0, -4, 5, 1))

    # ── Robot avec ses gènes ─────────────────────────────────────────────
    robot = RobotMobile(
        moteur=MoteurDifferentiel(),
        rayon=0.5,
        vitesse_max=3.0,
        capacite_charge=10.0,
        autonomie=8000.0,
    )
    env.ajouter_robot(robot)

    # Démarre la mission depuis le dépôt
    robot.reset(*env.position_depot)

    # ── Vue ──────────────────────────────────────────────────────────────
    vue = VuePygame(largeur=800, hauteur=800, scale=50)

    # ── Boucle principale ────────────────────────────────────────────────
    running = True
    dt = 0.05   # pas de temps plus petit = navigation plus fluide

    while running:
        running = vue.gerer_evenements()
        env.mise_a_jour(dt)
        vue.dessiner(env)
        vue.limiter_fps(60)

        # Arrêt automatique quand la mission est terminée
        if robot.etat in (EtatRobot.LIVRE, EtatRobot.EN_PANNE):
            print("\n── Simulation terminée ──")
            m = robot.metriques()
            print(f"  Succès        : {m['succes']}")
            print(f"  Pas effectués : {m['pas_effectues']}")
            print(f"  Énergie conso : {m['energie_consommee']:.1f} J")
            print(f"  Coût final    : {m['cout']:.2f}")
            pygame.time.wait(3000)   # Affiche le résultat 3 secondes
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