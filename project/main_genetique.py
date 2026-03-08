"""
Phase 3 — Lancement de l'algorithme génétique.
Lance l'optimisation, affiche la convergence et rejoue le meilleur robot.
"""

import pygame
from project.robot.modele.environnement import Environnement
from project.robot.modele.obstacle import ObstacleCirculaire, ObstacleRectangulaire
from project.robot.modele.grille_occupation import GrilleOccupation
from project.robot.modele.robot_mobile import EtatRobot
from project.robot.modele.moteur import MoteurDifferentiel
from project.robot.controleur.planificateur_a_star import PlanificateurAStar
from project.robot.controleur.controleur_pid import ControleurPID
from project.robot.controleur.algo_genetique import AlgorithmeGenetique
from project.robot.vue.vue import VuePygame


def creer_environnement() -> Environnement:
    """Environnement partagé par toutes les simulations."""
    env = Environnement(
        largeur=15, hauteur=15,
        positions_colis=[
            ( 6.0,  6.0),
            (-4.0,  5.0),
            ( 5.0, -3.0),
            ( 0.0,  6.0),
            (-5.0,  2.0),
        ],
        position_depot=(-5.0, -5.0),
    )
    env.ajouter_obstacle(ObstacleCirculaire(3, 3, 1.0))
    env.ajouter_obstacle(ObstacleCirculaire(-2, 4, 0.5))
    env.ajouter_obstacle(ObstacleRectangulaire(-4, -2, 2, 3))
    env.ajouter_obstacle(ObstacleRectangulaire(0, -4, 5, 1))
    return env


def rejouer_robots(meilleur, autres, env, planificateur) -> None:
    """Rejoue visuellement le meilleur robot + 3 autres en simultané."""
    print("\nRejoue les 4 robots...")

    # Couleurs distinctes pour chaque robot
    couleurs = [
        (0,   200, 100),   # vert   — meilleur
        (80,  160, 255),   # bleu
        (255, 180,   0),   # orange
        (220,  50,  50),   # rouge
    ]

    individus = [meilleur] + autres[:3]
    robots    = []

    env.robots.clear()
    env.position_paquet = env.positions_colis[0]

    for i, individu in enumerate(individus):
        robot             = individu.to_robot()
        robot.couleur     = couleurs[i]  # attribut custom pour la vue
        robot.label       = f"{'MEILLEUR' if i == 0 else f'Robot {i+1}'}"
        pid               = ControleurPID(v_max=robot.vitesse_max)
        robot._pid        = pid
        robot._planificateur = planificateur
        robot.reset(*env.position_depot)
        chemin = planificateur.trouver_chemin(env.position_depot, env.positions_colis[0])
        pid.set_chemin(chemin)
        env.ajouter_robot(robot)
        robots.append(robot)

    print(f"[DEBUG] Nombre de robots : {len(env.robots)}")

    vue     = VuePygame(largeur=800, hauteur=800, scale=50)
    running = True
    dt      = 0.05

    while running:
        running = vue.gerer_evenements()

        for robot in robots:
            if robot.etat not in (EtatRobot.LIVRE, EtatRobot.EN_PANNE):
                env.mise_a_jour_autonome_robot(robot, dt)

        vue.dessiner(env)
        vue.limiter_fps(60)

        if all(r.etat in (EtatRobot.LIVRE, EtatRobot.EN_PANNE) for r in robots):
            pygame.time.wait(10000)
            running = False

    vue.fermer()


def main_genetique():
    #  Environnement & planificateur 
    env           = creer_environnement()
    grille        = GrilleOccupation.construct(env,
                                                          resolution=0.25,
                                                          marge=0.8)
    planificateur = PlanificateurAStar(grille)

    #  Algorithme génétique 
    ag = AlgorithmeGenetique(
        taille_population = 15,
        nb_generations    = 20,
        taux_mutation     = 0.2,
        taux_croisement   = 0.8,
        taille_tournoi    = 3,
        elitisme          = 2,
    )

    print("╔══════════════════════════════════════════╗")
    print("║     OPTIMISATION — ALGORITHME GÉNÉTIQUE  ║")
    print("╚══════════════════════════════════════════╝")
    print(f"Population : {ag.taille_population} individus")
    print(f"Générations: {ag.nb_generations}")
    print(f"Mutation   : {ag.taux_mutation * 100:.0f}%")
    print()

    meilleur = ag.evoluer(env, planificateur)

    #  Rapport 
    ag.afficher_rapport()

    #  Relecture visuelle du meilleur robot 
    population_triee = sorted(ag.population, key=lambda ind: ind.fitness)
    n = len(population_triee)
    
    # Cherche un individu pour chaque capacite_charge différente (1, 2, 3, 4, 5)
    vus = {ag.meilleur_individu.capacite_charge}
    autres = []
    for ind in population_triee:
        if ind.capacite_charge not in vus:
            autres.append(ind)
            vus.add(ind.capacite_charge)
        if len(autres) == 3:
            break
    
    # Si pas assez de diversité, complète avec n//4, n//2, -1
    if len(autres) < 3:
        fallback = [population_triee[n // 4], population_triee[n // 2], population_triee[-10]]
        for ind in fallback:
            if len(autres) == 3:
                break
            if ind not in autres:
                autres.append(ind)
    rejouer_robots(meilleur, autres, env, planificateur)

    while True:
        print("\nQue voulez-vous faire ?")
        print("  [1] Rejouer la visualisation")
        print("  [2] Quitter")
        choix = input("Choix : ").strip()

        if choix == "1":
            rejouer_robots(meilleur, autres, env, planificateur)
        elif choix == "2":
            break
        else:
            print("Choix invalide.")

if __name__ == "__main__":
    main_genetique()