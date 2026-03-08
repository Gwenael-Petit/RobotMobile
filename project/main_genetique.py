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
        position_paquet=(5.0,  5.0),
        position_depot=(-5.0, -5.0),
    )
    env.ajouter_obstacle(ObstacleCirculaire(3, 3, 1.0))
    env.ajouter_obstacle(ObstacleCirculaire(-2, 4, 0.5))
    env.ajouter_obstacle(ObstacleRectangulaire(-4, -2, 2, 3))
    env.ajouter_obstacle(ObstacleRectangulaire(0, -4, 5, 1))
    return env


def rejouer_robot(meilleur, env, planificateur) -> None:
    """Rejoue visuellement le meilleur robot trouvé."""
    print("\nRejoue le meilleur robot...")

    robot = meilleur.to_robot()
    env.robots.clear()
    env.ajouter_robot(robot)
    robot.reset(*env.position_depot)

    pid = ControleurPID(v_max=robot.vitesse_max)
    robot._pid           = pid
    robot._planificateur = planificateur

    chemin = planificateur.trouver_chemin(env.position_depot, env.position_paquet)
    pid.set_chemin(chemin)

    vue     = VuePygame(largeur=800, hauteur=800, scale=50)
    running = True
    dt      = 0.05

    while running:
        running = vue.gerer_evenements()
        env.mise_a_jour_autonome(dt)
        vue.dessiner(env)
        vue.limiter_fps(60)

        if robot.etat in (EtatRobot.LIVRE, EtatRobot.EN_PANNE):
            m = robot.metriques()
            print(f"\n  Meilleur robot — résultats visuels :")
            print(f"  Succès   : {m['succes']}")
            print(f"  Distance : {m['distance_parcourue']:.2f} m")
            print(f"  Énergie  : {m['energie_consommee']:.1f} J")
            print(f"  Coût     : {m['cout']:.2f}")
            pygame.time.wait(3000)
            running = False

    vue.fermer()


def main_genetique():
    # ── Environnement & planificateur (construits une seule fois) ────────
    env           = creer_environnement()
    grille        = GrilleOccupation.construct(env,
                                                          resolution=0.25,
                                                          marge=0.8)
    planificateur = PlanificateurAStar(grille)

    # ── Algorithme génétique ─────────────────────────────────────────────
    ag = AlgorithmeGenetique(
        taille_population = 20,
        nb_generations    = 30,
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

    # ── Rapport ──────────────────────────────────────────────────────────
    ag.afficher_rapport()

    # ── Relecture visuelle du meilleur robot ─────────────────────────────
    rejouer_robot(meilleur, env, planificateur)


if __name__ == "__main__":
    main_genetique()