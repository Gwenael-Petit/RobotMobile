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
            ( 5.0,  3.0),   # devant rack droite
            (-5.0,  3.0),   # devant rack gauche
            ( 0.0,  2.0),   # devant rack centre
            ( 5.0, -0.5),   # couloir droite
            (-2.5, 5.5),   # couloir gauche
        ],
        position_depot=(-5.0, -5.5),
    )
    # Racks — bord inférieur à y=4.0, colis à y=2.0 → 1.5m de marge
    env.ajouter_obstacle(ObstacleRectangulaire( 5.0,  5.5, 1.5, 3.0))
    env.ajouter_obstacle(ObstacleRectangulaire(-5.0,  5.5, 1.5, 3.0))
    env.ajouter_obstacle(ObstacleRectangulaire( 0.0,  5.5, 1.5, 3.0))
    env.ajouter_obstacle(ObstacleRectangulaire( -5.0, -1.0, 1.0, 3.0))

    # Piliers
    env.ajouter_obstacle(ObstacleCirculaire( 2.5,  0.0, 0.4))
    env.ajouter_obstacle(ObstacleCirculaire(-2.5,  0.0, 0.4))
    env.ajouter_obstacle(ObstacleCirculaire( 2.5, -4.0, 0.4))

    # Séparateur central
    env.ajouter_obstacle(ObstacleRectangulaire(0.0, -1.5, 0.5, 4.0))
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


    vue     = VuePygame(largeur=1200, hauteur=820, scale=50)
    running = True
    dt      = 0.05
    pret = False
    while not pret:
        running = vue.gerer_evenements()
        if not running:
            vue.fermer()
            return

        # Affiche message d'attente
        vue.dessiner(env)
        msg = vue.font_md.render("Appuyez sur ESPACE pour lancer", True, (220, 220, 100))
        vue.screen.blit(msg, (400 - msg.get_width() // 2, vue.hauteur // 2))
        pygame.display.flip()
        vue.limiter_fps(60)

        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                pret = True
    while running:
        running = vue.gerer_evenements()

        for robot in robots:
            if robot.etat not in (EtatRobot.LIVRE, EtatRobot.EN_PANNE):
                env.mise_a_jour_autonome_robot(robot, dt)

        vue.dessiner(env)
        vue.limiter_fps(60)

        if all(r.etat in (EtatRobot.LIVRE, EtatRobot.EN_PANNE) for r in robots):
            running = vue.gerer_evenements()

    vue.fermer()


import pickle
import os

SAUVEGARDE = "population.pkl"

def sauvegarder_population(ag: AlgorithmeGenetique) -> None:
    with open(SAUVEGARDE, "wb") as f:
        pickle.dump({
            "population":        ag.population,
            "meilleur_individu": ag.meilleur_individu,
        }, f)
    print(f"Population sauvegardée dans {SAUVEGARDE}")

def charger_population() -> dict | None:
    if not os.path.exists(SAUVEGARDE):
        return None
    with open(SAUVEGARDE, "rb") as f:
        return pickle.load(f)
    

def main_genetique():
    env           = creer_environnement()
    grille        = GrilleOccupation.construct(env, resolution=0.25, marge=0.8)
    planificateur = PlanificateurAStar(grille)

    # ── Menu principal ───────────────────────────────────────────────
    sauvegarde = charger_population()

    if sauvegarde:
        print("\nSauvegarde détectée !")
        print("  [1] Relancer l'algorithme génétique")
        print("  [2] Rejouer la dernière visualisation")
        choix = input("Choix : ").strip()
    else:
        choix = "1"

    if choix == "1":
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
        ag.afficher_rapport()
        sauvegarder_population(ag)

        population_triee = sorted(ag.population, key=lambda ind: ind.fitness)
        n = len(population_triee)
        vus = {ag.meilleur_individu.capacite_charge}
        autres = []
        for ind in population_triee:
            if ind.capacite_charge not in vus:
                autres.append(ind)
                vus.add(ind.capacite_charge)
            if len(autres) == 3:
                break
        if len(autres) < 3:
            fallback = [population_triee[n // 4], population_triee[n // 2], population_triee[-1]]
            for ind in fallback:
                if len(autres) == 3:
                    break
                if ind not in autres:
                    autres.append(ind)

    else:
        meilleur = sauvegarde["meilleur_individu"]
        population_triee = sorted(sauvegarde["population"], key=lambda ind: ind.fitness)
        n = len(population_triee)
        vus = {meilleur.capacite_charge}
        autres = []
        for ind in population_triee:
            if ind.capacite_charge not in vus:
                autres.append(ind)
                vus.add(ind.capacite_charge)
            if len(autres) == 3:
                break
        if len(autres) < 3:
            fallback = [population_triee[n // 4], population_triee[n // 2], population_triee[-1]]
            for ind in fallback:
                if len(autres) == 3:
                    break
                if ind not in autres:
                    autres.append(ind)

    # ── Boucle de relecture ──────────────────────────────────────────
    while True:
        rejouer_robots(meilleur, autres, env, planificateur)
        print("\n  [1] Rejouer")
        print("  [2] Quitter")
        if input("Choix : ").strip() != "1":
            break

if __name__ == "__main__":
    main_genetique()