"""
Phase 3 — Lancement de l'algorithme génétique.
Lance l'optimisation, affiche la convergence et rejoue le meilleur robot.
"""

from matplotlib import pyplot as plt
import numpy as np
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
    dt      = 0.03
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
            "historique_individus": ag.historique_individus,
            "historique_meilleur":  ag.historique_meilleur,
            "historique_moyen":     ag.historique_moyen,
        }, f)
    print(f"Population sauvegardée dans {SAUVEGARDE}")

def charger_population() -> dict | None:
    if not os.path.exists(SAUVEGARDE):
        return None
    with open(SAUVEGARDE, "rb") as f:
        return pickle.load(f)
    

def afficher_region_faisable_2d(ag: AlgorithmeGenetique, env, planificateur) -> None:
    print("Calcul région faisable 2D...")

    autonomie_fixe = ag.meilleur_individu.autonomie
    vitesses = np.linspace(1.0, 10.0, 30)
    charges  = np.arange(1, 6)

    prix_ok  = np.zeros((len(charges), len(vitesses)), dtype=bool)

    for i, c in enumerate(charges):
        for j, v in enumerate(vitesses):
            prix = 3.0 * v + 5.0 * int(c) + 0.001 * autonomie_fixe
            prix_ok[i, j] = prix <= 50.0

    # Figure
    fig, ax = plt.subplots(figsize=(10, 6))
    fig.patch.set_facecolor('#1c1c20')
    ax.set_facecolor('#1c1c20')
    ax.set_xlim(0.5, 10.5)
    ax.set_ylim(0.5, 5.5)

    # Zone faisable — bleu comme ton image de référence
    ax.contourf(vitesses, charges, prix_ok.astype(float),
                levels=[0.5, 1.5], colors=['#1a3a5c'], alpha=0.8)

    # Frontière budget
    ax.contour(vitesses, charges, prix_ok.astype(float),
               levels=[0.5], colors=['#4a9eff'], linewidths=2.0)

    # Individus explorés — colorés par temps
    faisables = [ind for ind in ag.historique_individus if ind.fitness < 100_000]
    if faisables:
        vx = [ind.vitesse_max     for ind in faisables]
        vy = [ind.capacite_charge for ind in faisables]
        vc = [ind.fitness         for ind in faisables]
        sc = ax.scatter(vx, vy, c=vc, cmap='RdYlGn_r',
                        s=60, alpha=0.8, zorder=5,
                        edgecolors='white', linewidths=0.3)
        cbar = plt.colorbar(sc, ax=ax)
        cbar.set_label('Temps de mission (s)', color='#aaaaaa')
        plt.setp(cbar.ax.yaxis.get_ticklabels(), color='#aaaaaa')
        cbar.ax.yaxis.set_tick_params(color='#aaaaaa')

    # Solution optimale
    ax.scatter([ag.meilleur_individu.vitesse_max],
               [ag.meilleur_individu.capacite_charge],
               color='#ffb400', s=300, marker='*',
               zorder=10,
               label=f"Optimal — {ag.meilleur_individu.fitness:.1f}s")

    # Style 
    from matplotlib.patches import Patch
    legendes = [
        Patch(facecolor='#1a3a5c', edgecolor='#4a9eff', label='Région faisable (prix ≤ 50)'),
        Patch(facecolor='#1c1c20', edgecolor='#555566', label='Hors budget'),
    ]
    ax.legend(handles=legendes + [
        plt.scatter([], [], color='#ffb400', marker='*', s=150, label='Solution optimale'),
        plt.scatter([], [], color='grey', s=40, label='Individus explorés'),
    ], facecolor='#2a2a30', edgecolor='#444455',
       labelcolor='#cccccc', fontsize=9)

    ax.set_xlabel('Vitesse max (m/s)', color='#aaaaaa', fontsize=11)
    ax.set_ylabel('Capacité de charge (kg)', color='#aaaaaa', fontsize=11)
    ax.set_title(f'Région faisable et solution optimale',
                 color='#dddddd', fontsize=13)
    ax.set_yticks(charges)
    ax.tick_params(colors='#aaaaaa')
    for spine in ax.spines.values():
        spine.set_color('#444455')
    ax.grid(True, color='#2d2d35', linewidth=0.5, alpha=0.5)

    plt.tight_layout()
    plt.savefig('region_faisable_2d.png', dpi=150,
                facecolor='#1c1c20', bbox_inches='tight')
    print("Figure sauvegardée : region_faisable_2d.png")
    plt.show()

def main_genetique():
    env           = creer_environnement()
    grille        = GrilleOccupation.construct(env, resolution=0.25, marge=0.8)
    planificateur = PlanificateurAStar(grille)

    #  Menu principal 
    sauvegarde = charger_population()

    if sauvegarde:
        print("\nSauvegarde détectée !")
        print("  [1] Relancer l'algorithme génétique")
        print("  [2] Rejouer la dernière visualisation")
        print("  [3] Afficher la région faisable 2D")
        choix = input("Choix : ").strip()
    else:
        choix = "1"

    if choix == "1":
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
        ag.afficher_rapport()
        afficher_region_faisable_2d(ag, env, planificateur)
        sauvegarder_population(ag)

        population_triee = sorted(
            [ind for ind in ag.historique_individus if ind.fitness < 100_000 and ind.fitness != meilleur.fitness],
            key=lambda ind: ind.fitness
        )
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
    elif choix == "3":
            meilleur = sauvegarde["meilleur_individu"]
            ag = AlgorithmeGenetique()   # ← crée un ag vide
            ag.meilleur_individu    = sauvegarde["meilleur_individu"]
            ag.population           = sauvegarde["population"]
            ag.historique_individus = sauvegarde.get("historique_individus", sauvegarde["population"])
            ag.historique_meilleur  = sauvegarde.get("historique_meilleur", [])
            ag.historique_moyen     = sauvegarde.get("historique_moyen", [])
            afficher_region_faisable_2d(ag, env, planificateur)

    else:
        meilleur = sauvegarde["meilleur_individu"]
        historique       = sauvegarde.get("historique_individus",
                           sauvegarde["population"])
        population_triee = sorted(
            [ind for ind in historique if ind.fitness < 100_000 and ind.fitness != meilleur.fitness],
            key=lambda ind: ind.fitness
        )
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

    #  Boucle de relecture
    while True:
        rejouer_robots(meilleur, autres, env, planificateur)
        print("\n  [1] Rejouer")
        print("  [2] Quitter")
        if input("Choix : ").strip() != "1":
            break

if __name__ == "__main__":
    main_genetique()