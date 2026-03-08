"""
Algorithme Génétique
Optimise les 3 gènes du RobotMobile :
    - vitesse_max      [1.0 – 10.0]  m/s
    - capacite_charge  [1.0 – 50.0]  kg
    - autonomie        [500 – 10000] J

Minimise la fonction de coût de RobotMobile.calculer_cout().
"""

import random
import copy
from dataclasses import dataclass, field

from project.robot.modele.robot_mobile import RobotMobile
from project.robot.modele.moteur import MoteurDifferentiel


# ------------------------------------------------------------------
# Représentation d'un individu
# ------------------------------------------------------------------

@dataclass
class Individu:
    """
    Un individu = un robot avec ses 3 gènes + son score de fitness.
    fitness = coût (à minimiser) → plus petit = meilleur.
    """
    vitesse_max:      float
    capacite_charge:  float
    autonomie:        float
    fitness:          float = float('inf')   # non évalué par défaut

    def to_robot(self) -> RobotMobile:
        """Crée un RobotMobile à partir des gènes de cet individu."""
        return RobotMobile(
            moteur=MoteurDifferentiel(),
            rayon=0.5,
            vitesse_max=self.vitesse_max,
            capacite_charge=self.capacite_charge,
            autonomie=self.autonomie,
        )

    def __repr__(self) -> str:
        return (f"Individu(v={self.vitesse_max:.2f}, "
                f"charge={self.capacite_charge:.2f}, "
                f"auto={self.autonomie:.0f}, "
                f"fitness={self.fitness:.2f})")


# ------------------------------------------------------------------
# Bornes des gènes
# ------------------------------------------------------------------

BORNES = {
    "vitesse_max":     (1.0,   10.0),
    "capacite_charge": (1.0,   50.0),
    "autonomie":       (500.0, 10000.0),
}


# ------------------------------------------------------------------
# Algorithme Génétique
# ------------------------------------------------------------------

class AlgorithmeGenetique:
    """
    Algorithme génétique pour optimiser les paramètres d'un robot mobile.

    Paramètres :
        taille_population (int)   : nombre d'individus par génération
        nb_generations    (int)   : nombre de générations
        taux_mutation     (float) : probabilité de muter un gène [0-1]
        taux_croisement   (float) : probabilité de croisement [0-1]
        taille_tournoi    (int)   : nombre de candidats pour la sélection
        elitisme          (int)   : nombre des meilleurs à conserver
    """

    def __init__(self,
                 taille_population: int   = 20,
                 nb_generations:    int   = 30,
                 taux_mutation:     float = 0.2,
                 taux_croisement:   float = 0.8,
                 taille_tournoi:    int   = 3,
                 elitisme:          int   = 2):

        self.taille_population = taille_population
        self.nb_generations    = nb_generations
        self.taux_mutation     = taux_mutation
        self.taux_croisement   = taux_croisement
        self.taille_tournoi    = taille_tournoi
        self.elitisme          = elitisme

        # Résultats
        self.historique_meilleur: list[float] = []   # meilleur fitness par génération
        self.historique_moyen:    list[float] = []   # fitness moyen par génération
        self.meilleur_individu:   Individu | None = None

    # ------------------------------------------------------------------
    # Point d'entrée
    # ------------------------------------------------------------------

    def evoluer(self, environnement, planificateur,
                callback=None) -> Individu:
        """
        Lance l'évolution complète.

        Args:
            environnement  : Environnement avec obstacles, paquet, dépôt
            planificateur  : PlanificateurAStar pré-construit
            callback       : fonction appelée après chaque génération
                             callback(generation, population, meilleur)

        Returns:
            Le meilleur Individu trouvé.
        """
        # Génération de la population initiale
        population = self._population_initiale()

        for generation in range(self.nb_generations):
            # ── Évaluation ───────────────────────────────────────────────
            self._evaluer_population(population, environnement, planificateur)

            # Tri par fitness croissante (meilleur = plus petit coût)
            population.sort(key=lambda ind: ind.fitness)

            # ── Statistiques ─────────────────────────────────────────────
            meilleur = population[0]
            moyen    = sum(i.fitness for i in population) / len(population)
            self.historique_meilleur.append(meilleur.fitness)
            self.historique_moyen.append(moyen)

            if (self.meilleur_individu is None or
                    meilleur.fitness < self.meilleur_individu.fitness):
                self.meilleur_individu = copy.deepcopy(meilleur)

            print(f"Génération {generation + 1:3d}/{self.nb_generations} | "
                  f"Meilleur: {meilleur.fitness:10.2f} | "
                  f"Moyen: {moyen:10.2f} | "
                  f"Gènes: v={meilleur.vitesse_max:.2f} "
                  f"c={meilleur.capacite_charge:.2f} "
                  f"a={meilleur.autonomie:.0f}")

            if callback:
                callback(generation, population, meilleur)

            # ── Nouvelle génération ───────────────────────────────────────
            population = self._nouvelle_generation(population)

        return self.meilleur_individu

    # ------------------------------------------------------------------
    # Population initiale
    # ------------------------------------------------------------------

    def _population_initiale(self) -> list[Individu]:
        """Génère N individus avec des gènes aléatoires dans les bornes."""
        population = []
        for _ in range(self.taille_population):
            individu = Individu(
                vitesse_max=random.uniform(*BORNES["vitesse_max"]),
                capacite_charge=random.uniform(*BORNES["capacite_charge"]),
                autonomie=random.uniform(*BORNES["autonomie"]),
            )
            population.append(individu)
        return population

    # ------------------------------------------------------------------
    # Évaluation
    # ------------------------------------------------------------------

    def _evaluer_population(self, population: list[Individu],
                             environnement, planificateur) -> None:
        """Simule chaque individu et calcule son fitness."""
        for individu in population:
            robot    = individu.to_robot()
            metriques = environnement.simuler_robot(robot, planificateur)
            individu.fitness = metriques["cout"]

    # ------------------------------------------------------------------
    # Sélection par tournoi
    # ------------------------------------------------------------------

    def _selection_tournoi(self, population: list[Individu]) -> Individu:
        """
        Sélectionne un individu par tournoi :
        on tire k individus au hasard et on garde le meilleur.
        """
        candidats = random.sample(population, self.taille_tournoi)
        return min(candidats, key=lambda ind: ind.fitness)

    # ------------------------------------------------------------------
    # Croisement (BLX-alpha / arithmétique)
    # ------------------------------------------------------------------

    def _croisement(self, parent1: Individu, parent2: Individu) -> tuple[Individu, Individu]:
        """
        Croisement arithmétique : chaque gène de l'enfant est une
        combinaison linéaire des gènes des deux parents.
        """
        if random.random() > self.taux_croisement:
            return copy.deepcopy(parent1), copy.deepcopy(parent2)

        alpha = random.random()   # coefficient de mélange

        def melange(g1, g2, borne):
            val1 = alpha * g1 + (1 - alpha) * g2
            val2 = (1 - alpha) * g1 + alpha * g2
            return (
                max(borne[0], min(val1, borne[1])),
                max(borne[0], min(val2, borne[1])),
            )

        v1,  v2  = melange(parent1.vitesse_max,     parent2.vitesse_max,
                           BORNES["vitesse_max"])
        c1,  c2  = melange(parent1.capacite_charge, parent2.capacite_charge,
                           BORNES["capacite_charge"])
        a1,  a2  = melange(parent1.autonomie,       parent2.autonomie,
                           BORNES["autonomie"])

        enfant1 = Individu(vitesse_max=v1, capacite_charge=c1, autonomie=a1)
        enfant2 = Individu(vitesse_max=v2, capacite_charge=c2, autonomie=a2)
        return enfant1, enfant2

    # ------------------------------------------------------------------
    # Mutation gaussienne
    # ------------------------------------------------------------------

    def _muter(self, individu: Individu) -> Individu:
        """
        Mutation gaussienne : chaque gène peut être perturbé
        selon une loi normale centrée sur sa valeur actuelle.
        """
        individu = copy.deepcopy(individu)

        for gene, borne in BORNES.items():
            if random.random() < self.taux_mutation:
                amplitude = (borne[1] - borne[0]) * 0.15   # 15% de la plage
                bruit     = random.gauss(0, amplitude)
                valeur    = getattr(individu, gene) + bruit
                valeur    = max(borne[0], min(valeur, borne[1]))
                setattr(individu, gene, valeur)

        return individu

    # ------------------------------------------------------------------
    # Nouvelle génération
    # ------------------------------------------------------------------

    def _nouvelle_generation(self, population: list[Individu]) -> list[Individu]:
        """
        Construit la génération suivante :
        1. Élitisme : on garde les N meilleurs directement
        2. On remplit le reste par sélection + croisement + mutation
        """
        nouvelle = []

        # Élitisme
        for i in range(self.elitisme):
            nouvelle.append(copy.deepcopy(population[i]))

        # Remplissage
        while len(nouvelle) < self.taille_population:
            parent1 = self._selection_tournoi(population)
            parent2 = self._selection_tournoi(population)
            enfant1, enfant2 = self._croisement(parent1, parent2)
            nouvelle.append(self._muter(enfant1))
            if len(nouvelle) < self.taille_population:
                nouvelle.append(self._muter(enfant2))

        return nouvelle

    # ------------------------------------------------------------------
    # Rapport final
    # ------------------------------------------------------------------

    def afficher_rapport(self) -> None:
        """Affiche un résumé des résultats après évolution."""
        if not self.meilleur_individu:
            print("Aucune évolution effectuée.")
            return

        print("\n" + "═" * 55)
        print("  RÉSULTATS — ALGORITHME GÉNÉTIQUE")
        print("═" * 55)
        print(f"  Meilleur individu trouvé :")
        print(f"    vitesse_max      : {self.meilleur_individu.vitesse_max:.3f} m/s")
        print(f"    capacite_charge  : {self.meilleur_individu.capacite_charge:.3f} kg")
        print(f"    autonomie        : {self.meilleur_individu.autonomie:.0f} J")
        print(f"    fitness (coût)   : {self.meilleur_individu.fitness:.2f}")
        print("─" * 55)
        print(f"  Convergence :")
        print(f"    Génération 1   → {self.historique_meilleur[0]:.2f}")
        print(f"    Génération {len(self.historique_meilleur):3d} → "
              f"{self.historique_meilleur[-1]:.2f}")
        amelioration = (1 - self.historique_meilleur[-1] /
                        self.historique_meilleur[0]) * 100
        print(f"    Amélioration   : {amelioration:.1f}%")
        print("═" * 55)