import heapq
import math
from project.robot.modele.grille_occupation import GrilleOccupation


class PlanificateurAStar:
    """
    Planificateur global basé sur A*.

    Prend une GrilleOccupation par injection de dépendance (composition).
    Travaille en indices de grille, retourne des coordonnées monde (mètres).
    """

    def __init__(self, grille: GrilleOccupation):
        self.grille = grille

    # ------------------------------------------------------------------
    # Heuristique
    # ------------------------------------------------------------------

    def heuristique(self, idx_a: tuple[int, int],
                    idx_b: tuple[int, int]) -> float:
        """Distance euclidienne entre deux indices de grille."""
        return math.hypot(idx_a[0] - idx_b[0], idx_a[1] - idx_b[1])

    # ------------------------------------------------------------------
    # Calcul du chemin
    # ------------------------------------------------------------------

    def trouver_chemin(self,
                       depart_m: tuple[float, float],
                       arrivee_m: tuple[float, float]) -> list[tuple[float, float]]:
        """
        Calcule le chemin de `depart_m` vers `arrivee_m` (coordonnées monde).

        Returns:
            Liste de points (x, y) en mètres, du départ à l'arrivée.
            Liste vide si aucun chemin n'existe.
        """
        start = self.grille.coord2index(*depart_m)
        goal  = self.grille.coord2index(*arrivee_m)

        if self.grille.get_cellule(*goal) == GrilleOccupation.OCCUPE:
            print("PlanificateurAStar : objectif dans un obstacle !")
            return []

        # File de priorité : (f_score, nœud)
        open_set: list = []
        heapq.heappush(open_set, (0.0, start))

        came_from: dict[tuple, tuple] = {}
        g_score: dict[tuple, float]   = {start: 0.0}

        while open_set:
            _, courant = heapq.heappop(open_set)

            if courant == goal:
                return self._reconstruire_chemin(came_from, courant)

            for voisin in self._voisins(courant):
                # Coût : 1 pour déplacement cardinal, √2 pour diagonal
                dx = abs(voisin[0] - courant[0])
                dy = abs(voisin[1] - courant[1])
                cout_deplacement = math.sqrt(2) if dx == 1 and dy == 1 else 1.0

                tentative_g = g_score[courant] + cout_deplacement

                if tentative_g < g_score.get(voisin, float('inf')):
                    came_from[voisin] = courant
                    g_score[voisin]   = tentative_g
                    f = tentative_g + self.heuristique(voisin, goal)
                    heapq.heappush(open_set, (f, voisin))

        return []  # Pas de chemin trouvé

    # ------------------------------------------------------------------
    # Utilitaires privés
    # ------------------------------------------------------------------

    def _voisins(self, idx: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Retourne les 8 voisins accessibles (4 cardinaux + 4 diagonaux).
        Exclut les cellules occupées et hors grille.
        """
        ix, iy = idx
        candidats = [
            (ix + 1, iy),     (ix - 1, iy),
            (ix, iy + 1),     (ix, iy - 1),
            (ix + 1, iy + 1), (ix - 1, iy + 1),
            (ix + 1, iy - 1), (ix - 1, iy - 1),
        ]
        return [
            c for c in candidats
            if self.grille.get_cellule(*c) != GrilleOccupation.OCCUPE
        ]

    def _reconstruire_chemin(self,
                              came_from: dict,
                              courant: tuple) -> list[tuple[float, float]]:
        """Reconstruit le chemin en coordonnées monde depuis came_from."""
        chemin_indices = []
        while courant in came_from:
            chemin_indices.append(courant)
            courant = came_from[courant]
        chemin_indices.append(courant)
        chemin_indices.reverse()

        # Conversion indices → coordonnées monde
        return [self.grille.index2coord(ix, iy) for ix, iy in chemin_indices]