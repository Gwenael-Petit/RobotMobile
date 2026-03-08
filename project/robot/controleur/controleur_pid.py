import math


class ControleurPID:

    def __init__(self,
                 kp_lin: float    = 1.5,
                 kp_ang: float    = 3.0,
                 tolerance: float = 0.5,
                 v_max: float     = 5.0,
                 omega_max: float = 3.0):
        self.kp_lin    = kp_lin
        self.kp_ang    = kp_ang
        self.tolerance = tolerance
        self.v_max     = v_max
        self.omega_max = omega_max

        # État interne
        self._chemin: list[tuple[float, float]] = []
        self._index_waypoint: int = 0

    # ------------------------------------------------------------------
    # Interface publique
    # ------------------------------------------------------------------

    def set_chemin(self, chemin: list[tuple[float, float]]) -> None:
        """Charge un nouveau chemin et remet l'index à zéro."""
        self._chemin = chemin
        self._index_waypoint = 0

    def est_arrive(self) -> bool:
        """Retourne True quand tous les waypoints ont été atteints."""
        return self._index_waypoint >= len(self._chemin)

    def calculer_commande(self,
                      x: float,
                      y: float,
                      theta: float) -> dict[str, float]:
        if self.est_arrive():
            return {"v": 0.0, "omega": 0.0}

        # ── Waypoint courant ─────────────────────────────────────────────
        xc, yc = self._chemin[self._index_waypoint]

        dx = xc - x
        dy = yc - y
        distance = math.hypot(dx, dy)

        # Waypoint atteint → suivant
        if distance < self.tolerance:
            self._index_waypoint += 1
            if self.est_arrive():
                return {"v": 0.0, "omega": 0.0}
            xc, yc = self._chemin[self._index_waypoint]
            dx = xc - x
            dy = yc - y
            distance = math.hypot(dx, dy)

        #  Commande angulaire 
        theta_des = math.atan2(dy, dx)
        e_theta   = theta_des - theta
        e_theta   = (e_theta + math.pi) % (2 * math.pi) - math.pi

        omega = self.kp_ang * e_theta
        omega = max(-self.omega_max, min(omega, self.omega_max))

        #  Commande linéaire — exploite vraiment v_max 
        if abs(e_theta) < math.pi / 6:
            # Bien orienté → pleine vitesse
            v = self.v_max
        elif abs(e_theta) > math.pi / 2:
            # Mal orienté → tourne sur place
            v = 0.0
        else:
            # Intermédiaire → ralentit proportionnellement
            v = self.v_max * (1 - abs(e_theta) / (math.pi / 2))

        return {"v": v, "omega": omega}

    # ------------------------------------------------------------------
    # Propriété utile pour la Vue
    # ------------------------------------------------------------------

    @property
    def progression(self) -> float:
        """Retourne la progression entre 0.0 et 1.0."""
        if not self._chemin:
            return 1.0
        return self._index_waypoint / len(self._chemin)

    @property
    def chemin(self) -> list[tuple[float, float]]:
        return self._chemin