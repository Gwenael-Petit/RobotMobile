
from math import cos, sin, pi
from enum import Enum, auto
from .moteur import Moteur

class EtatRobot(Enum):
    EN_ATTENTE  = auto()   # Pas encore démarré
    VERS_PAQUET = auto()   # Se dirige vers le paquet
    CHARGE      = auto()   # Transporte le paquet vers le dépôt
    LIVRE       = auto()   # Mission accomplie
    EN_PANNE    = auto()   # Autonomie épuisée (garde-fou)
    VERS_BASE   = auto()   # Retourne à la base pour recharge
    EN_RECHARGE = auto()   # Recharge progressive à la base


# Seuil d'énergie (ratio) en dessous duquel le robot rentre se recharger
SEUIL_RECHARGE   = 0.20     # 20% de l'autonomie restante
VITESSE_RECHARGE = 500.0    # Joules rechargés par seconde


class RobotMobile:

    _nb_robots = 0
    _id_counter = 0

    def __init__(self, moteur=None, rayon=0.5, 
                 vitesse_max: float = 3.0,
                 capacite_charge: float = 10.0,
                 autonomie: float = 5000.0):
        if rayon <= 0:
            raise ValueError(f"Le rayon doit être strictement positif, reçu : {rayon}")
        
        self.__x = 0.0
        self.__y = 0.0
        self.__rotation = 0.0
        self.moteur = moteur
        self.rayon = rayon
        self.name = "pixie"

        # paramètres
        self.vitesse_max = vitesse_max          # m/s  [1.0 – 10.0]
        self.capacite_charge = capacite_charge  # kg   [1.0 – 50.0]
        self.autonomie = autonomie              # J    [500 – 10000]

        # État de mission
        self.etat             = EtatRobot.EN_ATTENTE
        self.energie_restante = autonomie
        self._etat_avant_recharge: EtatRobot = EtatRobot.VERS_PAQUET
        # Métriques collectées
        self.distance_parcourue = 0.0
        self.energie_consommee_total = 0.0
        self.nb_recharges = 0

        
        # Trajectoire
        self.trajectoire = []
        self.max_trajectoire = 100
        
        # Compteurs
        RobotMobile._nb_robots += 1
        RobotMobile._id_counter += 1
        self.id = RobotMobile._id_counter

    # ------------------------------------------------------------------
    # Reset pour une nouvelle simulation
    # ------------------------------------------------------------------

    def reset(self, x: float, y: float) -> None:
        """Réinitialise le robot pour une nouvelle simulation."""
        self.__x = x
        self.__y = y
        self.__rotation = 0.0
        self.etat = EtatRobot.VERS_PAQUET
        self.energie_restante = self.autonomie
        self._etat_avant_recharge    = EtatRobot.VERS_PAQUET
        self.distance_parcourue = 0.0
        self.energie_consommee_total = 0.0
        self.nb_recharges = 0
        self.trajectoire = [(x, y)]
        self.temps_mission = 0.0

        if self.moteur is not None:
            self.moteur.commander(v=0.0, omega=0.0)

    # ------------------------------------------------------------------
    # Recharge progressive
    # ------------------------------------------------------------------

    def recharger(self, dt: float) -> bool:
        """
        Recharge le robot progressivement.
        Appelée par Environnement quand le robot est EN_RECHARGE.

        Returns:
            True  → recharge terminée (énergie pleine)
            False → recharge en cours
        """
        self.energie_restante = min(
            self.autonomie,
            self.energie_restante + VITESSE_RECHARGE * dt
        )
        if self.energie_restante >= self.autonomie:
            self.energie_restante = self.autonomie
            self.nb_recharges    += 1
            return True
        return False

    def besoin_recharge(self) -> bool:
        """Retourne True si l'énergie est sous le seuil critique."""
        return (self.energie_restante / self.autonomie) < SEUIL_RECHARGE

    # ------------------------------------------------------------------
    # Mise à jour 
    # ------------------------------------------------------------------

    def mettre_a_jour(self, dt: float) -> None:
        """
        Met à jour la position via le moteur.
        Consomme de l'énergie proportionnellement à la vitesse et à la charge.
        """
        if dt <= 0:
            raise ValueError(f"dt doit être strictement positif, reçu : {dt}")

        if self.moteur is None:
            return

        x_avant, y_avant = self.__x, self.__y

        self.moteur.mettre_a_jour(self, dt)

        # Distance parcourue ce pas
        from math import hypot
        dp = hypot(self.__x - x_avant, self.__y - y_avant)
        self.distance_parcourue += dp
        self.temps_mission += dt

        # Consommation d'énergie
        charge = self.capacite_charge if self.etat == EtatRobot.CHARGE else 0.0
        conso  = self._consommation(dp, charge)
        self.energie_restante        -= conso
        self.energie_consommee_total += conso

        if self.energie_restante <= 0:
            self.energie_restante = 0
            self.etat = EtatRobot.EN_PANNE

        # Trajectoire
        self.trajectoire.append((self.__x, self.__y))
        if len(self.trajectoire) > self.max_trajectoire:
            self.trajectoire.pop(0)

    def _consommation(self, distance: float, charge: float) -> float:
        """
        Énergie consommée (J) pour un déplacement de `distance` mètres.
        Dépend de la vitesse max du robot et de la charge transportée.
        """
        k_charge = 0.5
        puissance = (self.vitesse_max * 10.0 + k_charge * charge) / 0.85
        # Énergie = puissance * temps = puissance * distance / vitesse
        if self.vitesse_max > 0:
            return puissance * distance / self.vitesse_max
        return 0.0

    # ------------------------------------------------------------------
    # Commande manuelle
    # ------------------------------------------------------------------

    def commander(self, **kwargs) -> None:
        """Envoie des commandes au moteur."""
        if self.moteur is not None:
            self.moteur.commander(**kwargs)

    def get_etat(self) -> tuple[float, float, float]:
        """Retourne (x, y, orientation) pour sauvegarde."""
        return (self.__x, self.__y, self.__rotation)

    def set_etat(self, etat: tuple[float, float, float]) -> None:
        """Restaure (x, y, orientation) après annulation de mouvement."""
        self.__x, self.__y, self.__rotation = etat

    # ------------------------------------------------------------------
    # Métriques & Fonction de coût
    # ------------------------------------------------------------------

    def metriques(self) -> dict:
        return {
            "succes"            : self.etat == EtatRobot.LIVRE,
            "distance_parcourue": self.distance_parcourue,
            "energie_consommee" : self.energie_consommee_total,
            "nb_recharges"       : self.nb_recharges,
            "cout"              : self.calculer_cout(),
            "etat_final"        : self.etat.name,
        }

    def calculer_cout(self) -> float:
        W_TEMPS    = 1.0 
        W_ENERGIE  = 0.01 
        PENALITE   = 100_000.0

        if self.etat == EtatRobot.EN_PANNE:
            return PENALITE
        if self.etat != EtatRobot.LIVRE:
            return PENALITE

        return W_TEMPS * self.temps_mission + W_ENERGIE * self.energie_consommee_total

    # ------------------------------------------------------------------
    # Utilitaires de classe
    # ------------------------------------------------------------------

    @classmethod
    def nombre_robots(cls) -> int:
        """Retourne le nombre total de robots créés."""
        return cls._nb_robots
    
    @staticmethod
    def moteur_valide(moteur) -> bool:
        """Vérifie si un objet est un moteur valide."""
        return isinstance(moteur, Moteur)
    
    def __del__(self):
        """Décrémente le compteur quand un robot est détruit."""
        RobotMobile._nb_robots -= 1
            
    # Getters and setters
    @property
    def x(self) -> float:
        return self.__x
    
    @property
    def y(self) -> float:
        return self.__y
    
    @property
    def orientation(self) -> float:
        return self.__rotation
    
    @x.setter
    def x(self, value: float) -> None:
        if not isinstance(value, (int, float)):
            raise TypeError(f"x doit être un nombre, reçu : {type(value)}")
        self.__x = float(value)

    @y.setter
    def y(self, value: float) -> None:
        if not isinstance(value, (int, float)):
            raise TypeError(f"y doit être un nombre, reçu : {type(value)}")
        self.__y = float(value)
    
    @orientation.setter
    def orientation(self, value: float) -> None:
        if not isinstance(value, (int, float)):
            raise TypeError(f"orientation doit être un nombre, reçu : {type(value)}")
        self.__rotation = float(value) % (2 * pi)

    def afficher(self) -> None:
        """Affiche la position du robot dans la console."""
        print(f"Robot #{self.id}: (x={self.x:.2f}, y={self.y:.2f}, "
              f"θ={self.orientation:.2f} rad, état={self.etat.name}, "
              f"énergie={self.energie_restante:.0f} J)"
              f"recharges={self.nb_recharges})")
        

    def __str__(self) -> str:
        return (f"Robot(x={self.x:.2f}, y={self.y:.2f}, "
                f"θ={self.orientation:.2f}, état={self.etat.name})")

    def __repr__(self) -> str:
        return (f"RobotMobile(vitesse={self.vitesse_max}, "
                f"charge={self.capacite_charge}, "
                f"autonomie={self.autonomie}, état={self.etat.name})")