from enum import auto, Enum
from math import atan2, cos, hypot, sin, pi
from .moteur import Moteur

class EtatRobot(Enum):
    EN_ATTENTE  = auto()   # Pas encore démarré
    VERS_PAQUET = auto()   # Se dirige vers le paquet
    CHARGE      = auto()   # Transporte le paquet vers le dépôt
    LIVRE       = auto()   # Mission accomplie
    EN_PANNE    = auto()   # Autonomie épuisée


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

        # Métriques collectées
        self.pas_effectues = 0
        self.energie_consommee_total = 0.0
        
        # Trajectoire
        self.trajectoire = []
        self.max_trajectoire = 50
        
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
        self.chemin_courant = []
        self.pas_effectues = 0
        self.energie_consommee_total = 0.0
        self.trajectoire = [(x, y)]

    # ------------------------------------------------------------------
    # Navigation autonome (appelée par Environnement.mise_a_jour)
    # ------------------------------------------------------------------
    
    def effectuer_pas_autonome(self, environnement, dt: float) -> bool:
        """
        Déplace le robot d'un pas vers sa cible courante en suivant
        le chemin calculé par A*.

        Returns:
            True  → mission en cours
            False → mission terminée (livré ou en panne)
        """
        if self.etat in (EtatRobot.LIVRE, EtatRobot.EN_PANNE, EtatRobot.EN_ATTENTE):
            return False

        # Recalcul du chemin si nécessaire
        if not self.chemin_courant:
            cible = self._cible_courante(environnement)
            chemin = environnement.calculer_chemin((self.__x, self.__y), cible)
            if not chemin or len(chemin) <= 1:
                self.etat = EtatRobot.EN_PANNE
                return False
            self.chemin_courant = chemin[1:]

        # Prochain waypoint
        cible_x, cible_y = self.chemin_courant[0]
        dx = cible_x - self.__x
        dy = cible_y - self.__y
        distance = hypot(dx, dy)

        # Consommation d'énergie
        charge = self.capacite_charge if self.etat == EtatRobot.CHARGE else 0.0
        conso = self._consommation_par_pas(charge)
        if self.energie_restante < conso:
            self.etat = EtatRobot.EN_PANNE
            return False

        # Déplacement vers le waypoint
        pas = self.vitesse_max * dt
        if distance <= pas:
            self.__x, self.__y = cible_x, cible_y
            self.chemin_courant.pop(0)
        else:
            self.__x += (dx / distance) * pas
            self.__y += (dy / distance) * pas

        # Orientation vers la cible
        self.__rotation = atan2(dy, dx) % (2 * pi)

        # Mise à jour énergie et métriques
        self.energie_restante -= conso
        self.energie_consommee_total += conso
        self.pas_effectues += 1

        # Enregistrement trajectoire
        self.trajectoire.append((self.__x, self.__y))
        if len(self.trajectoire) > self.max_trajectoire:
            self.trajectoire.pop(0)

        # Transitions d'état
        if self.etat == EtatRobot.VERS_PAQUET:
            px, py = environnement.position_paquet
            if hypot(self.__x - px, self.__y - py) < self.rayon + 0.3:
                self.etat = EtatRobot.CHARGE
                self.chemin_courant = []

        elif self.etat == EtatRobot.CHARGE:
            dx2, dy2 = environnement.position_depot
            if hypot(self.__x - dx2, self.__y - dy2) < self.rayon + 0.3:
                self.etat = EtatRobot.LIVRE
                return False

        return True

    def _consommation_par_pas(self, charge: float = 0.0) -> float:
        """Énergie consommée par pas (dépend de la vitesse et de la charge)."""
        k_charge = 0.5
        return (self.vitesse_max * 10.0 + k_charge * charge) / 0.85

    def _cible_courante(self, environnement) -> tuple[float, float]:
        if self.etat == EtatRobot.VERS_PAQUET:
            return environnement.position_paquet
        return environnement.position_depot
    
    # ------------------------------------------------------------------
    # Métriques & Fonction de coût
    # ------------------------------------------------------------------

    def metriques(self) -> dict:
        return {
            "succes": self.etat == EtatRobot.LIVRE,
            "pas_effectues": self.pas_effectues,
            "energie_consommee": self.energie_consommee_total,
            "cout": self.calculer_cout(),
            "etat_final": self.etat.name,
        }

    def calculer_cout(self) -> float:
        """
        Fonction de coût à minimiser par l'algorithme génétique.
        coût = w_temps * pas + w_energie * énergie + pénalité_si_échec
        """
        W_TEMPS   = 1.0
        W_ENERGIE = 0.01
        PENALITE  = 100_000.0

        if self.etat != EtatRobot.LIVRE:
            return PENALITE

        return W_TEMPS * self.pas_effectues + W_ENERGIE * self.energie_consommee_total
    
    # ------------------------------------------------------------------
    # Contrôle manuel
    # ------------------------------------------------------------------

    def move_forward(self, distance: float) -> None:
        """Déplace le robot en avant de la distance spécifiée."""
        self.__x += distance * cos(self.__rotation)
        self.__y += distance * sin(self.__rotation)

    def rotate(self, angle: float) -> None:
        """Tourne le robot de l'angle spécifié (en radians)."""
        self.__rotation += angle
        self.__rotation = self.__rotation % (2 * pi)

    def commander(self, **kwargs) -> None:
        """Envoie des commandes au moteur du robot."""
        if self.moteur is not None:
            self.moteur.commander(**kwargs)

    def mettre_a_jour(self, dt: float) -> None:
        """Met à jour la position du robot selon son modèle cinématique."""
        if dt <= 0:
            raise ValueError(f"dt doit être strictement positif, reçu : {dt}")
        
        if self.moteur is not None:
            self.moteur.mettre_a_jour(self, dt)
            
            # Enregistrer la position dans la trajectoire
            self.trajectoire.append((self.x, self.y))
            if len(self.trajectoire) > self.max_trajectoire:
                self.trajectoire.pop(0)

    def get_etat(self) -> tuple[float, float, float]:
        """
        Retourne l'état complet du robot (x, y, orientation).
        Utile pour sauvegarder/restaurer la position.
        """
        return (self.__x, self.__y, self.__rotation)

    def set_etat(self, etat: tuple[float, float, float]) -> None:
        """
        Restaure l'état complet du robot (x, y, orientation).
        Utile pour annuler un mouvement après collision.
        """
        self.__x, self.__y, self.__rotation = etat

    # ------------------------------------------------------------------
    # Utilitaires
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
              f"orientation={self.orientation:.2f} rad)")

    def __str__(self) -> str:
        return f"Robot(x={self.x:.2f}, y={self.y:.2f}, θ={self.orientation:.2f})"
    
    def __repr__(self) -> str:
        return (f"RobotMobile(x={self.x:.2f}, y={self.y:.2f}, "
                f"orientation={self.orientation:.2f}, rayon={self.rayon})")