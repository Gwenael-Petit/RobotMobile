from enum import auto, Enum
from math import cos, sin, pi
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

    def _consommation_par_pas(self, charge: float = 0.0) -> float:
        """Énergie consommée par pas (dépend de la vitesse et de la charge)."""
        k_charge = 0.5
        return (self.vitesse_max * 10.0 + k_charge * charge) / 0.85

    def _cible_courante(self, environnement) -> tuple[float, float]:
        if self.etat == EtatRobot.VERS_PAQUET:
            return environnement.position_paquet
        return environnement.position_depot

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