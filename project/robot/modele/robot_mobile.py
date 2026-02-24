from math import cos, sin, pi
from .moteur import Moteur


class RobotMobile:

    _nb_robots = 0
    _id_counter = 0

    def __init__(self, moteur=None, rayon=0.5):
        if rayon <= 0:
            raise ValueError(f"Le rayon doit être strictement positif, reçu : {rayon}")
        
        self.__x = 0.0
        self.__y = 0.0
        self.__rotation = 0.0
        self.moteur = moteur
        self.rayon = rayon
        self.name = "pixie"
        
        # Trajectoire
        self.trajectoire = []
        self.max_trajectoire = 50
        
        # Compteurs
        RobotMobile._nb_robots += 1
        RobotMobile._id_counter += 1
        self.id = RobotMobile._id_counter

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