import math


class GrilleOccupation:
    """
    Grille 2D représentant l'environnement du robot.
    
    Le repère monde est centré au milieu de la grille :
    - x : axe horizontal (droite positif)
    - y : axe vertical (haut positif)
    """

    INCONNU = -1
    LIBRE   =  0
    OCCUPE  =  1

    def __init__(self, largeur_m: float, hauteur_m: float, resolution: float = 0.25):
        """
        Args:
            largeur_m  : largeur du monde en mètres
            hauteur_m  : hauteur du monde en mètres
            resolution : taille d'une cellule en mètres (ex: 0.25 = 25 cm)
        """
        self.largeur_m  = largeur_m
        self.hauteur_m  = hauteur_m
        self.resolution = resolution

        # Nombre de cellules sur chaque axe
        self.nx = math.ceil(largeur_m / resolution)
        self.ny = math.ceil(hauteur_m / resolution)

        # Origine du repère (coin bas-gauche en indices)
        self._origin_x = largeur_m / 2
        self._origin_y = hauteur_m / 2

        # Grille initialisée à LIBRE
        self._grid = [[self.LIBRE] * self.ny for _ in range(self.nx)]

    # ------------------------------------------------------------------
    # Conversions coordonnées ↔ indices
    # ------------------------------------------------------------------

    def coord2index(self, x: float, y: float) -> tuple[int, int]:
        """
        Convertit des coordonnées monde (mètres) en indices de grille.
        Les valeurs sont clampées pour rester dans la grille.
        """
        ix = int((x + self._origin_x) / self.resolution)
        iy = int((y + self._origin_y) / self.resolution)
        ix = max(0, min(ix, self.nx - 1))
        iy = max(0, min(iy, self.ny - 1))
        return ix, iy

    def index2coord(self, ix: int, iy: int) -> tuple[float, float]:
        """
        Convertit des indices de grille en coordonnées monde (mètres).
        Retourne le centre de la cellule.
        """
        x = (ix + 0.5) * self.resolution - self._origin_x
        y = (iy + 0.5) * self.resolution - self._origin_y
        return x, y

    # ------------------------------------------------------------------
    # Accesseurs (encapsulation de _grid)
    # ------------------------------------------------------------------

    def get_cellule(self, ix: int, iy: int) -> int:
        """Retourne l'état de la cellule aux indices donnés."""
        if not self._dans_grille(ix, iy):
            return self.OCCUPE  # Hors grille = considéré occupé
        return self._grid[ix][iy]

    def set_cellule(self, ix: int, iy: int, etat: int) -> None:
        """Modifie l'état de la cellule aux indices donnés."""
        if self._dans_grille(ix, iy):
            self._grid[ix][iy] = etat

    def _dans_grille(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    # ------------------------------------------------------------------
    # Construction depuis un Environnement
    # ------------------------------------------------------------------

    @classmethod
    def construct(cls, env, resolution: float = 0.25,
                              marge: float = 0.8) -> "GrilleOccupation":
        """
        Construit une GrilleOccupation à partir d'un Environnement existant
        en marquant les cellules proches d'obstacles comme OCCUPÉES.

        Args:
            env        : Environnement contenant les obstacles
            resolution : résolution de la grille en mètres
            marge      : marge autour des obstacles (rayon du robot)
        """
        grille = cls(env.largeur, env.hauteur, resolution)

        for ix in range(grille.nx):
            for iy in range(grille.ny):
                x, y = grille.index2coord(ix, iy)
                for obs in env.obstacles:
                    # On simule un robot-point avec rayon = marge
                    class _FakeRobot:
                        pass
                    fake = _FakeRobot()
                    fake.x, fake.y, fake.rayon = x, y, marge
                    if obs.collision(fake):
                        grille.set_cellule(ix, iy, cls.OCCUPE)
                        break

        return grille

    def __repr__(self) -> str:
        return (f"GrilleOccupation({self.largeur_m}m × {self.hauteur_m}m, "
                f"résolution={self.resolution}m, "
                f"taille={self.nx}×{self.ny} cellules)")