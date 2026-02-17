from abc import ABC, abstractmethod
import pygame
class Controleur(ABC):
    @abstractmethod
    def lire_commande(self):
        """Retourne une commande pour le robot"""
        pass

class ControleurTerminal(Controleur):
    def lire_commande(self):
        print("Commande differentiel : v omega (ex: 1.0 0.5)")
        # Utiliser input pour recuperer les entree clavier
        v = float(input("v: "))
        omega = float(input("omega: "))
        return {"v": v, "omega": omega}
    
class ControleurClavierPygame(Controleur):
    def lire_commande(self):
        keys = pygame.key.get_pressed()
        v = 0.0
        omega = 0.0
        if keys[pygame.K_UP]:
            v += 1.0
        if keys[pygame.K_DOWN]:
            v -= 1.0
        if keys[pygame.K_LEFT]:
            omega += 0.5
        if keys[pygame.K_RIGHT]:
            omega -= 0.5
        return {"v": v, "omega": omega}
