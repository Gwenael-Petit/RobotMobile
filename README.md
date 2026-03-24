# 🤖 Simulateur de Robot Mobile

![Python](https://img.shields.io/badge/python-3.8+-blue.svg)
![Pygame](https://img.shields.io/badge/pygame-2.0+-green.svg)

Simulation de robots mobiles dans un entrepôt 2D. Un robot autonome doit récupérer des colis dispersés dans le hangar et les livrer à une zone de dépôt, en évitant les obstacles et en gérant son autonomie énergétique.

Le projet intègre :

- **Navigation autonome** : planification de chemin A* sur grille d'occupation, suivi de trajectoire par contrôleur PID
- **Gestion énergétique** : consommation dynamique, détection de seuil critique et recharge automatique à la base
- **Optimisation génétique** : algorithme génétique qui optimise 3 paramètres du robot (vitesse maximale, capacité de charge, autonomie) pour minimiser le coût de mission
- **Visualisation temps réel** : rendu Pygame avec affichage du chemin A*, barres d'énergie, métriques et comparaison simultanée de 4 robots
- **Analyse des résultats** : courbe de convergence et visualisation 3D de l'espace des solutions exploré

## Pour commencer

Ces instructions vous permettront de lancer le simulateur sur votre machine locale.

### Pré-requis

Ce dont vous avez besoin pour exécuter le projet :

* Python 3.8 ou supérieur
* pip (gestionnaire de paquets Python)
* Pygame 2.0+

### Installation

1. Clonez le repository

```bash
git clone https://github.com/votre-username/robot-simulator.git
cd robot-simulator
```

1. Installez les dépendances

```bash
pip install pygame
```

1. Lancez le simulateur

```bash
python -m project.main
```

Vous devriez voir une fenêtre Pygame s'ouvrir avec un robot contrôlable au clavier.

## Démarrage

Pour lancer la simulation :

```bash
python -m project.main
```

### Contrôles

* `↑` : Avancer
* `↓` : Reculer
* `←` : Tourner à gauche
* `→` : Tourner à droite
* `Espace` : Arrêt d'urgence

## Fabriqué avec

* **Python 3.13** - Langage de programmation
* **Pygame 2.0** - Bibliothèque graphique
* **Visual Studio Code** - Éditeur de code

## Auteurs

* **Gwénaël PETIT** - [@Gwenael-Petit](https://github.com/Gwenael-Petit)
* **Anita RIVOT** - [@username](https://github.com/username)
