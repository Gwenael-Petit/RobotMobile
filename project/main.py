import math
from robot.robot_mobile import RobotMobile
from robot.moteur import *

robot = RobotMobile(...)

"""
robot.display_robot()
robot.move_forward(1.0)
robot.display_robot()
robot.rotate(0.7854) # Rotate 45 degrees in radians = 0.7854
robot.display_robot()
robot.move_forward(3.0)
robot.display_robot()
"""

moteur_diff = MoteurDifferentiel()
robot = RobotMobile(moteur=moteur_diff)
dt = 1.0 # pas de temps (s)
robot.afficher()
# On doit nommer les arguments (v = ..., omega = ...) car on utilise **kwargs !
robot.commander(v = 3.0, omega = math.pi/2) # avance en tournant
robot.mettre_a_jour(dt)
robot.afficher()
robot.commander(v = 1.0, omega = 0.0) # avance tout droit
robot.mettre_a_jour(dt)
robot.afficher()

moteur_omni = MoteurOmnidirectionnel()
robot = RobotMobile(moteur=moteur_omni)
robot.afficher()
robot.commander(vx = 3.0, vy = 1.0, omega = 0.0) # avance en tournant
robot.mettre_a_jour(dt)
robot.afficher()
print(robot)