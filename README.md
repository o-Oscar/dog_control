

# Dog Control

All the stuff you need to control a robot dog.

## TODO

Goal : having a controller with low PD gains that keeps the robot upright both in simulation and in real life.

- calculer une estimation des torques que les moteurs ont appliqués à la dernière boucle. -> done
    - calculer kp * delta_pos + kd * estimated_vitesse -> done
    - cliper les torques (mesurer les torques max que les joints peuvent appliquer) -> done
- calculer une estimation des forces que les pieds ont appliquées à la dernière boucle -> done
    - /!\ ne marche que à l'extérieur des singularités -> mettre un flag quand on approche d'une singularité
    - /!\ On suppose qu'on est en régime quasi-statique -> ne peut pas être utilisé quand le chien commence à bouger (masse * accélération de la patte grand). 
- faire un estimateur de quand le chien est posé par terre ou non (somme des forces en Z plus grand qu'un certain seuil). 
- tester ça en simulation (on se dit que ça marche si la somme des forces compense la gravité)
    - en faisant descendre le chien au fur et à mesure -> ??
    - en désactivant de la contrainte de mocap -> en cours
- tester le tout en vrai (on se dit que ça marche si la somme des forces compense la gravité)

Then

- faire le bout de code qui transforme la force désirée au niveau d'un pied à une target en position.
    - On a : longueur vectoriel force égal moment (cross(L,F)=M) pour tous les joints
    - On projette le moment dans ce que peux faire le joint. On clip le moment dans un truc que le joint peut faire.
    - On déduit un delta de position target (en prenant en compte le kp différent pour chaque joint)
- solve a least-square problem to compute the forces at the feets needed to put the dog back into balance (QP problem, use quadprog) (need to think about what we want as force / torque applied on the main body)
    - calculer ce qu'on veut comme force/torque en fonction de l'orientation/vitesse de rotation du torse
    - commencer par contraindre un seul axe ? -> J'aimerai pouvoir mettre les axes uns par uns (hauteur, puis tanguage, puis déplacement d'avant en arrière en utilisant uniquement les moteurs des hip et des lower leg ?)
- add the constraints that the forces must be over the ground
- add the constraint that the torques at each motor must be smaller than some threshold (measure the max torque for each motor)

## Notes

### mujoco

mujoco xml reference : https://mujoco.readthedocs.io/en/latest/XMLreference.html#xml-reference

And the other very good one : https://mujoco.readthedocs.io/en/latest/APIreference.html#mjdata

### raspberry

To setup a network on the raspberry :

- sudo raspi-config
- System option
- Wireless LAN
- Enter Box name and password