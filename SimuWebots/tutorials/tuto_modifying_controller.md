# Tutoriel : Modification du contrôleur du robot

Ce tutoriel décrit la procédure de modification du contrôleur du robot simulé dans Webots.

## Déscription du contrôleur utilisé

Le contrôleur utilisé par le robot simulé est un contrôleur ROS. Il est donc possible de le modifier en utilisant les outils de ROS. Le contrôleur est implémenté dans le dossier [controllers/simu_ROS_standard/](../controllers/simu_ROS_standard/). Ce dossier contient les fichiers nécéssaires à l'utilisation du contrôleur ROS dans Webots.

Lors du démarage de la simulation, Webots lance le script python [simu_ROS_standard.py](../controllers/simu_ROS_standard/simu_ROS_standard.py). Ce script python est le point d'entrée du contrôleur ROS.

Pour standardiser le fonctionnement du véhicule simulé, ce point d'entrée instancie des classes python situées dans le même dossier et qui correspondent aux différentes parties du robot simulé. On peut trouver **une classe pour le control** du robot, c'est à dire l'application des commandes de vitesse et de direction et **une classe par capteur** utilisé par le robot simulé.

Chaque classe capteur définit un topic ROS sur lequel elle publie les données du capteur. De même, la classe control définit un topic ROS sur lequel elle souscrit pour recevoir les commandes de vitesse et de direction à appliquer au robot simulé.

Les noms des topics ROS utilisés par le contrôleur sont définis de façon à être cohérents avec les noms des topics ROS utilisés par le robot réel. Ainsi, il est possible de tester un algorithme de navigation sur le robot simulé avant de l'implémenter sur le robot réel sans avoir à modifier le code de l'algorithme.

## Ajout d'un capteur

Pour ajouter un capteur au robot simulé, il faut créer une nouvelle classe dans le dossier [simu_ROS_standard/](../controllers/simu_ROS_standard/). Il est conseillé de copier le code d'une classe déjà existante et de le modifier pour l'adapter au nouveau capteur.


### Pré-requis

Pour publier les données d'un nouveau capteur, il est nécéssaire d'avoir au préalable ajouté le capteur au modèle 3D du robot simulé (voir [tuto_modifying_robot.md](tuto_modifying_robot.md) pour plus d'informations).

### Import des modules python
Importer les modules python nécéssaires à son fonctionnement.
- Le module ```rospy``` est nécéssaire pour la communication avec ROS.
- Le module ```Driver``` est nécéssaire pour la communication avec Webots.
- Il existe enfin un module par type de capteur utilisé par le robot simulé dans Webots (voir https://cyberbotics.com/doc/guide/sensors). 
- Importer le type de message ROS correspondant aux données publiées par le capteur. (Il est recommandé d'utiliser les ```sensor_msgs``` pour les capteurs standards, voir http://wiki.ros.org/sensor_msgs).

Exemple pour la caméra :
```python
from vehicle import Driver
from controller import Camera

from sensor_msgs.msg import Image as SensorImage
import rospy
```
