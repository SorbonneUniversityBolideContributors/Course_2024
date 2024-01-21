# Tutoriel : Utilisation du simulateur

Ce tutoriel décrit la procédure d'utilisation du simulateur Webots pour la course de voitures autonomes.

## Démarrage

### ROS Master
Dans un premier temps, il est **impératif** de lancer un ROS Master. Sans celui-ci, le véhicule simulé ne pourra pas souscrire ou publier dans des topics ROS.
```bash
roscore
```

### Lancement du simulateur
Pour lancer **webots**, il faut alors utiliser la commande :
```bash
webots
```

### Chargement d'un monde de simulation
Depuis la fenêtre principale de WeBots, aller dans ```File``` puis ```Open World``` et sélectionner un des fichiers ```.wbt``` de [worlds](worlds/)

Le fichier [Piste_SU_2023b.wbt](../worlds/Piste_SU_2023b.wbt) est le monde de simulation utilisé pour la course de voitures autonomes. Il est possible de le modifier pour tester les algorithmes de navigation dans un environnement différent. (voir [tuto_modifying_world.md](tuto_modifying_world.md) pour plus d'informations)

Pour lancer la simulation, il faut alors cliquer sur le **bouton play** en haut de la fenêtre de visualisation graphique (Pour vérifier si le simulateur est bien en route, un indicateur temporel est situé à gauche du bouton play. Si cet indicateur affiche un chronomètre, alors la simulation est bien en cours de fonctionnement.). Si besoin, la simulation peut être arrétée et relancée avec les boutons pause et restart, voisins au bouton play.

## Erreurs courantes
### Erreur de lancement du simulateur
Si le simulateur ne se lance pas, il est possible que le package ROS **webots_ros** ne soit pas installé. Dans ce cas, il faut utiliser la commande :
```bash
sudo apt-get install ros-noetic-webots-ros
```
Comme indiqué dans le tutoriel d'installation.

### Erreur de module non trouvé
Si le simulateur se lance mais affiche une erreur de module non trouvé, il s'agit plutôt d'un problème de configuration de l'environnement ROS. Il faut alors vérifier que les autres packages ROS utilisés par le simulateur sont bien installés et que l'environnement ROS est bien configuré (oubli d'un package ROS par exemple dans ```catkin_ws/src/```). N'oubliez pas d'effectuer la commande
```bash
catkin_make
```
dans le dossier ```catkin_ws``` après avoir installé un nouveau package ROS.

## Plus d'informations sur l'utilisation de Webots
Pour plus d'informations sur l'utilisation de Webots, se référer à la [documentation officielle de Webots](https://cyberbotics.com/doc/guide/tutorials?tab-language=python).

Nous avons également réalisé un certain nombre de tutoriels pour vous aider à modifier les mondes de simulation et véhicules utilisés en fonction de l'avançée du projet. Ces tutoriels sont disponibles dans le dossier [tutorials](../tutorials/).