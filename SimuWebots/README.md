# Installation et Utilisation du simulateur :
**Note importante 1 :** Ce simulateur a été intégralement développé sous Linux, et n'a pas été testé sous Windows ou MacOS. Il est possible d'installer WeBots sous ces deux OS, mais l'utilisation de ROS dans ces environnements est moins naturelle que sous Linux. N'ayant pas accès à une machine équipée de Windows ou MacOS, je ne peux pas fournir d'instructions d'installation du simulateur pour ces deux OS.

**Note importante 2 :** L'utilisation de ce simulateur requiert de maîtriser les concepts de base de ROS (lecture et publication efficace dans des topics ROS, utilsation du rosmaster et de roslaunch, rosbag, rosmsg, timers ...). Ce document est rédigé en supposant que le lecteur maîtrise ces points théoriques. Si ce n'est pas le cas, se référer à la documentation de ROS et aux exemples proposés dans le Google Drive du projet.

## I - Installation - SANS Docker : 
### Pré-requis :
- disposer d'un environnement de PC compatible avec ROS1 Noetic (par exemple : Ubuntu 20.xx)
- disposer des droits administrateurs dans cet environnement (sudo sous Linux)
- Disposer d'une installation fonctionnelle de ROS1 (de préférence ROS Noetic)

Une connexion Internet sera nécéssaire pour l'installation du logiciel et pour le téléchargement de certaines données du simulateur (modèles 3D, textures ...). 

### Procédure d'installation de Webots :
Sous Linux, dans un terminal, entrer les commandes :
```bash
wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add -
sudo apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/
```
```bash
sudo apt-get update
sudo apt-get install webots
```

Une fois l'installation terminée, il est possible de lancer WeBots directement depuis le terminal en utilisant la commande :
```bash
webots
```
 
Pour pouvoir utiliser l'environement de simulation du robot, il faut égalmenet disposer du package ROS **webots_ros** :

```bash
sudo apt-get install ros-noetic-webots-ros
```

## III - Utilisation du simulateur : 
### Démarrage :
Dans un premier temps, il est **impératif** de lancer un ROS Master. Sans celui-ci, le véhicule simulé ne pourra pas souscrire ou publier dans des topics ROS.
```bash
roscore
```
Pour lancer **webots**, il faut alors utiliser la commande :
```bash
webots
```

Depuis la fenêtre principale de WeBots, aller dans ```File``` puis ```Open World``` et sélectionner un des fichiers ```.wbt``` de [worlds](worlds/)
 
Pour lancer la simulation, il faut alors cliquer sur le **bouton play** en haut de la fenêtre de visualisation graphique (Pour vérifier si le simulateur est bien en route, un indicateur temporel est situé à gauche du bouton play. Si cet indicateur affiche un chronomètre, alors la simulation est bien en cours de fonctionnement.). Si besoin, la simulation peut être arrétée et relancée avec les boutons pause et restart, voisins au bouton play.

### Utilisation : 
Le simulateur est implémenté de sorte que le robot simulé soit le plus proche possible du robot réel, que ce soit en termes d'architecture électronique (capteurs), physique (poids, inertie...) ou d'implémentation (interfaçage avec ROS). Les algorithmes réalisés pour le calcul des trajectoire de ce robot devront être facilement compréhensibles (commentaires, documentation, codes lisibles), légers en consommation CPU et surtout paramétrables (l'environnement physique réel du robot pourra être bien différent de son environnement simulé : les paramètres utilisé dans les algorithmes simulé seront donc amenés à évoluer une fois dans le monde réel).

## Contenu de SIMU

- [controllers](controllers/) : Ce dossier contient les codes pythons excécutés par webots lors de l'aparition d'un véhicule employant le controleur cité.
- [protos](protos/)
- [worlds](worlds/)