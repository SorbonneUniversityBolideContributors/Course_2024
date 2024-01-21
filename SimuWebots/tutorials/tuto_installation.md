
# Tutoriel : Installation de Webots

Ce tutoriel décrit la procédure d'installation du simulateur Webots sur un environnement Linux de façon à pouvoir l'utiliser avec ROS1 Noetic.

## Pré-requis
- disposer d'un environnement de PC compatible avec ROS1 Noetic (par exemple : Ubuntu 20.xx)
- disposer des droits administrateurs dans cet environnement (sudo sous Linux)
- Disposer d'une installation fonctionnelle de ROS1 (de préférence ROS Noetic)

Une connexion Internet sera nécéssaire pour l'installation du logiciel et pour le téléchargement de certaines données du simulateur (modèles 3D, textures ...). 

## Procédure d'installation de Webots :
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