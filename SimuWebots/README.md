# Installation et Utilisation du simulateur :
**Note importante 1 :** Ce simulateur a été intégralement développé sous Linux, et n'a pas été testé sous Windows ou MacOS. Il est possible d'installer WeBots sous ces deux OS, mais l'utilisation de ROS dans ces environnements est moins naturelle que sous Linux. N'ayant pas accès à une machine équipée de Windows ou MacOS, je ne peux pas fournir d'instructions d'installation du simulateur pour ces deux OS.   
  
**Note importante 2 :** L'utilisation de ce simulateur requiert de maîtriser les concepts de base de ROS (lecture et publication efficace dans des topics ROS, utilsation du rosmaster et de roslaunch, rosbag, rosmsg, timers ...). Ce document est rédigé en supposant que le lecteur maîtrise ces points théoriques. Si ce n'est pas le cas, se référer à la documentation de ROS et aux exemples proposés dans le Google Drive du projet.  

## I - Installation - SANS Docker : 
#### Pré-requis : 
- disposer d'un environnement de PC compatible avec ROS1 Noetic (par exemple : Ubuntu 20.xx)
- disposer des droits administrateurs dans cet environnement (sudo sous Linux)
- Disposer d'une installation fonctionnelle de ROS1 (de préférence ROS Noetic)

Une connexion Internet sera nécéssaire pour l'installation du logiciel et pour le téléchargement de certaines données du simulateur (modèles 3D, textures ...). 

#### Procédure d'installation de Webots : 
Sous Linux, dans un terminal, entrer les commandes :   
$>>$ wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add -  
$>>$ sudo apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/'  
$>>$ sudo apt-get update
$>>$ sudo apt-get install webots  
  
Une fois l'installation terminée,  il est possible de lancer WeBots directement depuis le terminal en utilisant la commande :  
$>>$ webots  
  
Pour pouvoir utiliser l'environement de simulation du robot, il faut égalmenet disposer du package ROS **webots_ros** :    
$>>$ sudo apt-get install ros-noetic-webots-ros

#### Téléchargement du simulateur : 
Télécharger le dossier $/SIMU$ au lien suivant : https://github.com/Intelligent-Systems-MSc/cva-su/tree/main/SimuWebots et placez le dans un répertoire de votre choix sur votre machine. Ce dossier contient tous les fichier nécéssaires à la compilation du simulateur. 

## II - Installation - AVEC Docker 
Si vous n'avez pas accès à une machine équipée d'Ubuntu 20 et de ROS noetic, nous avons mis en place une image **Docker** qui vous permettra tout de même d'utiliser le simulateur. Cette image n'a été testée que sous Linux et est disponible au lien suivant : https://github.com/Teiwin/ros-webots-docker

## III - Utilisation du simulateur : 
#### Démarrage : 
Dans un premier temps, il est **impératif** de lancer un ROS Master. Sans celui-ci, le véhicule simulé ne pourra pas souscrire ou publier dans des topics ROS ($>>$ roscore).  Pour lancer **webots**, il faut alors utiliser la commande : $>> webots$.  
  
Depuis la fenêtre principale de WeBots, aller dans File puis Open World  et sélectionner, dans le dossier précédemment décompréssé, le fichier simu_vx.wbt (dossier worlds de l'archive décompressée).  La simulation devrait s'ouvrir er ressembler à l'image ci-dessous. Un second environnement avec un circuit plus complexe est également disponible.   

<img src="https://github.com/Intelligent-Systems-MSc/cva-su/blob/main/SimuWebots/SIMU.png" width="600">  
  
Pour lancer la simulation, il faut alors cliquer sur le bouton  play en haut de la fenêtre de visualisation graphique (Pour vérifier si le simulateur est bien en route, un indicateur temporel est situé à gauche du bouton play. Si cet indicateur affiche un chronomètre, alors la simulation est bien en cours de fonctionnement.). Si besoin, la simulation peut être arrétée et relancée avec les boutons pause et restart, voisins au bouton play. 

#### Utilisation : 
Le simulateur est implémenté de sorte que le robot simulé soit le plus proche possible du robot réel, que ce soit en termes d'archtecture électronique (capteurs), physique (poids, inertie...) ou d'implémentation (interfaçage avec ROS). Les algorithmes réalisés pour le calcul des trajectoire de ce robot devront être facilement compréhensibles (commentaires, documentation, codes lisibles), légers en consommation CPU et surtout paramétrables (l'environnement physique réel du robot pourra être bien différent de son environnement simulé : les paramètres utilisé dans les algorithmes simulé seront donc amenés à évoluer une fois dans le monde réel).    
  
Le robot publie, à différentes fréquences, dans trois topics ROS : 
- **/LidarScan** : ici, le robot simulé publie un vecteur (Float32MultiArray) de valeurs correspondants aux distances captées par le lidar à chacun des angles de mesures. 
- **/ImageScan** : Dans ce topic, le robot publie des images RGBA issues de sa caméra.
- **/SensorsScan** : Le robot simulé est équipé de 4 capteurs TOFs disposés à l'avant et à l'arrière (à gauche et à droite) du véhicule. Le robot publie les données issues de ces deux capteurs dans le topic /SensorsScan sous forme d'un vecteur de deux valeurs, chacune de ces deux valeurs correspondant aux distances perçues par les deux capteurs.  
  
Le robot souscrit à deux topics qui lui permettent de lire des valeurs de commande des roues :
- **/SpeedCommand**  : Ce topic attend des valeurs en Float32 comprises entre -1 et 1. Il permet de commander la vitesse de déplacement du véhicule (1 : vitesse max en marche avant, 0 : arrrêt et 1 : vitesse max en marche arrière).  

- **/AngleCommand**  : Ce topic attend des valeurs en Float32 comprises entre -1 et 1. Il permet de régler la direction de circulation du véhicule (1 : aller complètement à droite, -1 à gauche, 0 au centre).  
  
Des exemples d'utilisation de ces topics sont disponibles dans le package ROS d'exemple (SIMU_webots) : par exemple, le fichier teleop_robot.py permet de télé-opérer le véhicule grâce aux flèches du clavier (+ une petite interface graphique).   
  
Pour toute question ou remarque concernant ce simulateur, ne pas hésiter à me contacter (par exemple avec une *issue*  GitHub afin que tout le monde puisse profiter de ces remarques !). 
