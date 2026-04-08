# GoPiGo3 Vision Sorter

## Description
GoPiGo3 Vision Sorter est un projet de robotique basé sur un robot GoPiGo3 équipé d’une caméra Raspberry Pi.
Le robot détecte des cubes colorés en temps réel grâce à OpenCV et publie les résultats via ROS2.

Une interface web permet de visualiser le flux vidéo et de sélectionner dynamiquement la couleur à détecter.

---

## Fonctionnalités
- Détection de cubes par couleur : rouge, vert, bleu, jaune
- Affichage du flux caméra en temps réel via interface web
- Sélection dynamique de la couleur à détecter
- Publication ROS2 (`cube_detection`)
- Architecture modulaire (vision + contrôle robot)

---

## Prérequis
- Raspberry Pi (Raspberry Pi OS ou Ubuntu)
- Robot GoPiGo3
- Python 3.8+
- OpenCV (`opencv-python`)
- ROS2 (Humble / Rolling recommandé)
- Git

---

## Installation

### 1. Cloner le projet
```bash
cd ~/ROS2_WS/src
git clone https://github.com/Math-Baba/GoPiGo3_Vision_Sorter.git 
```

### 2. Installer les dépendances
```bash
cd ~/ROS2_WS/src/robot_controller
pip install opencv-python numpy
```

### 3. Build ROS2
```bash
cd ~/ROS2_WS
colcon build
source install/setup.bash
```

---

## Lancer le projet

### 1. Lancer la caméra + serveur web
```bash
ros2 run robot_controller camera_node
```

### 2. Accéder à l’interface web
Dans ton navigateur :
```
http://<IP_DU_RASPBERRY_PI>:8080/
```
> Trouver ton IP :
```bash
hostname -I
```

### 3. Sélection de couleur
Via URL :
```
http://<IP>:8080/set_color?color=red
http://<IP>:8080/set_color?color=green
http://<IP>:8080/set_color?color=blue
http://<IP>:8080/set_color?color=yellow
```

---

## Structure du projet
```
robot_controller/
├── camera_node.py        # Caméra + détection + serveur web
├── motion_node.py        # Contrôle du robot
└── web/
    ├── index.html        # Interface utilisateur
    ├── style.css         # Style
    └── app.js            # Logique frontend
```

---

## Fonctionnement
La caméra capture des images en continu.
OpenCV détecte les objets selon la couleur sélectionnée.
Les informations sont publiées sur ROS2 :
- `color`
- `detected`
- `cx` (position horizontale)
- `frame_width`

Le flux vidéo est envoyé au frontend via HTTP.
L’utilisateur peut changer la couleur via l’interface web.

---
