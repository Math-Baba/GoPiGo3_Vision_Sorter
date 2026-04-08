# GoPiGo3 Vision Sorter

**Description**
GoPiGo3 Vision Sorter est un projet pour contrôler un robot GoPiGo3 équipé d’une caméra Raspberry Pi. Le robot détecte et trie des cubes colorés en utilisant la vision par ordinateur avec OpenCV et ROS2. Il inclut un serveur web pour visualiser la caméra et sélectionner la couleur des cubes à détecter.

---

## Fonctionnalités

- Détection de cubes par couleur : rouge, vert, bleu, jaune.
- Affichage en temps réel via un serveur web (HTML + JS + CSS).
- Publication ROS2 du statut des cubes détectés (`cube_detection` topic).
- Contrôle et stratégie de tri avec modules séparés (`cube_strategy`, `motion_node`).
- Calibration HSV avec interface simple pour ajuster les couleurs.
- Support pour GoPiGo3 et Raspberry Pi avec ROS2.


---

## Prérequis

- Raspberry Pi avec Raspberry Pi OS ou Ubuntu.
- Robot GoPiGo3.
- Python 3.8+ et OpenCV.
- ROS2 (ex: Humble ou Rolling).
- Git et pip installés.
- Accès à Internet pour les dépendances.

---

## Installation

1. Cloner le projet :
   ```bash
   cd ~/ROS2_WS/src
   git clone https://github.com/Math-Baba/GoPiGo3_Vision_Sorter.git 
   ```

2. Installer les dépendances Python :
   ```bash
   cd ~/ROS2_WS/src/robot_controller
   pip install -r requirements.txt
   ```
   Si `requirements.txt` n’existe pas, installe au moins :
   ```bash
   pip install opencv-python numpy
   ```

3. Construire le workspace ROS2 :
   ```bash
   cd ~/ROS2_WS
   colcon build
   source install/setup.bash
   ```

4. Vérifier la structure du projet :
   ```
   robot_controller/
   ├── __init__.py
   ├── arena_map.py
   ├── camera_node.py
   ├── cube_detector.py
   ├── cube_strategy.py
   ├── follow_test.py
   ├── gopigo3_driver.py
   ├── hsv_calibration.py
   ├── motion_node.py
   └── web/
       ├── index.html
       ├── style.css
       └── app.js
   ```

---

## Lancer le projet

1. Lancer ROS2 et le serveur web :
   ```bash
   ros2 run robot_controller camera_node
   ```
   Le serveur web sera accessible sur `http://<IP_DU_PI>:8080/`.
   Sélection de couleur via `http://<IP_DU_PI>:8080/set_color?color=red` (ou `green`/`blue`/`yellow`).

2. Lancer le contrôle du robot :
   ```bash
   ros2 run robot_controller motion_node
   ros2 run robot_controller cube_strategy
   ```
   Ces modules utilisent les messages `CubeDetection` pour décider des mouvements.

---

## Structure des fichiers

- `camera_node.py` : capture caméra, détection couleur et serveur web.
- `cube_detector.py` : fonctions de détection avancées (si nécessaire).
- `cube_strategy.py` : logique de tri et décisions du robot.
- `motion_node.py` : commandes moteur GoPiGo3.
- `hsv_calibration.py` : calibration couleur HSV.
- `web/` : interface web (`index.html`, `style.css`, `app.js`).