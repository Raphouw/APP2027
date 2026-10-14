# Navigation fluide, obstacles nouveaux et caméra

## Pourquoi le robot faisait « avance / arrête »

Deux causes, mesurées en simulation (1 % de rayons LiDAR invalides, comme un vrai LDS) :

1. **Un seul rayon invalide arrêtait tout.** Les rayons à 0,0 m (fréquents sur un LDS) rendaient la « zone avant inconnue » et coupaient les commandes jusqu'au scan suivant, que ce soit en navigation sur carte ou en exploration. `lidar_filter.tolerate_front_gaps` comble désormais jusqu'à 3 rayons consécutifs (4 au total) dans le cône avant, avec la distance du voisin le plus proche : un obstacle ne peut pas être masqué. Au-delà, le robot s'arrête toujours.
2. **Le chemin était recalculé en bloquant le robot.** Chaque variation des échos LiDAR déclenchait un A* synchrone, précédé d'une commande nulle, toutes les 0,5 s. Hypothèse de départ confirmée, avec une précision : ce n'était pas le calcul qui était long, c'était le recalcul permanent sans raison.

Mesure (navigation, 1 % de rayons invalides, même but) : **183 arrêts/reprises en 60 s et 1,7 m parcourus avant ; 0 arrêt et but atteint en 48 s après.** Exploration : 77 arrêts/reprises avant (voir la section Exploration ci-dessous pour l'après).

## Ce qui a changé dans la navigation sur carte

- Le chemin est calculé dans un fil de travail ; la boucle de contrôle continue de suivre l'ancien chemin. Plus aucune commande nulle pendant un calcul.
- Un nouveau calcul n'est lancé que si le chemin restant (1,5 m devant) traverse une cellule occupée, si aucun chemin n'existait (nouvel essai par seconde) ou sur nouvelle destination. Au plus un calcul toutes les 0,4 s.
- La poursuite vise un point à 0,30 m sur le chemin (au lieu d'une cellule de 5 cm) : le cap varie doucement. Les vitesses passent par des rampes (accélération 0,45 m/s², freinage 1,2 m/s²), ralentissent dans les virages, à l'approche du but et près d'un obstacle (arrêt seulement sous 0,30 m).
- Réglages dans `slam_components/settings.py`.

## Chemins plus directs, destinations moins refusées

- A* 8-connexe, coût diagonal √2, heuristique octile exacte, puis **lissage par lignes de vue** : une diagonale libre est suivie en une seule ligne droite. Un raccourci n'est accepté que s'il n'est pas plus coûteux que le détour (on ne rase pas un mur).
- Marge dure autour des murs réduite de 0,20 m à 0,14 m (le Burger fait 0,105 m de rayon), plus une **pénalité douce jusqu'à 0,30 m** : le chemin reste au milieu des couloirs mais passe encore dans les passages étroits.
- Une destination dans la zone de marge est ramenée à la cellule libre la plus proche (≤ 0,60 m) au lieu d'être refusée ; le robot posé contre un mur est traité de même (≤ 0,40 m). Une destination hors carte ou entourée d'obstacles reste refusée (`NO_PATH`).

## Obstacles nouveaux

Les murs de la carte restent noirs. Les échos qui n'ont pas de mur correspondant sur la carte sont des **obstacles nouveaux** :

1. les échos sont d'abord décalés (±0,4 m) pour coller au mieux aux murs de la carte, car l'odométrie dérive ;
2. un écho est « nouveau » s'il reste à plus de 0,12 m de tout mur ou zone inconnue de la carte, et s'il se répète au scan suivant ;
3. il bloque la planification 1,5 s après le dernier écho, et reste affiché **en rose** 8 s (carte de l'atelier et fenêtre `map_view`).

Simulation : une boîte absente de la carte est détectée (24 à 31 cellules roses) et contournée. Avec une odométrie deux fois plus mauvaise que la normale, il reste ~12 faux positifs en 90 s (223 sans le recalage). Une dérive très forte finit par produire de faux obstacles : la relocalisation visuelle reste désactivée.

## Destination « indisponible »

Le clic est refusé tant que la position du robot n'est pas connue. Le message dit maintenant pourquoi et quoi faire (replacer le robot, calibration IMU, capteur en attente) et l'outil **Replacer le robot** est sélectionné automatiquement. Après « Ouvrir une session », la navigation démarre avec cet outil actif : cliquez là où se trouve le robot (glissez pour son orientation), puis choisissez une destination. Parcours vérifié en simulation : carte → pose → destination → arrivée.

## Caméra

Le serveur `serveur_camera.py` doit tourner **sur le robot** : il n'était lancé par rien (connexion refusée sur le port 9999). Dans l'onglet **Robot (SSH)** : *Connecter*, puis **Lancer la caméra**. L'atelier copie le script sur le robot (`~/atelier_slam/`), le lance avec ROS chargé et le même `ROS_DOMAIN_ID`, et pré-remplit l'adresse de l'onglet **Caméra**, qui réessaie jusqu'à la connexion. **Arrêter la caméra** envoie Ctrl-C.

Le serveur prend le topic ROS `/camera/image_raw` s'il publie (5 s d'attente), sinon une caméra V4L2 (`/dev/video0`, demande `python3-opencv` sur le robot). Il redimensionne en 640×480 et n'envoie chaque image qu'une fois. Un TurtleBot3 Burger n'a pas de caméra : il faut en brancher une (USB) ou lancer un nœud caméra ; sinon le message `[CAMERA] ERREUR` s'affiche dans la fenêtre de l'onglet Robot.

Vérifié : image ROS simulée → TCP → pixels corrects ; capture V4L2 locale ; lancement SSH testé avec un faux robot. **Non vérifié sur un vrai TurtleBot3.**
