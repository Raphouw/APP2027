> Mise à jour du 7 octobre 2026 : l’interface et le moteur SLAM ont évolué. Consulter le [lancement simplifié et le diagnostic](../LANCEMENT_ET_DIAGNOSTIC.md), le [guide Atelier SLAM](../GUIDE_ATELIER_SLAM.md), [l’algorithme et les mesures](../SLAM_ALGORITHME_ET_MESURES.md) et [l’architecture actuelle](../ARCHITECTURE_CODE.md). Le résumé ci-dessous conserve son rôle de description historique.

# Synthèse détaillée du projet TurtleBot3 : cartographie, localisation et navigation

Document de reprise — état du code examiné le **7 octobre 2026**.

> **Mise à jour :** cette synthèse conserve l’état initial avant découpage. Les quatre scripts principaux ont ensuite été répartis par responsabilité dans `mapping/slam_components/`. Voir [l’architecture actuelle](../ARCHITECTURE_CODE.md) pour les nouveaux fichiers et les tests. Les priorités ont ensuite été corrigées : voir [les améliorations appliquées et leur validation](../AMELIORATIONS_ET_VALIDATION.md). Les constats ci-dessous décrivent uniquement l’état historique avant ces corrections.

Ce document décrit les fichiers présents dans `mapping/`, leur organisation, leurs échanges et leurs limites. Il s'appuie sur la lecture des dix scripts Python, de la configuration RViz, des métadonnées de carte, des 24 CSV et du contenu de l'archive ZIP. Il distingue le comportement effectivement programmé des intentions annoncées dans les commentaires.

**Périmètre de validation :** analyse statique et inventaire des données. La syntaxe des dix scripts a été vérifiée sans les exécuter. Aucun déplacement de robot, lancement de contrôleur, essai ROS en réseau ou validation physique n'a été effectué. Les problèmes signalés ci-dessous sont des constats de lecture ou des conséquences attendues du code, pas des incidents reproduits sur le robot.

## Sommaire

1. [Vue d'ensemble](#1-vue-densemble)
2. [Organisation des fichiers](#2-organisation-des-fichiers)
3. [Architecture et circulation des données](#3-architecture-et-circulation-des-données)
4. [Interface de gestion](#4-interface-de-gestion--managerpy)
5. [Odométrie et navigation réactive](#5-odométrie-et-navigation-réactive--tb3_nav_nodepy)
6. [Construction de carte](#6-construction-de-carte--carto_matpy)
7. [Navigation sur carte](#7-navigation-sur-carte--nav_odompy)
8. [Planificateur D* Lite](#8-planificateur--dstar_litepy)
9. [Vision et reconnaissance de lieux](#9-vision-et-reconnaissance-de-lieux)
10. [Vidéo](#10-vidéo)
11. [Données sauvegardées](#11-données-sauvegardées)
12. [Environnement et commandes](#12-environnement-et-commandes)
13. [État fonctionnel et incohérences](#13-état-fonctionnel-et-incohérences)
14. [Proposition de reprise](#14-proposition-de-reprise-pour-la-nouvelle-année)
15. [Glossaire](#15-glossaire)
16. [Repères de lecture](#16-repères-de-lecture-du-code)

## 1. Vue d'ensemble

Le projet vise un **TurtleBot3 Burger** équipé de roues différentielles, d'encodeurs, d'une IMU, d'un LiDAR et éventuellement d'une caméra. Il utilise **Python et ROS 2** pour acquérir les capteurs, estimer une pose, commander le robot, reconstruire une carte 2D et afficher les résultats.

Il contient cinq ensembles principaux :

- un nœud d'odométrie et de navigation réactive, avec évitement d'obstacles et journalisation ;
- un cartographe qui transforme les scans LiDAR en grille d'occupation, en direct ou depuis des CSV ;
- un navigateur sur carte sauvegardée, avec planification et suivi de points ;
- une interface Tkinter qui lance les scripts et RViz comme processus séparés ;
- des extensions de vision : reconnaissance de lieux, tentative de relocalisation et retour vidéo.

Le code contient des briques utiles pour un projet de SLAM, mais **il ne constitue pas encore une chaîne SLAM globale cohérente et validée**. Il n'y a notamment pas d'optimisation de graphe de poses appliquée après reconnaissance d'une boucle. Le scan matching reste local et les différents consommateurs n'utilisent pas tous la même odométrie.

Deux navigations coexistent :

| Navigation | Entrée objectif | Pose utilisée | Décision de déplacement |
|---|---|---|---|
| `tb3_nav_node.py` | `/goal_pose`, notamment depuis RViz | Pose calculée à l'intérieur du nœud | Direction du but et passages libres du LiDAR |
| `nav_odom.py` | Clic dans une fenêtre Matplotlib | `/odom` avec offsets locaux | Chemin sur grille puis suivi de points |

Cette distinction est centrale : le second programme ne prolonge pas simplement le contrôleur du premier. Ce sont deux contrôleurs qui peuvent publier sur le même topic de commande.

## 2. Organisation des fichiers

```text
APP2027_V2/
├── RESUME_PROJET_SLAM.md       ← ce document de reprise
├── mapping/
│   ├── manager.py             ← interface et orchestration
│   ├── tb3_nav_node.py         ← odométrie, navigation réactive, CSV
│   ├── carto_mat.py            ← grille LiDAR live/offline et export
│   ├── nav_odom.py             ← navigation sur carte et interface graphique
│   ├── dstar_lite.py           ← recherche de chemin
│   ├── loop_closure.py         ← reconnaissance visuelle de revisites
│   ├── global_localizer.py     ← tentative de relocalisation SIFT
│   ├── ros_raw_stream.py       ← caméra ROS vers TCP brut
│   ├── video_stream.py         ← lecture TCP ou FFmpeg, vidéo de test
│   ├── test_manager.py         ← essai d'ouverture de l'interface
│   ├── tb3_auto_config.rviz    ← configuration de visualisation
│   ├── Code APP.zip           ← copie archivée des fichiers
│   ├── __pycache__/           ← fichiers Python compilés générés
│   └── logs/
│       ├── run_20260401_*.csv  ← 24 sessions enregistrées
│       ├── map_out.pgm        ← image de carte
│       ├── map_out.yaml       ← paramètres de la carte
│       ├── map_origin.txt     ← origine en mètres
│       └── last_pose.txt      ← dernière pose sauvegardée
└── dossiers de configuration/outillage locaux
```

Les dix scripts représentent **4 419 lignes**, commentaires et lignes vides compris. Avec les 44 lignes de configuration RViz, l'ensemble atteint 4 463 lignes.

| Fichier | Lignes | Responsabilité principale |
|---|---:|---|
| `tb3_nav_node.py` | 1 474 | Gros nœud réunissant estimation, perception, contrôle et sauvegarde |
| `manager.py` | 678 | Interface, démarrage et arrêt des processus |
| `carto_mat.py` | 566 | Cartographie probabiliste et export |
| `nav_odom.py` | 489 | Chargement de carte, interface et contrôle de suivi |
| `video_stream.py` | 302 | Réception vidéo asynchrone |
| `loop_closure.py` | 262 | Base d'images et détection de revisites |
| `dstar_lite.py` | 230 | Planification sur grille |
| `global_localizer.py` | 217 | Correspondances SIFT et homographie |
| `ros_raw_stream.py` | 157 | Serveur vidéo TCP |
| `test_manager.py` | 44 | Vérification manuelle de démarrage Tkinter |

L'organisation est celle d'un ensemble de scripts exécutés directement. Aucun `package.xml`, `setup.py`, `pyproject.toml`, manifeste de dépendances, fichier launch ROS ou suite de tests structurée n'a été trouvé dans les fichiers accessibles du projet. L'en-tête du nœud évoque un package ROS 2, mais ce packaging n'est pas présent dans ce dossier.

L'archive `Code APP.zip` contient 49 entrées, dont les logs et des caches. **Les dix scripts Python de l'archive sont identiques octet par octet aux scripts courants.** Elle ne fournit donc pas une variante du code permettant de résoudre les incohérences actuelles.

## 3. Architecture et circulation des données

### 3.1 Vue logique

```text
Robot / pilotes ROS externes
  ├─ /joint_states ───┐
  ├─ /imu ────────────┼─> tb3_nav_node ──> /my_odom + TF + /path
  ├─ /scan ───────────┤          ├──────> /cmd_vel
  └─ /camera/image_raw┘          └──────> buffers puis CSV à l'arrêt

Source externe de /odom
  ├─ /odom + /scan ─────> carto_mat live ──> grille + export PGM/YAML
  └─ /odom + carte ─────> nav_odom ────────> /cmd_vel

CSV sauvegardé ─────────> carto_mat offline ──> PGM/YAML
PGM/YAML ───────────────> nav_odom ──> DStarLite ──> chemin

/camera/image_raw ──────> ros_raw_stream ──> TCP:9999 ──> video_stream
manager ───────────────> lancement de scripts, RViz et affichage vidéo
```

**Point de raccordement à corriger :** le nœud maison publie `/my_odom`, alors que le cartographe live et le navigateur sur carte écoutent `/odom`. Sans source externe ou remappage approprié, ils ne reçoivent pas la pose du nœud maison. En offline, le CSV contient bien la pose interne de ce nœud, ce qui crée aussi une différence entre les sources de pose live et offline.

### 3.2 Topics présents

| Topic | Type | Producteur dans le projet | Consommateurs dans le projet |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Aucun : pilote externe attendu | `tb3_nav_node` |
| `/imu` | `sensor_msgs/Imu` | Aucun : pilote externe attendu | `tb3_nav_node` |
| `/scan` | `sensor_msgs/LaserScan` | Aucun : pilote externe attendu | Nœud principal, cartographe live, navigateur sur carte |
| `/camera/image_raw` | `sensor_msgs/Image` | Aucun : caméra externe attendue | Nœud principal, navigateur, serveur vidéo |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Outil SetGoal de RViz ou source externe | `tb3_nav_node` |
| `/my_odom` | `nav_msgs/Odometry` | `tb3_nav_node` | RViz configuré |
| `/odom` | `nav_msgs/Odometry` | Aucun dans ces scripts | Cartographe live et `nav_odom` |
| `/cmd_vel` | `geometry_msgs/Twist` | Les deux contrôleurs | Base mobile externe |
| `/path` | `nav_msgs/Path` | `tb3_nav_node` | RViz configuré |
| `/nav_state` | `std_msgs/String` | `tb3_nav_node` | Aucun abonné trouvé ici |
| `/selected_gap_angle` | `std_msgs/Float32` | `tb3_nav_node` | Aucun abonné trouvé ici |
| `/loop_state` | `std_msgs/Bool` | `tb3_nav_node` | Aucun abonné trouvé ici |
| `/loop_repulsion_angle` | `std_msgs/Float32` | `tb3_nav_node` | Aucun abonné trouvé ici |

Les commentaires mentionnent parfois un `live_mapper`, mais aucun fichier correspondant n'est fourni.

### 3.3 Repères et unités

Les positions physiques sont exprimées en mètres, les orientations internes en radians et les vitesses en m/s ou rad/s. Certaines sorties console convertissent l'orientation en degrés.

Le nœud principal publie réellement la transformation **`my_odom → base_link`**. Plusieurs commentaires annoncent encore `odom → base_footprint` : ils ne correspondent plus aux noms exécutés.

RViz utilise `my_odom` comme repère fixe. Le projet ne fournit pas toute la chaîne de transformations entre la base, le LiDAR et la caméra. Les pilotes ou la description du robot doivent compléter cette chaîne. Le callback d'objectif ne transforme pas le repère du message reçu : il prend directement ses coordonnées x/y.

La carte utilise des indices de matrice, avec 10 cellules par mètre par défaut. Les conventions lignes/colonnes, axes physiques et retournement vertical de l'image doivent être harmonisées avant de considérer l'export comme fiable.

## 4. Interface de gestion — `manager.py`

La classe `TB3Manager` construit une fenêtre Tkinter de 1 000 × 700 pixels. Elle gère quatre références de processus : nœud principal, RViz, cartographe et navigateur sur carte. Elle définit son dossier de travail à partir de l'emplacement du script et crée `logs/` si nécessaire.

### 4.1 Mode Normal

Le parcours comporte trois boutons :

1. **Démarrer Mesures & RViz** : lance `tb3_nav_node.py` et RViz avec la configuration locale. Le robot doit rester immobile pendant la calibration IMU. Le message propose d'attendre 30 secondes, mais le nœud attend réellement 200 échantillons IMU ; la durée dépend donc de la fréquence du capteur.
2. **Arrêter & Générer Carte** : envoie SIGINT au nœud, attend son arrêt et ferme RViz. Sélectionne le CSV le plus récemment modifié, puis lance le cartographe offline avec `--auto --speed 3.0`.
3. **Lancer Navigation Odométrie** : cherche `logs/map_out.yaml`, relance le nœud principal s'il est arrêté, puis lance `nav_odom.py` avec cette carte.

Le démarrage de la phase de mesures ne crée pas d'objectif tout seul. Le nœud reste en `IDLE` jusqu'à réception d'un objectif, par exemple avec l'outil RViz.

Le mode Normal sélectionne le dernier CSV du dossier, sans garantir qu'il appartient au processus tout juste arrêté. Un échec de sauvegarde peut donc conduire à reconstruire une ancienne session.

### 4.2 Mode Live

Ce mode affiche une vidéo optionnelle, une entrée d'URL, un bouton vidéo de test et des commandes de cartographie.

Le démarrage lance `tb3_nav_node.py` et `carto_mat.py -m live --speed 3.0`. **Le cartographe ainsi lancé reste soumis à son menu console** : le gestionnaire ne lui transmet pas le choix « lancer la cartographie ». Aucun lancement RViz n'est programmé dans cette méthode.

L'arrêt termine les deux processus et arrête le lecteur vidéo. Il ne déclenche pas explicitement la sauvegarde de la carte live.

### 4.3 Mode Auto

Le gestionnaire lance le nœud principal, RViz et le cartographe live. Il surveille ensuite la fin du processus cartographe toutes les secondes. Lorsque ce processus se termine, il cherche un YAML puis lance `nav_odom.py`.

Les limites sont importantes :

- aucun critère de couverture ou de fin d'exploration n'est calculé ;
- la fin du processus est assimilée à une cartographie terminée, sans contrôle de son code de sortie ;
- le lancement du cartographe reste interactif ;
- aucune sauvegarde live automatique n'est demandée ;
- un ancien YAML peut être repris ;
- aucun but de navigation n'est envoyé au navigateur sur carte : il faut cliquer dans sa fenêtre.

Le terme « Auto » désigne donc une intention d'enchaînement de programmes, pas une exploration complète suivie d'une mission autonome déjà implémentée.

### 4.4 Processus et fermeture

Le gestionnaire utilise `subprocess.Popen`, `run`, `wait` et des signaux. Plusieurs méthodes essaient d'arrêter les anciennes instances avant de relancer. À la fermeture, il arrête les processus connus et la vidéo, puis détruit Tkinter.

Des attentes synchrones et `time.sleep()` sont exécutés sur le fil de l'interface ; la fenêtre peut rester bloquée pendant ces opérations. `stop_and_generate()` attend le nœud sans délai maximal. Les références ne sont pas toujours remises à zéro et les protections contre les lancements multiples varient selon le mode.

La fermeture est enregistrée à la fois sur les signaux, le bouton de fenêtre et `atexit`. Elle gagnerait à devenir idempotente, pour éviter une deuxième destruction de widgets déjà fermés.

## 5. Odométrie et navigation réactive — `tb3_nav_node.py`

La classe `TurtleBot3NavNode` concentre de nombreuses responsabilités. Elle s'exécute avec une boucle de contrôle prévue à **50 Hz**, soit un pas `dt = 0.02 s`.

### 5.1 Initialisation et pose sauvegardée

Le nœud part normalement de `(0, 0, 0)`. Le paramètre ROS `use_saved_pose` vaut `False` par défaut. Quand il vaut `True`, le nœud tente de lire `logs/last_pose.txt`.

Le gestionnaire ne passe pas ce paramètre lors de ses relances. Son commentaire « relancer pour fournir l'odométrie » ne signifie donc pas que la pose précédente est restaurée.

### 5.2 Encodeurs et IMU

Le callback des roues recherche `wheel_left_joint` et `wheel_right_joint`. Il calcule les variations d'angles en les ramenant dans l'intervalle autour de ±π, puis déduit les déplacements des roues et les vitesses linéaire et angulaire.

Les constantes physiques sont : rayon de roue 0,033 m, entraxe 0,160 m et rayon de robot 0,105 m.

L'IMU fournit la vitesse de rotation autour de z. Les 200 premiers échantillons servent à estimer un biais moyen, avec le robot supposé immobile. Ensuite, le biais est soustrait et une zone morte de 0,012 rad/s annule les petites valeurs.

La vitesse angulaire fusionnée est :

```text
fused_w = 0.98 × gyro_corrigé + 0.02 × vitesse_angulaire_encodeurs
```

Cette pondération ne supprime pas à elle seule la dérive à long terme du gyroscope. Les différences de roues sont divisées par le `dt` fixe du contrôleur, plutôt que par l'intervalle réel entre messages encodeurs.

### 5.3 Intégration de pose : point essentiel

La méthode RK4 intègre le modèle différentiel :

```text
dx/dt = v × cos(theta)
dy/dt = v × sin(theta)
dtheta/dt = w
```

**Cependant, la boucle transmet `prev_v`, c'est-à-dire la vitesse commandée précédente, et non `enc_v`, à l'intégration de translation.** L'estimation de position avance donc selon la commande, même si le déplacement réel diffère. Le cap utilise `fused_w` lorsqu'il dépasse la zone morte, sinon la commande angulaire précédente.

RK4 améliore l'intégration numérique du modèle, mais ne corrige pas le fait que sa vitesse d'entrée peut différer de la vitesse réelle du robot.

Le nœud publie `/my_odom`, la TF `my_odom → base_link` et une trajectoire `/path` enrichie lorsque le déplacement dépasse 5 cm. Cette trajectoire représente l'historique estimé, pas un chemin global planifié.

### 5.4 Machine à états

| État | Fonction |
|---|---|
| `IDLE` | Attend un objectif, publie une commande tendant vers zéro |
| `MOVE_TO_GOAL` | Oriente le robot vers le but et adapte l'avance |
| `FOLLOW_GAP` | Suit une direction libre détectée dans le scan |
| `GOAL_REACHED` | Signale l'arrivée et revient à une commande d'arrêt |

Un objectif `/goal_pose` réinitialise les PID, certains historiques, le verrou de côté et la trajectoire. Seule sa position x/y est exploitée ; l'orientation finale souhaitée n'est pas suivie.

L'arrivée est déclarée à moins de **0,12 m** du but.

### 5.5 Régulation et vitesse

La classe PID comprend un terme proportionnel, une intégrale limitée, une dérivée filtrée, une saturation de sortie et une remise à zéro.

- PID de cap : `kp=1.4`, `ki=0.03`, `kd=0.12`.
- PID d'évitement : `kp=0.8`, `ki=0.1`, `kd=0.12`.
- PID linéaire : instancié, mais la vitesse est finalement déterminée par une loi directe.

Plusieurs commentaires affirment que l'intégrale d'évitement est nulle ; la valeur réellement configurée est **0,1**.

La vitesse linéaire maximale vaut 0,16 m/s et la vitesse angulaire maximale 1 rad/s. Les rampes limitent l'accélération à 0,18 m/s², la décélération à 0,55 m/s² et les changements de vitesse angulaire à 2,5 rad/s².

En approche du but, l'avance diminue à partir de 0,40 m, avec un plancher de 0,04 m/s. Un grand écart de cap réduit aussi la vitesse, sans imposer systématiquement une rotation sur place.

### 5.6 Perception LiDAR et passages libres

Le callback LiDAR remplace les mesures NaN, infinies ou hors limites par la portée maximale. Il produit ensuite une version « gonflée » des obstacles : les rayons voisins sont raccourcis dans un cône dépendant du rayon du robot.

Le contrôleur mesure le dégagement devant lui et dans la direction de l'objectif. Les seuils principaux sont 0,50 m pour commencer à éviter et 0,20 m pour demander l'arrêt de l'avance en `FOLLOW_GAP`.

Les décisions d'obstacle passent par une fenêtre de huit votes. La bascule vers l'évitement utilise un ratio de 0,65 ; le retour vers le but utilise 0,25 et une durée minimale d'évitement de 1,2 s. Un retour anticipé est possible près du but si le dégagement frontal dépasse le seuil d'arrêt.

Les passages candidats sont des suites de rayons suffisamment libres. Ils sont évalués selon leur largeur, leur profondeur, leur direction par rapport au but, la quantité de virage et un score prédictif. La largeur minimale est de **35 indices LiDAR**, pas nécessairement 35 degrés : cela dépend du pas angulaire du capteur.

Le score prédictif considère une position virtuelle un mètre plus loin, tout en utilisant le scan courant comme approximation de dégagement. Il ne simule pas une carte complète depuis cette position.

Le choix du passage est lissé par moyenne exponentielle. Un verrou de côté de 2,5 s limite les changements de direction ; il peut être libéré si le passage disparaît pendant six cycles ou si le but devient accessible.

### 5.7 Détection de confinement et vision

Le mécanisme anti-boucle de navigation conserve 50 positions espacées de 0,1 s. Si au moins 80 % restent dans un rayon de 0,30 m autour du centre moyen, il peut déclencher une répulsion qui éloigne le cap de ce centre, avec un délai de réarmement de 4 s.

Ce mécanisme détecte un robot qui progresse peu. Il est distinct de la reconnaissance visuelle d'un lieu déjà visité.

La caméra est convertie avec `cv_bridge`, puis transmise à `LoopClosureDetector`. Une revisite détectée est journalisée. **Aucune correction de pose ni réoptimisation de carte n'est appliquée dans ce callback.**

### 5.8 Journalisation et arrêt

Environ toutes les 0,10 s, le nœud ajoute en mémoire la pose, l'état, la distance frontale et un scan disponible. Le CSV est écrit lors de l'arrêt ; il ne s'agit pas d'un enregistrement continu sur disque.

L'arrêt tente également de publier une vitesse nulle et de sauver la dernière pose. La logique de sauvegarde est dupliquée entre `cleanup_handler()` et `finally`, et enregistrée avec `atexit`. Elle peut donc être appelée plusieurs fois. Une interruption brutale peut perdre les buffers en mémoire.

## 6. Construction de carte — `carto_mat.py`

La classe `Cartographer`, nommée `lidar_visualizer` côté ROS, gère une grille d'occupation. Le même programme propose un mode live et un mode offline.

### 6.1 Représentation

La grille initiale mesure **200 × 200 cellules**. Avec 10 cellules par mètre, elle représente nominalement 20 × 20 m, à une résolution de 0,10 m/cellule. Le robot est initialement placé au centre.

Chaque cellule conserve un log-odds : zéro correspond à une probabilité de 0,5, donc à l'inconnu. Une observation occupée ajoute environ +0,847 ; une observation libre ajoute environ −0,847.

La probabilité affichée est calculée par :

```text
p = 1 / (1 + exp(-log_odds))
```

Les log-odds sont bornés entre −3 et +3. L'affichage associe des couleurs au libre, à l'inconnu et à l'occupé.

### 6.2 Traitement d'un scan

Le pipeline est le suivant :

1. conversion des distances polaires en points cartésiens ;
2. conservation des points à moins de 3,5 m ;
3. tentative d'alignement local sur les cellules déjà occupées ;
4. transformation homogène rotation/translation vers la grille ;
5. recherche des cellules traversées par les rayons ;
6. mise à jour des cellules libres et des extrémités occupées ;
7. oubli conditionnel, saturation et conversion en probabilités ;
8. actualisation de la carte et de l'historique de trajectoire.

La projection utilise une matrice homogène 3 × 3. Le tracé des cellules traversées est vectorisé avec NumPy pour traiter plusieurs rayons simultanément.

Les angles sont reconstruits avec `linspace(angle_min, angle_max, nombre_de_mesures)`. Les valeurs infinies sont transformées en zéro, tandis que les NaN ne font pas l'objet d'un traitement explicite. Une distance zéro peut finir projetée sur le robot au lieu de représenter une absence de retour.

### 6.3 Scan matching

Pour les points projetés, le code cherche des cellules occupées dans un voisinage de rayon deux pixels. Il calcule ensuite la médiane des décalages et corrige la translation si suffisamment de correspondances existent.

Ce procédé ne corrige pas l'orientation et ne réalise pas une recherche explicite du voisin le plus proche. La pose est de nouveau fixée par les messages `/odom` en live ou par chaque ligne du CSV en offline : la correction n'est pas conservée comme un offset global durable.

### 6.4 Oubli et extension

Le code compare les probabilités à leur état précédent. Si l'écart dépasse 0,2, un compteur augmente. À partir de cinq changements, le log-odds de la cellule est multiplié par 0,95. Ce mécanisme cherche à limiter la persistance de zones changeantes.

Les paramètres `decay`, `log_decay` et les timestamps de cellule ne pilotent pas effectivement une décroissance temporelle générale dans la mise à jour actuelle.

L'extension ajoute 50 cellules sur chaque côté. Toutefois, `grow_map()` n'agrandit pas `cell_change_count` et `cell_prev_state` ; la comparaison suivante de tableaux peut échouer par incompatibilité de dimensions. Le déclenchement considère surtout les dépassements positifs, et le recentrage de la pose et des points déjà calculés n'est pas traité de façon cohérente.

### 6.5 Live et offline

En live, le programme écoute `/scan` et `/odom`. Un verrou protège le dernier scan reçu ; la boucle graphique consomme ce scan et un thread fait tourner ROS. Il n'y a pas de synchronisation temporelle précise entre scan et pose.

En offline, chaque ligne fournit la pose et les distances. L'affichage est rafraîchi tous les vingt scans. Mais le calcul des indices ajoute une origine fixe de −10 m en plus du centre de grille :

```text
live    : indice_x = world_x × scale + x0
offline : indice_x = (world_x - (-10)) × scale + x0
```

Pour une grille initiale de 200 cellules, une position physique x=0 donne ainsi l'indice 100 en live et 200 en offline. **Les deux modes n'utilisent donc pas la même conversion.**

`--speed` réduit surtout les pauses d'affichage ; il ne multiplie pas la vitesse physique du robot. `--auto` supprime le menu seulement en offline.

### 6.6 Export

La sauvegarde écrit dans `./logs/` :

- `map_<nom>.pgm` : image de probabilités ;
- `map_<nom>.yaml` : résolution, origine, nom de l'image et seuils ;
- `map_origin.txt` : origine x/y.

Le nom par défaut `out` produit `map_out.pgm` et `map_out.yaml`.

**Incohérence majeure :** l'image est enregistrée avec `p × 255`, donc une occupation élevée devient claire. `nav_odom.py` relit l'occupation comme `(255 - pixel) / 255`, donc il considère plutôt les pixels sombres comme occupés. Le YAML annonce `negate: 0`. L'export et la lecture inversent ainsi le sens de l'occupation.

De plus, le cartographe indexe la matrice sous la forme `[x, y]`, tandis que le navigateur traite les tableaux comme `[ligne_y, colonne_x]` et retourne l'image verticalement. Ces conventions doivent être vérifiées ensemble, avec une petite carte de référence asymétrique.

## 7. Navigation sur carte — `nav_odom.py`

Ce programme prend le chemin d'un YAML en argument. Il utilise deux processus : un processus graphique Matplotlib et un nœud ROS de contrôle. Un dictionnaire partagé via `multiprocessing.Manager` transporte la pose, le chemin, les clics et les commandes d'interface.

### 7.1 Interface

La fenêtre affiche la carte, la pose et l'orientation du robot, des points LiDAR, le chemin calculé et le but.

- En mode normal, un clic fixe une destination.
- `SET POSE` permet de définir une position ; un glissement de plus de 0,2 m dans le repère affiché donne aussi un angle.
- `STOP` vide le chemin et remet le contrôleur au repos via l'état partagé.

Le bouton STOP est une commande logicielle d'interface ; il ne remplace pas un arrêt physique ni un arbitre de commandes entre les deux contrôleurs.

### 7.2 Chargement et grille

Le programme lit la résolution et l'origine du YAML, charge le PGM en niveaux de gris, puis le retourne verticalement. Les obstacles sont détectés avec une probabilité supérieure à 0,60 et dilatés par un noyau 3 × 3.

À 0,1 m/cellule, ce noyau étend typiquement l'obstacle d'une cellule dans chaque direction ; le commentaire « 0,15 m » ne décrit pas exactement ce rayon discret.

Les cellules inconnues ne sont pas conservées comme une catégorie interdite dans la grille de planification : tout ce qui n'est pas classé obstacle devient traversable. Les seuils YAML et `negate` ne sont pas utilisés comme une configuration générale de décodage.

### 7.3 Pose et recalage

Le contrôleur écoute `/odom` et ajoute un offset. Si `map_origin.txt` existe, il initialise l'offset à l'opposé de l'origine. Avec l'origine fournie, cela ajoute +10 m sur x et y.

`last_pose.txt` initialise temporairement la pose, mais le callback `/odom` l'écrase ensuite avec la pose brute et l'offset. Une restauration durable de localisation ne découle donc pas de cette seule lecture.

Le recalage manuel calcule une rotation et une translation, mais le callback odométrique applique ensuite seulement la translation aux coordonnées x/y, tout en ajoutant l'angle. Les deux formules sont incompatibles si le recalage modifie l'orientation.

La relocalisation caméra est tentée toutes les dix secondes. Lorsqu'elle réussit, elle change `robot_pos` sans mettre à jour l'offset odométrique ; le message `/odom` suivant peut donc effacer la correction.

### 7.4 Planification et suivi

À réception d'un clic, les coordonnées sont converties en cellules et transmises à `DStarLite.plan()`. Les coordonnées hors carte sont ramenées à la bordure, plutôt que rejetées.

Le code supprime les trois premiers points avec `res[3:]`. Un chemin très court peut donc devenir vide sans que le but ait été suivi.

La boucle de contrôle fonctionne toutes les 0,05 s, soit **20 Hz**. Elle retire un point lorsque le robot est à moins de 0,15 m, et utilise un PID pour orienter le robot vers le point suivant. L'avance est de 0,15 m/s si l'erreur de cap est inférieure à 0,3 rad ; sinon le robot tourne sans avancer.

Le PID de cap est configuré à `kp=0.9`, `ki=0.05`, `kd=0.3`, avec une sortie bornée à ±0,7 rad/s.

### 7.5 Obstacles dynamiques et commandes concurrentes

Les scans LiDAR sont projetés pour l'affichage, avec un point conservé sur huit. **Ils ne mettent pas à jour la grille du planificateur et n'imposent pas d'arrêt frontal dans cette boucle de suivi.** La replanification dynamique annoncée n'est donc pas raccordée au contrôle actuel.

En parallèle, le gestionnaire garde ou relance `tb3_nav_node.py`. Ce dernier publie aussi `/cmd_vel`, y compris des zéros au repos. Les commandes des deux processus peuvent ainsi s'intercaler. Il faut un propriétaire unique de la commande, ou un mécanisme explicite d'arbitrage.

La fermeture du nœud de navigation sur carte ne comporte pas de publication finale explicite d'un `Twist` nul dans son `finally`.

## 8. Planificateur — `dstar_lite.py`

La classe `DStarLite` stocke une grille où une valeur ≥ 2 est un obstacle. Elle utilise huit voisins, avec un coût 1 pour un déplacement axial et √2 pour une diagonale.

Elle possède une file de priorité, des scores `g` et `rhs`, une heuristique euclidienne, une mémoire du dernier départ et du dernier but, ainsi qu'un terme `km` pour les déplacements du départ.

`plan(start, goal)` initialise ou réutilise les scores, appelle le calcul du chemin, puis reconstruit une suite de cellules. `update_obstacle()` tente de réévaluer les cellules affectées.

Cette implémentation doit être considérée comme **un planificateur expérimental à vérifier**, pour plusieurs raisons :

- la gestion des entrées périmées ou dupliquées dans la file n'est pas accompagnée d'un mécanisme complet d'invalidation ;
- la clé utilise une heuristique vers le but, et les conditions de poursuite demandent une vérification de conformité avec l'algorithme visé ;
- `update_obstacle()` ne modifie pas lui-même la grille ;
- le navigateur ne l'appelle pas sur les observations LiDAR ;
- les diagonales ne vérifient pas explicitement le passage entre deux obstacles adjacents ;
- aucun test de terminaison, d'optimalité ou de replanification n'est fourni.

Les affirmations de complexité `O(log n)` présentes dans les commentaires ne démontrent pas le coût de la replanification complète. Le nombre de cellules affectées et les opérations de recherche comptent également.

Pour la reprise, une comparaison avec une recherche de référence sur de petites grilles permettrait de vérifier chemins, coûts et cas impossibles avant utilisation physique.

## 9. Vision et reconnaissance de lieux

### 9.1 `loop_closure.py`

`LoopClosureDetector` extrait des points SIFT et conserve pour chaque image de référence : les points caractéristiques, les descripteurs, la pose, un timestamp et un numéro de frame.

Pour une nouvelle image, il compare les descripteurs aux références avec `BFMatcher` et un appariement à deux voisins. Le test de Lowe conserve une correspondance si le premier voisin est suffisamment meilleur que le second.

Les principaux filtres sont : écart minimal de 30 images de référence, distance spatiale maximale codée à 3 m, nombre minimal de correspondances et seuil de certitude. Le nœud principal configure 15 correspondances minimales.

Malgré son titre « Bag of Words », le fichier ne construit pas de vocabulaire visuel ni d'histogramme de mots. Il compare directement des descripteurs SIFT. Le paramètre `distance_threshold` n'est pas utilisé comme seuil spatial effectif : la valeur 3 m est écrite dans la détection.

L'erreur de pose retournée est la différence entre les poses déjà enregistrées ; aucune transformation géométrique entre les images n'est estimée pour corriger cette erreur. Les contraintes peuvent être récupérées, mais aucun optimiseur ne les consomme dans ce projet.

Les références s'accumulent sans politique de réduction. Le coût mémoire et le temps de comparaison peuvent augmenter pendant une longue session.

### 9.2 `global_localizer.py`

`GlobalLocalizer` charge directement l'image PGM, calcule ses points SIFT et tente d'apparier les caractéristiques d'une image caméra avec celles de la carte. Il estime une homographie avec RANSAC, transforme le centre de l'image et produit une position en pixels, un angle approximatif et un score fondé sur les correspondances géométriquement retenues.

Les étapes mathématiques sont implémentées, mais leur application au robot reste fragile : une image caméra perspective et une grille d'occupation vue de dessus ne représentent généralement pas les mêmes textures. Aucun modèle calibré reliant ces deux représentations n'est fourni.

Dans `nav_odom.py`, la conversion pixel/monde utilise `(pixel - origine_en_mètres) × résolution`, ce qui mélange des unités. Elle ne reprend pas la conversion standard locale déjà utilisée ailleurs, et ne traite pas explicitement le retournement vertical de l'image.

L'orientation issue de l'homographie est approximative. `refine_pose()` ne raffine rien actuellement : la boucle contient `pass`, puis la pose initiale est renvoyée.

Ces modules sont donc des prototypes de vision à évaluer séparément avant de les présenter comme une relocalisation fiable.

## 10. Vidéo

### 10.1 Serveur `ros_raw_stream.py`

`ROSRawStreamServer` écoute `/camera/image_raw`, convertit les images en BGR avec `cv_bridge` et conserve la dernière frame sous verrou. Il ouvre un serveur TCP sur toutes les interfaces, port 9999, puis crée un thread par client.

Les images sont envoyées comme une suite d'octets BGR24, sans compression ni en-tête de dimensions. Le client doit connaître exactement la largeur et la hauteur.

La boucle d'envoi ne limite pas le rythme quand une image est disponible et peut retransmettre plusieurs fois la même frame. À titre de repère, 640 × 480 × 3 représente 921 600 octets par image ; trente images distinctes par seconde représentent environ 27,6 Mo/s avant les surcoûts réseau.

### 10.2 Lecteur `video_stream.py`

`RTSPStreamReader` utilise un thread de réception et un verrou sur l'image partagée. Il propose deux chemins :

- URL `tcp://...` : lecture directe d'images brutes de taille fixe ;
- autre URL : sous-processus FFmpeg qui décode en BGR24 et redimensionne.

La classe expose `start`, `stop`, `get_frame`, `is_connected` et `get_fps`. Le mode TCP brut n'a pas de délai de réception explicite ; l'arrêt du thread peut rester incomplet si la socket est bloquée. L'état de connexion peut aussi rester positif après une panne puisqu'une ancienne frame demeure stockée.

`MockStreamReader` génère un dégradé avec une bordure blanche pour tester l'affichage sans caméra. Son compteur FPS renvoie une valeur fixe de 30.

### 10.3 Intégration au gestionnaire

L'URL par défaut est `tcp://10.141.177.146:9999`. Elle doit être adaptée au réseau de l'équipe. L'affichage Tkinter se rafraîchit toutes les 50 ms, soit environ 20 fois par seconde.

Le gestionnaire demande FFmpeg avant de démarrer toute vidéo réelle, même pour le chemin TCP brut qui ne l'utilise pas. Pillow est optionnel à l'import mais nécessaire à l'affichage des frames. OpenCV est importé tardivement pour la conversion BGR/RGB, malgré un commentaire indiquant sa désactivation pour des problèmes antérieurs de segmentation.

Le retour vidéo affiché n'alimente pas directement les détecteurs de lieux : ceux-ci écoutent le topic caméra ROS dans leurs propres nœuds.

## 11. Données sauvegardées

### 11.1 CSV

Les 24 fichiers sont nommés avec la date du **1er avril 2026**, entre `093436` et `112726`. Ils représentent environ **71,4 Mo** et **17 779 lignes de données**, hors en-têtes.

Le format attendu est :

```text
timestamp,x,y,theta,state,front_min,angle_min,angle_max,scan_0,scan_1,...
```

`theta`, `angle_min` et `angle_max` sont en radians. Le timestamp provient de l'horloge ROS ; son interprétation dépend de la configuration de l'horloge utilisée lors de la session.

Le nombre de colonnes varie selon les fichiers, de 230 à 290, soit 222 à 282 colonnes de scan après les huit champs fixes. Le lecteur ne doit donc pas supposer un nombre unique de rayons.

Répartition des états enregistrés :

| État | Lignes |
|---|---:|
| `IDLE` | 10 931 |
| `MOVE_TO_GOAL` | 1 772 |
| `FOLLOW_GAP` | 1 236 |
| `GOAL_REACHED` | 3 840 |

Ces données montrent que plusieurs états ont été enregistrés. Elles ne prouvent pas que les trajectoires étaient exactes, que les objectifs ont été atteints physiquement ou que la carte est cohérente. Plusieurs sessions ne contiennent que `IDLE`.

Les CSV permettent surtout de reconstruire les cartes, comparer les sessions et développer des contrôles reproductibles sans robot. Ils ne remplacent pas un enregistrement complet des messages ROS : ils ne contiennent ni tous les en-têtes capteurs, ni les images, ni toute la chronologie originale.

### 11.2 Carte et pose fournies

Le PGM présent mesure 200 × 200 pixels, avec une profondeur de 8 bits. Le YAML contient :

```yaml
image: map_out.pgm
resolution: 0.1
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

`map_origin.txt` contient `-10.0,-10.0`. `last_pose.txt` contient `0.0,0.0,0.0` au moment de l'analyse.

L'origine désigne la position physique de la grille selon sa convention d'export. Elle ne constitue pas automatiquement un décalage à ajouter à toute odométrie. La dernière pose n'est exploitable que si son repère, la carte et la session sont compatibles.

## 12. Environnement et commandes

### 12.1 Dépendances identifiées

| Ensemble | Éléments attendus |
|---|---|
| ROS 2 | `rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `std_msgs`, `tf2_ros` |
| Calcul | NumPy, SciPy pour la dilatation |
| Affichage | Matplotlib, Tkinter, Pillow |
| Vision | OpenCV avec SIFT, `cv_bridge` pour les images ROS |
| Métadonnées | PyYAML |
| Outils externes | RViz2 ; FFmpeg pour le décodage vidéo concerné |

Le mécanisme de découverte Python trouve `rclpy`, NumPy, OpenCV, Matplotlib, SciPy, Pillow, PyYAML et Tkinter dans l'environnement examiné. Cela ne constitue pas un test d'import complet, de compatibilité binaire, d'affichage graphique ou de connexion ROS.

La distribution ROS 2, les versions de bibliothèques, la configuration réseau et le lancement des pilotes ne sont pas définis par des fichiers de projet. Des caches `cpython-310` indiquent des exécutions antérieures sous Python 3.10, sans constituer une exigence de version formalisée.

### 12.2 Commandes correspondant au code actuel

Ces commandes sont documentaires et **n'ont pas été exécutées pendant cette analyse**. Les contrôleurs peuvent commander le robot ; ils doivent être utilisés après préparation des pilotes, du réseau et de l'environnement ROS de votre installation.

Depuis la racine du dépôt :

```bash
cd mapping
python3 manager.py
```

Lancement séparé du nœud et de RViz :

```bash
python3 tb3_nav_node.py
rviz2 -d tb3_auto_config.rviz
```

Restauration explicitement demandée de la pose sauvegardée :

```bash
python3 tb3_nav_node.py --ros-args -p use_saved_pose:=true
```

Reconstruction offline d'une session fournie :

```bash
python3 carto_mat.py -m offline -i logs/run_20260401_093436.csv -oname reprise --auto --speed 1.0
```

Cette reconstruction écrirait `map_reprise.pgm`, `map_reprise.yaml` et remplacerait le fichier partagé `map_origin.txt`. Elle reste affectée par les incohérences du cartographe décrites plus haut.

Cartographie live interactive :

```bash
python3 carto_mat.py -m live -oname reprise_live
```

Dans le menu : `1` lance la cartographie ; pendant l'acquisition live, `0` puis Entrée revient au menu ; `2` sauvegarde ; `0` dans le menu quitte le programme.

Navigation sur carte :

```bash
python3 nav_odom.py logs/map_out.yaml
```

Serveur vidéo sur la machine recevant le topic caméra :

```bash
python3 ros_raw_stream.py
```

Ces exemples reflètent les entrées réellement codées. Ils ne constituent pas une procédure de mise en service validée. En particulier, les deux contrôleurs ne doivent pas être considérés comme coordonnés simplement parce que le gestionnaire les lance ensemble.

## 13. État fonctionnel et incohérences

### 13.1 Maturité des fonctions

| Fonction | État constaté à la lecture |
|---|---|
| Interface Tkinter et lancement de processus | Implémentés, avec attentes bloquantes et différences entre modes |
| Acquisition IMU, roues, LiDAR | Callbacks présents ; fonctionnement physique non vérifié ici |
| Navigation réactive vers un objectif | Implémentée avec FSM, PID et recherche de passages |
| Pose RK4 | Implémentée, mais translation fondée sur la commande |
| Sauvegarde CSV et dernière pose | Présente, essentiellement lors de l'arrêt |
| Grille probabiliste et rayons libres | Implémentés, avec incohérences de coordonnées et export |
| Construction live | Présente, dépend de `/odom` et du menu console |
| Reconstruction offline | Présente, avec conversion de coordonnées différente du live |
| Planification sur carte | Raccordée au clic, algorithme à vérifier |
| Évitement dynamique dans `nav_odom` | Non raccordé : LiDAR utilisé pour l'affichage |
| Reconnaissance visuelle de revisites | Détection présente, correction globale absente |
| Relocalisation caméra/carte | Prototype avec difficultés géométriques et de persistance |
| Exploration automatique | Aucun choix autonome de frontières ni critère de couverture |
| Retour vidéo | Serveur et lecteurs présents, protocole brut à fiabiliser |
| Tests automatisés | Aucun ensemble structuré fourni |

### 13.2 Priorités de correction avant une remise en service cohérente

| Priorité proposée | Constat | Conséquence attendue |
|---|---|---|
| Très haute | Deux publishers `/cmd_vel` simultanés | Commandes contradictoires, y compris zéros du nœud au repos |
| Très haute | `/my_odom` publié, `/odom` consommé | Chaîne de pose maison non raccordée au live/navigation |
| Très haute | Export `p×255`, lecture `(255-pixel)/255` | Occupé et libre interprétés en sens opposés |
| Très haute | Axes matrice, retournement et conversions différents | Carte, robot et objectifs peuvent ne pas se superposer |
| Haute | `prev_v` utilisé à la place de la vitesse mesurée | Pose éloignée du déplacement réel |
| Haute | Offsets manuels et relocalisation non persistants/cohérents | Recalage faux ou effacé au message odométrique suivant |
| Haute | Tableaux de suivi non agrandis avec la carte | Erreur attendue lors de l'extension |
| Haute | LiDAR sans arrêt/replanification dans le navigateur sur carte | Obstacles nouveaux non intégrés à la conduite |
| Haute | Planificateur sans tests ni garanties de terminaison établies | Chemins et cas limites non fiables avant validation |
| Moyenne | Mode Auto basé sur la fin du processus et un YAML existant | Passage prématuré ou réutilisation d'une carte ancienne |
| Moyenne | Buffers CSV sauvegardés seulement à l'arrêt | Perte possible d'une session interrompue brutalement |
| Moyenne | Arrêts dupliqués, attentes bloquantes, états vidéo incomplets | Interface figée ou processus mal nettoyés |

### 13.3 Tests et code provisoire

`test_manager.py` importe le gestionnaire, crée une fenêtre et prévoit de la quitter après 500 ms. Mais il contient `print(..., flbush=True)` au lieu de `flush=True`. Cet argument invalide provoquera un `TypeError` si l'exécution atteint cette ligne, avant le `try` qui entoure `mainloop()`.

Il s'agit donc d'un essai d'interface actuellement défectueux, pas d'un test du SLAM. La vérification syntaxique ne détecte pas ce type d'erreur d'appel.

Autres signes de travail en cours : anciennes portions commentées dans `carto_mat.py`, imports et attributs inutilisés, deux classes PID très proches, paramètres inscrits dans le code, titres « Phase 5/6 » sans document de référence, et mélange de logique robotique et d'affichage.

## 14. Proposition de reprise pour la nouvelle année

Cette partie propose une direction de travail. **Aucune réorganisation ni correction du code n'a été réalisée pour produire ce résumé.**

### 14.1 Stabiliser les conventions communes

Avant de régler les gains PID ou d'ajouter de la vision, fixer collectivement :

1. la source de pose retenue et le nom de son topic ;
2. les repères ROS et les transformations attendues ;
3. la convention carte : lignes/colonnes, axes, origine, résolution et sens des pixels ;
4. le programme qui possède `/cmd_vel` dans chaque mode ;
5. le comportement en cas de capteur absent, ancien ou interrompu.

Une petite scène de test avec un obstacle uniquement à droite et un autre uniquement devant permettrait de vérifier simultanément l'orientation de la carte, les pixels, la pose et les objectifs.

### 14.2 Découper les responsabilités

Une structure possible à terme :

```text
projet/
├── README.md                   # installation et démarrage
├── docs/                       # architecture, conventions, compte rendu
├── config/                     # paramètres robot, navigation, RViz
├── launch/                     # combinaisons de nœuds par mode
├── src/
│   ├── odometry/               # capteurs et estimation de pose
│   ├── mapping/                # grille, scan matching et export
│   ├── planning/               # recherche et replanification
│   ├── control/                # suivi, évitement, commande unique
│   ├── vision/                 # expériences de reconnaissance de lieux
│   ├── streaming/              # transport caméra
│   └── ui/                     # interface et supervision
├── tests/                      # cas déterministes et intégration
└── data/
    ├── sessions/               # enregistrements et métadonnées
    └── maps/                   # cartes liées à leur session
```

Le détail du package ROS peut être choisi ensuite. L'objectif est de séparer estimation, décision, commande, interface et données pour pouvoir les vérifier indépendamment.

### 14.3 Ordre de travail proposé

**Étape 1 — Préserver une référence.** Conserver les scripts actuels, les CSV et la carte avec une description de l'environnement. Répertorier les pilotes nécessaires et les versions réellement utilisées.

**Étape 2 — Rendre la pose cohérente.** Séparer l'odométrie de la navigation, utiliser les mesures et des intervalles temporels adaptés, vérifier les mouvements droits et les rotations. Faire consommer la même pose au cartographe et au navigateur.

**Étape 3 — Fiabiliser la carte.** Uniformiser live/offline, filtrer les retours LiDAR, corriger l'extension et l'export. Vérifier le cycle carte → PGM/YAML → rechargement sur une scène connue.

**Étape 4 — Fiabiliser la navigation.** Valider le planificateur sur de petites grilles, traiter les chemins courts et les destinations invalides, raccorder les obstacles dynamiques et attribuer une seule source de commande active.

**Étape 5 — Remettre le gestionnaire en cohérence.** Définir des états de processus explicites, supprimer les menus console dans les parcours graphiques, contrôler les codes de sortie, identifier la carte de chaque session et éviter les attentes longues sur le fil Tkinter.

**Étape 6 — Évaluer la vision.** Tester d'abord la reconnaissance de revisites sur des images enregistrées. Définir ensuite une méthode de correction compatible avec la représentation cartographique. Une détection de boucle doit produire une contrainte géométrique exploitable pour améliorer la pose ou la carte.

**Étape 7 — Définir l'automatisation réelle.** Décider si « Auto » signifie enchaînement guidé, navigation vers un but ou exploration autonome. Pour l'exploration, il faudra notamment un choix de zones à visiter et un critère de fin qui n'existent pas encore.

### 14.4 Vérifications utiles à construire

- Conversions monde/grille et aller-retour export/import sur une carte asymétrique.
- Intégration cinématique sur translation, rotation et arc connus.
- Passage dans un couloir, impossibilité de traverser un mur et traitement des diagonales.
- Planification avec but inaccessible, départ=but, chemin court et modification d'obstacle.
- Cartographie qui dépasse chacun des quatre bords.
- Relecture d'un CSV avec lignes sans scan et longueurs variables.
- Perte de LiDAR, d'odométrie ou de caméra et comportement de la commande.
- Démarrages/arrêts répétés et garantie d'une seule source de commande active.

Ces vérifications visent les comportements attendus. Elles permettraient de mesurer les améliorations au lieu de se fier aux messages console ou au rendu graphique.

## 15. Glossaire

| Terme | Sens dans le projet |
|---|---|
| SLAM | Construction d'une carte et estimation de pose conjointes |
| Odométrie | Estimation du mouvement à partir de mesures et/ou du modèle utilisé |
| IMU | Capteur inertiel, dont le gyroscope sert ici au cap |
| LiDAR | Mesures de distances en fonction de l'angle |
| Pose | Position x/y et orientation theta |
| TF | Transformations entre repères ROS |
| Grille d'occupation | Tableau de probabilités de présence d'obstacles |
| Log-odds | Représentation logarithmique utilisée pour accumuler les observations |
| Scan matching | Alignement d'un scan avec une représentation déjà disponible |
| PID | Régulateur proportionnel, intégral et dérivé |
| FSM | Machine à états pour organiser les comportements |
| Gap | Passage libre candidat dans un scan |
| Inflation | Agrandissement des obstacles pour tenir compte du volume du robot |
| EWMA | Moyenne exponentielle pour lisser une valeur |
| RK4 | Méthode numérique d'intégration d'ordre quatre |
| SIFT | Extraction et description de caractéristiques d'image |
| Homographie | Transformation projective entre deux plans d'image |
| RANSAC | Estimation géométrique avec rejet de correspondances aberrantes |
| Loop closure | Reconnaissance d'un lieu déjà visité, pouvant fournir une contrainte globale |
| Relocalisation | Recherche d'une pose sur une carte ou une référence existante |
| RViz | Outil de visualisation des données ROS |
| PGM/YAML | Image de grille et métadonnées associées |

## 16. Repères de lecture du code

Les chemins ci-dessous sont relatifs à `mapping/`. Les méthodes permettent de retrouver les passages sans dépendre de numéros de ligne qui changeront lors des corrections.

| Sujet | Points d'entrée |
|---|---|
| Modes et enchaînement | `manager.py` : `start_mapping`, `stop_and_generate`, `start_navigation`, `start_live_mapping`, `start_auto_mode`, `check_auto_completion` |
| Arrêt de l'interface | `manager.py` : `on_closing` |
| Mesures et fusion | `tb3_nav_node.py` : `_cb_joint`, `_cb_imu` |
| Pose réellement intégrée | `tb3_nav_node.py` : `_control_loop`, `_rk4_update` |
| Topics et repères publiés | `tb3_nav_node.py` : `__init__`, `_publish_odom` |
| Évitement | `tb3_nav_node.py` : `_cb_scan`, `_find_best_gap_angle`, `_update_side_lock`, `_control_loop` |
| Confinement | `tb3_nav_node.py` : `_update_loop_detection` |
| Sauvegarde de session | `tb3_nav_node.py` : `_log_data`, `cleanup_handler`, `main` |
| Mise à jour de grille | `carto_mat.py` : `process_data`, `scan_match`, `get_free_indices`, `write_map` |
| Extension et export | `carto_mat.py` : `grow_map`, `save_map` |
| Différence live/offline | `carto_mat.py` : `pose_cb`, `run_carto_offline`, `main` |
| Pose et navigation sur carte | `nav_odom.py` : `odom_cb`, `process_pose_command`, `control_loop` |
| Recalage caméra | `nav_odom.py` : `relocalize` ; `global_localizer.py` : `localize`, `refine_pose` |
| Planification | `dstar_lite.py` : `_key`, `_compute_shortest_path`, `plan`, `update_obstacle` |
| Reconnaissance de lieux | `loop_closure.py` : `add_reference_frame`, `detect_loop` |
| Transport vidéo | `ros_raw_stream.py` : `_handle_client` ; `video_stream.py` : `_read_raw_stream`, `_read_ffmpeg_stream` |

Ce document fournit l'état de référence pour la reprise. Il devra évoluer lorsque les conventions, l'architecture ou les comportements auront été corrigés et vérifiés.
