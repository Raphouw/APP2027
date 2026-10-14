# Architecture actuelle du code

Mise à jour du 7 octobre 2026. Le [résumé initial](../RESUME_PROJET_SLAM.md) décrit l'état historique. Le [suivi des améliorations et validations](AMELIORATIONS_ET_VALIDATION.md) décrit les corrections appliquées après découpage.

## Points d'entrée

Depuis `mapping/`, les scripts restent `manager.py`, `tb3_nav_node.py`, `carto_mat.py` et `nav_odom.py`. Ils initialisent et assemblent les composants. Les fonctions de traitement sont dans `slam_components/`.

| Ensemble | Modules | Responsabilité |
|---|---|---|
| Paramètres | `settings.py` | Topic/repère communs, seuils de fraîcheur et distances |
| Commandes | `command_ownership.py` | Propriétaire local exclusif de la commande robot |
| Régulation | `pid.py` | PID partagé par les deux navigateurs |
| Odométrie | `tb3_odometry.py` | Roues, IMU, intégration et publication de pose/TF |
| Perception | `tb3_perception.py` | Filtrage LiDAR, inflation, choix de passages |
| Récupération | `tb3_recovery.py` | Confinement et verrou de côté |
| Navigation réactive | `tb3_control.py` | FSM, objectif, rampes, surveillance et commande |
| Vision réactive | `tb3_vision.py` | Appel au détecteur de revisites |
| Sessions | `tb3_logging.py` | CSV progressif et sauvegarde de pose |
| Grille | `occupancy_grid.py` | Rayons, scan matching, log-odds et extension |
| Format de carte | `map_format.py` | Conventions, validation et chargement du format versionné |
| Entrées/sorties carte | `mapping_io.py` | CSV anciens/nouveaux et export atomique |
| Exécution cartographe | `mapping_runtime.py` | Callbacks ROS, menu et acquisition live |
| Localisation sur carte | `map_localization.py` | Conversions, recalage et observations dynamiques |
| Navigation sur carte | `map_control.py` | Surveillance, chemin et suivi de points |
| Affichage navigation | `map_view.py` | Processus Matplotlib et interaction utilisateur |
| Gestionnaire | `manager_ui.py`, `manager_processes.py` | Panneaux, worker, sessions et transitions de processus |
| Vidéo gestionnaire | `manager_video.py`, `video_dependencies.py`, `system_tools.py` | Affichage, dépendances optionnelles et recherche FFmpeg |

Les autres modules identifiables restent séparés : `dstar_lite.py` (API historique, implémentation A*), `loop_closure.py`, `global_localizer.py`, `video_stream.py` et `ros_raw_stream.py`.

## Composition

Les mixins apportent des méthodes aux classes principales. Les attributs sont initialisés dans leurs constructeurs et partagés au sein d'un même objet. Ils ne constituent pas des services indépendants et ne doivent pas être instanciés seuls sans leur état requis.

L'odométrie est intégrée lors des mesures de roues. La boucle de commande publie la pose seulement tant que les mesures amont sont fraîches. La navigation sur carte reçoit cette pose depuis `/my_odom`. Le gestionnaire relance le nœud principal en mode odométrie seule avant de transférer la commande à `nav_odom.py`.

Le cartographe exporte sa grille interne `[x,y]` vers le format commun `[y,x]`. Le navigateur et son affichage utilisent le même chargeur. Les cartes sont liées explicitement à leur session.

## Vérification

Depuis la racine, après chargement de ROS :

```bash
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 MPLCONFIGDIR=/tmp/slam-matplotlib python3 -m unittest discover -s tests -v
```

Les tests se trouvent dans `tests/test_components.py` et `tests/test_safety_and_mapping.py`. Ils ne lancent aucun contrôleur ROS ni interface graphique.


## Extension Atelier SLAM (7 octobre 2026)

Voir [le guide de l’interface](GUIDE_ATELIER_SLAM.md) et [l’algorithme et ses mesures](SLAM_ALGORITHME_ET_MESURES.md).

| Module | Responsabilité |
|---|---|
| `se2.py` | Transformations rigides 2D |
| `scan_matching.py` | ICP, rejets et qualité |
| `pose_buffer.py` | Historique et interpolation temporelle |
| `pose_graph.py` | Contraintes et optimisation robuste creuse |
| `slam_engine.py` | Images clés, correction persistante, revisites |
| `slam_mapping.py` | Projection, reconstruction, aperçu et événements SLAM |
| `session_ipc.py` | Messages atomiques locaux, jetons, expiration |
| `run_logging.py` | Capture console des lancements autonomes |
| `manager_ui.py` | Parcours guidé, carte intégrée, états et destinations |
| `manager_processes.py` | Supervision, sessions, logs et cycle de vie |
| `evaluate_trajectory.py` | Comparaison à une trajectoire de référence |

La navigation lancée par le manager utilise `--headless --ipc-dir ... --token ...` ; le lancement direct garde sa fenêtre historique. Le cartographe utilise `--headless` dans le manager. Les acquisitions précédentes et les points d’entrée CLI sont conservés.


### Démarrage et panne native

`lancer.sh` prépare un environnement système ROS/Tk cohérent sans installation ; `manager.py` y redirige ses lancements directs. `check_environment.py` sonde les modules natifs dans des sous-processus et archive les résultats. `installer_dependances.sh` vérifie les paquets et installe uniquement les absents sur demande explicite. `tk_text.py` protège l’affichage contre les pictogrammes hors BMP qui déclenchaient la segmentation de l’ancien manager sur la machine de validation. Voir [le diagnostic détaillé](LANCEMENT_ET_DIAGNOSTIC.md).


`lidar_filter.py` partage le filtrage angulaire conservateur entre perception réactive et navigation sur carte. La provenance des petits trous estimés est séparée des rayons bruts. Voir [la correction des oscillations](CORRECTION_OSCILLATIONS_LIDAR.md).
