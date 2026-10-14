# Améliorations appliquées et validation

État du 7 octobre 2026. Les six priorités et la fiabilisation des sessions ont été traitées par étapes. Les contrôleurs du robot n'ont pas été lancés pendant ces travaux.

## 1. Une seule source de commande locale

Le gestionnaire arrête et attend l'ancien contrôleur avant de lancer le suivant. Pendant la navigation sur carte, `tb3_nav_node.py` tourne avec `odometry_only:=true` : il ne crée pas de publisher `/cmd_vel` et n'écoute pas les objectifs de navigation réactive.

Les deux contrôleurs utilisent également `CommandLease`, un verrou interprocessus. Si un contrôleur du projet possède déjà la commande, le second refuse de démarrer. La sortie du processus libère le verrou. Les arrêts normaux publient une vitesse nulle avant de libérer le propriétaire.

Ce verrou concerne les programmes locaux exécutés par le même utilisateur. Il ne coordonne pas un autre ordinateur, la télécommande ou des nœuds externes au projet. Pour une commande distribuée, un arbitre ROS reste nécessaire.

Fichiers : `command_ownership.py`, `manager_processes.py`, `tb3_nav_node.py`, `nav_odom.py`.

## 2. Une odométrie commune

Le topic utilisé par les trois consommateurs est désormais `/my_odom`, dans le repère `my_odom`. Les pilotes peuvent conserver leur `/odom` séparé. Les constantes sont regroupées dans `settings.py`.

Les objectifs réactifs et les poses reçues par le navigateur sont rejetés si leur repère ne correspond pas. Les cartes nouvelles portent explicitement leur `frame_id`.

L'origine de l'image ne sert plus de translation artificielle à la pose du navigateur. Un recalage manuel utilise désormais la rotation et la translation complètes, et reste appliqué aux messages suivants. Un déplacement physique du robot entre deux sessions nécessite toujours un recalage ; restaurer un fichier de pose n'est pas une relocalisation physique.

La TF publiée reste `my_odom → base_link`. La description du robot et les pilotes doivent fournir les transformations des capteurs. L'absence de conflits TF avec le lancement réel des pilotes reste à vérifier sur l'installation de l'équipe.

Fichiers : `settings.py`, `tb3_odometry.py`, `mapping_runtime.py`, `map_localization.py`.

## 3. Une convention de carte vérifiable

Le format exporté est versionné (`format_version: 2`). Il utilise :

- une grille physique `[ligne y, colonne x]` ;
- une origine en mètres, en bas à gauche ;
- des pixels blancs libres, noirs occupés et gris inconnus ;
- un retournement vertical unique entre la grille et le fichier image ;
- une transposition explicite de la grille interne historique `[x, y]` lors de l'export.

La conversion monde/grille est la même en live et en offline. Les destinations hors carte sont rejetées, et non ramenées artificiellement à sa bordure. Les cellules inconnues sont bloquées pour la planification. L'inflation est exprimée en mètres (`ROBOT_CLEARANCE`), puis convertie en cellules.

Les mesures NaN, négatives ou nulles ne créent plus d'obstacle au centre du robot. Les retours positifs infinis représentent un rayon libre jusqu'à la portée cartographiée, sans mur terminal inventé. Les scans bruts sont conservés dans les nouveaux enregistrements pour garder cette distinction.

Une scène asymétrique teste l'export et la relecture : un obstacle à droite reste à droite après sauvegarde.

**Migration : les anciens YAML sans version sont refusés par le navigateur.** Ils doivent être régénérés à partir des CSV. Les fichiers historiques du dépôt n'ont pas été écrasés. Les positions déjà erronées enregistrées dans un ancien CSV ne sont pas corrigées rétroactivement par le nouveau cartographe.

Fichiers : `map_format.py`, `occupancy_grid.py`, `mapping_io.py`, `map_localization.py`, `map_view.py`.

## 4. Une pose fondée sur le mouvement mesuré

L'intégration RK4 est exécutée à réception des mesures de roues, avec l'intervalle entre leurs timestamps. La translation utilise les déplacements des encodeurs, et non la vitesse commandée. Les messages temporellement dupliqués ou désordonnés sont ignorés. Après une interruption trop longue, une nouvelle référence évite un saut de pose.

La rotation utilise le gyroscope corrigé et la rotation des roues. Une mesure gyroscopique trop ancienne n'est pas réutilisée comme si elle était récente. Le contrôleur réactif utilise également l'intervalle réel de sa boucle pour ses régulateurs et rampes.

Quand les roues ou l'IMU ne sont plus reçues à temps, le nœud cesse de publier une odométrie qui semblerait fraîche. Cela permet au navigateur sur carte de détecter la panne en amont.

Tests : trajet sur arc connu, déplacement de roues sur 100 ms, roues immobiles malgré une consigne non nulle, timestamp dupliqué et arrêt de publication en cas de roues périmées.

Fichiers : `tb3_odometry.py`, `tb3_control.py`.

## 5. Arrêt LiDAR et replanification

Le suivi sur carte vérifie la réception récente du LiDAR et de l'odométrie, à partir d'une horloge monotone. Il publie une vitesse nulle en cas de perte de données, de mesures frontales invalides, de fermeture de l'affichage ou de commande STOP.

Le seuil frontal est `STOP_DISTANCE = 0.30 m`. L'arrêt frontal précède la planification et ne dépend pas de l'épuisement du chemin. Le contrôleur réactif bloque également l'avance devant un obstacle proche et s'arrête quand ses capteurs sont périmés.

Les impacts LiDAR sont projetés dans la grille et gonflés selon le volume de sécurité. Ils expirent après une seconde sans nouvelle observation. Une modification déclenche un nouveau chemin, au plus toutes les 0,5 s ; le robot est immobilisé pendant ce calcul. Un chemin impossible ne laisse pas subsister l'ancien chemin.

Les petits chemins sont conservés, y compris un but dans la cellule courante. STOP supprime aussi le but et les clics en attente.

Ces paramètres sont des valeurs de départ à mesurer sur le robot : la distance d'arrêt réelle dépend du matériel et de la vitesse. Les tests logiciels ne certifient pas le freinage physique.

Fichiers : `map_control.py`, `map_localization.py`, `tb3_perception.py`, `tb3_control.py`, `settings.py`.

## 6. Planification et extension fiables

L'ancienne recherche incrémentale expérimentale a été remplacée par une recherche **A\*** déterministe avec une file de priorité et des coûts positifs. Le nom public `DStarLite` reste disponible pour conserver l'import historique, mais l'implémentation ne revendique plus D* Lite ni ses performances incrémentales.

Le départ et le but doivent être accessibles. Les diagonales ne traversent plus un coin bloqué. Les cas départ=but, but inaccessible, sortie de grille et ajout d'obstacle sont traités. Les coûts ont été comparés à ceux du solveur de graphes de SciPy sur trente grilles déterministes.

L'agrandissement de carte déplace ensemble la grille, les tableaux de suivi, la pose, l'origine en cellules et l'historique de trajectoire. Il fonctionne pour les dépassements sur les quatre côtés.

Le choix d'A* privilégie une base vérifiable. Sur de grandes cartes, une mesure de performance puis une véritable implémentation incrémentale pourront être envisagées.

Fichiers : `dstar_lite.py`, `occupancy_grid.py`.

## 7. Sessions et gestionnaire

Chaque acquisition du gestionnaire reçoit un dossier unique :

```text
mapping/logs/sessions/<identifiant>/
├── session.json
├── mapping.csv
├── last_pose.txt
├── map_out.yaml
├── map_out_<identifiant-image>.pgm
└── navigation_<identifiant>.csv
```

Le CSV est écrit et vidé progressivement, avec synchronisation disque périodique. Les poses sont sauvegardées par remplacement atomique. La mémoire ne conserve plus toute la session dans deux listes.

Le nouveau CSV conserve les huit premiers champs historiques et utilise ensuite `ranges_json`, qui porte la liste exacte des distances de chaque scan. La relecture accepte ce format et les anciens CSV à colonnes `scan_0`, `scan_1`, etc.

Le YAML porte l'identifiant de session et le chemin du CSV source. Le gestionnaire vérifie ces informations avant de naviguer. Il ne choisit plus le fichier le plus récemment modifié dans l'ensemble des logs. Une reconstruction vide, échouée ou interrompue ne déclenche pas la navigation.

L'image est écrite sous un nom unique, puis le YAML est remplacé atomiquement. En live supervisé, une carte intermédiaire est enregistrée toutes les cinq secondes et une sauvegarde finale est effectuée à l'arrêt normal.

Les opérations longues se déroulent dans un worker, avec suivi depuis le fil Tkinter. Les arrêts des processus sont bornés, attendent leur sortie et signalent les arrêts forcés. Un échec arrête les processus partiellement lancés. STOP et la fermeture restent pris en compte pendant une opération.

Le mode Auto lance directement le cartographe live sans menu console. Un bouton **« Terminer carte et naviguer »** rend explicite la fin de la phase guidée. Fermer normalement la fenêtre du cartographe peut aussi terminer cette phase. La navigation attend ensuite une destination choisie sur la carte. Le mode ne prétend pas effectuer une exploration autonome.

Fichiers : `tb3_logging.py`, `manager_processes.py`, `manager_ui.py`, `mapping_io.py`, `mapping_runtime.py`, `carto_mat.py`.

## Commandes

Depuis la racine du projet, avec ROS 2 Humble chargé :

```bash
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 MPLCONFIGDIR=/tmp/slam-matplotlib python3 -m unittest discover -s tests -v
```

Interface habituelle :

```bash
cd mapping
python3 manager.py
```

Régénération manuelle d'une carte historique dans un nouveau dossier :

```bash
cd mapping
python3 carto_mat.py -m offline -i logs/run_20260401_093436.csv --auto --output-dir logs/reconstruction -oname reprise
```

Le résultat est `logs/reconstruction/map_reprise.yaml`, accompagné de son image référencée. Le mode offline ne crée plus de nœud ROS et peut être exécuté avec `MPLBACKEND=Agg` sans affichage graphique. Les imports ROS exigent toujours l'environnement installé.

Mode odométrie seule, pour un lancement manuel coordonné :

```bash
python3 tb3_nav_node.py --ros-args -p odometry_only:=true
```

Ne pas ajouter `use_saved_pose:=true` sans vérifier que le robot est dans la pose et le repère de la session à restaurer.

## Résultats et limites

- **30 tests automatisés passent**, sans nœud ROS actif et sans commande robot.
- La reconstruction offline de **douze scans d'un CSV historique**, lancée par le vrai point d'entrée, a réussi ; l'image, les conventions et les métadonnées de session ont été relues et vérifiées.
- Les fichiers de données historiques n'ont pas été modifiés.
- La validation graphique Tkinter et les essais physiques restent à faire sur l'installation de l'équipe.
- La relocalisation SIFT caméra/PGM reste expérimentale et est désormais désactivée par défaut. Le recalage manuel est disponible. La correction globale par fermeture de boucle et l'exploration autonome restent des développements distincts.

## Ordre de validation sur robot

1. Vérifier les transformations capteurs et l'absence de conflit TF avec les pilotes ; contrôler qu'un seul contrôleur du projet publie les vitesses.
2. Robot immobile, attendre la calibration et vérifier que la pose ne se déplace pas.
3. Mesurer une translation courte et une rotation, puis comparer à l'odométrie.
4. Cartographier une scène asymétrique simple ; vérifier les axes et le sens des obstacles après rechargement.
5. Tester la disparition d'un topic et la présence d'un obstacle frontal, avant un suivi de chemin complet.
6. Tester un chemin court, un but inaccessible, le passage Normal/Auto et les arrêts répétés.

Ces essais complètent la validation logicielle ; ils n'ont pas été exécutés pendant cette modification.


## Extension du 7 octobre 2026 : algorithme et ergonomie

Le SLAM SE(2), la correction persistante, la synchronisation des nouvelles acquisitions, les rejets de recalage, les images clés, les fermetures de boucle et la reconstruction de grille sont maintenant implémentés. Le manager propose un parcours guidé et une carte intégrée sous Tkinter/ttk, avec états des capteurs, arrêt permanent, reprises de session, reconstruction CSV, destinations et repositionnement dans la même fenêtre.

Les logs sont liés au run du manager et à la session, avec fichiers distincts par lancement de sous-processus, stdout/stderr, traces Python, logs ROS et événements SLAM JSONL. Les lancements autonomes produisent également un dossier de debug unique.

Détails : [guide utilisateur](GUIDE_ATELIER_SLAM.md), [algorithme, mesures et limites](SLAM_ALGORITHME_ET_MESURES.md). Les anciennes descriptions ci-dessus documentent les étapes précédentes ; le guide utilisateur décrit le parcours actuel.

Validation : tests de calcul et de sécurité, tests de widgets sur affichage virtuel isolé, reconstruction de 150 scans historiques et capture de la fenêtre. Aucun essai physique, aucun résultat de précision réel et aucune comparaison SLAM Toolbox ne sont revendiqués.


### Lancement simplifié et segmentation

Un lanceur unique charge ROS et vérifie l’environnement sans installation ni `colcon build`. La commande historique `python3 manager.py` redirige vers ce lanceur. Le crash natif de l’ancien manager a été reproduit ; retirer uniquement ses pictogrammes hors BMP en mémoire permettait le démarrage. La nouvelle interface et `tk_safe_text` évitent ce déclencheur. Les traces de lancement, tests d’import isolés et logs natifs sont conservés par tentative. Détails et limites : [lancement et diagnostic](LANCEMENT_ET_DIAGNOSTIC.md).


## Correction des oscillations après l’essai robot

Les objectifs étaient reçus, mais les NaN du LiDAR devenaient des obstacles à zéro puis étaient gonflés. Le filtre de contrôle partage désormais une politique de petits trous bornés, conserve les grandes zones inconnues comme bloquées et interdit les rotations forcées lorsque l’avant est inconnu. Voir [le diagnostic et la validation](CORRECTION_OSCILLATIONS_LIDAR.md).
