# Simulation sans robot

Un robot virtuel remplace le TurtleBot3, sans Gazebo. Les nœuds de l'atelier tournent sans modification.

## Utilisation

Dans le manager, cochez **Mode simulation (sans robot)** avant de lancer une cartographie. La case se désactive seule si une session est en cours. Le simulateur publie `/scan` (10 Hz), `/joint_states`, `/imu` (100 Hz) et la vérité terrain sur `/sim/ground_truth`, et lit `/cmd_vel`.

En ligne de commande (ROS chargé, depuis `atelier_slam/`) :

```bash
python3 simulateur.py --truth-csv /tmp/verite.csv   # robot + salle intégrée
python3 pilote_simulation.py --laps 2                     # pilote automatique : tours autour de l'îlot central
```

Options utiles de `simulateur.py` : `--noise 0` (capteurs parfaits), `--seed N`, `--pose X Y THETA`, `--map carte.pgm --resolution 0.05 --origin X Y` (votre propre carte, noir = obstacle).

## Ce qui est simulé

- Salle de 10 m x 8 m avec un îlot central (on peut en faire le tour, donc fermer une boucle), une cloison, un muret et deux piliers (`slam_components/sim_world.py`).
- LiDAR de 360 rayons, portée 0,12–3,5 m, bruit de 1 cm. Roues de 33 mm, voie de 160 mm.
- Erreurs d'odométrie : rayon de roue faux de ~1,5 % par roue (fixe pour une graine donnée), glissement aléatoire de 1 %, biais et bruit du gyroscope.
- Pas de dynamique complète : rampes d'accélération simples et arrêt sur collision.

## Mesurer l'erreur

```bash
source /opt/ros/humble/setup.bash
python3 scripts/mesurer_simulation.py --laps 2
```

Le script lance simulateur, odométrie, cartographe et pilote, puis écrit `resultats.json` avec l'erreur de position (RMSE, 95e centile) et de cap de l'odométrie seule et du SLAM face à la vérité terrain, via `evaluer_trajectoire.py`.

## Première mesure (2 tours, ~25 m par tour, graine 0, bruit 1)

| | RMSE position | 95e centile | RMSE cap |
|---|---:|---:|---:|
| Odométrie seule | 6,8 cm | 11,0 cm | 0,018 rad |
| SLAM (trajectoire corrigée) | 10,4 cm | 16,9 cm | 0,022 rad |

Réserves : une seule exécution, sur un PC qui faisait tourner cinq processus à la fois. Le cartographe n'a traité que 1 457 des ~2 900 scans publiés (il ne tient pas le rythme en temps réel ici), et sa trajectoire couvre donc ~260 s sur 294 s. 4 fermetures de boucle ont été détectées. La comparaison n'est donc pas encore concluante : le SLAM n'améliore pas l'odométrie dans cette mesure, et la charge de calcul du cartographe est un point à étudier.
