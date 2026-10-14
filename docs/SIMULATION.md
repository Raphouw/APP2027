# Simulation sans robot

Un robot virtuel remplace le TurtleBot3, sans Gazebo. Les nœuds de l'atelier tournent sans modification.

## Utilisation

Dans le manager, cochez **Mode simulation (sans robot)** avant de lancer une cartographie. La case se désactive seule si une session est en cours. Le simulateur publie `/scan` (10 Hz), `/joint_states`, `/imu` (100 Hz) et la vérité terrain sur `/sim/ground_truth`, et lit `/cmd_vel`.

En ligne de commande (ROS chargé, depuis `atelier_slam/`) :

```bash
python3 simulateur.py --truth-csv /tmp/verite.csv   # robot + salle intégrée
python3 pilote_simulation.py --laps 2                     # pilote automatique : tours autour de l'îlot central
```

Options utiles de `simulateur.py` : `--dropout 0.01` (1 % de rayons LiDAR invalides à 0,0 m, comme un vrai LDS ; c'est ce qui reproduit les saccades), `--noise 0` (capteurs parfaits), `--seed N`, `--pose X Y THETA`, `--map carte.pgm --resolution 0.05 --origin X Y` (votre propre carte, noir = obstacle).

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

## Mesures (2 tours, ~25 m par tour, graine 0, bruit 1)

| | RMSE position | 95e centile | RMSE cap |
|---|---:|---:|---:|
| Odométrie seule | 5,9 cm | 9,0 cm | 0,015 rad |
| SLAM (trajectoire corrigée) | 6,2 cm | 10,6 cm | 0,012 rad |

Avant l'optimisation du cartographe (voir ci-dessous), le SLAM donnait 10,4 cm d'erreur de position et ne traitait que 1 457 scans sur ~2 640.

Réserves : une seule exécution par ligne, sur un PC qui fait tourner cinq processus à la fois (les résultats varient d'une exécution à l'autre). Le cartographe traite maintenant 1 905 scans sur ~2 640 (56 fermetures de boucle) : il ne tient pas encore tout le flux. Dans cette simulation, le SLAM n'améliore pas la position par rapport à l'odométrie ; il améliore un peu le cap. L'odométrie simulée est peut-être trop bonne (erreurs de roues de 1,5 %) : un bruit plus fort (`--noise 3`) permettrait de mieux voir l'apport du SLAM.

## Pourquoi le cartographe était lent

1. Le tracé des rayons libres de chaque scan se faisait en boucles Python avec des ensembles et des tris : ~70 % du temps par scan. Il est maintenant vectorisé (carte produite identique, vérifiée sur 300 scans).
2. Chaque fermeture de boucle déclenchait une reconstruction complète de la carte (tous les scans depuis le début), soit plusieurs secondes de blocage à chaque fois. La reconstruction est maintenant différée : au moins 20 scans et dix fois sa propre durée entre deux reconstructions, et une dernière avant l'enregistrement de la carte.
3. L'optimisation du graphe de poses calculait ses résidus arête par arête en Python ; elle est vectorisée (résultat identique à 1e-15) et s'arrête à une tolérance un peu plus large (30 évaluations au plus au lieu de 80).

Effet sur un rejeu de 1 500 scans : plus de 2 minutes avant (estimé d'après la progression mesurée), 35 s après.
