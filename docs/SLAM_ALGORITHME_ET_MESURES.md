# SLAM SE(2), fermeture de boucle et mesures

## Ce qui change

L’ancien `scan_match` ajustait seulement x/y dans la grille du scan courant. Le moteur `slam_engine.py` travaille en mètres et radians, séparément de ROS et de la grille. Il corrige x, y **et l’angle**, conserve une transformation `map_from_odom`, construit des images clés, cherche des revisites et optimise un graphe de poses. Une optimisation réussie provoque la reconstruction de la grille à partir des observations retenues, y compris les rayons libres.

L’odométrie reste continue dans `my_odom`. La carte est exprimée dans `map`. Le cartographe live publie TF `map → my_odom`. Le YAML conserve la transformation ; la navigation l’applique à `/my_odom` et le recalage manuel la remplace durablement. Le `my_odom → base_link` existant doit être compatible avec la configuration TF du pilote : ne pas laisser deux producteurs concurrents pour le parent d’un même repère.

## Chaîne

1. Les nouvelles acquisitions interpolent la pose des roues à l’horodatage du scan. Elles enregistrent chaque scan une seule fois. Les odométries publiées portent l’horodatage de la mesure de roues lorsqu’il existe.
2. En live, un tampon ordonné interpole les poses, avec traitement correct du passage de +π à −π. Un scan légèrement en avance attend la mesure suivante ; un scan trop ancien ou séparé par un trou de plus de 0,3 s est écarté et comptabilisé.
3. L’ICP local utilise les impacts LiDAR finis et une sous-carte des six dernières images clés. Une recherche KD-tree, un rejet des correspondances éloignées et un ajustement SVD calculent une transformation SE(2).
4. Le résultat est accepté seulement si la géométrie, le recouvrement, le résidu et l’amplitude de correction sont suffisants. Sinon, la prédiction odométrique corrigée reste utilisée.
5. Une image clé est ajoutée après environ 22 cm ou 0,15 rad de déplacement. Le graphe contient un a priori odométrique et, lorsqu’il existe, une contrainte de recalage.
6. Les revisites comparent jusqu’à trois images clés proches, séparées d’au moins seize images clés. Elles exigent un recouvrement plus élevé et une vérification dans les deux sens.
7. Le graphe optimise les poses avec `scipy.optimize.least_squares`, une perte robuste et un Jacobien creux. La première pose fixe le repère. Une solution non finie, non convergée ou augmentant le coût robuste est refusée.
8. Les observations conservent une pose relative à leur image clé. Après optimisation, les anciens scans sont reprojetés avec les nouvelles poses : corriger le trajet seul laisserait les murs dédoublés.

Le recalage historique est conservé uniquement pour les objets de grille qui n’activent pas le moteur ; le point d’entrée `carto_mat.py` utilise le nouveau moteur.

## Paramètres initiaux

| Contrôle | Valeur initiale |
|---|---:|
| Itérations ICP maximales | 25 |
| Distance initiale de correspondance | 0,35 m |
| Recouvrement local minimal | 65 % |
| RMSE local maximal | 0,09 m |
| Correction locale maximale | 0,55 m / 0,35 rad |
| Rayon de recherche de boucle | 0,75 m |
| Recouvrement boucle minimal | 85 % dans les deux sens |
| RMSE boucle maximal | 0,045 m dans les deux sens |
| Recherche historique bornée | 400 images clés |
| Publication de l’aperçu | au plus 1 Hz |

Ces valeurs sont regroupées dans le moteur et doivent être calibrées avec vos acquisitions. Une boucle ne sera pas cherchée si la dérive éloigne les poses de plus de 0,75 m ; le système n’effectue pas de relocalisation globale.

## Mesures reproductibles

`trajectory.csv` contient les poses corrigées après optimisation. `mapping.csv` conserve l’acquisition brute. Une référence doit utiliser les colonnes `timestamp,x,y,theta`, en secondes, mètres, radians, dans **le même repère et avec la même origine temporelle**. L’évaluateur interpole la référence sans extrapolation, ne masque pas un décalage de repère par un ajustement automatique et donne RMSE position, percentile 95 et RMSE angulaire.

```bash
python3 mapping/evaluate_trajectory.py \
  mapping/logs/sessions/MA_SESSION/trajectory.csv \
  reference.csv --output evaluation.json
```

Comparer également l’odométrie brute à cette référence. Une référence mesurée indépendante est nécessaire pour conclure à un gain réel ; fermer visuellement une boucle ne suffit pas.

Le test synthétique déterministe `square_run` utilise 25 poses sur un carré, une dérive odométrique progressive et un nuage de points fixe. Résultat de validation : RMSE position brute **0,188 m**, RMSE des estimations SLAM **0,0074 m**, trois contraintes de boucle acceptées. C’est un cas favorable, sans simulation réaliste des occultations, de la dynamique ni du bruit LiDAR ; ce résultat ne prédit pas la précision sur le TurtleBot.

Une relecture de **150 scans réels historiques**, essentiellement stationnaires, a également réussi. Elle valide le chemin de reconstruction et les fichiers produits, pas une réduction de dérive mesurée. Les anciens CSV associent une pose au moment de leur ancien enregistrement ; leurs horodatages exacts de scan ne peuvent pas être recréés.

## Limites et essais à poursuivre

- ICP reste local et peut échouer sur des couloirs répétitifs, des symétries ou des obstacles mobiles. Le refus d’une paroi quasi colinéaire ne détecte pas toutes les ambiguïtés possibles.
- La recherche de boucle est géométrique, sans descripteur global ni vérification probabiliste complète. Des faux positifs restent possibles ; inspecter les contraintes dans `pose_graph.json`.
- Les pondérations du graphe sont fixes ; elles ne représentent pas une covariance estimée à partir des capteurs. Les contraintes odométriques et scan peuvent être corrélées.
- Les scans et observations sont gardés en mémoire pendant la session. Les très longues sessions demandent une politique de sous-cartes et de stockage disque. Une reconstruction globale est synchrone dans le cartographe ; mesurer son coût et ses pertes de scans sur le matériel réel.
- Pas de correction du mouvement à l’intérieur d’un scan, ni de calibration automatique de l’extrinsèque LiDAR. Se déplacer lentement pendant les essais.
- La navigation conserve sa surveillance LiDAR et son recalage manuel. Elle ne réalise pas encore une localisation probabiliste continue sur carte sauvegardée.
- Une comparaison avec SLAM Toolbox reste une expérimentation à faire sur les mêmes bagfiles et la même référence, avant de décider d’une migration. Aucun remplacement silencieux par une pile ROS externe n’a été réalisé.

## Tests

```bash
python3 -m unittest discover -s tests -v
```

Les tests Tk sont optionnels par défaut. Sur un affichage **virtuel isolé** déjà démarré :

```bash
DISPLAY=:97 TB3_TEST_DISPLAY=:97 python3 -m unittest discover -s tests -v
```

Ils construisent les widgets, vérifient les états et la carte intégrée sans démarrer de processus robot. Les autres tests couvrent SE(2), ICP rotation/translation, refus de géométrie et de non-recouvrement, persistance du recalage, interpolation, optimisation de boucle, reconstruction passée, messages périmés et logs stdout/stderr par run.
