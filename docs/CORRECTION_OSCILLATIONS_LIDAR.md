# Correction des oscillations après un objectif

## Diagnostic de l’essai du 7 octobre 2026

La session `20261007_101800_7a416423` contient des objectifs effectivement reçus par le contrôleur (`[GOAL]`). Le robot reste près de l’origine, passe immédiatement en `FOLLOW_GAP` et signale `front=0.00m` / « Aucun gap libre trouvé ». Sur 675 lignes d’acquisition, 641 sont en `FOLLOW_GAP`.

La carte affiche pourtant les obstacles correctement : elle exclut les rayons NaN du calcul. Le contrôleur utilisait une autre chaîne : NaN → zéro → inflation d’un prétendu obstacle à zéro jusqu’à un demi-tour. Des trous dans les retours latéraux pouvaient ainsi créer un faux obstacle devant le robot. Les distances finies devant le robot dans les exemples examinés sont autour de 0,85 m, sans obstacle réel à zéro.

Le passage par RViz n’est donc pas la cause identifiée : le blocage intervient après réception de l’objectif, dans la perception du contrôleur. Le chemin de l’interface publie également `/goal_pose` dans `my_odom`, après conversion de la destination depuis `map`.

## Correction

- Une mesure inconnue n’est plus gonflée comme un obstacle physique à zéro.
- Les petits trous **NaN de largeur angulaire ≤ 5°**, bornés par deux distances finies valides, reçoivent la distance du voisin le plus proche pour le contrôle. Il s’agit d’une estimation prudente, pas d’un nouvel impact mesuré.
- Les grands trous, extrémités non bornées d’un scan partiel, zéros et valeurs numériques invalides restent inconnus. Un trou entre des retours infinis ne suffit pas à inventer une distance finie.
- Une zone avant inconnue interdit la translation **et la rotation forcée**. L’interface indique « LiDAR · zone avant inconnue » plutôt que de laisser le robot alterner ses rotations d’évasion.
- Les mêmes règles sont utilisées pendant la cartographie et la navigation sur carte.
- Les obstacles réels proches restent présents et l’arrêt de sécurité à 0,30 m est conservé.
- Les données brutes restent inchangées pour la carte et les CSV. Les valeurs estimées ne servent pas à dessiner des murs.

Les nouveaux logs donnent `unknown_front`, `inferred`, la distance avant et les bornes `range_min/range_max`. Les nouveaux CSV ajoutent les deux bornes du capteur après `ranges_json` ; les anciens fichiers restent lisibles. Ces métadonnées manquaient dans l’acquisition examinée : une relecture ne peut pas retrouver exactement les limites déclarées par le pilote à cet instant.

## Validation et reprise

Les tests couvrent les petits trous, grands trous, jointure d’un scan complet, limites d’un scan partiel, discontinuités entre surfaces, vrais obstacles proches, arrêt des deux vitesses lorsque l’avant est inconnu, et autorisation de translation sur un scan libre avec un petit trou. Tous les éditeurs de commandes utilisés dans les tests sont simulés ; aucun mouvement physique n’est déclenché.

Pour charger le correctif : arrêter le robot depuis l’interface, fermer le manager et relancer `./lancer.sh`. Les processus déjà lancés conservent l’ancien code jusqu’à leur redémarrage. Vérifier ensuite dans une zone dégagée, avec un objectif proche et l’arrêt accessible. Si « zone avant inconnue » reste affiché, examiner les nouveaux diagnostics LiDAR plutôt que de réduire les protections.

Le message vidéo « No route to host » visible dans la capture concerne la connexion caméra facultative ; il ne démontre pas une panne de réception du LiDAR et n’explique pas les faux obstacles à zéro enregistrés ici.


## À-coups : continuité courte et limites (7 octobre 2026)

La session `20261007_104112_cde9b8ee` contient des trous LiDAR intermittents
qui font alterner autorisation et arrêt de la commande. La carte accumule les
mesures dans le temps : une carte visible ne garantit pas un scan avant complet
à cet instant. Le dernier état observé `GOAL_REACHED` correspond à un arrêt normal.

Les deux contrôleurs disposent désormais du même historique de retours réels :
200 ms maximum, reprojection avec la pose du robot, déplacement inférieur à
8 cm et rotation inférieure à 0,25 rad. Seuls les petits trous NaN bornés
(jusqu’à 8 degrés) peuvent bénéficier de cet historique. Une marge conservatrice
de 2 cm + 0,5 m/s × âge est retirée des distances précédentes. Les estimations
ne sont jamais réinjectées dans l’historique, ni dans la cartographie. Les
obstacles mesurés à l’instant courant restent prioritaires. Les trous larges,
les zéros et les données expirées restent bloquants.

Le diagnostic JSON contient `lidar_temporal` (nombre de rayons récupérés),
`lidar_front_unknown` (rayons avant encore inconnus) et `state`. Les logs
ODOM affichent `recent=...`. L’interface distingue désormais calibration,
mesures LiDAR insuffisantes, interruption de capteur et destination atteinte.

Validation hors robot : 7 nouvelles régressions de l’historique et suite
complète. Sur un extrait de 2 900 scans, le nombre de scans avant incomplets
passe de 1 387 à 1 350 seulement : cette protection ne résout pas les pertes
persistantes. Si les arrêts continuent, conserver les logs du run et vérifier
la source `/scan` (distribution des NaN, fréquence, alimentation et transport
du LiDAR) avant d’élargir les tolérances. Aucun essai de déplacement physique
n’a été réalisé pendant cette correction.
