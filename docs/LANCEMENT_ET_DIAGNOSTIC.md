# Lancement simplifié et diagnostic du manager

## Utilisation quotidienne

Sur le PC, une commande suffit, depuis la racine du projet :

```bash
./lancer.sh
```

La commande historique fonctionne également :

```bash
cd mapping
python3 atelier.py
```

Elle redirige automatiquement vers le même lanceur. Vous pouvez aussi double-cliquer sur l’icône « Atelier SLAM » créée par `./scripts/installer_icone.sh` ; selon le gestionnaire de fichiers Ubuntu, il faudra autoriser son lancement une première fois. Ce raccourci vise `/home/app-sea/Bureau/APP2027` : si vous déplacez le projet, adapter les champs `Exec` et `Path`. Le script `lancer.sh` lui-même trouve son dossier et fonctionne depuis n’importe quel répertoire.

Le lanceur :

1. choisit le Python système `/usr/bin/python3`, compatible avec les paquets ROS installés ;
2. retire les chemins Python, Qt et bibliothèques natives hérités d’environnements Conda/venv, puis charge `/opt/ros/humble/setup.bash` ;
3. conserve les paramètres réseau ROS, notamment `ROS_DOMAIN_ID` ;
4. force le backend graphique TkAgg et utilise un cache Matplotlib dans le projet ;
5. vérifie les modules dans des processus séparés, avec traces de panne native ;
6. démarre le manager, qui supervise la cartographie et la navigation.

**Aucune installation et aucune compilation ne sont effectuées au lancement.** Le manager lance directement les scripts Python du projet ; une modification de ces scripts ne nécessite donc pas `colcon build`. RViz devient un outil avancé optionnel. Le bringup TurtleBot reste à lancer sur le robot : le PC ne l’active pas à distance et aucun identifiant SSH n’est mémorisé.

Les anciennes instructions de `DEMODirectives.txt` décrivent une version empaquetée dans `ros2_ws`. Elles ont servi de contexte pour simplifier le parcours ; leurs commandes ne sont pas exécutées automatiquement. Éviter de laisser tourner leurs anciens contrôleurs en parallèle du manager.

## Installation, seulement si nécessaire

```bash
./scripts/installer_dependances.sh --verifier
```

Cette commande lit l’état des paquets APT. Si tout est présent, elle termine sans modification. Pour installer explicitement **les paquets absents uniquement** :

```bash
./scripts/installer_dependances.sh --installer
```

Cela demande les droits APT habituels. Les dépôts Ubuntu et ROS doivent être déjà configurés. Le script n’utilise pas `pip`, ne réinstalle pas les paquets présents et ne reconstruit pas vos workspaces. Sur la machine de validation, tous les paquets listés étaient déjà installés ; aucune installation système n’a été nécessaire.

Par défaut, ROS Humble est utilisé. Pour une autre distribution déjà installée, préciser `TB3_ROS_DISTRO`. Pour un overlay supplémentaire réellement nécessaire, préciser explicitement `TB3_EXTRA_SETUP=/chemin/install/setup.bash`. Le lanceur ne charge pas automatiquement l’ancien `ros2_ws`.

## Vérifier un problème de lancement

```bash
./lancer.sh --diagnostic
```

Cette commande vérifie les imports, versions, chemins des bibliothèques, affichage déclaré, distribution ROS et outils optionnels. Elle n’ouvre pas le manager et ne crée aucun nœud ROS. Un diagnostic sans `DISPLAY` peut réussir pour les dépendances ; l’application graphique exige ensuite une session de bureau ou un affichage X11 transféré.

```bash
./lancer.sh --smoke-test
```

Ce mode vérifie les dépendances, ouvre le vrai manager puis le ferme automatiquement. **Il ne lance aucun contrôleur robot.** Il nécessite un affichage graphique valide.

Chaque tentative possède un dossier distinct :

```text
atelier_slam/logs/launches/<run_id>/
  launcher.log
  report.json
  probe_tkinter.log
  probe_numpy.log
  probe_rclpy.log
  probe_*.log
```

Le rapport donne le code de retour de chaque module et l’emplacement de sa bibliothèque. Une erreur d’import, un délai dépassé ou une segmentation dans une dépendance est isolé et empêche le lancement. Le diagnostic ne tente pas de réinstaller des paquets pour masquer un crash natif.

Après démarrage du manager, ses sorties et erreurs sont dans `logs/manager_runs/<run_id>/console.log`, avec son journal d’actions dans `manager.log`. La session référence aussi le dossier du lanceur dans `session.json`. Voir [le guide des logs par session](GUIDE_ATELIER_SLAM.md).

## Segmentation reproduite et correction

Le démarrage de l’ancien manager a été reproduit avec Python `faulthandler` sur un affichage virtuel isolé. Il se terminait avec le **code 139** avant tout lancement ROS. La trace atteignait `tkinter.Label`, appelé depuis `create_normal_ui`.

Une comparaison contrôlée a conservé l’ancien manager et son environnement, en retirant uniquement en mémoire les pictogrammes Unicode hors BMP des boutons et libellés, notamment l’icône robot. Son démarrage et sa fermeture ont alors réussi. Un essai séparé des polices Arial et DejaVu Sans réussissait : changer seulement la police ne suffisait donc pas à identifier le déclencheur.

La nouvelle interface évite ces pictogrammes. Les messages dynamiques et noms de session passent par `tk_safe_text` avant affichage ; les caractères hors BMP sont remplacés par `[symbole]`. Les accents sont conservés et **les logs conservent le texte original**. Des tests vérifient cette protection et le contenu des widgets. Le déclencheur est ainsi isolé sur cette machine ; l’identification de la bibliothèque native précise demanderait un backtrace natif supplémentaire.

Le lanceur ajoute une protection contre les mélanges d’environnements et active `faulthandler` pour que toute autre segmentation laisse une trace exploitable. Le démarrage complet de la nouvelle application est vérifié avec `--smoke-test`, y compris avec des variables Python/Conda volontairement invalides à l’entrée du lanceur.
