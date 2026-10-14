# Se connecter au robot et lancer le bringup (onglet « Robot (SSH) »)

Dans l'atelier, l'onglet **Robot (SSH)** remplace les lignes de commande :

1. Renseignez l'**adresse** du robot (nom ou IP) et l'**utilisateur** (souvent `ubuntu`). Le port SSH est 22.
2. **Mot de passe** : saisissez-le, ou laissez vide pour utiliser une clé SSH (champ « Clé SSH » ou clé/agent habituels). Le champ est vidé dès le clic sur *Connecter*. Pour ne plus le ressaisir, cochez **Mémoriser dans le trousseau du système** : le mot de passe est alors rangé dans le trousseau de votre session (Secret Service / gnome-keyring), qui le conserve après un redémarrage du PC. Il n'est écrit ni dans le dépôt ni dans un fichier de l'atelier. **Oublier le mot de passe** le supprime du trousseau. Sans trousseau disponible, la case est grisée avec une explication : utilisez alors une clé SSH.
3. Cliquez **Connecter**. À la première connexion, l'empreinte du robot s'affiche : acceptez-la pour l'enregistrer dans `~/.ssh/known_hosts` (comme avec `ssh`).
4. Cliquez **Lancer le bringup** : l'atelier exécute sur le robot `ros2 launch turtlebot3_bringup robot.launch.py` dans un shell interactif du robot (il lit donc son `~/.bashrc`, comme une session ouverte à la main), avec le `ROS_DOMAIN_ID` choisi (30 par défaut, comme `lancer.sh`). « Modèle » et « Modèle LiDAR » ne remplacent les valeurs du robot que si vous les choisissez (LDS-01, LDS-02 ou LDS-03). La sortie du robot s'affiche dans la fenêtre.
5. **Arrêter le bringup** envoie un Ctrl-C au robot ; fermer l'atelier arrête aussi le bringup lancé d'ici.

6. **Lancer la caméra** copie et démarre `serveur_camera.py` sur le robot (port 9999) ; l'onglet Caméra est pré-rempli. Voir `NAVIGATION_FLUIDE.md`.

L'onglet **Connexion ROS** confirme ensuite que `/imu`, `/joint_states` et `/scan` sont bien publiés.

Les réglages sans secret (adresse, utilisateur, clé, domaine, modèle) sont gardés dans `~/.config/atelier_slam/robot.json`, hors du dépôt. Pour une autre commande de bringup, ajoutez `"bringup_command": "..."` dans ce fichier.

Erreurs expliquées en clair : identifiants refusés, robot injoignable, SSH désactivé (port 22 refusé), adresse introuvable.

Prérequis : `python3-paramiko` (installé par `./scripts/installer_dependances.sh --installer`), SSH activé sur le robot, PC et robot sur le même réseau.

Vérifié : tests unitaires et tests contre un vrai serveur SSH local simulé (mot de passe, empreinte, bringup, Ctrl-C). Non vérifié sur un vrai TurtleBot3.

**Caméra** : l'adresse du robot et le port caméra (9999 par défaut) remplissent automatiquement le champ « Adresse du flux » de l'onglet « Caméra optionnelle » dès que l'adresse du robot est saisie.

**Si le mot de passe n'est pas repris** : l'onglet l'indique sous le champ. « Aucun mot de passe enregistré pour ce robot » : saisissez-le une fois avec la case cochée puis *Connecter* (il n'est enregistré qu'après une connexion réussie, pour l'adresse et l'utilisateur exacts ; une autre adresse, par exemple un nom au lieu d'une IP, compte comme un autre robot). « Le trousseau est verrouillé » : déverrouillez-le avec le mot de passe de votre session Ubuntu (au démarrage avec connexion automatique, il reste verrouillé), puis réessayez. L'état de la case est conservé d'une session à l'autre.
