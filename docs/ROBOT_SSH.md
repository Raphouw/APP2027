# Se connecter au robot et lancer le bringup (onglet « Robot (SSH) »)

Dans l'atelier, l'onglet **Robot (SSH)** remplace les lignes de commande :

1. Renseignez l'**adresse** du robot (nom ou IP) et l'**utilisateur** (souvent `ubuntu`). Le port SSH est 22.
2. **Mot de passe** : saisissez-le, ou laissez vide pour utiliser une clé SSH (champ « Clé SSH » ou clé/agent habituels). Le mot de passe n'est jamais enregistré et le champ est vidé dès le clic sur *Connecter*.
3. Cliquez **Connecter**. À la première connexion, l'empreinte du robot s'affiche : acceptez-la pour l'enregistrer dans `~/.ssh/known_hosts` (comme avec `ssh`).
4. Cliquez **Lancer le bringup** : l'atelier exécute sur le robot `ros2 launch turtlebot3_bringup robot.launch.py` avec le modèle et le `ROS_DOMAIN_ID` choisis (30 par défaut, comme `lancer.sh`). La sortie du robot s'affiche dans la fenêtre.
5. **Arrêter le bringup** envoie un Ctrl-C au robot ; fermer l'atelier arrête aussi le bringup lancé d'ici.

L'onglet **Connexion ROS** confirme ensuite que `/imu`, `/joint_states` et `/scan` sont bien publiés.

Les réglages sans secret (adresse, utilisateur, clé, domaine, modèle) sont gardés dans `~/.config/atelier_slam/robot.json`, hors du dépôt. Pour une autre commande de bringup, ajoutez `"bringup_command": "..."` dans ce fichier.

Erreurs expliquées en clair : identifiants refusés, robot injoignable, SSH désactivé (port 22 refusé), adresse introuvable.

Prérequis : `python3-paramiko` (installé par `./scripts/installer_dependances.sh --installer`), SSH activé sur le robot, PC et robot sur le même réseau.

Vérifié : tests unitaires et tests contre un vrai serveur SSH local simulé (mot de passe, empreinte, bringup, Ctrl-C). Non vérifié sur un vrai TurtleBot3.
