# Atelier SLAM · TurtleBot3

Projet scolaire de démonstration SLAM : odométrie, cartographie 2D (ICP + graphe de poses), localisation et navigation d'un TurtleBot3 Burger sous ROS 2 Humble, avec une interface Tkinter.

## Lancer

```bash
./lancer.sh                 # vérifie l'environnement puis ouvre le manager
./lancer.sh --diagnostic    # vérifications seulement
./lancer.sh --smoke-test    # ouvre puis ferme le manager, sans robot
```

Pour une icône cliquable (menu des applications et Bureau) :

```bash
./scripts/installer_icone.sh
```

Dépendances manquantes : `./scripts/installer_dependances.sh --verifier` puis `--installer`. Les dépendances Python sont aussi listées dans `requirements.txt` / `pyproject.toml`.

## Que fait chaque fichier (`atelier_slam/`)

| Fichier | Rôle |
|---|---|
| `atelier.py` | Interface (manager) : lance et arrête tout le reste |
| `noeud_odometrie.py` | Odométrie roues + IMU, navigation réactive, enregistrement CSV |
| `cartographe.py` | Cartographie SLAM (live ou à partir d'un CSV), export de la carte |
| `navigation_carte.py` | Navigation sur une carte sauvegardée |
| `planificateur_chemin.py` | Recherche de chemin sur grille (nom de classe historique `DStarLite`) |
| `revisites_visuelles.py`, `relocalisation_visuelle.py` | Vision : détection de lieux déjà vus, relocalisation SIFT (expérimentale) |
| `flux_video.py`, `serveur_camera.py` | Retour caméra du robot |
| `simulateur.py`, `pilote_simulation.py` | Robot virtuel et pilote automatique |
| `evaluer_trajectoire.py`, `diagnostic_environnement.py` | Mesure d'erreur ; vérification de l'installation |
| `slam_components/` | Modules internes (moteur SLAM, grille, contrôle, interface…) |

## Simulation

Cochez « Mode simulation (sans robot) » dans le manager : un robot virtuel remplace le TurtleBot3. Détails et mesures d'erreur : [docs/SIMULATION.md](docs/SIMULATION.md).

## Organisation

| Dossier | Contenu |
|---|---|
| `atelier_slam/` | Code : points d'entrée (`atelier.py`, `noeud_odometrie.py`, `cartographe.py`, `navigation_carte.py`), modules dans `slam_components/`, configuration RViz dans `config/` |
| `data/demo_map/` | Carte de démonstration (PGM + YAML) |
| `docs/` | Guides (dont [SIMULATION.md](docs/SIMULATION.md), [ROBOT_SSH.md](docs/ROBOT_SSH.md)) ; `docs/archive/` garde l'ancien résumé d'avant refonte |
| `tests/` | Tests unitaires (ROS doit être chargé) ; `tests/manual/` : essai manuel de l'interface |
| `scripts/` | Installation des dépendances et de l'icône |
| `assets/` | Icône et modèle de lanceur |

Les logs et sessions générés vont dans `atelier_slam/logs/` (non versionné).

## Tests

```bash
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s tests
```

## Documentation

[Lancement et diagnostic](docs/LANCEMENT_ET_DIAGNOSTIC.md) · [Guide de l'atelier](docs/GUIDE_ATELIER_SLAM.md) · [Architecture](docs/ARCHITECTURE_CODE.md) · [Algorithme et mesures](docs/SLAM_ALGORITHME_ET_MESURES.md) · [Améliorations et validation](docs/AMELIORATIONS_ET_VALIDATION.md)
