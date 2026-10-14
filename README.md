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

## Organisation

| Dossier | Contenu |
|---|---|
| `atelier_slam/` | Code : points d'entrée (`manager.py`, `tb3_nav_node.py`, `carto_mat.py`, `nav_odom.py`), modules dans `slam_components/`, configuration RViz dans `config/` |
| `data/demo_map/` | Carte de démonstration (PGM + YAML) |
| `docs/` | Guides ; `docs/archive/` garde l'ancien résumé d'avant refonte |
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
