#!/usr/bin/env bash
# Crée l'icône « Atelier SLAM » dans le menu des applications et sur le Bureau.
# Utilise le dossier actuel du projet : à relancer si le projet est déplacé.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
desktop_dir="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Bureau")"
apps_dir="$HOME/.local/share/applications"
mkdir -p "$apps_dir" "$desktop_dir"
sed -e "s#@PROJECT_DIR@#$project_dir#g" "$project_dir/assets/Atelier_SLAM.desktop.template" > "$apps_dir/Atelier_SLAM.desktop"
chmod +x "$apps_dir/Atelier_SLAM.desktop"
cp "$apps_dir/Atelier_SLAM.desktop" "$desktop_dir/Atelier_SLAM.desktop"
chmod +x "$desktop_dir/Atelier_SLAM.desktop"
command -v gio >/dev/null && gio set "$desktop_dir/Atelier_SLAM.desktop" metadata::trusted true 2>/dev/null || true
sed -e "s#@PROJECT_DIR@#$project_dir#g" "$project_dir/assets/Arreter_Atelier_SLAM.desktop.template" > "$apps_dir/Arreter_Atelier_SLAM.desktop"
chmod +x "$apps_dir/Arreter_Atelier_SLAM.desktop"
cp "$apps_dir/Arreter_Atelier_SLAM.desktop" "$desktop_dir/Arreter_Atelier_SLAM.desktop"
chmod +x "$desktop_dir/Arreter_Atelier_SLAM.desktop"
command -v gio >/dev/null && gio set "$desktop_dir/Arreter_Atelier_SLAM.desktop" metadata::trusted true 2>/dev/null || true
echo "Icône créée dans $apps_dir et $desktop_dir."
echo "Si elle apparaît grisée sur le Bureau : clic droit > « Autoriser le lancement »."
