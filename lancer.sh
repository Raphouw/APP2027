#!/bin/bash
# No installs, no builds, no robot bringup: prepare this PC and launch the manager.
set -eo pipefail
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
action=launch
manager_args=()
for argument in "$@"; do
  case "$argument" in
    --diagnostic) action=diagnostic ;;
    --smoke-test) manager_args+=(--smoke-test) ;;
    --help|-h)
      echo 'Usage : ./lancer.sh [--diagnostic | --smoke-test]'
      echo 'Charge ROS et vérifie les dépendances, sans installer ni compiler.'
      echo '--diagnostic : vérifications et rapport, sans ouvrir le manager.'
      echo '--smoke-test : ouvre puis ferme le manager, sans lancer de robot.'
      exit 0 ;;
    *) echo "Argument inconnu : $argument" >&2; exit 2 ;;
  esac
done
# Preserve ROS networking options, but discard mixed Python/Qt/Conda runtime paths.
unset PYTHONHOME PYTHONPATH LD_PRELOAD LD_LIBRARY_PATH QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH
unset VIRTUAL_ENV CONDA_PREFIX CONDA_DEFAULT_ENV
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1 PYTHONFAULTHANDLER=1 MPLBACKEND=TkAgg
export MPLCONFIGDIR="$project_dir/atelier_slam/logs/cache/matplotlib"
run_id="$(date +%Y%m%d_%H%M%S_%N)"
launch_dir="$project_dir/atelier_slam/logs/launches/$run_id"
mkdir -p "$launch_dir" "$MPLCONFIGDIR"
exec > >(tee -a "$launch_dir/launcher.log") 2>&1
echo "Atelier SLAM · démarrage $run_id"
echo "Diagnostic : $launch_dir"
ros_setup="/opt/ros/${TB3_ROS_DISTRO:-humble}/setup.bash"
if [[ ! -f "$ros_setup" ]]; then
  echo "ROS introuvable : $ros_setup. Voir docs/LANCEMENT_ET_DIAGNOSTIC.md."
  exit 2
fi
# ROS setup scripts are not compatible with nounset.
source "$ros_setup"
if [[ -n "${TB3_EXTRA_SETUP:-}" ]]; then
  if [[ ! -f "$TB3_EXTRA_SETUP" ]]; then echo "Overlay absent : $TB3_EXTRA_SETUP"; exit 2; fi
  source "$TB3_EXTRA_SETUP"
fi
export TB3_LAUNCH_READY=1 TB3_LAUNCH_LOG_DIR="$launch_dir"
cd "$project_dir"
set +e
/usr/bin/python3 -s -X faulthandler "$project_dir/atelier_slam/diagnostic_environnement.py" --output "$launch_dir"
check_code=$?
set -e
if [[ "$check_code" -ne 0 ]]; then
  echo "Diagnostic en échec ($check_code). Aucun composant robot n’a été démarré."
  echo 'Consultez report.json et les fichiers probe_*.log dans le dossier ci-dessus.'
  echo 'Pour des paquets absents uniquement : ./scripts/installer_dependances.sh (vérification).'
  exit "$check_code"
fi
if [[ "$action" == diagnostic ]]; then
  echo 'Diagnostic terminé. Aucune installation ni compilation effectuée.'
  exit 0
fi
if [[ -z "${DISPLAY:-}" ]]; then
  echo 'Aucun affichage X11 disponible. Ouvrez une session graphique, puis relancez ./lancer.sh.'
  echo 'Le diagnostic reste disponible avec ./lancer.sh --diagnostic.'
  exit 4
fi
set +e
/usr/bin/python3 -s -X faulthandler "$project_dir/atelier_slam/atelier.py" "${manager_args[@]}"
manager_code=$?
set -e
if [[ "$manager_code" -ne 0 ]]; then
  echo "Le manager s’est arrêté avec le code $manager_code."
  echo "Consultez $launch_dir/launcher.log et atelier_slam/logs/manager_runs/."
  if [[ "$manager_code" -eq 139 ]]; then echo 'Crash natif : la trace faulthandler est conservée dans console.log.'; fi
fi
exit "$manager_code"
