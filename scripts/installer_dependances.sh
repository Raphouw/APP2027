#!/usr/bin/env bash
# Explicit, optional first-time setup. Never invoked by lancer.sh.
set -euo pipefail
install=false
case "${1:-}" in
  --installer) install=true ;;
  ''|--verifier) ;;
  --help|-h) echo 'Usage : ./scripts/installer_dependances.sh [--verifier | --installer]'; exit 0 ;;
  *) echo 'Argument inconnu.' >&2; exit 2 ;;
esac
ros_distro="${TB3_ROS_DISTRO:-humble}"
packages=(python3-tk python3-numpy python3-scipy python3-matplotlib python3-pil
  python3-pil.imagetk python3-yaml python3-opencv python3-paramiko ffmpeg
  "ros-$ros_distro-rclpy" "ros-$ros_distro-sensor-msgs" "ros-$ros_distro-nav-msgs"
  "ros-$ros_distro-geometry-msgs" "ros-$ros_distro-std-msgs" "ros-$ros_distro-tf2-ros"
  "ros-$ros_distro-cv-bridge" "ros-$ros_distro-rviz2")
missing=()
for package in "${packages[@]}"; do
  status="$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)"
  if [[ "$status" != 'install ok installed' ]]; then missing+=("$package"); fi
done
if [[ ${#missing[@]} -eq 0 ]]; then
  echo 'Toutes les dépendances système sont déjà présentes. Rien à installer.'
  exit 0
fi
printf 'Paquets absents :\n'
printf '  %s\n' "${missing[@]}"
if [[ "$install" != true ]]; then
  echo 'Pour installer uniquement ces paquets : ./scripts/installer_dependances.sh --installer'
  echo 'Le dépôt APT ROS de votre distribution doit déjà être configuré.'
  exit 2
fi
if [[ "$EUID" -eq 0 ]]; then
  apt-get install --no-install-recommends "${missing[@]}"
else
  sudo apt-get install --no-install-recommends "${missing[@]}"
fi
