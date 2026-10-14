"""Détection des outils système utilisés par le gestionnaire."""

import subprocess

def check_ffmpeg():
    """Vérifier que ffmpeg est installé"""
    result = subprocess.run(["which", "ffmpeg"], capture_output=True)
    return result.returncode == 0
