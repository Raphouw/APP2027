"""Démarrage des lecteurs vidéo et affichage des images dans Tkinter.

Les attributs sont initialisés par le point d’entrée ; ce mixin ne lance aucun processus.
"""

import json
import numpy as np
import os
import tkinter as tk
from pathlib import Path
from tkinter import messagebox
from flux_video import RTSPStreamReader, MockStreamReader
from .video_dependencies import HAS_PILLOW, Image, ImageTk
from .system_tools import check_ffmpeg

UI_SETTINGS = Path(os.environ.get('ATELIER_CONFIG_DIR', Path.home()/'.config'/'atelier_slam'))/'ui.json'


def load_ui_settings(path=UI_SETTINGS):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return {}


def save_ui_setting(key, value, path=UI_SETTINGS):
    data = load_ui_settings(path)
    data[key] = value
    try:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(data, indent=2))
    except OSError:
        pass


def rotate_180(frame):
    """Image retournée de 180° (caméra montée à l'envers) : vue sans copie, ordre BGR/RGB inchangé."""
    return frame[::-1, ::-1]


class ManagerVideoMixin:

    def start_video(self):
        """Démarrer la lecture du flux RTSP"""
        if not check_ffmpeg():
            messagebox.showerror(
                "FFMPEG manquant",
                "FFMPEG n'est pas installé sur ce système.\n\n"
                "Installez-le avec:\n"
                "  sudo apt-get update\n"
                "  sudo apt-get install ffmpeg\n\n"
                "Ceci est nécessaire sur la machine exécutant atelier.py,\n"
                "pas sur le robot."
            )
            return

        url = self.rtsp_entry.get()
        if not url:
            messagebox.showerror("Erreur", "Entrez une URL RTSP")
            return

        print(f"[MANAGER] Tentative de connexion au flux vidéo: {url}")
        print("[MANAGER] Le serveur caméra se lance depuis l'onglet « Robot (SSH) » → « Lancer la caméra ».")
        print("[MANAGER] Les logs s'afficheront dans le terminal...")

        self.video_reader = RTSPStreamReader(url, width=640, height=480)
        self.video_reader.start()
        self.video_enabled = True
        self.btn_video.config(state=tk.DISABLED, text="Vidéo active")
        self.info_label.config(text="Vidéo: Connexion en cours...", fg="#f39c12")
        self.update_video_frame()

    def start_mock_video(self):
        """Démarrer une vidéo de test (sans caméra réelle)"""
        self.video_reader = MockStreamReader(width=640, height=480)
        self.video_reader.start()
        self.video_enabled = True
        self.btn_video.config(state=tk.DISABLED, text="Vidéo active")
        self.update_video_frame()

    def update_video_frame(self):
        """Mettre à jour l'affichage vidéo (appelé périodiquement)"""
        if not self.video_enabled or not self.video_reader:
            return

        try:
            # Vérifier que le widget n'a pas été détruit
            self.video_label.winfo_exists()

            frame = self.video_reader.get_frame()
            fps = self.video_reader.get_fps()
            is_connected = self.video_reader.is_connected()
            status = "Connecté" if is_connected else "Connexion en cours..."

            if frame is not None and HAS_PILLOW:
                try:
                    import cv2
                    # Convertir BGR vers RGB et redimensionner
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    frame_rgb = cv2.resize(frame_rgb, (640, 480))
                    if self.camera_flip.get():
                        frame_rgb = rotate_180(frame_rgb)
                except ImportError:
                    frame_rgb = rotate_180(frame) if self.camera_flip.get() else frame  # Fallback: use frame as-is

                # Convertir en PhotoImage
                img = Image.fromarray(np.ascontiguousarray(frame_rgb))
                photo = ImageTk.PhotoImage(img)

                self.video_label.config(image=photo)
                self.video_label.image = photo
                self.info_label.config(text=f"Vidéo: {status} | FPS: {fps}", fg="#27ae60" if is_connected else "#f39c12")
            else:
                # Afficher un message d'attente si pas de frame
                if not HAS_PILLOW:
                    self.info_label.config(text="PIL/Pillow non installé", fg="#e74c3c")
                else:
                    error = getattr(self.video_reader, 'error', '')
                    self.info_label.config(text=f"Vidéo : {error}" if error else f"Vidéo: {status} (en attente de frames...)",
                                           fg="#e74c3c" if error else "#f39c12")

            if self.video_enabled:
                self.root.after(50, self.update_video_frame)  # ~20 FPS
        except tk.TclError:
            # Widget détruit, arrêter les callbacks
            self.video_enabled = False
