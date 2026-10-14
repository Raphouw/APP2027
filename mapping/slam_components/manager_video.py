"""Démarrage des lecteurs vidéo et affichage des images dans Tkinter.

Les attributs sont initialisés par le point d’entrée ; ce mixin ne lance aucun processus.
"""

import tkinter as tk
from tkinter import messagebox
from video_stream import RTSPStreamReader, MockStreamReader
from .video_dependencies import HAS_PILLOW, Image, ImageTk
from .system_tools import check_ffmpeg

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
                "Ceci est nécessaire sur la machine exécutant manager.py,\n"
                "pas sur le robot."
            )
            return

        url = self.rtsp_entry.get()
        if not url:
            messagebox.showerror("Erreur", "Entrez une URL RTSP")
            return

        print(f"[MANAGER] Tentative de connexion au flux vidéo: {url}")
        print("[MANAGER] Assurez-vous que ros_raw_stream.py est en cours sur le robot:")
        print("  - Sur le robot: python3 ros_raw_stream.py &")
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
                except ImportError:
                    frame_rgb = frame  # Fallback: use frame as-is

                # Convertir en PhotoImage
                img = Image.fromarray(frame_rgb)
                photo = ImageTk.PhotoImage(img)

                self.video_label.config(image=photo)
                self.video_label.image = photo
                self.info_label.config(text=f"Vidéo: {status} | FPS: {fps}", fg="#27ae60" if is_connected else "#f39c12")
            else:
                # Afficher un message d'attente si pas de frame
                if not HAS_PILLOW:
                    self.info_label.config(text="PIL/Pillow non installé", fg="#e74c3c")
                else:
                    self.info_label.config(text=f"Vidéo: {status} (en attente de frames...)", fg="#f39c12")

            if self.video_enabled:
                self.root.after(50, self.update_video_frame)  # ~20 FPS
        except tk.TclError:
            # Widget détruit, arrêter les callbacks
            self.video_enabled = False
