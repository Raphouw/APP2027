#!/usr/bin/env python3
"""
Video Stream Reader — RTSP/HTTP MJPEG via FFMPEG
==================================================

Lit un flux vidéo (RTSP ou HTTP MJPEG) via FFMPEG.
Supporte :
- HTTP MJPEG (ex: http://10.141.177.146:8080/stream)
- RTSP (ex: rtsp://host:8554/stream)

Décode et met à jour les frames en temps-réel dans un thread.
"""

import subprocess
import numpy as np
import threading
import time
import socket


class RTSPStreamReader:
    """
    Lecteur de flux RTSP asynchrone.
    Compatible avec :
    - Raspberry Pi Camera V2 (via picamera ou libcamera)
    - Caméras IP (RTSP, HTTP)
    - Serveurs streaming
    """

    def __init__(self, rtsp_url, width=640, height=480, fps=30):
        """
        Initialiser le lecteur de flux vidéo.

        Args:
            rtsp_url: URL (ex: tcp://10.141.177.146:9999 pour flux brut, ou http://... MJPEG)
            width: Largeur de la vidéo décodée (pixels)
            height: Hauteur de la vidéo décodée (pixels)
            fps: Framerate cible
        """
        self.rtsp_url = rtsp_url
        self.width = width
        self.height = height
        self.fps = fps
        self.is_raw_stream = rtsp_url.startswith('tcp://')

        self.frame = None
        self.frame_lock = threading.Lock()
        self.is_running = False
        self.thread = None
        self.process = None

        self.error = ''
        self.frame_count = 0
        self.fps_actual = 0
        self.last_fps_time = time.time()

    def start(self):
        """Démarrer la lecture du flux RTSP."""
        if self.is_running:
            return

        self.is_running = True
        self.thread = threading.Thread(target=self._read_stream, daemon=True)
        self.thread.start()
        print(f"[RTSP] Stream started: {self.rtsp_url}")

    def stop(self):
        """Arrêter la lecture du flux."""
        self.is_running = False

        if self.process:
            try:
                self.process.terminate()
                self.process.wait(timeout=2)
            except:
                self.process.kill()

        if self.thread:
            self.thread.join(timeout=2)

        print("[RTSP] Stream stopped")

    def _read_stream(self):
        """
        Boucle de lecture du flux vidéo (RTSP/MJPEG via FFMPEG ou TCP brut).
        Cette fonction s'exécute dans un thread séparé.
        """
        if self.is_raw_stream:
            self._read_raw_stream()
        else:
            self._read_ffmpeg_stream()

    def _read_raw_stream(self):
        """Lire un flux TCP brut (raw BGR24)"""
        try:
            print(f"[RTSP] Connexion au flux brut: {self.rtsp_url}")

            # Parser l'URL TCP
            url_parts = self.rtsp_url.replace('tcp://', '').split(':')
            host = url_parts[0]
            port = int(url_parts[1]) if len(url_parts) > 1 else 9999

            sock = None
            while self.is_running and sock is None:
                # Le serveur peut démarrer après le client (lancement depuis l'onglet Robot) : on réessaie.
                candidate = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                candidate.settimeout(3)
                try:
                    candidate.connect((host, port))
                except OSError as exc:
                    candidate.close()
                    self.error = ('Aucun serveur caméra ne répond sur le robot (nouvel essai toutes les 2 s). '
                                  'Onglet « Robot (SSH) » → « Lancer la caméra ». ') + f'({exc})'
                    time.sleep(2)
                    continue
                candidate.settimeout(None)
                sock = candidate
            if sock is None:
                return
            self.error = ''
            print(f"[RTSP] ✓ Connecté à {host}:{port}")

            frame_size = self.width * self.height * 3  # BGR24 (anciens serveurs, sans compression)
            frames_received = 0
            pending = b''

            def read_exact(count):
                nonlocal pending
                while len(pending) < count:
                    chunk = sock.recv(max(65536, count-len(pending)))
                    if not chunk:
                        raise ConnectionError('Le serveur caméra du robot a fermé la connexion.')
                    pending += chunk
                data, pending = pending[:count], pending[count:]
                return data

            while self.is_running:
                try:
                    head = read_exact(4)
                    if head == b'JPG1':
                        # Serveur récent : image JPEG précédée de sa taille (≈ 40 Ko au lieu de 900 Ko).
                        size = int.from_bytes(read_exact(4), 'big')
                        packed = np.frombuffer(read_exact(size), dtype=np.uint8)
                        import cv2
                        frame = cv2.imdecode(packed, cv2.IMREAD_COLOR)
                        if frame is None:
                            continue
                        data = b'x'
                    else:
                        data = head+read_exact(frame_size-4)
                        frame = np.frombuffer(data, dtype=np.uint8).reshape((self.height, self.width, 3))

                    frames_received += 1
                    if frames_received == 1:
                        print(f"[RTSP] ✓ Premier frame reçu! {frame.shape[1]}x{frame.shape[0]}")

                    # Mettre à jour le frame
                    with self.frame_lock:
                        self.frame = frame
                        self.frame_count += 1

                    # Calculer FPS
                    now = time.time()
                    if now - self.last_fps_time >= 1.0:
                        self.fps_actual = self.frame_count
                        print(f"[RTSP] FPS: {self.fps_actual} | Frames reçus: {frames_received}")
                        self.frame_count = 0
                        self.last_fps_time = now

                except ConnectionError as e:
                    self.error = str(e)
                    return
                except Exception as e:
                    print(f"[RTSP] Read error: {e}")
                    break

        except Exception as e:
            print(f"[RTSP] TCP connection error: {e}")
            self.error = ('Connexion refusée : aucun serveur caméra ne tourne sur le robot. '
                          'Onglet « Robot (SSH) » → « Lancer la caméra ».') if isinstance(e, ConnectionRefusedError) else f'Connexion impossible : {e}'
        finally:
            try:
                sock.close()
            except:
                pass

    def _read_ffmpeg_stream(self):
        """Lire un flux MJPEG/RTSP via FFMPEG"""
        try:
            # Command FFMPEG pour lire HTTP MJPEG ou RTSP et décoder en raw video
            cmd = [
                'ffmpeg',
                '-rtsp_transport', 'tcp',  # TCP si RTSP
                '-i', self.rtsp_url,
                '-f', 'rawvideo',
                '-pix_fmt', 'bgr24',  # BGR pour OpenCV
                '-vf', f'scale={self.width}:{self.height}',  # Redimensionner si nécessaire
                '-framerate', str(self.fps),
                '-'  # stdout
            ]

            print(f"[RTSP] FFMPEG command: {' '.join(cmd)}")

            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=10
            )

            # Lancer thread pour capturer stderr (logs FFMPEG)
            self._stderr_thread = threading.Thread(target=self._read_stderr, daemon=True)
            self._stderr_thread.start()

            frame_size = self.width * self.height * 3  # 3 channels BGR

            while self.is_running:
                try:
                    # Lire une frame complète
                    data = self.process.stdout.read(frame_size)

                    if len(data) != frame_size:
                        print(f"[RTSP] Frame incomplete: {len(data)}/{frame_size}")
                        break

                    # Convertir les données brutes en np.ndarray
                    frame = np.frombuffer(data, dtype=np.uint8)
                    frame = frame.reshape((self.height, self.width, 3))

                    # Mettre à jour le frame avec verrou
                    with self.frame_lock:
                        self.frame = frame
                        self.frame_count += 1

                    # Calculer FPS réel
                    now = time.time()
                    if now - self.last_fps_time >= 1.0:
                        self.fps_actual = self.frame_count
                        self.frame_count = 0
                        self.last_fps_time = now

                except Exception as e:
                    print(f"[RTSP] Read error: {e}")
                    break

        except FileNotFoundError:
            print("[RTSP] ERROR: ffmpeg not found. Install with: sudo apt-get install ffmpeg")
        except Exception as e:
            print(f"[RTSP] Stream error: {e}")
        finally:
            if self.process:
                self.process.terminate()

    def _read_stderr(self):
        """
        Lire et afficher les messages d'erreur FFMPEG.
        Utile pour déboguer les problèmes de connexion RTSP.
        """
        if self.process and self.process.stderr:
            try:
                for line in iter(self.process.stderr.readline, b''):
                    if line:
                        msg = line.decode('utf-8', errors='ignore').strip()
                        if msg:
                            print(f"[FFMPEG] {msg}")
            except:
                pass

    def get_frame(self):
        """
        Retourner le dernier frame reçu.

        Returns:
            np.ndarray BGR ou None si pas de frame
        """
        with self.frame_lock:
            return self.frame.copy() if self.frame is not None else None

    def is_connected(self):
        """Vérifier si la stream est connectée."""
        return self.is_running and self.frame is not None

    def get_fps(self):
        """Retourner le FPS réel."""
        return self.fps_actual


class MockStreamReader:
    """
    Mock reader pour tester sans RTSP réel.
    Génère des frames de test.
    """

    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.frame_count = 0
        self.is_running = False

    def start(self):
        self.is_running = True
        print("[MOCK] Mock stream started")

    def stop(self):
        self.is_running = False
        print("[MOCK] Mock stream stopped")

    def get_frame(self):
        if not self.is_running:
            return None

        # Générer une frame de test (gradients)
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Gradient horizontal bleu
        for y in range(self.height):
            frame[y, :, 0] = int(255 * y / self.height)

        # Gradient vertical vert
        for x in range(self.width):
            frame[:, x, 1] = int(255 * x / self.width)

        # Ajouter du texte (cadre blanc)
        frame[:, :2] = [255, 255, 255]
        frame[-2:, :] = [255, 255, 255]

        self.frame_count += 1
        return frame

    def is_connected(self):
        return self.is_running

    def get_fps(self):
        return 30
