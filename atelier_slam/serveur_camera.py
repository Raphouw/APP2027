#!/usr/bin/env python3
"""Serveur de flux caméra pour le robot : images BGR brutes en TCP (port 9999 par défaut).

Ce script tourne SUR LE ROBOT (il est copié et lancé depuis l'onglet « Robot (SSH) »
de l'atelier, ou à la main : `python3 serveur_camera.py`). Il n'a besoin ni de
`cv_bridge` ni de l'atelier : Python 3, numpy, et au moins une source parmi
  - un topic ROS 2 d'images (`/camera/image_raw`, via rclpy), ou
  - une caméra V4L2 (`/dev/video0`, via OpenCV `python3-opencv`).

Chaque image est redimensionnée à largeur×hauteur (640×480 par défaut, ce que l'atelier
attend) puis envoyée telle quelle, sans en-tête : largeur×hauteur×3 octets BGR24.
"""
import argparse
import socket
import sys
import threading
import time

import numpy as np

try:
    import cv2
except ImportError:           # le robot n'a pas forcément OpenCV
    cv2 = None


def log(message):
    print(f'[CAMERA] {message}', flush=True)


def resize_nearest(frame, width, height):
    """Redimensionnement sans OpenCV (plus proche voisin)."""
    rows = np.arange(height)*frame.shape[0]//height
    cols = np.arange(width)*frame.shape[1]//width
    return frame[rows][:, cols]


def to_bgr(data, height, width, encoding, step=None):
    """Convertit le tampon d'un sensor_msgs/Image en tableau BGR uint8 (sans cv_bridge)."""
    encoding = encoding.lower()
    raw = np.frombuffer(data, dtype=np.uint8)
    channels = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1, '8uc1': 1, '8uc3': 3,
                'yuv422_yuy2': 2, 'yuyv': 2, 'yuv422': 2, 'uyvy': 2}.get(encoding)
    if channels is None:
        raise ValueError(f'encodage non pris en charge : {encoding}')
    row_bytes = step or width*channels
    image = raw[:height*row_bytes].reshape(height, row_bytes)[:, :width*channels].reshape(height, width, channels)
    if encoding in ('bgr8', '8uc3'):
        return image
    if encoding == 'rgb8':
        return image[:, :, ::-1]
    if encoding == 'bgra8':
        return image[:, :, :3]
    if encoding == 'rgba8':
        return image[:, :, 2::-1]
    if channels == 1:
        return np.repeat(image, 3, axis=2)
    if cv2 is None:
        raise ValueError('YUV422 demande OpenCV (python3-opencv) sur le robot')
    code = cv2.COLOR_YUV2BGR_UYVY if encoding in ('uyvy', 'yuv422') else cv2.COLOR_YUV2BGR_YUY2
    return cv2.cvtColor(image, code)


class FrameHub:
    """Dernière image reçue, partagée entre la source et les clients TCP."""

    def __init__(self, width, height):
        self.width, self.height = width, height
        self.condition = threading.Condition()
        self.payload, self.sequence = None, 0
        self.received = 0
        self.started = time.time()

    def push(self, frame):
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = (cv2.resize(frame, (self.width, self.height)) if cv2 is not None
                     else resize_nearest(frame, self.width, self.height))
        payload = np.ascontiguousarray(frame, dtype=np.uint8).tobytes()
        with self.condition:
            self.payload, self.sequence = payload, self.sequence+1
            self.condition.notify_all()
        self.received += 1
        if self.received == 1:
            log(f'première image reçue ({frame.shape[1]}x{frame.shape[0]})')
        elif self.received % 150 == 0:
            log(f'{self.received/(time.time()-self.started):.1f} images/s en moyenne')

    def wait_next(self, last, timeout=2.0):
        with self.condition:
            self.condition.wait_for(lambda: self.sequence != last, timeout)
            return self.payload, self.sequence


def serve(hub, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', port))
    sock.listen(5)
    log(f'serveur prêt sur tcp://0.0.0.0:{port}')

    def client(connection, address):
        log(f'client connecté : {address[0]}')
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        last, sent = 0, 0
        try:
            while True:
                payload, sequence = hub.wait_next(last)
                if payload is None or sequence == last:
                    continue               # rien de nouveau : on n'envoie pas deux fois la même image
                connection.sendall(payload)
                last, sent = sequence, sent+1
        except OSError:
            pass
        finally:
            connection.close()
            log(f'client déconnecté : {address[0]} ({sent} images)')

    while True:
        connection, address = sock.accept()
        threading.Thread(target=client, args=(connection, address), daemon=True).start()


def describe_video_devices():
    """Texte de diagnostic : périphériques /dev/video*, leur nom et leurs formats (v4l2-ctl si présent)."""
    import glob
    import shutil
    import subprocess
    devices = sorted(glob.glob('/dev/video*'))
    if not devices:
        return 'aucun /dev/video* : pas de caméra USB/V4L2 détectée (une caméra CSI Raspberry Pi n’en crée pas d’utilisable ici)'
    lines = ['périphériques : '+' '.join(devices)]
    if shutil.which('v4l2-ctl'):
        for command in (['v4l2-ctl', '--list-devices'],):
            try:
                lines.append(subprocess.run(command, capture_output=True, text=True, timeout=5).stdout.strip())
            except Exception:
                pass
        try:
            lines.append(subprocess.run(['v4l2-ctl', '-d', devices[0], '--list-formats-ext'],
                                        capture_output=True, text=True, timeout=5).stdout.strip()[:600])
        except Exception:
            pass
    else:
        lines.append('(installez `sudo apt install v4l-utils` pour lister les formats)')
    return '\n'.join(lines)


def _open_v4l2(device, width, height):
    """Ouvre un périphérique avec le backend V4L2 explicite (et non GStreamer) et lit des images d'essai."""
    candidates = [(cv2.CAP_V4L2, 'MJPG'), (cv2.CAP_V4L2, 'YUYV'), (cv2.CAP_V4L2, None)]
    for backend, fourcc in candidates:
        capture = cv2.VideoCapture(device, backend)
        if not capture.isOpened():
            capture.release()
            continue
        if fourcc:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        for _ in range(30):                        # la 1re image d'une webcam met souvent 1 à 2 s
            ok, frame = capture.read()
            if ok and frame is not None:
                log(f'{device} : image reçue (format {fourcc or "par défaut"}, {frame.shape[1]}x{frame.shape[0]})')
                return capture
            time.sleep(0.1)
        capture.release()
    return None


def run_v4l2(hub, device, fps):
    if cv2 is None:
        raise RuntimeError('OpenCV absent : installez `sudo apt install python3-opencv` sur le robot')
    import glob
    wanted = str(device)
    if wanted.isdigit():
        wanted = f'/dev/video{wanted}'
    # Le périphérique demandé d'abord, puis les autres (sur un Raspberry Pi, video0 n'est souvent pas la caméra).
    others = [d for d in sorted(glob.glob('/dev/video*'), key=lambda d: int(d[10:] or 0)) if d != wanted]
    capture = None
    for candidate in [wanted]+others:
        log(f'essai de {candidate}…')
        capture = _open_v4l2(candidate, hub.width, hub.height)
        if capture is not None:
            break
    if capture is None:
        raise RuntimeError('aucun périphérique vidéo ne délivre d’images.\n'+describe_video_devices()+
                           '\nCaméra CSI (Raspberry Pi) ? Lancez plutôt un nœud ROS qui publie /camera/image_raw '
                           '(`sudo apt install ros-humble-camera-ros`, puis `ros2 run camera_ros camera_node`), '
                           'ou testez la caméra avec `libcamera-hello`.')
    period = 1.0/fps
    failures = 0
    while True:
        started = time.time()
        ok, frame = capture.read()
        if not ok or frame is None:
            failures += 1
            if failures > 50:
                raise RuntimeError('la caméra a cessé de délivrer des images (câble, alimentation USB ?)')
            time.sleep(0.05)
            continue
        failures = 0
        hub.push(frame)
        time.sleep(max(0.0, period-(time.time()-started)))


def run_ros(hub, topic, wait_for_images=None):
    """Souscrit au topic d'images. Retourne si `wait_for_images` s'écoule sans image."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node('serveur_camera')

    def on_image(msg):
        try:
            hub.push(to_bgr(bytes(msg.data), msg.height, msg.width, msg.encoding, msg.step))
        except Exception as exc:
            log(f'image ignorée : {exc}')

    node.create_subscription(Image, topic, on_image, qos_profile_sensor_data)
    log(f'abonné à {topic}')
    deadline = time.time()+wait_for_images if wait_for_images else None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if deadline and not hub.received and time.time() > deadline:
                log(f'aucune image sur {topic} après {wait_for_images:.0f} s')
                return False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=9999)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=15.0, help='cadence maximale (caméra V4L2)')
    parser.add_argument('--source', choices=('auto', 'ros', 'v4l2'), default='auto',
                        help='auto : le topic ROS s’il publie, sinon la caméra V4L2')
    parser.add_argument('--topic', default='/camera/image_raw')
    parser.add_argument('--device', default='0', help='caméra V4L2 : numéro (0) ou chemin (/dev/video0)')
    args = parser.parse_args(argv)
    hub = FrameHub(args.width, args.height)
    threading.Thread(target=serve, args=(hub, args.port), daemon=True).start()
    time.sleep(0.3)
    try:
        if args.source in ('ros', 'auto'):
            try:
                finished = run_ros(hub, args.topic, wait_for_images=5.0 if args.source == 'auto' else None)
            except ImportError:
                log('rclpy introuvable (ROS non chargé dans ce terminal ?)')
                finished = False
                if args.source == 'ros':
                    return 2
            if finished or args.source == 'ros':
                return 0
        run_v4l2(hub, args.device, args.fps)
    except KeyboardInterrupt:
        return 0
    except RuntimeError as exc:
        log(f'ERREUR : {exc}')
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
