"""Serveur de flux caméra (côté robot) et son lancement depuis l'onglet Robot (SSH)."""
import socket
import sys
import threading
import time
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'atelier_slam'))
import serveur_camera as cam
from slam_components import robot_ssh as r

GOOD = dict(r.DEFAULTS, host='192.168.1.50', user='ubuntu')


class ServerTests(unittest.TestCase):
    def test_encodings_are_converted_without_cv_bridge(self):
        rgb = np.zeros((2, 2, 3), np.uint8)
        rgb[:, :] = (10, 20, 30)
        bgr = cam.to_bgr(rgb.tobytes(), 2, 2, 'rgb8')
        self.assertEqual(tuple(bgr[0, 0]), (30, 20, 10))
        mono = cam.to_bgr(bytes([7, 8, 9, 10]), 2, 2, 'mono8')
        self.assertEqual(mono.shape, (2, 2, 3))
        with self.assertRaises(ValueError):
            cam.to_bgr(b'', 1, 1, 'bayer_rggb8')

    def test_nv21_nv12_colours_and_stride(self):
        import cv2
        h, w = 48, 64
        bgr = np.zeros((h, w, 3), np.uint8)
        bgr[:, :] = (0, 0, 255)                                   # rouge
        flat = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).reshape(-1)
        y, u, v = flat[:h*w].reshape(h, w), flat[h*w:h*w*5//4].reshape(h//2, w//2), flat[h*w*5//4:].reshape(h//2, w//2)
        vu = np.empty((h//2, w), np.uint8)
        vu[:, 0::2], vu[:, 1::2] = v, u
        nv21 = np.vstack([y, vu])
        out = cam.to_bgr(nv21.tobytes(), h, w, 'nv21', step=w)
        self.assertLessEqual(abs(int(out[5, 5, 2])-255)+int(out[5, 5, 0])+int(out[5, 5, 1]), 6)
        padded = np.pad(nv21, ((0, 0), (0, 16)))
        self.assertTrue((cam.to_bgr(padded.tobytes(), h, w, 'nv21', step=w+16) == out).all())
        uv = np.empty((h//2, w), np.uint8)
        uv[:, 0::2], uv[:, 1::2] = u, v
        self.assertTrue((cam.to_bgr(np.vstack([y, uv]).tobytes(), h, w, 'nv12', step=w) == out).all())

    def test_row_padding_is_ignored(self):
        data = bytes([1, 2, 3, 0, 0]*2)   # 1 pixel BGR + 2 octets de bourrage par ligne
        self.assertEqual(cam.to_bgr(data, 2, 1, 'bgr8', step=5).shape, (2, 1, 3))

    def test_frames_are_resized_and_streamed_once_each(self):
        hub = cam.FrameHub(8, 6)
        sock = socket.socket()
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
        sock.close()
        threading.Thread(target=cam.serve, args=(hub, port), daemon=True).start()
        time.sleep(0.3)
        client = socket.create_connection(('127.0.0.1', port), timeout=3)
        hub.push(np.full((12, 16, 3), 5, np.uint8))     # 16x12 -> 8x6
        data = b''
        while len(data) < 8*6*3:
            data += client.recv(8*6*3-len(data))
        self.assertEqual(data, bytes([5])*(8*6*3))
        client.settimeout(0.5)
        with self.assertRaises(socket.timeout):          # pas de renvoi de la même image
            client.recv(1)


class FlipTests(unittest.TestCase):
    def test_rotation_180_and_saved_setting(self):
        import tempfile
        from slam_components.manager_video import rotate_180, load_ui_settings, save_ui_setting
        frame = np.arange(2*3*3, dtype=np.uint8).reshape(2, 3, 3)
        flipped = rotate_180(frame)
        self.assertTrue((flipped[0, 0] == frame[1, 2]).all() and (flipped[1, 2] == frame[0, 0]).all())
        self.assertTrue((rotate_180(flipped) == frame).all())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'ui.json'
            self.assertEqual(load_ui_settings(path), {})
            save_ui_setting('camera_flip_180', False, path)
            self.assertFalse(load_ui_settings(path)['camera_flip_180'])


class _Channel:
    def __init__(self):
        self.command, self.sent, self.closed = None, [], False

    def get_pty(self): pass
    def exec_command(self, command): self.command = command
    def recv(self, size): time.sleep(0.05); return b'' if self.closed else b'[CAMERA] serveur pret\n'
    def send(self, data): self.sent.append(data)
    def exit_status_ready(self): return True
    def recv_exit_status(self): return 0
    def close(self): self.closed = True


class _Sftp:
    def __init__(self): self.uploaded = []
    def mkdir(self, path): pass
    def put(self, local, remote): self.uploaded.append((local, remote))
    def close(self): pass


class _Client:
    def __init__(self):
        self.channels, self.sftp = [], _Sftp()

    def connect(self, host, **kwargs): pass
    def open_sftp(self): return self.sftp
    def close(self): pass

    def get_transport(self):
        client = self
        class Transport:
            def open_session(self):
                channel = _Channel()
                client.channels.append(channel)
                return channel
        return Transport()


class RemoteStartTests(unittest.TestCase):
    def test_script_is_uploaded_then_started_and_stopped_with_ctrl_c(self):
        client = _Client()
        link = r.RobotLink(lambda: client)
        self.assertFalse(link.start_camera(GOOD))     # pas connecté
        link.connect(GOOD)
        self.assertTrue(link.start_camera(GOOD))
        self.assertEqual(client.sftp.uploaded[0][1], 'atelier_slam/serveur_camera.py')
        self.assertTrue(Path(client.sftp.uploaded[0][0]).is_file())
        kill, launch = client.channels
        self.assertIn('pkill', kill.command)
        self.assertIn('serveur_camera.py --port 9999', launch.command)
        self.assertIn('ROS_DOMAIN_ID=30', launch.command)
        self.assertEqual(r.camera_url(GOOD), 'tcp://192.168.1.50:9999')
        self.assertTrue(link.camera_running)
        link.stop_camera()
        self.assertEqual(launch.sent, ['\x03'])
        self.assertFalse(link.camera_running)

    def test_camera_error_from_robot_is_explained(self):
        self.assertIn('caméra', r.hint_for('[CAMERA] ERREUR : impossible d’ouvrir la caméra 0'))


if __name__ == '__main__':
    unittest.main()
