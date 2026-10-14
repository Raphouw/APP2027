"""Teste RobotLink contre un vrai serveur SSH (paramiko) local, sans robot."""
import socket
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'atelier_slam'))
import paramiko
from slam_components import robot_ssh as r


class FakeRobot(paramiko.ServerInterface):
    def __init__(self):
        self.command = None
        self.got_ctrl_c = threading.Event()
        self.started = threading.Event()

    def check_auth_password(self, username, password):
        ok = username == 'ubuntu' and password == 'turtlebot'
        return paramiko.AUTH_SUCCESSFUL if ok else paramiko.AUTH_FAILED

    def get_allowed_auths(self, username): return 'password'
    def check_channel_request(self, kind, chanid): return paramiko.OPEN_SUCCEEDED
    def check_channel_pty_request(self, channel, *args): return True

    def check_channel_exec_request(self, channel, command):
        self.command = command.decode()
        threading.Thread(target=self._run, args=(channel,), daemon=True).start()
        return True

    def _run(self, channel):
        channel.send(b'[INFO] bringup demarre\r\n')
        self.started.set()
        while not channel.closed:
            if channel.recv_ready() and b'\x03' in channel.recv(16):
                self.got_ctrl_c.set()
                channel.send(b'^C arret\r\n')
                channel.send_exit_status(0)
                channel.close()
                return
            time.sleep(0.02)


class SshServerTests(unittest.TestCase):
    def setUp(self):
        self.key = paramiko.RSAKey.generate(2048)
        self.listener = socket.socket()
        self.listener.bind(('127.0.0.1', 0))
        self.listener.listen(1)
        self.port = self.listener.getsockname()[1]
        self.robot = FakeRobot()
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()
        self.directory = tempfile.TemporaryDirectory()
        self.patch = patch.object(r, 'KNOWN_HOSTS', Path(self.directory.name)/'known_hosts')
        self.patch.start()

    def tearDown(self):
        self.patch.stop()
        self.listener.close()
        self.directory.cleanup()

    def _serve(self):
        try:
            connection, _ = self.listener.accept()
        except OSError:
            return
        transport = paramiko.Transport(connection)
        transport.add_server_key(self.key)
        transport.start_server(server=self.robot)
        self.transport = transport

    def settings(self):
        return dict(r.DEFAULTS, host='127.0.0.1', port=self.port, user='ubuntu')

    def test_full_flow_with_password_host_key_and_ctrl_c(self):
        asked = []
        link = r.RobotLink()
        ok = link.connect(self.settings(), 'turtlebot', lambda host, fp: asked.append(fp) or True)
        self.assertTrue(ok, link.message)
        self.assertTrue(asked[0].startswith('SHA256:'))
        self.assertIn('ssh-rsa', (Path(self.directory.name)/'known_hosts').read_text())
        self.assertTrue(link.start_bringup(self.settings()))
        self.assertTrue(self.robot.started.wait(5))
        self.assertIn('turtlebot3_bringup', self.robot.command)
        link.stop_bringup(timeout=5)
        self.assertTrue(self.robot.got_ctrl_c.wait(5))
        lines = []
        while not link.output.empty():
            lines.append(link.output.get())
        self.assertIn('[INFO] bringup demarre', lines)
        link.disconnect()
        self.assertEqual(link.state, r.DISCONNECTED)

    def test_wrong_password_and_refused_host_key(self):
        link = r.RobotLink()
        self.assertFalse(link.connect(self.settings(), 'mauvais', lambda host, fp: True))
        self.assertIn('Identifiants refusés', link.message)

    def test_host_key_refusal_cancels_the_connection(self):
        link = r.RobotLink()
        self.assertFalse(link.connect(self.settings(), 'turtlebot', lambda host, fp: False))
        self.assertEqual(link.state, r.ERROR)
        self.assertIn('annulée', link.message)


if __name__ == '__main__':
    unittest.main()
