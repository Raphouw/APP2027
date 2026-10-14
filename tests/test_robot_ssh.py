import json
import sys
import tempfile
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'atelier_slam'))
from slam_components import robot_ssh as r


class FakeChannel:
    def __init__(self, chunks):
        self.chunks, self.sent, self.closed, self.command = list(chunks), [], False, None
        self.done = False

    def get_pty(self): pass
    def exec_command(self, command): self.command = command

    def recv(self, size):
        if self.chunks:
            return self.chunks.pop(0)
        while not self.done and not self.closed:
            time.sleep(0.01)
        return b''

    def send(self, data):
        self.sent.append(data)
        if data == '\x03':
            self.done = True

    def exit_status_ready(self): return self.done
    def recv_exit_status(self): return 0
    def close(self): self.closed = True


class FakeTransport:
    def __init__(self, channel): self.channel = channel
    def open_session(self): return self.channel


class FakeClient:
    def __init__(self, channel=None, error=None):
        self.channel, self.error, self.kwargs, self.closed = channel, error, None, False

    def connect(self, host, **kwargs):
        if self.error:
            raise self.error
        self.host, self.kwargs = host, kwargs

    def get_transport(self): return FakeTransport(self.channel)
    def close(self): self.closed = True


GOOD = dict(r.DEFAULTS, host='192.168.1.50', user='ubuntu')


class SettingsTests(unittest.TestCase):
    def test_validation_messages(self):
        self.assertIn('adresse', r.validate(dict(GOOD, host='')).lower())
        self.assertIn('invalide', r.validate(dict(GOOD, host='a b; rm -rf /')))
        self.assertIn('0 et 232', r.validate(dict(GOOD, ros_domain_id=999)))
        self.assertIn('introuvable', r.validate(dict(GOOD, key_file='/nope/id_rsa')))
        self.assertIsNone(r.validate(GOOD))

    def test_default_bringup_command_uses_model_and_domain(self):
        command = r.bringup_command(dict(GOOD, model='waffle', ros_domain_id=7))
        self.assertIn('TURTLEBOT3_MODEL=waffle', command)
        self.assertIn('ROS_DOMAIN_ID=7', command)
        self.assertIn('turtlebot3_bringup robot.launch.py', command)
        self.assertEqual(r.bringup_command(dict(GOOD, bringup_command='ros2 run x y')), 'ros2 run x y')

    def test_saved_settings_never_contain_a_password(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'robot.json'
            r.save_settings(dict(GOOD, password='secret', passwd='x'), path)
            self.assertNotIn('secret', path.read_text())
            self.assertEqual(set(json.loads(path.read_text())), set(r.DEFAULTS))
            self.assertEqual(r.load_settings(path)['host'], '192.168.1.50')
            self.assertEqual(r.load_settings(Path(directory)/'absent.json'), r.DEFAULTS)


class LinkTests(unittest.TestCase):
    def test_connect_passes_credentials_and_reports_state(self):
        client = FakeClient()
        link = r.RobotLink(lambda: client)
        self.assertTrue(link.connect(GOOD, password='pw'))
        self.assertEqual(link.state, r.CONNECTED)
        self.assertEqual(client.kwargs['password'], 'pw')
        self.assertFalse(client.kwargs['look_for_keys'])

    def test_invalid_settings_do_not_open_a_connection(self):
        called = []
        link = r.RobotLink(lambda: called.append(1))
        self.assertFalse(link.connect(dict(GOOD, host='')))
        self.assertEqual((link.state, called), (r.ERROR, []))

    def test_connection_errors_are_explained(self):
        import socket
        link = r.RobotLink(lambda: FakeClient(error=socket.timeout()))
        self.assertFalse(link.connect(GOOD))
        self.assertIn('ne répond pas', link.message)
        import paramiko
        link = r.RobotLink(lambda: FakeClient(error=paramiko.AuthenticationException()))
        link.connect(GOOD)
        self.assertIn('Identifiants refusés', link.message)

    def test_bringup_streams_output_and_stops_with_ctrl_c(self):
        channel = FakeChannel([b'[INFO] lds started\r\n[INFO] ready', b'\n'])
        link = r.RobotLink(lambda: FakeClient(channel))
        link.connect(GOOD)
        self.assertTrue(link.start_bringup(GOOD))
        self.assertEqual(link.state, r.BRINGUP)
        self.assertTrue(channel.command.startswith('bash -lc '))
        deadline = time.monotonic()+2
        lines = []
        while len(lines) < 3 and time.monotonic() < deadline:
            while not link.output.empty():
                lines.append(link.output.get())
            time.sleep(0.01)
        self.assertEqual(lines[1:3], ['[INFO] lds started', '[INFO] ready'])
        link.stop_bringup(timeout=2)
        self.assertEqual(channel.sent, ['\x03'])
        self.assertEqual(link.state, r.CONNECTED)

    def test_bringup_requires_a_connection(self):
        link = r.RobotLink(lambda: FakeClient())
        self.assertFalse(link.start_bringup(GOOD))
        self.assertIn('Connectez-vous', link.message)


if __name__ == '__main__':
    unittest.main()
