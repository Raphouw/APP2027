"""Instance unique, arrêt de tout l'atelier, fils qui meurent avec le manager (sans ROS ni fenêtre)."""
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'atelier_slam'))
from slam_components import instance


class InstanceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.old = (instance.LOCK_PATH, instance.APP_DIR)
        instance.LOCK_PATH = Path(self.tmp.name)/'lock'
        instance.APP_DIR = Path(self.tmp.name).resolve()
        self.children = []

    def tearDown(self):
        for child in self.children:
            child.kill()
            child.wait()
        instance.LOCK_PATH, instance.APP_DIR = self.old
        self.tmp.cleanup()

    def fake_node(self, name='navigation_carte.py'):
        script = instance.APP_DIR/name
        script.write_text('import time\ntime.sleep(60)\n')
        child = subprocess.Popen([sys.executable, str(script)], cwd=instance.APP_DIR)
        self.children.append(child)
        time.sleep(0.3)
        return child

    def test_second_instance_is_refused_and_names_the_holder(self):
        lock, holder = instance.acquire()
        self.assertIsNotNone(lock)
        # un autre processus voit le verrou pris
        code = ('import sys; sys.path.insert(0, %r); from slam_components import instance as i; '
                'from pathlib import Path; i.LOCK_PATH = Path(%r); l, h = i.acquire(); print(l is None, h)'
                % (str(Path(instance.__file__).resolve().parents[1]), str(instance.LOCK_PATH)))
        out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True).stdout.split()
        self.assertEqual(out, ['True', str(os.getpid())])
        lock.close()
        self.assertIsNotNone(instance.acquire()[0])

    def test_stop_all_kills_orphan_nodes_of_this_folder_only(self):
        ours = self.fake_node('navigation_carte.py')
        foreign = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)', 'navigation_carte.py'])
        self.children.append(foreign)
        self.assertIn(ours.pid, instance.atelier_processes())
        self.assertNotIn(foreign.pid, instance.atelier_processes())
        instance.stop_all(grace=1.0)
        self.assertIsNotNone(ours.wait(timeout=5))
        self.assertIsNone(foreign.poll())

    def test_children_die_with_their_parent(self):
        code = ('import subprocess, sys, os; sys.path.insert(0, %r); from slam_components import instance as i; '
                'p = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"], preexec_fn=i.child_dies_with_parent); '
                'print(p.pid, flush=True); os.kill(os.getpid(), 9)'
                % str(Path(instance.__file__).resolve().parents[1]))
        out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True).stdout.split()
        pid = int(out[0])
        def state():
            try:
                return Path(f'/proc/{pid}/stat').read_text().split(')')[1].split()[0]
            except OSError:
                return 'gone'
        deadline = time.monotonic()+5
        while time.monotonic() < deadline and state() not in ('Z', 'gone'):
            time.sleep(0.1)
        state = state()
        self.assertIn(state, ('Z', 'gone'))


if __name__ == '__main__':
    unittest.main()
