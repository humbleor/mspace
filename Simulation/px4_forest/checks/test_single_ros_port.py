import errno
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import unittest


class SingleRosPortTests(unittest.TestCase):
    def test_rejects_occupied_ros_master_port_before_startup(self):
        runner = Path(__file__).resolve().parents[1] / "run.py"
        with socket.socket() as listener, tempfile.TemporaryDirectory() as tmp:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                listener.bind(("127.0.0.1", 41887))
            except OSError as exc:
                if exc.errno != errno.EADDRINUSE:
                    raise
            else:
                listener.listen()

            output = Path(tmp) / "run"
            result = subprocess.run(
                [sys.executable, str(runner), "run", "--stage", "hover", "--headless",
                 "--output", str(output), "--px4", str(Path(tmp) / "no-build")],
                capture_output=True, text=True, timeout=15,
            )
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertRegex(result.stderr, r"ROS Master.*41887.*already in use")
            self.assertFalse(output.exists())
