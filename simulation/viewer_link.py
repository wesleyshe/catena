"""The control windows' link to the MuJoCo viewer: start play.py under mjpython, send it commands, read its readouts.

The window is the socket server, on a port the OS picks; play.py (--panel PORT) connects back. Commands and readouts
are JSON lines (see "Panel <-> viewer protocol" in AGENTS.md). Stdlib only, like the windows that use it.
"""
import json
import os
import socket
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
MJPYTHON = os.path.join(HERE, ".venv", "bin", "mjpython")


class ViewerLink:
    def __init__(self):
        self.server = socket.socket()
        self.server.bind(("127.0.0.1", 0))
        self.server.listen(1)
        self.server.setblocking(False)
        self.conn, self.buf, self.proc = None, b"", None

    def open(self, setup_path):
        """(Re)start the viewer on this setup file: the chain in its start pose, waiting for Play."""
        self.close()
        self.proc = subprocess.Popen([MJPYTHON, os.path.join(HERE, "play.py"), "--setup", setup_path, "--log", "",
                                      "--panel", str(self.server.getsockname()[1])], cwd=HERE)

    def close(self):
        if self.proc and self.proc.poll() is None:
            self.send("quit")
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.terminate()
        self.conn, self.buf = None, b""

    def send(self, cmd, **args):
        """Send one command; False if the viewer isn't connected."""
        if not self.conn:
            return False
        try:
            self.conn.sendall((json.dumps({"cmd": cmd, **args}) + "\n").encode())
            return True
        except OSError:
            self.conn = None
            return False

    def poll(self):
        """Call often (every 100 ms): accept the viewer's connection and read what it sent.
        Returns (just_connected, latest readouts or None, just_closed)."""
        connected = closed = False
        if self.conn is None and self.proc and self.proc.poll() is None:
            try:
                self.conn, _ = self.server.accept()
                self.conn.setblocking(False)
                connected = True
            except BlockingIOError:
                pass
        info = None
        if self.conn:
            try:
                chunk = self.conn.recv(65536)
                self.buf += chunk
                if not chunk:
                    self.conn = None
            except BlockingIOError:
                pass
            lines = self.buf.split(b"\n")
            self.buf = lines[-1]
            if len(lines) > 1:
                info = json.loads(lines[-2])
        if self.proc and self.proc.poll() is not None:
            self.proc, closed = None, True
        return connected, info, closed
