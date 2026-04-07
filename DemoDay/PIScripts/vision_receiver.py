import socket
import json
import time
import threading


class VisionReceiver:
    def __init__(self, host="127.0.0.1", port=5801):
        self.host = host
        self.port = port

        self.running = False
        self.thread = None
        self.sock = None

        self.latest_msg = None

        self.visible_since = None
        self.lost_since = None

        self.history = []

        self.lock = threading.Lock()

    def start(self):
        if self.running:
            print("[WARNING] VisionReceiver already running")
            return

        self.running = True
        self.thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.thread.start()
        print(f"[INFO] VisionReceiver listening on {self.host}:{self.port}")

    def stop(self):
        self.running = False

        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

        if self.thread is not None:
            self.thread.join(timeout=1.0)

        print("VisionReceiver stopped")

    def _listen_loop(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(0.2)

            while self.running:
                try:
                    data, addr = self.sock.recvfrom(4096)
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"[WARNING] recvfrom error: {e}")
                    continue

                try:
                    text = data.decode("utf-8")
                    msg = json.loads(text)
                except Exception as e:
                     print(f"[WARNING] Bad vision packet: {e}")
                     continue
                with self.lock:
                     self.latest_msg = msg

                # Detect the continuous presence and loss of the signal
                     det = msg.get("det", 0)
                     now = time.time()

                     # check the percentage of available signal of past 5 secs

                     self.history.append((now, det))
                     cutoff = now - 5.0
                     self.history = [(ts, d) for ts, d in self.history if ts >= cutoff]

                     if det == 1:
                        if self.visible_since is None:
                            self.visible_since = time.time()
                        self.lost_since = None
                     elif det == 0:
                        if self.lost_since is None:
                            self.lost_since = time.time()
                        self.visible_since = None

        except Exception as e:
            print(f"[ERROR] VisionReceiver socket error: {e}")

    # return the last vector msg
    def get_latest(self):
        with self.lock:
            return self.latest_msg

    # return if target visible in last msg
    def target_visible_now(self):
        with self.lock:
            if self.latest_msg is None:
                return False
            return self.latest_msg.get("det", 0) == 1

    # return if target can visible for x sec
    def visible_for(self, seconds):
        with self.lock:
            if self.visible_since is None:
                return False
            return (time.time() - self.visible_since) >= seconds

    # return if target LOST for x sec
    def lost_for(self, seconds):
        with self.lock:
            if self.lost_since is None:
                return False
            return (time.time() - self.lost_since) >= seconds

    # return the percentage of available signal in the past 5 secs
    def visible_ratio_over(self, window_seconds):
        with self.lock:
            now = time.time()
            cutoff = now - window_seconds
            recent = [(ts, d) for ts, d in self.history if ts >= cutoff]
            if not recent:
                return 0.0
            visible_count = sum(1 for _, d in recent if d == 1)
            return visible_count / len(recent)

    def mostly_visible_for(self, window_seconds, threshold_ratio):
        return self.visible_ratio_over(window_seconds) >= threshold_ratio
