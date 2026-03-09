import threading
import time
from pymavlink import mavutil

class MavlinkMonitor:
    def __init__(self, connection, baud):
        self.connection = connection
        self.baud = baud
        self.master = None
        self.connected = False
        self.armed = True
        self.last_heartbeat = 0
        self._stop = False

    def start(self):
        t = threading.Thread(target=self._run, daemon=True)
        t.start()

    def stop(self):
        self._stop = True
        if self.master:
            try:
                self.master.close()
            except Exception:
                pass

    def _run(self):
        try:
            self.master = mavutil.mavlink_connection(self.connection, baud=self.baud)
            self.master.wait_heartbeat(timeout=15)
            self.connected = True
            self.last_heartbeat = time.time()
        except Exception:
            self.connected = False
            return

        while not self._stop:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg:
                self.last_heartbeat = time.time()
                self.connected = True
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            else:
                if time.time() - self.last_heartbeat > 3:
                    self.connected = False