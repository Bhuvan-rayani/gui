from PyQt6.QtCore import QThread, pyqtSignal
from pymavlink import mavutil
import time
from datetime import datetime


class MavlinkGPSWorker(QThread):
    gps_update = pyqtSignal(float, float, float, str, bool)
    heartbeat = pyqtSignal(bool)

    def __init__(self, udp_port: int):
        super().__init__()
        self.udp_port = udp_port
        self.running = True

    def run(self):
        conn = mavutil.mavlink_connection(
            f"udp:127.0.0.1:{self.udp_port}"
        )

        conn.wait_heartbeat()
        self.heartbeat.emit(True)

        last_hb_time = time.time()
        hb_alive = True

        while self.running:
            msg = conn.recv_match(blocking=False)

            if msg:
                if msg.get_type() == "HEARTBEAT":
                    last_hb_time = time.time()
                    if not hb_alive:
                        self.heartbeat.emit(True)
                        hb_alive = True

                elif msg.get_type() == "GLOBAL_POSITION_INT":
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.relative_alt / 1000.0
                    ts = datetime.now().strftime("%H:%M:%S")
                    self.gps_update.emit(lat, lon, alt, ts, True)

            if time.time() - last_hb_time > 2:
                if hb_alive:
                    self.heartbeat.emit(False)
                    hb_alive = False

            time.sleep(0.05)

    def stop(self):
        self.running = False
