from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import Qt, QUrl
from PyQt6.QtGui import QFont, QPixmap
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QSpacerItem,
    QVBoxLayout,
    QWidget,
)

ASSETS_DIR = Path(__file__).resolve().parents[2] / "assets"


def _safe_pixmap(path: Path, *, height: int | None = None) -> QPixmap | None:
    if not path.exists():
        return None
    pix = QPixmap(str(path))
    if pix.isNull():
        return None
    if height is not None:
        pix = pix.scaledToHeight(height, Qt.TransformationMode.SmoothTransformation)
    return pix


class DroneStatusCard(QFrame):
    def __init__(
        self,
        name: str,
        status_text: str = "Status: Offline",
        image_path: Path | None = None,
        *,
        latitude: float | None = None,
        longitude: float | None = None,
        altitude_m: float | None = None,
        updated_text: str | None = None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)

        self.setObjectName("DroneStatusCard")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        name_lbl = QLabel(name)
        name_lbl.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        name_lbl.setObjectName("DroneName")

        img_lbl = QLabel()
        img_lbl.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        img_lbl.setObjectName("DroneImage")
        img_lbl.setMinimumHeight(86)

        pix = _safe_pixmap(image_path, height=78) if image_path else None
        if pix is not None:
            img_lbl.setPixmap(pix)
        else:
            img_lbl.setText("[drone]")

        self.status_lbl = QLabel(status_text)
        self.status_lbl.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self.status_lbl.setObjectName("DroneStatus")
        self.status_lbl.setProperty("live", False)

        info = QFrame()
        info.setObjectName("InfoGrid")
        info_layout = QVBoxLayout(info)
        info_layout.setContentsMargins(0, 0, 0, 0)
        info_layout.setSpacing(10)

        row1 = QHBoxLayout()
        row2 = QHBoxLayout()

        self.lat_card = self._make_kv_card("Latitude", "--")
        self.lon_card = self._make_kv_card("Longitude", "--")
        self.alt_card = self._make_kv_card("Altitude", "--")
        self.updated_card = self._make_kv_card("Updated", "--")

        row1.addWidget(self.lat_card)
        row1.addWidget(self.lon_card)
        row2.addWidget(self.alt_card)
        row2.addWidget(self.updated_card)

        info_layout.addLayout(row1)
        info_layout.addLayout(row2)

        self.gps_lbl = QLabel("GPS: --")
        self.gps_lbl.setObjectName("GpsStatus")
        self.gps_lbl.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self.gps_lbl.setProperty("gps", "unknown")

        layout.addWidget(name_lbl)
        layout.addWidget(img_lbl)
        layout.addWidget(self.status_lbl)
        layout.addWidget(info)
        layout.addWidget(self.gps_lbl)

    def _make_kv_card(self, key: str, value: str) -> QFrame:
        card = QFrame()
        card.setObjectName("KvCard")
        v = QVBoxLayout(card)
        v.setContentsMargins(10, 8, 10, 8)
        v.setSpacing(2)

        k = QLabel(key)
        k.setObjectName("KvKey")
        val = QLabel(value)
        val.setObjectName("KvValue")
        val.setWordWrap(True)

        v.addWidget(k)
        v.addWidget(val)
        card._value_label = val
        return card

    def _set_kv_value(self, card: QFrame, value: str) -> None:
        card._value_label.setText(value)

    def set_position(self, *, latitude=None, longitude=None, altitude_m=None, updated_text=None):
        if latitude is not None:
            self._set_kv_value(self.lat_card, f"{latitude:.6f}")
        if longitude is not None:
            self._set_kv_value(self.lon_card, f"{longitude:.6f}")
        if altitude_m is not None:
            self._set_kv_value(self.alt_card, f"{altitude_m:.1f} m")
        if updated_text is not None:
            self._set_kv_value(self.updated_card, updated_text)

    def set_gps_active(self, active: bool | None) -> None:
        if active is None:
            self.gps_lbl.setText("GPS: --")
            self.gps_lbl.setProperty("gps", "unknown")
        else:
            self.gps_lbl.setText("GPS: Active" if active else "GPS: Inactive")
            self.gps_lbl.setProperty("gps", "active" if active else "inactive")
        self.gps_lbl.style().polish(self.gps_lbl)

    def set_live(self, live: bool) -> None:
        self.status_lbl.setText("Status: Live" if live else "Status: Offline")
        self.status_lbl.setProperty("live", live)
        self.status_lbl.style().polish(self.status_lbl)


class MissionPlannerWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        
        # Start offline tile server (runs once, non-blocking)
        import threading
        from .. import tile_server

        threading.Thread(
            target=tile_server.run,
            daemon=True
        ).start()



        self.setWindowTitle("Manas Planner")
        self.resize(1200, 700)

        root = QWidget()
        self.setCentralWidget(root)

        outer = QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)

        outer.addWidget(self._build_header())
        outer.addWidget(self._build_body())

        self._apply_styles()

    def _build_header(self) -> QFrame:
        header = QFrame()
        header.setObjectName("Header")
        header.setFixedHeight(70)

        layout = QHBoxLayout(header)
        layout.setContentsMargins(18, 10, 18, 10)

        logo = QLabel()
        logo_pix = _safe_pixmap(ASSETS_DIR / "manas-full-white.png", height=50)
        if logo_pix:
            logo.setPixmap(logo_pix)
        else:
            logo.setText("PROJECT MANAS")

        self.global_status = QLabel("Status: Offline")
        self.global_status.setObjectName("GlobalStatus")

        layout.addWidget(logo)
        layout.addStretch()
        layout.addWidget(self.global_status)

        return header

    def _build_body(self) -> QFrame:
        body = QFrame()
        body.setObjectName("Body")

        layout = QHBoxLayout(body)
        layout.setContentsMargins(0, 0, 0, 0)

        # ================= MAP AREA =================
        map_frame = QFrame()
        map_frame.setObjectName("MapArea")
        map_layout = QVBoxLayout(map_frame)
        map_layout.setContentsMargins(16, 16, 16, 16)

        self.map_view = QWebEngineView()
        map_html = Path(__file__).resolve().parent / "map.html"
        self.map_view.setUrl(QUrl.fromLocalFile(str(map_html)))

        map_layout.addWidget(self.map_view)
        # ============================================

        # Sidebar
        sidebar = QFrame()
        sidebar.setObjectName("Sidebar")
        sidebar.setFixedWidth(280)

        sb = QVBoxLayout(sidebar)
        sb.setContentsMargins(18, 18, 18, 18)

        self.freyja_card = DroneStatusCard("Freyja", image_path=ASSETS_DIR / "drone.png")
        self.cleo_card = DroneStatusCard("Cleo", image_path=ASSETS_DIR / "drone.png")

        sb.addWidget(self.freyja_card)
        sb.addWidget(self.cleo_card)
        sb.addStretch()

        btn_start = QPushButton("Start Mission")
        btn_start.setObjectName("PrimaryButton")

        sb.addWidget(btn_start)

        layout.addWidget(map_frame, 1)
        layout.addWidget(sidebar, 0)

        return body


    def _apply_styles(self) -> None:
        self.setStyleSheet(
            """
            /*
              Theme: black surfaces + #f49221 accents
              - Keep contrast high, avoid flat gray blocks
              - Use orange borders to match the reference screenshot
            */
            QMainWindow { background: #070707; }

            #Header {
                background: #070707;
                border-bottom: 2px solid #f49221;
            }
            #GlobalStatus { color: #eaeaea; }
            #GlobalStatus[live="true"] { color: #69e36b; }
            #GlobalStatus[live="false"] { color: #ff5c5c; }
            #Logo { color: #eaeaea; }

            #Body { background: #070707; }

            #MapArea {
                background: #0d0d0d;
                border-right: 2px solid rgba(244, 146, 33, 0.55);
            }
            #MapLabel { color: rgba(244, 146, 33, 0.85); }

            #Sidebar {
                background: #070707;
            }

            /* Drone card: orange outline + dark panel */
            #DroneStatusCard {
                background: #0e0e0e;
                border: 2px solid #f49221;
                border-radius: 14px;
                padding: 12px;
            }

            #DroneName {
                color: #f4f4f4;
                font-size: 20px;
                font-weight: 700;
            }

            #DroneImage { color: #f2f2f2; }

            /* Status badge (pill) */
            #DroneStatus {
                color: #cfcfcf;
                font-size: 14px;
                padding: 6px 10px;
                border-radius: 10px;
                background: #121212;
                border: 1px solid rgba(244, 146, 33, 0.45);
            }

            /* Info grid */
            #InfoGrid { background: transparent; }

            #KvCard {
                background: #121212;
                border-radius: 10px;
                border: 1px solid rgba(244, 146, 33, 0.22);
            }

            #KvKey {
                color: rgba(244, 146, 33, 0.85);
                font-size: 11px;
                font-weight: 700;
            }

            #KvValue {
                color: #e6e6e6;
                font-size: 13px;
                font-weight: 700;
            }

            #DroneMode {
                color: rgba(244, 146, 33, 0.95);
                font-size: 13px;
                font-weight: 800;
                padding-top: 2px;
            }

            #GpsStatus {
                color: #bdbdbd;
                font-size: 12px;
                font-weight: 700;
                padding: 4px 8px;
                border-radius: 10px;
                background: #111111;
                border: 1px solid rgba(244, 146, 33, 0.20);
            }
            #GpsStatus[gps="active"] {
                color: #69e36b;
                background: rgba(105, 227, 107, 0.10);
                border: 1px solid rgba(105, 227, 107, 0.55);
            }
            #GpsStatus[gps="inactive"] {
                color: #ff5c5c;
                background: rgba(255, 92, 92, 0.10);
                border: 1px solid rgba(255, 92, 92, 0.55);
            }

            #DroneStatus[live="true"] {
                color: #69e36b;
                background: rgba(105, 227, 107, 0.10);
                border: 1px solid rgba(105, 227, 107, 0.55);
            }

            #DroneStatus[live="false"] {
                color: #ff5c5c;
                background: rgba(255, 92, 92, 0.10);
                border: 1px solid rgba(255, 92, 92, 0.55);
            }

            QPushButton#PrimaryButton,
            QPushButton#SmallButton {
                background: #f49221;
                color: #0b0b0b;
                border: 1px solid rgba(244, 146, 33, 0.85);
                border-radius: 12px;
                font-weight: 700;
                letter-spacing: 0.2px;
            }

            QPushButton#PrimaryButton {
                font-size: 16px;
                padding: 14px 14px;
            }

            QPushButton#SmallButton {
                font-size: 14px;
                padding: 10px 12px;
                min-height: 40px;
            }

            QPushButton#PrimaryButton:hover,
            QPushButton#SmallButton:hover {
                background: #ffad55;
                border: 1px solid rgba(244, 146, 33, 1.0);
            }

            QPushButton#PrimaryButton:pressed,
            QPushButton#SmallButton:pressed {
                background: #d97813;
                border: 1px solid rgba(244, 146, 33, 0.95);
            }

            QPushButton#PrimaryButton:focus,
            QPushButton#SmallButton:focus {
                outline: none;
                border: 2px solid rgba(244, 146, 33, 0.95);
            }
            """
        )
