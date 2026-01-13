from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont, QPixmap
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
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)

        self.setObjectName("DroneStatusCard")
        self.setStyleSheet(
            "#DroneStatusCard{border:0px; background: transparent;}"
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        name_lbl = QLabel(name)
        name_lbl.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        name_lbl.setObjectName("DroneName")

        img_lbl = QLabel()
        img_lbl.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        img_lbl.setObjectName("DroneImage")

        pix = _safe_pixmap(image_path, height=46) if image_path else None
        if pix is not None:
            img_lbl.setPixmap(pix)
        else:
            img_lbl.setText("[drone]")

        self.status_lbl = QLabel(status_text)
        self.status_lbl.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self.status_lbl.setObjectName("DroneStatus")
        # Default to offline unless updated by backend.
        self.status_lbl.setProperty("live", False)

        layout.addWidget(name_lbl)
        layout.addWidget(img_lbl)
        layout.addWidget(self.status_lbl)

    def set_live(self, live: bool) -> None:
        # Text + an objectName used by QSS for coloring.
        self.status_lbl.setText("Status: Live" if live else "Status: Offline")
        self.status_lbl.setProperty("live", bool(live))
        self.status_lbl.style().unpolish(self.status_lbl)
        self.status_lbl.style().polish(self.status_lbl)
        self.status_lbl.update()


class MissionPlannerWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()

        self.setWindowTitle("Manas Planner")
        self.resize(1200, 700)

        root = QWidget()
        self.setCentralWidget(root)

        outer = QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        header = self._build_header()
        body = self._build_body()

        outer.addWidget(header)
        outer.addWidget(body)

        self._apply_styles()

    def _build_header(self) -> QFrame:
        header = QFrame()
        header.setObjectName("Header")
        header.setFixedHeight(70)

        layout = QHBoxLayout(header)
        layout.setContentsMargins(18, 10, 18, 10)
        layout.setSpacing(16)

        logo = QLabel()
        logo.setObjectName("Logo")
        logo.setFixedHeight(50)
        # Prefer the repo-provided logo if present; keep backwards-compatible fallbacks.
        logo_pix = (
            _safe_pixmap(ASSETS_DIR / "manas-full-white.png", height=50)
            or _safe_pixmap(ASSETS_DIR / "logo.png", height=50)
        )
        if logo_pix is not None:
            logo.setPixmap(logo_pix)
        else:
            logo.setText("PROJECT\nMANAS")
            logo.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            logo.setFont(QFont("Sans Serif", 14, QFont.Weight.Bold))

        self.global_status = QLabel("Status: Offline")
        self.global_status.setObjectName("GlobalStatus")
        self.global_status.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignTop)
        self.global_status.setFont(QFont("Sans Serif", 18, QFont.Weight.Bold))
        # Default to offline unless updated by backend.
        self.global_status.setProperty("live", False)

        layout.addWidget(logo, 0, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        layout.addItem(QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum))
        layout.addWidget(self.global_status, 0, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignTop)

        return header

    def _build_body(self) -> QFrame:
        body = QFrame()
        body.setObjectName("Body")

        layout = QHBoxLayout(body)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Map placeholder
        map_frame = QFrame()
        map_frame.setObjectName("MapArea")
        map_layout = QVBoxLayout(map_frame)
        map_layout.setContentsMargins(16, 16, 16, 16)

        map_lbl = QLabel("Map")
        map_lbl.setObjectName("MapLabel")
        map_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        map_lbl.setFont(QFont("Sans Serif", 48, QFont.Weight.Bold))
        map_layout.addWidget(map_lbl)

        # Right sidebar
        sidebar = QFrame()
        sidebar.setObjectName("Sidebar")
        sidebar.setFixedWidth(280)

        sb = QVBoxLayout(sidebar)
        sb.setContentsMargins(18, 18, 18, 18)
        sb.setSpacing(18)

        freyja_img = (
            ASSETS_DIR / "Freyja.png"
            if (ASSETS_DIR / "Freyja.png").exists()
            else ASSETS_DIR / "drone.png"
        )
        cleo_img = (
            ASSETS_DIR / "Cleo.png"
            if (ASSETS_DIR / "Cleo.png").exists()
            else ASSETS_DIR / "drone.png"
        )

        self.freyja_card = DroneStatusCard("Freyja", "Status: Offline", freyja_img)
        self.cleo_card = DroneStatusCard("Cleo", "Status: Offline", cleo_img)

        sb.addWidget(self.freyja_card)
        sb.addWidget(self.cleo_card)
        sb.addItem(QSpacerItem(20, 18, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))

        btn_start_mission = QPushButton("Start Mission")
        btn_start_mission.setObjectName("PrimaryButton")
        btn_start_spray = QPushButton("Start Spray")
        btn_start_spray.setObjectName("PrimaryButton")

        kill_row = QHBoxLayout()
        kill_row.setSpacing(12)
        btn_kill_left = QPushButton("Kill Freyja")
        btn_kill_left.setObjectName("SmallButton")
        btn_kill_right = QPushButton("Kill Cleo")
        btn_kill_right.setObjectName("SmallButton")

        kill_row.addWidget(btn_kill_left)
        kill_row.addWidget(btn_kill_right)

        for b in (btn_start_mission, btn_start_spray):
            b.setMinimumHeight(56)

        sb.addWidget(btn_start_mission)
        sb.addWidget(btn_start_spray)
        sb.addLayout(kill_row)

        layout.addWidget(map_frame, 1)
        layout.addWidget(sidebar, 0)

        return body

    def set_global_live(self, live: bool) -> None:
        self.global_status.setText("Status: Live" if live else "Status: Offline")
        self.global_status.setProperty("live", bool(live))
        self.global_status.style().unpolish(self.global_status)
        self.global_status.style().polish(self.global_status)
        self.global_status.update()

    def set_drone_live(self, drone_name: str, live: bool) -> None:
        name = drone_name.strip().lower()
        if name == "freyja":
            self.freyja_card.set_live(live)
        elif name == "cleo":
            self.cleo_card.set_live(live)

    def _apply_styles(self) -> None:
        # Roughly matches the screenshot: dark header + dark sidebar, grey map.
        self.setStyleSheet(
            """
            QMainWindow { background: #0f0f0f; }

            #Header { background: #0b0b0b; }
            #GlobalStatus { color: #f2f2f2; }
            #GlobalStatus[live="true"],
            #DroneStatus[live="true"] { color: #69e36b; }
            #GlobalStatus[live="false"],
            #DroneStatus[live="false"] { color: #ff5c5c; }
            #Logo { color: #f2f2f2; }

            #MapArea { background: #777777; }
            #MapLabel { color: #111111; }

            #Sidebar { background: #3a3737; }

            #DroneName { color: #f2f2f2; font-size: 22px; }
            #DroneStatus { color: #f2f2f2; font-size: 16px; }
            #DroneImage { color: #f2f2f2; }

            QPushButton#PrimaryButton,
            QPushButton#SmallButton {
                background: #f08a00;
                color: #111111;
                border: 1px solid rgba(0, 0, 0, 0.35);
                border-radius: 10px;
                font-weight: 600;
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
                background: #ff9c1a;
            }

            QPushButton#PrimaryButton:pressed,
            QPushButton#SmallButton:pressed {
                background: #d97700;
            }

            QPushButton#PrimaryButton:focus,
            QPushButton#SmallButton:focus {
                outline: none;
                border: 2px solid rgba(255, 255, 255, 0.65);
            }
            """
        )
