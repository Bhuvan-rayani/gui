from __future__ import annotations

import sys
import os
import cv2
import numpy as np
from pathlib import Path
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QUrl
from PyQt6.QtGui import QPixmap, QImage
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebEngineCore import QWebEngineSettings, QWebEnginePage
from PyQt6.QtWidgets import (
    QApplication,
    QFrame,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QHBoxLayout,
    QWidget,
    QTabWidget,
)

ASSETS_DIR = Path(__file__).parent / "assets"

# =========================
# Yellow Detection Pipeline
# (EXACT LOGIC, LIVE)
# =========================

min_contour = 100

def detect_yellow_live(img_bgr, edge_margin_px=100):
    if img_bgr is None:
        return []

    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    # EXACT HSV RANGE
    lower = np.array([20, 170, 40])
    upper = np.array([40, 255, 255])

    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((6, 6), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.dilate(mask, kernel, iterations=1)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    img_h, img_w = img_bgr.shape[:2]
    centroids = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_contour:
            continue

        M = cv2.moments(cnt)
        if M.get("m00", 0) == 0:
            continue

        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]

        # Ignore detections near image edges
        if (
            cx < edge_margin_px or
            cx > img_w - edge_margin_px or
            cy < edge_margin_px or
            cy > img_h - edge_margin_px
        ):
            continue

        centroids.append((cx, cy))

    return centroids

# =========================
# OpenCV Worker (THREAD)
# =========================
import time  # REQUIRED


class OpenCVWorker(QThread):
    frame_ready = pyqtSignal(QImage)

    def __init__(self, source=0, target_fps=None):
        super().__init__()
        self.source = source
        self.running = True
        self.target_fps = target_fps
        self._frame_interval = 1.0 / target_fps if target_fps else None

    # -----------------------------
    # YUV → BGR helper
    # -----------------------------
    def _convert_yuv(self, frame):
        if frame is None:
            return None

        # YUYV sometimes arrives as HxWx2
        if frame.ndim == 3 and frame.shape[2] == 2:
            try:
                return cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUY2)
            except cv2.error:
                pass

        return frame

    # -----------------------------
    # MAIN THREAD LOOP
    # -----------------------------
    def run(self):
        cap = cv2.VideoCapture(self.source, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        if not cap.isOpened():
            print("[OpenCVWorker] ERROR: Camera could not be opened")
            return

        last_time = time.perf_counter()

        while self.running:
            # ---------- FPS LIMIT ----------
            if self._frame_interval is not None:
                now = time.perf_counter()
                elapsed = now - last_time
                if elapsed < self._frame_interval:
                    time.sleep(self._frame_interval - elapsed)
                last_time = time.perf_counter()

            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            # ---------- Decode ----------
            frame = self._convert_yuv(frame)
            if frame is None:
                continue

            # ==========================================
            # 🔥 LIVE YELLOW DETECTION (ONCE)
            # ==========================================
            centroids = detect_yellow_live(frame)

            for (cx, cy) in centroids:
                cx_i, cy_i = int(round(cx)), int(round(cy))

                arrow_start = (cx_i - 40, cy_i - 40)
                arrow_end = (cx_i, cy_i)

                cv2.arrowedLine(
                    frame,
                    arrow_start,
                    arrow_end,
                    (0, 0, 255),
                    1,
                    tipLength=0.25
                )

                cv2.putText(
                    frame,
                    "Diseased Plant",
                    (arrow_start[0], arrow_start[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    1,
                    cv2.LINE_AA
                )
            # ==========================================

            # ---------- Convert to Qt ----------
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape

            qimg = QImage(
                rgb.data,
                w,
                h,
                ch * w,
                QImage.Format.Format_RGB888
            ).copy()  # REQUIRED for thread safety

            self.frame_ready.emit(qimg)

        cap.release()

    # -----------------------------
    # Clean shutdown
    # -----------------------------
    def stop(self):
        self.running = False
        self.wait()




# =========================
# Camera Tab
# =========================
class CameraTab(QWidget):
    def __init__(self):
        super().__init__()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        title = QLabel("Camera Feed (OpenCV Output)")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet(
            "color:#f49221; font-size:18px; font-weight:700;"
        )

        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # 🔥 CRITICAL FIXES
        self.video_label.setScaledContents(False)
        self.video_label.setMinimumSize(1, 1)
        self.video_label.setSizePolicy(
            QSizePolicy.Policy.Ignored,
            QSizePolicy.Policy.Ignored,
        )

        self.video_label.setStyleSheet(
            "background:#000; border:2px solid #f49221;"
        )

        layout.addWidget(title)
        layout.addWidget(self.video_label, 1)

        self._last_pixmap = None

        self.worker = OpenCVWorker(0)
        self.worker.frame_ready.connect(self.update_frame)
        self.worker.start()

    def update_frame(self, img: QImage):
        self._last_pixmap = QPixmap.fromImage(img)
        self._update_scaled_pixmap()

    def _update_scaled_pixmap(self):
        if self._last_pixmap is None:
            return

        target_size = self.video_label.size()
        if target_size.width() <= 0 or target_size.height() <= 0:
            return

        scaled = self._last_pixmap.scaled(
            target_size,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.video_label.setPixmap(scaled)

    def resizeEvent(self, event):
        self._update_scaled_pixmap()
        super().resizeEvent(event)

    def shutdown(self):
        self.worker.stop()




# =========================
# Mission Planner Window
# =========================
class MissionPlannerWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Manas Planner")
        self.resize(1200, 700)

        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)

        outer.addWidget(self._build_header())
        outer.addWidget(self._build_tabs())

        self._apply_styles()

    # ---------- HEADER ----------
    def _build_header(self):
        header = QFrame()
        header.setFixedHeight(70)
        header.setObjectName("Header")

        layout = QHBoxLayout(header)
        layout.setContentsMargins(18, 10, 18, 10)

        logo = QLabel("PROJECT MANAS")
        logo.setStyleSheet("color:white; font-size:22px; font-weight:800;")

        self.global_status = QLabel("Status: Offline")
        self.global_status.setStyleSheet("color:#ccc;")

        layout.addWidget(logo)
        layout.addStretch()
        layout.addWidget(self.global_status)

        return header

    # ---------- TABS ----------
    def _build_tabs(self):
        tabs = QTabWidget()

        tabs.addTab(self._build_map_tab(), "Map")
        self.camera_tab = CameraTab()
        tabs.addTab(self.camera_tab, "Camera")

        return tabs

    # ---------- MAP TAB ----------
    def _build_map_tab(self):
        body = QFrame()
        layout = QHBoxLayout(body)

        map_frame = QFrame()
        map_layout = QVBoxLayout(map_frame)

        self.map_view = QWebEngineView()

        class DebugPage(QWebEnginePage):
            def javaScriptConsoleMessage(self, level, msg, line, source):
                print(f"[JS] {msg}")

        self.map_view.setPage(DebugPage(self.map_view))
        self.map_view.settings().setAttribute(
            QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls,
            True,
        )

        map_html = Path(__file__).parent / "map.html"
        self.map_view.setUrl(QUrl.fromLocalFile(str(map_html)))

        map_layout.addWidget(self.map_view)

        sidebar = QFrame()
        sidebar.setFixedWidth(280)
        sb = QVBoxLayout(sidebar)

        btn = QPushButton("Start Mission")
        btn.setObjectName("PrimaryButton")

        sb.addStretch()
        sb.addWidget(btn)

        layout.addWidget(map_frame, 1)
        layout.addWidget(sidebar)

        return body

    def closeEvent(self, event):
        self.camera_tab.close()
        event.accept()


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

                        #Header {
                background:#070707;
                border-bottom:2px solid #f49221;
            }

            QTabWidget::pane {
                border-top:2px solid #f49221;
                background:#070707;
            }

            QTabBar::tab {
                background:#0d0d0d;
                color:#cfcfcf;
                padding:10px 22px;
                font-weight:700;
            }

            QTabBar::tab:selected {
                color:#f49221;
                border-bottom:2px solid #f49221;
            }

            QPushButton#PrimaryButton {
                background:#f49221;
                border-radius:12px;
                padding:14px;
                font-weight:800;
            }
            
            """
        )
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MissionPlannerWindow()
    win.show()
    sys.exit(app.exec())