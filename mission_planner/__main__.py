from __future__ import annotations

import sys


def main() -> int:
    # Prefer PyQt6; if someone swaps requirements later, fail loudly.
    from PyQt6.QtWidgets import QApplication

    from .ui.main_window import MissionPlannerWindow

    
    app = QApplication(sys.argv)
    app.setApplicationName("manasPlanner")

    window = MissionPlannerWindow()
    window.show()

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
