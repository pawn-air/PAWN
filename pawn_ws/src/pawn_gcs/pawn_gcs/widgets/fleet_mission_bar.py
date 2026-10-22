from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QWidget, QHBoxLayout, QLabel, QCheckBox, QSlider, QPushButton
from pawn_gcs.theme import get_theme

class FleetMissionBar(QWidget):
    frontiers_toggled = Signal(bool)
    theme_toggled = Signal(str)

    def __init__(self):
        super().__init__()
        self.current_theme = "dark"
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        badge = QLabel("  PAWN MISSION CONTROL | AUTONOMOUS MULTI-DRONE TRIAGE  ")
        badge.setStyleSheet("background-color: #0284c7; color: white; font-weight: bold; padding: 7px; border-radius: 4px;")
        layout.addWidget(badge)
        layout.addStretch()

        self.chk_humans = QCheckBox("Survivors (P0)")
        self.chk_humans.setChecked(True)
        self.chk_cracks = QCheckBox("Cracks (P2)")
        self.chk_cracks.setChecked(True)
        self.chk_frontiers = QCheckBox("Frontiers")
        self.chk_frontiers.setChecked(True)
        self.chk_frontiers.toggled.connect(self.frontiers_toggled.emit)

        layout.addWidget(self.chk_humans)
        layout.addWidget(self.chk_cracks)
        layout.addWidget(self.chk_frontiers)

        layout.addWidget(QLabel("  ML Conf:"))
        self.conf_slider = QSlider(Qt.Orientation.Horizontal)
        self.conf_slider.setRange(20, 95)
        self.conf_slider.setValue(65)
        self.conf_slider.setFixedWidth(100)
        layout.addWidget(self.conf_slider)

        self.btn_theme = QPushButton("☀️ Light Mode")
        self.btn_theme.setStyleSheet(
            "QPushButton { background-color: #1e293b; color: #f8fafc; font-weight: bold; "
            "padding: 7px 12px; border: 1px solid #475569; border-radius: 4px; } "
            "QPushButton:hover { background-color: #334155; }"
        )
        self.btn_theme.clicked.connect(self._toggle_theme)
        layout.addWidget(self.btn_theme)

        btn_estop = QPushButton(" HOLD ALL DRONES ")
        btn_estop.setStyleSheet(
            "QPushButton { background-color: #dc2626; color: white; font-weight: bold; "
            "padding: 7px 12px; border-radius: 4px; } QPushButton:hover { background-color: #b91c1c; }"
        )
        layout.addWidget(btn_estop)

    def _toggle_theme(self):
        new_theme = "light" if self.current_theme == "dark" else "dark"
        self.set_theme(new_theme)
        self.theme_toggled.emit(new_theme)

    def set_theme(self, theme_name: str):
        self.current_theme = theme_name
        theme = get_theme(theme_name)
        self.btn_theme.setText(theme["toggle_btn_text"])
        self.btn_theme.setStyleSheet(theme["toggle_btn_style"])