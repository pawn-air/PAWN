import numpy as np
from PySide6.QtCore import Qt, QPointF, Signal
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QBrush, QImage
from PySide6.QtWidgets import QFrame, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QWidget
from pawn_gcs.theme import get_theme

class DroneHudCard(QFrame):
    mode_changed = Signal(int, str)       # (drone_id, "AUTO" | "MANUAL")
    flight_command = Signal(int, str)     # (drone_id, "takeoff" | "land")
    focus_requested = Signal(int)         # (drone_id)

    def __init__(self, drone_id: int, role_title: str):
        super().__init__()
        self.drone_id = drone_id
        self.role_title = role_title

        # Telemetry & States
        self.t = 0.0
        self.risk_score = 2.0
        self.show_cracks = True
        self.show_humans = True
        self.conf_thresh = 0.65
        self.current_theme = "dark"
        self.control_mode = "AUTO"
        self.is_manual_focused = False
        self.is_flying = True
        self.battery = 88
        self.altitude = 1.4
        self.video_frame: QImage | None = None

        self.setMinimumSize(360, 260)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        # Setup internal layout
        self.root_layout = QVBoxLayout(self)
        self.root_layout.setContentsMargins(6, 6, 6, 6)
        self.root_layout.addStretch(1)

        # Bottom Control Toolbar
        self.ctrl_bar = QWidget()
        ctrl_layout = QHBoxLayout(self.ctrl_bar)
        ctrl_layout.setContentsMargins(6, 4, 6, 4)
        ctrl_layout.setSpacing(6)

        # Mode Toggle Button
        self.btn_mode = QPushButton("🤖 AUTO")
        self.btn_mode.setToolTip("Switch between Autonomous Exploration and Manual Keyboard Override")
        self.btn_mode.clicked.connect(self._toggle_mode)
        ctrl_layout.addWidget(self.btn_mode)

        # Takeoff / Land Button
        self.btn_flight = QPushButton("🛬 Land")
        self.btn_flight.setToolTip("Command drone to Takeoff or Land")
        self.btn_flight.clicked.connect(self._toggle_flight)
        ctrl_layout.addWidget(self.btn_flight)

        # Telemetry & Battery Badges
        self.lbl_telemetry = QLabel("Alt: 1.4m")
        self.lbl_battery = QLabel("🔋 88%")
        ctrl_layout.addWidget(self.lbl_telemetry)
        ctrl_layout.addWidget(self.lbl_battery)

        ctrl_layout.addStretch()

        self.lbl_manual_hint = QLabel("")
        self.lbl_manual_hint.setStyleSheet("font-size: 10px; font-weight: bold; color: #f59e0b;")
        ctrl_layout.addWidget(self.lbl_manual_hint)

        self.root_layout.addWidget(self.ctrl_bar)

        self._apply_styles()

    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        self.focus_requested.emit(self.drone_id)

    def _toggle_mode(self):
        new_mode = "MANUAL" if self.control_mode == "AUTO" else "AUTO"
        self.set_control_mode(new_mode)
        self.mode_changed.emit(self.drone_id, new_mode)

    def _toggle_flight(self):
        cmd = "land" if self.is_flying else "takeoff"
        self.is_flying = not self.is_flying
        self.btn_flight.setText("🛬 Land" if self.is_flying else "🛫 Takeoff")
        self.flight_command.emit(self.drone_id, cmd)
        self._apply_styles()

    def set_control_mode(self, mode: str):
        self.control_mode = mode
        if mode == "MANUAL":
            self.btn_mode.setText("🎮 MANUAL")
            self.lbl_manual_hint.setText("KEYBOARD READY [WASD/Space]")
        else:
            self.btn_mode.setText("🤖 AUTO")
            self.lbl_manual_hint.setText("")
        self._apply_styles()
        self.update()

    def set_manual_focused(self, focused: bool):
        self.is_manual_focused = focused
        self._apply_styles()
        self.update()

    def _apply_styles(self):
        theme = get_theme(self.current_theme)

        # Frame border: glow with accent when manual focus is active
        if self.control_mode == "MANUAL" and self.is_manual_focused:
            accent = theme["focus_border_d1"] if self.drone_id == 1 else theme["focus_border_d2"]
            self.setStyleSheet(
                f"background-color: {theme['card_bg']}; border: 2px solid {accent}; border-radius: 8px;"
            )
        else:
            self.setStyleSheet(
                f"background-color: {theme['card_bg']}; border: 1px solid {theme['card_border']}; border-radius: 6px;"
            )

        # Control Bar Styling
        self.ctrl_bar.setStyleSheet(
            f"background-color: {theme['hud_ctrl_bg']}; border: 1px solid {theme['card_border']}; border-radius: 4px;"
        )

        # Mode button
        if self.control_mode == "MANUAL":
            self.btn_mode.setStyleSheet(theme["btn_mode_manual"])
        else:
            self.btn_mode.setStyleSheet(theme["btn_mode_auto"])

        # Flight button
        if self.is_flying:
            self.btn_flight.setStyleSheet(theme["btn_land"])
        else:
            self.btn_flight.setStyleSheet(theme["btn_takeoff"])

        badge_style = f"background-color: {theme['telemetry_badge_bg']}; color: {theme['telemetry_badge_text']}; padding: 3px 6px; border-radius: 3px; font-weight: bold; font-size: 11px;"
        self.lbl_telemetry.setStyleSheet(badge_style)
        self.lbl_battery.setStyleSheet(badge_style)

    def set_theme(self, theme_name: str):
        self.current_theme = theme_name
        self._apply_styles()
        self.update()

    def set_video_frame(self, frame: QImage | None):
        self.video_frame = frame
        self.update()

    def update_hud(
        self,
        t: float,
        risk_score: float,
        show_cracks: bool,
        show_humans: bool,
        conf_thresh: float,
        frame: QImage | None = None,
        battery: int = 88,
        altitude: float = 1.4,
    ):
        self.t = t
        self.risk_score = risk_score
        self.show_cracks = show_cracks
        self.show_humans = show_humans
        self.conf_thresh = conf_thresh
        self.battery = battery
        self.altitude = altitude
        if frame is not None:
            self.video_frame = frame

        self.lbl_telemetry.setText(f"Alt: {self.altitude:.1f}m")
        self.lbl_battery.setText(f"🔋 {self.battery}%")
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        theme = get_theme(self.current_theme)
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        view_h = h - self.ctrl_bar.height() - 12

        # 1. Background Video Feed or Synthetic Grid Canvas
        header_color = QColor(*theme["hud_header_bg"])
        if self.video_frame and not self.video_frame.isNull():
            p.drawImage(self.rect(), self.video_frame)
            # Subtle dark vignette / gradient for HUD legibility
            p.fillRect(0, 0, w, 28, header_color)
        else:
            p.fillRect(0, 0, w, view_h, QColor(theme["hud_bg"]))
            p.setPen(QPen(QColor(theme["hud_grid"]), 1))
            for x in range(0, w, 40):
                p.drawLine(x, 0, x, view_h)
            for y in range(0, view_h, 40):
                p.drawLine(0, y, w, y)

            # Center crosshair when waiting for video
            p.setPen(QPen(QColor(theme["hud_grid"]), 1, Qt.PenStyle.DashLine))
            p.drawLine(w // 2 - 20, view_h // 2, w // 2 + 20, view_h // 2)
            p.drawLine(w // 2, view_h // 2 - 20, w // 2, view_h // 2 + 20)

        # 2. Vector AI Overlays: Shear Cracks
        if self.show_cracks:
            p.setPen(QPen(QColor(theme["crack_draw"]), 2.5))
            cx = int(w * 0.45 + np.sin(self.t + self.drone_id) * 35)
            cy = int(view_h * 0.45)
            pts = [QPointF(cx - 40, cy - 50), QPointF(cx - 10, cy - 15), QPointF(cx + 15, cy + 10), QPointF(cx + 45, cy + 55)]
            for i in range(len(pts) - 1):
                p.drawLine(pts[i], pts[i + 1])
            p.setFont(QFont("Menlo", 9, QFont.Weight.Bold))
            p.setPen(QColor(theme["crack_text"]))
            p.drawText(cx + 15, cy, f"SHEAR CRACK | Risk: {self.risk_score:.1f}")

        # 3. Vector AI Overlays: Survivor Human Detection
        if self.show_humans and self.drone_id == 1 and 0.94 >= self.conf_thresh:
            bx = int(w * 0.22 + np.cos(self.t * 0.8) * 30)
            by = int(view_h * 0.40)
            p.setPen(QPen(QColor(theme["human_box"]), 2))
            p.setBrush(QBrush(QColor(*theme["human_fill"])))
            p.drawRect(bx, by, 85, 95)
            p.setPen(QColor(theme["human_text"]))
            p.drawText(bx, by - 6, "SURVIVOR [0.94] P0")

        # 4. Top Telemetry Header
        p.fillRect(0, 0, w, 28, header_color)

        accent_color = theme["hud_text_d1"] if self.drone_id == 1 else theme["hud_text_d2"]
        p.setPen(QColor(accent_color))
        p.setFont(QFont("SansSerif", 10, QFont.Weight.Bold))
        status_mode = f"[{self.control_mode}]"
        p.drawText(10, 19, f"DRONE {self.drone_id} {status_mode}: {self.role_title}")

        # Active Manual Focus Pill Badge
        if self.control_mode == "MANUAL":
            p.setFont(QFont("SansSerif", 9, QFont.Weight.Bold))
            p.setPen(QColor("#f59e0b"))
            p.drawText(w - 180, 19, "● MANUAL ACTIVE")

        # 5. Right-side Structural Risk Meter
        bar_h = int((self.risk_score / 10.0) * (view_h - 45))
        bar_col = QColor(theme["hazard_colors"]["HUMAN"]) if self.risk_score < 4 else (
            QColor(theme["hazard_colors"]["CRACK"]) if self.risk_score < 7.5 else QColor(theme["hazard_colors"]["FALL_RISK"])
        )
        p.fillRect(w - 14, view_h - 10 - bar_h, 8, bar_h, bar_col)