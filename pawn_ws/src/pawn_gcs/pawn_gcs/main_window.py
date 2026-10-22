from PySide6.QtCore import Qt, Slot, QTimer
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QSplitter, QApplication

from pawn_gcs.widgets.fleet_mission_bar import FleetMissionBar
from pawn_gcs.widgets.slam_viewport_3d import SlamViewport3D
from pawn_gcs.widgets.drone_hud_card import DroneHudCard
from pawn_gcs.widgets.triage_inspector import TriageInspector
from pawn_gcs.workers.mock_sim_worker import MockMissionWorker
from pawn_gcs.theme import apply_palette

class PawnGroundStation(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PAWN (PİYON) — Multi-Drone SAR & Structural Risk Ground Control Station")
        self.resize(1650, 960)
        self.hazard_actors = {}
        self.current_theme = "dark"

        # Manual control states
        self.active_manual_drone = 1
        self.pressed_keys = set()

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        self.top_bar = FleetMissionBar()
        root.addWidget(self.top_bar)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        root.addWidget(splitter, stretch=1)

        self.slam_view = SlamViewport3D()
        splitter.addWidget(self.slam_view)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(4, 0, 0, 0)

        hud_row = QHBoxLayout()
        self.hud1 = DroneHudCard(1, "Scout / Survivor & Crack CV")
        self.hud2 = DroneHudCard(2, "Mapper / Overhang Fall-Risk CV")
        hud_row.addWidget(self.hud1)
        hud_row.addWidget(self.hud2)
        right_layout.addLayout(hud_row, stretch=4)

        self.triage = TriageInspector()
        right_layout.addWidget(self.triage, stretch=3)
        splitter.addWidget(right_panel)
        splitter.setSizes([900, 750])

        # Wire UI Signals
        self.top_bar.frontiers_toggled.connect(
            lambda v: (self.slam_view.frontier_actor.SetVisibility(v), self.slam_view.plotter.update())
        )
        self.top_bar.theme_toggled.connect(self.set_theme)
        self.triage.hazard_focused.connect(lambda xyz: self.slam_view.plotter.fly_to(xyz))
        self.triage.hazard_dismissed.connect(self._remove_hazard_actor)

        # Wire Drone HUD Manual Controls
        self.hud1.mode_changed.connect(self._on_drone_mode_changed)
        self.hud2.mode_changed.connect(self._on_drone_mode_changed)
        self.hud1.flight_command.connect(self._on_flight_command)
        self.hud2.flight_command.connect(self._on_flight_command)
        self.hud1.focus_requested.connect(self._set_manual_focus)
        self.hud2.focus_requested.connect(self._set_manual_focus)

        # Start Worker
        self.worker = MockMissionWorker()
        self.worker.telemetry_tick.connect(self._on_tick)
        self.worker.new_hazard_event.connect(self._on_hazard)
        self.worker.start()

        # Keyboard Manual Control Dispatcher (25 Hz)
        self.manual_timer = QTimer(self)
        self.manual_timer.setInterval(40)
        self.manual_timer.timeout.connect(self._dispatch_manual_velocity)
        self.manual_timer.start()

    @Slot(str)
    def set_theme(self, theme_name: str):
        self.current_theme = theme_name
        app = QApplication.instance()
        if app:
            apply_palette(app, theme_name)
        self.top_bar.set_theme(theme_name)
        self.slam_view.set_theme(theme_name)
        self.hud1.set_theme(theme_name)
        self.hud2.set_theme(theme_name)
        self.triage.set_theme(theme_name)

    @Slot(int, str)
    def _on_drone_mode_changed(self, drone_id: int, mode: str):
        self.worker.set_drone_mode(drone_id, mode)
        if mode == "MANUAL":
            self._set_manual_focus(drone_id)
        else:
            if self.active_manual_drone == drone_id:
                hud = self.hud1 if drone_id == 1 else self.hud2
                hud.set_manual_focused(False)

    @Slot(int, str)
    def _on_flight_command(self, drone_id: int, action: str):
        self.worker.trigger_flight_action(drone_id, action)

    @Slot(int)
    def _set_manual_focus(self, drone_id: int):
        self.active_manual_drone = drone_id
        self.hud1.set_manual_focused(drone_id == 1 and self.hud1.control_mode == "MANUAL")
        self.hud2.set_manual_focused(drone_id == 2 and self.hud2.control_mode == "MANUAL")

    def keyPressEvent(self, event: QKeyEvent):
        key = event.key()
        self.pressed_keys.add(key)

        # Quick drone selection: key '1' or '2'
        if key == Qt.Key.Key_1:
            self._set_manual_focus(1)
        elif key == Qt.Key.Key_2:
            self._set_manual_focus(2)
        # Spacebar: Flight toggle for active manual drone
        elif key == Qt.Key.Key_Space:
            active_hud = self.hud1 if self.active_manual_drone == 1 else self.hud2
            if active_hud.control_mode == "MANUAL":
                active_hud._toggle_flight()
        # Escape: Reset manual mode to AUTO
        elif key == Qt.Key.Key_Escape:
            active_hud = self.hud1 if self.active_manual_drone == 1 else self.hud2
            if active_hud.control_mode == "MANUAL":
                active_hud.set_control_mode("AUTO")
                self.worker.set_drone_mode(self.active_manual_drone, "AUTO")
                self.worker.set_manual_velocity(self.active_manual_drone, 0.0, 0.0, 0.0, 0.0)

        super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QKeyEvent):
        self.pressed_keys.discard(event.key())
        super().keyReleaseEvent(event)

    def _dispatch_manual_velocity(self):
        active_hud = self.hud1 if self.active_manual_drone == 1 else self.hud2
        if active_hud.control_mode != "MANUAL":
            return

        vx, vy, vz, vyaw = 0.0, 0.0, 0.0, 0.0

        # WASD for Pitch / Roll
        if Qt.Key.Key_W in self.pressed_keys:
            vx += 1.0  # Forward pitch
        if Qt.Key.Key_S in self.pressed_keys:
            vx -= 1.0  # Backward pitch
        if Qt.Key.Key_A in self.pressed_keys:
            vy += 1.0  # Roll Left
        if Qt.Key.Key_D in self.pressed_keys:
            vy -= 1.0  # Roll Right

        # Arrow Keys for Altitude (Throttle) / Yaw
        if Qt.Key.Key_Up in self.pressed_keys:
            vz += 1.0  # Ascend
        if Qt.Key.Key_Down in self.pressed_keys:
            vz -= 1.0  # Descend
        if Qt.Key.Key_Left in self.pressed_keys:
            vyaw += 1.0  # Yaw Rotate Left
        if Qt.Key.Key_Right in self.pressed_keys:
            vyaw -= 1.0  # Yaw Rotate Right

        self.worker.set_manual_velocity(self.active_manual_drone, vx, vy, vz, vyaw)

    @Slot(dict)
    def _on_tick(self, d: dict):
        self.slam_view.update_drones(d["d1_pos"], d["d1_path"], d["d2_pos"], d["d2_path"])
        conf = self.top_bar.conf_slider.value() / 100.0
        show_c = self.top_bar.chk_cracks.isChecked()
        show_h = self.top_bar.chk_humans.isChecked()

        self.hud1.update_hud(
            d["t"],
            d["d1_risk"],
            show_c,
            show_h,
            conf,
            frame=d.get("d1_frame"),
            battery=d.get("d1_bat", 88),
            altitude=d.get("d1_alt", 1.4),
        )
        self.hud2.update_hud(
            d["t"],
            d["d2_risk"],
            show_c,
            show_h,
            conf,
            frame=d.get("d2_frame"),
            battery=d.get("d2_bat", 94),
            altitude=d.get("d2_alt", 2.2),
        )

    @Slot(object)
    def _on_hazard(self, hz):
        actors = self.slam_view.add_hazard_pin(hz)
        self.hazard_actors[hz.id] = actors
        self.triage.add_hazard(hz)

    def _remove_hazard_actor(self, hid: int):
        actors = self.hazard_actors.pop(hid, None)
        if actors:
            self.slam_view.plotter.remove_actor(actors[0])
            if actors[1]:
                self.slam_view.plotter.remove_actor(actors[1])
            self.slam_view.plotter.update()

    def closeEvent(self, event):
        self.manual_timer.stop()
        self.worker.stop()
        self.slam_view.plotter.close()
        super().closeEvent(event)