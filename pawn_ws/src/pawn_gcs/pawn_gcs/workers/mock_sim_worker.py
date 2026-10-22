import numpy as np
from PySide6.QtCore import QThread, Signal, Qt, QPointF
from PySide6.QtGui import QImage, QPainter, QColor, QPen, QLinearGradient
from pawn_gcs.data_models import HazardEvent

class MockMissionWorker(QThread):
    telemetry_tick = Signal(dict)
    new_hazard_event = Signal(object)  # Emits HazardEvent

    def __init__(self):
        super().__init__()
        self._running = True

        # Drone states
        self.d1_mode = "AUTO"
        self.d2_mode = "AUTO"
        self.d1_manual_vel = np.zeros(4)  # [vx, vy, vz, vyaw]
        self.d2_manual_vel = np.zeros(4)
        self.d1_pos = np.array([3.2, 0.0, 1.4])
        self.d2_pos = np.array([0.0, 3.5, 2.2])
        self.d1_bat = 88.0
        self.d2_bat = 94.0
        self.d1_flying = True
        self.d2_flying = True

    def set_drone_mode(self, drone_id: int, mode: str):
        if drone_id == 1:
            self.d1_mode = mode
        elif drone_id == 2:
            self.d2_mode = mode

    def set_manual_velocity(self, drone_id: int, vx: float, vy: float, vz: float, vyaw: float):
        if drone_id == 1:
            self.d1_manual_vel = np.array([vx, vy, vz, vyaw], dtype=float)
        elif drone_id == 2:
            self.d2_manual_vel = np.array([vx, vy, vz, vyaw], dtype=float)

    def trigger_flight_action(self, drone_id: int, action: str):
        if drone_id == 1:
            self.d1_flying = (action == "takeoff")
        elif drone_id == 2:
            self.d2_flying = (action == "takeoff")

    def _generate_synthetic_camera_frame(self, t: float, drone_id: int, vx: float, vy: float) -> QImage:
        """Procedurally render a simulated 720p/360p drone onboard camera stream."""
        w, h = 360, 240
        img = QImage(w, h, QImage.Format.Format_RGB32)

        p = QPainter(img)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        # 1. Concrete Room Gradient Background
        grad = QLinearGradient(0, 0, 0, h)
        if drone_id == 1:
            grad.setColorAt(0.0, QColor(25, 32, 44))
            grad.setColorAt(0.65, QColor(45, 55, 72))
            grad.setColorAt(1.0, QColor(15, 20, 28))
        else:
            grad.setColorAt(0.0, QColor(30, 28, 42))
            grad.setColorAt(0.65, QColor(50, 48, 68))
            grad.setColorAt(1.0, QColor(18, 16, 26))
        p.fillRect(0, 0, w, h, grad)

        # 2. Vanishing Perspective Lines (Wall / Corridor Depth)
        tilt_x = int(np.sin(t * 1.5 + drone_id) * 6 + vy * 12)
        tilt_y = int(np.cos(t * 1.2) * 4 + vx * 10)
        vp_x = w // 2 + tilt_x
        vp_y = h // 2 + tilt_y

        p.setPen(QPen(QColor(70, 85, 105, 120), 1.5))
        p.drawLine(0, 0, vp_x, vp_y)
        p.drawLine(w, 0, vp_x, vp_y)
        p.drawLine(0, h, vp_x, vp_y)
        p.drawLine(w, h, vp_x, vp_y)

        # Wall horizontal joint seams
        p.setPen(QPen(QColor(60, 75, 95, 80), 1))
        for seam_y in [h * 0.35, h * 0.70]:
            p.drawLine(0, int(seam_y), w, int(seam_y))

        # 3. Simulated Structural Crack on Wall
        p.setPen(QPen(QColor(30, 35, 45, 180), 2))
        crack_base_x = vp_x + int(np.cos(t * 0.5) * 20)
        pts = [
            QPointF(crack_base_x, vp_y - 20),
            QPointF(crack_base_x + 15, vp_y + 15),
            QPointF(crack_base_x + 8, vp_y + 40),
            QPointF(crack_base_x + 22, vp_y + 70),
        ]
        for i in range(len(pts) - 1):
            p.drawLine(pts[i], pts[i + 1])

        # 4. Subtle Optical Center Reticle
        p.setPen(QPen(QColor(255, 255, 255, 60), 1))
        p.drawEllipse(vp_x - 12, vp_y - 12, 24, 24)
        p.drawLine(vp_x - 18, vp_y, vp_x + 18, vp_y)
        p.drawLine(vp_x, vp_y - 18, vp_x, vp_y + 18)

        # 5. Timestamp Watermark
        p.setPen(QColor(200, 210, 225, 180))
        p.setFont(p.font())
        p.drawText(12, h - 12, f"TELLO-CAM-{drone_id:02d} | 720p 30FPS | T+{t:.1f}s")

        p.end()
        return img

    def run(self):
        t = 0.0
        hazard_counter = 0
        templates = [
            ("HUMAN", "Survivor Detected (Prone)", 9.6, "Conf: 94% | Pose: Stationary"),
            ("FALL_RISK", "Unstable Concrete Overhang", 8.9, "Spalling: 0.8m² | Drop Radius: 1.5m"),
            ("CRACK", "Diagonal Shear Wall Crack", 7.4, "Angle: 46° | Width: 4.8mm"),
            ("CRACK", "Vertical Flexural Crack", 4.2, "Angle: 88° | Width: 1.5mm"),
            ("FALL_RISK", "Exposed Buckled Ceiling Beam", 9.2, "Beam #3 | Collapse Hazard"),
        ]

        while self._running:
            t += 0.05
            self.d1_bat = max(5.0, self.d1_bat - 0.005)
            self.d2_bat = max(5.0, self.d2_bat - 0.006)

            # DRONE 1: Position computation (Manual vs Auto)
            if self.d1_mode == "MANUAL":
                # Manual velocity integration
                vx, vy, vz, _ = self.d1_manual_vel
                self.d1_pos[0] += vx * 0.06
                self.d1_pos[1] += vy * 0.06
                self.d1_pos[2] = float(np.clip(self.d1_pos[2] + vz * 0.04, 0.2, 3.5))
                p1 = self.d1_pos.copy()
                # Project forward trajectory
                path1 = np.array([p1 + [vx * step * 0.1, vy * step * 0.1, vz * step * 0.05] for step in range(15)])
            else:
                p1 = np.array([np.cos(t) * 3.2, np.sin(t) * 3.2, 1.4 + 0.3 * np.sin(t * 2)])
                self.d1_pos = p1.copy()
                ft1 = np.linspace(t, t + 1.4, 15)
                path1 = np.column_stack([np.cos(ft1) * 3.2, np.sin(ft1) * 3.2, 1.4 + 0.3 * np.sin(ft1 * 2)])

            # DRONE 2: Position computation (Manual vs Auto)
            if self.d2_mode == "MANUAL":
                vx, vy, vz, _ = self.d2_manual_vel
                self.d2_pos[0] += vx * 0.06
                self.d2_pos[1] += vy * 0.06
                self.d2_pos[2] = float(np.clip(self.d2_pos[2] + vz * 0.04, 0.2, 3.5))
                p2 = self.d2_pos.copy()
                path2 = np.array([p2 + [vx * step * 0.1, vy * step * 0.1, vz * step * 0.05] for step in range(15)])
            else:
                p2 = np.array([np.cos(-t * 0.7) * 2.0, np.sin(-t * 0.7) * 3.5, 2.2])
                self.d2_pos = p2.copy()
                ft2 = np.linspace(t, t + 1.4, 15)
                path2 = np.column_stack([np.cos(-ft2 * 0.7) * 2.0, np.sin(-ft2 * 0.7) * 3.5, np.full(15, 2.2)])

            # Generate synthetic video feeds
            d1_frame = self._generate_synthetic_camera_frame(t, 1, self.d1_manual_vel[0], self.d1_manual_vel[1])
            d2_frame = self._generate_synthetic_camera_frame(t, 2, self.d2_manual_vel[0], self.d2_manual_vel[1])

            self.telemetry_tick.emit({
                "t": t,
                "d1_pos": p1,
                "d1_path": path1,
                "d1_risk": 5.0 + 4.5 * np.sin(t * 1.3) ** 2,
                "d1_frame": d1_frame,
                "d1_bat": int(self.d1_bat),
                "d1_alt": float(p1[2]),
                "d2_pos": p2,
                "d2_path": path2,
                "d2_risk": 3.0 + 6.0 * np.cos(t * 0.9) ** 2,
                "d2_frame": d2_frame,
                "d2_bat": int(self.d2_bat),
                "d2_alt": float(p2[2]),
            })

            # Periodic hazard events
            if int(t * 20) % 70 == 0 and hazard_counter < 10:
                htype, title, score, metrics = templates[hazard_counter % len(templates)]
                angle = hazard_counter * 1.1
                wall_xyz = np.array([np.cos(angle) * 4.5, np.sin(angle) * 4.5, 1.0 + (hazard_counter % 3) * 0.8])
                hazard_counter += 1
                self.new_hazard_event.emit(HazardEvent(
                    id=hazard_counter,
                    drone_id=1 if hazard_counter % 2 == 1 else 2,
                    hazard_type=htype,
                    title=title,
                    score=score,
                    metrics=metrics,
                    xyz=wall_xyz,
                ))

            self.msleep(50)

    def stop(self):
        self._running = False
        self.wait()