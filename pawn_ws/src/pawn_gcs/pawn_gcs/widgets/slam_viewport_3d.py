import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton
from pawn_gcs.theme import get_theme

class SlamViewport3D(QWidget):
    def __init__(self):
        super().__init__()
        self.current_theme = "dark"
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        cam_bar = QHBoxLayout()
        cam_bar.addWidget(QLabel("<b>3D SLAM & Structural Risk Map</b>"))
        cam_bar.addStretch()
        btn_iso = QPushButton("3D Isometric")
        btn_top = QPushButton("2D Floorplan")
        btn_reset = QPushButton("Reset View")
        btn_iso.clicked.connect(self.set_view_isometric)
        btn_top.clicked.connect(self.set_view_topdown)
        btn_reset.clicked.connect(lambda: self.plotter.reset_camera())
        cam_bar.addWidget(btn_iso)
        cam_bar.addWidget(btn_top)
        cam_bar.addWidget(btn_reset)
        layout.addLayout(cam_bar)

        self.plotter = QtInteractor(self)
        self.plotter.set_background("#090d16")
        self.plotter.show_axes()
        self.plotter.show_grid(color="#1e293b")
        layout.addWidget(self.plotter.interactor, stretch=1)

        self._init_scene()

    def set_theme(self, theme_name: str):
        self.current_theme = theme_name
        theme = get_theme(theme_name)
        self.plotter.set_background(theme["pyvista_bg"])
        self.plotter.show_grid(color=theme["pyvista_grid"])
        self.plotter.update()


    def _init_scene(self):
        rng = np.random.default_rng(42)
        angles = rng.uniform(0, 2 * np.pi, 3500)
        z_wall = rng.uniform(0.0, 3.2, 3500)
        r_wall = 4.6 + rng.normal(0, 0.06, 3500)
        wall_pts = np.column_stack([np.cos(angles) * r_wall, np.sin(angles) * r_wall, z_wall])

        self.slam_cloud = pv.PolyData(wall_pts)
        self.slam_cloud["Elevation"] = wall_pts[:, 2]
        self.plotter.add_mesh(self.slam_cloud, scalars="Elevation", cmap="viridis", point_size=4, show_scalar_bar=False)

        self.d1_mesh = pv.Sphere(radius=0.22)
        self.d2_mesh = pv.Sphere(radius=0.22)
        self.plotter.add_mesh(self.d1_mesh, color="#38bdf8")
        self.plotter.add_mesh(self.d2_mesh, color="#c084fc")

        frontiers = pv.PolyData(np.array([[3.5, 2.5, 1.8], [-3.2, 2.8, 1.5], [0.0, -3.8, 2.2]]))
        self.frontier_actor = self.plotter.add_mesh(frontiers, color="#22d3ee", point_size=16, render_points_as_spheres=True)

        self.path1_actor = None
        self.path2_actor = None
        self.set_view_isometric()

    def set_view_isometric(self):
        self.plotter.disable_parallel_projection()
        self.plotter.view_isometric()

    def set_view_topdown(self):
        self.plotter.enable_parallel_projection()
        self.plotter.view_xy()

    def update_drones(self, d1_pos, d1_path, d2_pos, d2_path):
        self.d1_mesh.points = pv.Sphere(radius=0.22, center=d1_pos).points
        self.d2_mesh.points = pv.Sphere(radius=0.22, center=d2_pos).points
        if self.path1_actor:
            self.plotter.remove_actor(self.path1_actor)
        if self.path2_actor:
            self.plotter.remove_actor(self.path2_actor)
        self.path1_actor = self.plotter.add_mesh(pv.Spline(d1_path, 15), color="#38bdf8", line_width=3)
        self.path2_actor = self.plotter.add_mesh(pv.Spline(d2_path, 15), color="#c084fc", line_width=3)
        self.plotter.update()

    def add_hazard_pin(self, hz):
        theme = get_theme(self.current_theme)
        pin_color = theme["hazard_colors"].get(hz.hazard_type, "white")
        pin_actor = self.plotter.add_mesh(pv.Sphere(radius=0.22, center=hz.xyz), color=pin_color)
        drop_actor = None
        if hz.hazard_type == "FALL_RISK":
            disc = pv.Disc(center=(hz.xyz[0], hz.xyz[1], 0.02), inner=0.0, outer=1.1, normal=(0, 0, 1))
            drop_actor = self.plotter.add_mesh(disc, color=pin_color, opacity=0.35)
        return pin_actor, drop_actor