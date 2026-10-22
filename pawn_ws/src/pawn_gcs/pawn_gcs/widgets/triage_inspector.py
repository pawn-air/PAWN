from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import QGroupBox, QHBoxLayout, QVBoxLayout, QListWidget, QListWidgetItem, QFrame, QLabel, QPushButton
from pawn_gcs.theme import get_theme

class TriageInspector(QGroupBox):
    hazard_focused = Signal(object)    # Emits xyz array to fly camera
    hazard_dismissed = Signal(int)     # Emits hazard_id

    def __init__(self):
        super().__init__("Post-Earthquake Structural & Survivor Triage Log (Click to Fly 3D Camera)")
        self.hazards = {}
        self.current_theme = "dark"
        layout = QHBoxLayout(self)

        self.list_widget = QListWidget()
        self.list_widget.itemClicked.connect(self._on_item_clicked)
        layout.addWidget(self.list_widget, stretch=3)

        self.card = QFrame()
        c_layout = QVBoxLayout(self.card)
        self.lbl_title = QLabel("<b>Select a hazard from the log</b>")
        self.lbl_title.setWordWrap(True)
        self.lbl_coords = QLabel("3D Pos: —")
        self.lbl_metrics = QLabel("Metrics: —")
        self.lbl_metrics.setWordWrap(True)

        self.btn_dismiss = QPushButton("Dismiss False Positive")
        self.btn_dismiss.clicked.connect(self._on_dismiss)

        c_layout.addWidget(self.lbl_title)
        c_layout.addWidget(self.lbl_coords)
        c_layout.addWidget(self.lbl_metrics)
        c_layout.addStretch()
        c_layout.addWidget(self.btn_dismiss)
        layout.addWidget(self.card, stretch=2)

        self._apply_theme_styles()

    def _apply_theme_styles(self):
        theme = get_theme(self.current_theme)
        self.card.setStyleSheet(
            f"background-color: {theme['card_bg']}; border: 1px solid {theme['card_border']}; "
            f"border-radius: 6px; padding: 8px;"
        )
        self.list_widget.setStyleSheet(
            f"QListWidget {{ background-color: {theme['list_bg']}; border: 1px solid {theme['list_border']}; "
            f"border-radius: 4px; padding: 4px; }} "
            f"QListWidget::item {{ padding: 6px; border-bottom: 1px solid {theme['alt_base']}; }} "
            f"QListWidget::item:selected {{ background-color: {theme['highlight']}; color: {theme['highlight_text']}; }}"
        )

    def set_theme(self, theme_name: str):
        self.current_theme = theme_name
        self._apply_theme_styles()
        theme = get_theme(theme_name)
        # Update colors of existing list items
        for i in range(self.list_widget.count()):
            item = self.list_widget.item(i)
            hid = item.data(Qt.ItemDataRole.UserRole)
            hz = self.hazards.get(hid)
            if hz:
                item.setForeground(QColor(theme["hazard_colors"].get(hz.hazard_type, theme["text"])))

    def add_hazard(self, hz):
        self.hazards[hz.id] = hz
        theme = get_theme(self.current_theme)
        color = theme["hazard_colors"].get(hz.hazard_type, theme["text"])
        prefix = "P0 SURVIVOR" if hz.hazard_type == "HUMAN" else ("P1 FALL RISK" if hz.hazard_type == "FALL_RISK" else "P2 CRACK")
        item = QListWidgetItem(f"[{prefix}] #{hz.id} {hz.title} ({hz.score:.1f}/10)")
        item.setData(Qt.ItemDataRole.UserRole, hz.id)
        item.setForeground(QColor(color))
        self.list_widget.insertItem(0, item)

    def _on_item_clicked(self, item):
        hz = self.hazards.get(item.data(Qt.ItemDataRole.UserRole))
        if hz:
            self.lbl_title.setText(f"<b>#{hz.id} — {hz.title}</b> (Drone {hz.drone_id})")
            self.lbl_coords.setText(f"<b>3D Pos:</b> X={hz.xyz[0]:.2f}m, Y={hz.xyz[1]:.2f}m, Z={hz.xyz[2]:.2f}m")
            self.lbl_metrics.setText(f"<b>Severity:</b> {hz.score:.1f}/10<br>{hz.metrics}")
            self.hazard_focused.emit(hz.xyz)

    def _on_dismiss(self):
        item = self.list_widget.currentItem()
        if item:
            hid = item.data(Qt.ItemDataRole.UserRole)
            self.hazards.pop(hid, None)
            self.list_widget.takeItem(self.list_widget.row(item))
            self.lbl_title.setText("<b>Hazard Dismissed</b>")
            self.hazard_dismissed.emit(hid)