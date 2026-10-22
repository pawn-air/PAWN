from PySide6.QtGui import QPalette, QColor
from PySide6.QtWidgets import QApplication

DARK_THEME = {
    "name": "dark",
    "window_bg": "#0f172a",       # Slate 900
    "window_text": "#f8fafc",     # Slate 50
    "base": "#020617",            # Slate 950
    "alt_base": "#1e293b",        # Slate 800
    "text": "#f8fafc",
    "button": "#1e293b",
    "button_text": "#f8fafc",
    "highlight": "#0284c7",       # Sky 600
    "highlight_text": "#ffffff",
    "card_bg": "#0f172a",
    "card_border": "#334155",
    "hud_bg": "#090d16",
    "hud_grid": "#1e293b",
    "hud_header_bg": (15, 23, 42, 220),
    "hud_text_d1": "#38bdf8",     # Cyan
    "hud_text_d2": "#c084fc",     # Purple
    "pyvista_bg": "#090d16",
    "pyvista_grid": "#1e293b",
    "list_bg": "#020617",
    "list_border": "#1e293b",
    "hazard_colors": {
        "HUMAN": "#22c55e",       # Green
        "FALL_RISK": "#ef4444",   # Red
        "CRACK": "#eab308",       # Amber
    },
    "crack_draw": "#f59e0b",
    "crack_text": "#fbbf24",
    "human_box": "#22c55e",
    "human_fill": (34, 197, 94, 35),
    "human_text": "#4ade80",
    "hud_ctrl_bg": "#0f172a",
    "focus_border_d1": "#38bdf8",
    "focus_border_d2": "#c084fc",
    "btn_mode_auto": "QPushButton { background-color: #0369a1; color: white; font-weight: bold; border-radius: 3px; padding: 4px 8px; border: none; }",
    "btn_mode_manual": "QPushButton { background-color: #f59e0b; color: #020617; font-weight: bold; border-radius: 3px; padding: 4px 8px; border: none; }",
    "btn_takeoff": "QPushButton { background-color: #16a34a; color: white; font-weight: bold; border-radius: 3px; padding: 4px 8px; border: none; } QPushButton:hover { background-color: #15803d; }",
    "btn_land": "QPushButton { background-color: #dc2626; color: white; font-weight: bold; border-radius: 3px; padding: 4px 8px; border: none; } QPushButton:hover { background-color: #b91c1c; }",
    "telemetry_badge_bg": "#1e293b",
    "telemetry_badge_text": "#94a3b8",
    "toggle_btn_text": "☀️ Light Mode",
    "toggle_btn_style": (
        "QPushButton { background-color: #1e293b; color: #f8fafc; font-weight: bold; "
        "padding: 6px 12px; border: 1px solid #475569; border-radius: 4px; } "
        "QPushButton:hover { background-color: #334155; }"
    ),
}

LIGHT_THEME = {
    "name": "light",
    "window_bg": "#f8fafc",       # Slate 50
    "window_text": "#0f172a",     # Slate 900
    "base": "#ffffff",            # Pure white
    "alt_base": "#f1f5f9",        # Slate 100
    "text": "#0f172a",
    "button": "#e2e8f0",          # Slate 200
    "button_text": "#0f172a",
    "highlight": "#0284c7",       # Sky 600
    "highlight_text": "#ffffff",
    "card_bg": "#ffffff",
    "card_border": "#cbd5e1",     # Slate 300
    "hud_bg": "#f8fafc",
    "hud_grid": "#e2e8f0",
    "hud_header_bg": (241, 245, 249, 230),
    "hud_text_d1": "#0284c7",     # Darker cyan/sky for contrast on light
    "hud_text_d2": "#7c3aed",     # Darker purple for contrast on light
    "pyvista_bg": "#f1f5f9",      # Slate 100
    "pyvista_grid": "#cbd5e1",
    "list_bg": "#ffffff",
    "list_border": "#cbd5e1",
    "hazard_colors": {
        "HUMAN": "#16a34a",       # High contrast green
        "FALL_RISK": "#dc2626",   # High contrast red
        "CRACK": "#b45309",       # High contrast amber
    },
    "crack_draw": "#d97706",
    "crack_text": "#92400e",
    "human_box": "#16a34a",
    "human_fill": (22, 163, 74, 30),
    "human_text": "#15803d",
    "hud_ctrl_bg": "#f1f5f9",
    "focus_border_d1": "#0284c7",
    "focus_border_d2": "#7c3aed",
    "btn_mode_auto": "QPushButton { background-color: #0284c7; color: white; font-weight: bold; border-radius: 3px; padding: 4px 8px; border: none; }",
    "btn_mode_manual": "QPushButton { background-color: #d97706; color: white; font-weight: bold; border-radius: 3px; padding: 4px 8px; border: none; }",
    "btn_takeoff": "QPushButton { background-color: #15803d; color: white; font-weight: bold; border-radius: 3px; padding: 4px 8px; border: none; } QPushButton:hover { background-color: #166534; }",
    "btn_land": "QPushButton { background-color: #dc2626; color: white; font-weight: bold; border-radius: 3px; padding: 4px 8px; border: none; } QPushButton:hover { background-color: #b91c1c; }",
    "telemetry_badge_bg": "#e2e8f0",
    "telemetry_badge_text": "#475569",
    "toggle_btn_text": "🌙 Dark Mode",
    "toggle_btn_style": (
        "QPushButton { background-color: #f1f5f9; color: #0f172a; font-weight: bold; "
        "padding: 6px 12px; border: 1px solid #94a3b8; border-radius: 4px; } "
        "QPushButton:hover { background-color: #e2e8f0; }"
    ),
}

THEMES = {
    "dark": DARK_THEME,
    "light": LIGHT_THEME,
}

def get_theme(name: str) -> dict:
    return THEMES.get(name, DARK_THEME)

def apply_palette(app: QApplication, mode: str = "dark"):
    theme = get_theme(mode)
    app.setStyle("Fusion")
    p = QPalette()
    p.setColor(QPalette.ColorRole.Window, QColor(theme["window_bg"]))
    p.setColor(QPalette.ColorRole.WindowText, QColor(theme["window_text"]))
    p.setColor(QPalette.ColorRole.Base, QColor(theme["base"]))
    p.setColor(QPalette.ColorRole.AlternateBase, QColor(theme["alt_base"]))
    p.setColor(QPalette.ColorRole.Text, QColor(theme["text"]))
    p.setColor(QPalette.ColorRole.Button, QColor(theme["button"]))
    p.setColor(QPalette.ColorRole.ButtonText, QColor(theme["button_text"]))
    p.setColor(QPalette.ColorRole.Highlight, QColor(theme["highlight"]))
    p.setColor(QPalette.ColorRole.HighlightedText, QColor(theme["highlight_text"]))
    app.setPalette(p)
