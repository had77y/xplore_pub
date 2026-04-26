# ──────────────────────────────────────────────────────────────────────────────
# rover_gui.py — Interface graphique de contrôle du rover XPlore
#
# Remplace controller_node + video_viewer_node : tout est centralisé ici.
#
# Fonctionnalités :
#   - Menu principal cliquable à la souris (Autonome / Téléop / Quitter)
#   - Mode race : flux caméra à droite, panneau commandes à gauche, clavier WASD
#   - Mode autonome : état du rover + détections ArUco en direct
#   - Logs ArUco via /aruco_detected
#
# Topics publiés :
#   /rover/mode     (std_msgs/String)        — autonomous / race / arm / idle
#   /rover/cmd_vel  (geometry_msgs/Twist)    — uniquement en mode race
#
# Topics écoutés :
#   /camera/image_compressed (sensor_msgs/CompressedImage)
#   /aruco_detected          (std_msgs/Float32MultiArray)
#   /rover_status            (std_msgs/String, optionnel)
# ──────────────────────────────────────────────────────────────────────────────

import os
import sys
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist

from PySide6.QtCore import (
    Qt, QObject, Signal, QTimer, QSize,
    QPropertyAnimation, QEasingCurve,
)
from PySide6.QtGui import QImage, QPixmap, QFont, QFontDatabase, QKeyEvent
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QStackedWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QFrame, QSizePolicy, QSpacerItem,
    QGraphicsOpacityEffect,
)


# ══════════════════════════════════════════════════════════════════════════════
# PALETTE XPLORE — bleu marine profond inspiré du logo
# ══════════════════════════════════════════════════════════════════════════════

# Fonds — bleu marine exact du logo XPlore (sobre, profond)
BG_DEEP     = '#0E1A33'  # fond principal — couleur du logo
BG_SURFACE  = '#16223D'  # cartes (subtilement plus claire)
BG_ELEVATED = '#1C2A48'  # cartes élevées / hover

# Bordures (subtiles, ne ressortent que peu)
BORDER       = '#243756'
BORDER_HOVER = '#36507A'

# Identité
PRIMARY     = '#E53935'  # rouge XPlore
PRIMARY_HOV = '#FF5A55'
ACCENT      = '#00AEEF'  # cyan
ACCENT_2    = '#5AE8B5'  # vert succès (ArUco détecté)

# Texte
TEXT        = '#FFFFFF'
TEXT_DIM    = '#B8C5D9'
TEXT_MUTED  = '#6B7A95'

LOGO_PATH = os.path.join(os.path.dirname(__file__), 'xplore_logo.jpg')


# ══════════════════════════════════════════════════════════════════════════════
# QSS — STYLE GLOBAL
# ══════════════════════════════════════════════════════════════════════════════

GLOBAL_QSS = f"""
* {{
    color: {TEXT};
    font-family: "Inter", "SF Pro Display", "Helvetica Neue", "Segoe UI", sans-serif;
    outline: 0;
}}

QMainWindow, QWidget#root {{
    background-color: {BG_DEEP};
}}

/* ── Header / Footer ─────────────────────────────────────── */

QFrame#header {{
    background-color: {BG_DEEP};
    border-bottom: 1px solid {BORDER};
}}

QFrame#footer {{
    background-color: {BG_DEEP};
    border-top: 1px solid {BORDER};
}}

QLabel#breadcrumb {{
    font-size: 12px;
    font-weight: 500;
    color: {TEXT_MUTED};
    letter-spacing: 2px;
}}

QLabel#breadcrumb_active {{
    font-size: 12px;
    font-weight: 600;
    color: {TEXT};
    letter-spacing: 2px;
}}

QLabel#footer_label {{
    font-size: 11px;
    color: {TEXT_MUTED};
    letter-spacing: 2px;
}}

QLabel#footer_value {{
    font-size: 11px;
    color: {ACCENT};
    font-weight: 600;
    letter-spacing: 2px;
}}

/* ── Typographie ─────────────────────────────────────────── */

QLabel#hero {{
    font-size: 44px;
    font-weight: 200;
    letter-spacing: 6px;
    color: {TEXT};
}}

QLabel#title {{
    font-size: 30px;
    font-weight: 300;
    letter-spacing: 4px;
    color: {TEXT};
}}

QLabel#subtitle {{
    font-size: 13px;
    font-weight: 400;
    letter-spacing: 4px;
    color: {TEXT_MUTED};
}}

QLabel#section {{
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 3px;
    color: {TEXT_MUTED};
}}

QLabel#status_label {{
    font-size: 12px;
    color: {TEXT_DIM};
    letter-spacing: 1px;
}}

QLabel#status_value {{
    font-size: 13px;
    color: {ACCENT};
    font-weight: 600;
    letter-spacing: 1px;
}}

/* ── Boutons menu (gros, soft) ───────────────────────────── */

QPushButton#menu_btn {{
    background-color: {BG_SURFACE};
    border: 1px solid {BORDER};
    border-radius: 16px;
    padding: 28px 40px;
    font-size: 16px;
    font-weight: 600;
    letter-spacing: 4px;
    min-width: 340px;
    min-height: 72px;
    color: {TEXT};
    text-align: center;
}}
QPushButton#menu_btn:hover {{
    background-color: {BG_ELEVATED};
    border-color: {PRIMARY};
    color: {TEXT};
}}
QPushButton#menu_btn:pressed {{
    background-color: {PRIMARY};
    border-color: {PRIMARY};
}}

QPushButton#menu_btn_quit {{
    background-color: transparent;
    border: 1px solid {BORDER};
    border-radius: 16px;
    padding: 22px 40px;
    font-size: 12px;
    font-weight: 500;
    letter-spacing: 4px;
    min-width: 340px;
    color: {TEXT_MUTED};
}}
QPushButton#menu_btn_quit:hover {{
    border-color: {PRIMARY};
    color: {PRIMARY};
    background-color: rgba(229, 57, 53, 25);
}}

QPushButton#ghost_btn {{
    background-color: transparent;
    border: 1px solid {BORDER};
    border-radius: 10px;
    padding: 12px 24px;
    font-size: 12px;
    font-weight: 600;
    letter-spacing: 3px;
    color: {TEXT_DIM};
}}
QPushButton#ghost_btn:hover {{
    border-color: {ACCENT};
    color: {ACCENT};
}}

/* ── Cards ───────────────────────────────────────────────── */

QFrame#card {{
    background-color: {BG_SURFACE};
    border-radius: 18px;
    border: 1px solid {BORDER};
}}

QFrame#card_elevated {{
    background-color: {BG_ELEVATED};
    border-radius: 18px;
    border: 1px solid {BORDER_HOVER};
}}

/* ── Vidéo ───────────────────────────────────────────────── */

QLabel#video {{
    background-color: #050B17;
    border-radius: 12px;
    border: 1px solid {BORDER};
    color: {TEXT_MUTED};
}}

/* ── Badges ──────────────────────────────────────────────── */

QLabel#mode_badge {{
    background-color: {PRIMARY};
    color: {TEXT};
    padding: 8px 18px;
    border-radius: 14px;
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 4px;
}}

QLabel#mode_badge_auto {{
    background-color: {ACCENT};
    color: {BG_DEEP};
    padding: 8px 18px;
    border-radius: 14px;
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 4px;
}}

/* ── Console ArUco ──────────────────────────────────────── */

QLabel#aruco_log {{
    background-color: rgba(5, 11, 23, 200);
    border: 1px solid {BORDER};
    border-radius: 10px;
    padding: 16px;
    font-family: "JetBrains Mono", "Menlo", "Consolas", monospace;
    font-size: 11px;
    color: {ACCENT_2};
    letter-spacing: 1px;
}}
"""


# ══════════════════════════════════════════════════════════════════════════════
# ROS2 BRIDGE — Node + Signaux Qt
# ══════════════════════════════════════════════════════════════════════════════

VIDEO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class RosBridge(QObject):
    """Pont ROS2 ↔ Qt. Émet des signaux Qt depuis les callbacks ROS2.

    Composition plutôt qu'héritage multiple (QObject + Node sont incompatibles
    avec super().__init__ — MRO conflict sur node_name).
    """

    frame_ready = Signal(QImage)
    aruco_update = Signal(bool, int, float, float, float)  # detected, id, cx, cy, area
    status_update = Signal(str)

    def __init__(self):
        super().__init__()
        self._node = _RosNode(self)

    # ── Publications ──

    def publish_mode(self, mode: str):
        self._node.publish_mode(mode)

    def publish_cmd(self, linear: float, angular: float):
        self._node.publish_cmd(linear, angular)

    def destroy_node(self):
        self._node.destroy_node()

    @property
    def node(self):
        return self._node


class _RosNode(Node):
    """Node ROS2 interne — callbacks émettent les signaux Qt via bridge."""

    def __init__(self, bridge: RosBridge):
        super().__init__('rover_gui')
        self._bridge = bridge

        self.pub_mode = self.create_publisher(String, '/rover/mode', 10)
        self.pub_cmd  = self.create_publisher(Twist, '/rover/cmd_vel', 10)

        self.create_subscription(
            CompressedImage, '/camera/image_compressed',
            self._on_image, VIDEO_QOS,
        )
        self.create_subscription(
            Float32MultiArray, '/aruco_detected', self._on_aruco, 10
        )
        self.create_subscription(
            String, '/rover_status', self._on_status, 10
        )

    def publish_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.pub_mode.publish(msg)

    def publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub_cmd.publish(msg)

    def _on_image(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if bgr is None:
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w, _ = rgb.shape
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888).copy()
        self._bridge.frame_ready.emit(qimg)

    def _on_aruco(self, msg: Float32MultiArray):
        if len(msg.data) < 5:
            return
        detected = msg.data[0] > 0.5
        self._bridge.aruco_update.emit(
            detected, int(msg.data[1]), msg.data[2], msg.data[3], msg.data[4]
        )

    def _on_status(self, msg: String):
        self._bridge.status_update.emit(msg.data)


# ══════════════════════════════════════════════════════════════════════════════
# UTILITAIRES UI
# ══════════════════════════════════════════════════════════════════════════════

def make_logo(max_height=120) -> QLabel:
    label = QLabel()
    if os.path.exists(LOGO_PATH):
        pix = QPixmap(LOGO_PATH).scaledToHeight(
            max_height, Qt.TransformationMode.SmoothTransformation
        )
        label.setPixmap(pix)
    else:
        label.setText('XPLORE')
        label.setObjectName('title')
    label.setAlignment(Qt.AlignmentFlag.AlignCenter)
    return label


def make_section_label(text: str) -> QLabel:
    lbl = QLabel(text.upper())
    lbl.setObjectName('section')
    return lbl


# ══════════════════════════════════════════════════════════════════════════════
# ANIMATIONS
# ══════════════════════════════════════════════════════════════════════════════

class PulsingDot(QLabel):
    """Point coloré qui pulse en opacité — utilisé dans les badges de mode."""

    def __init__(self, color: str, size: int = 14):
        super().__init__('●')
        self.setStyleSheet(f'color: {color}; font-size: {size}px;')

        self._eff = QGraphicsOpacityEffect(self)
        self._eff.setOpacity(1.0)
        self.setGraphicsEffect(self._eff)

        self._anim = QPropertyAnimation(self._eff, b'opacity')
        self._anim.setDuration(1400)
        self._anim.setKeyValueAt(0.0, 1.0)
        self._anim.setKeyValueAt(0.5, 0.25)
        self._anim.setKeyValueAt(1.0, 1.0)
        self._anim.setLoopCount(-1)
        self._anim.setEasingCurve(QEasingCurve.Type.InOutSine)
        self._anim.start()

    def set_color(self, color: str, size: int = 14):
        self.setStyleSheet(f'color: {color}; font-size: {size}px;')


def fade_in(widget: QWidget, duration: int = 350):
    """Anime l'opacité d'un widget de 0 à 1 — pour transitions de pages."""
    eff = QGraphicsOpacityEffect(widget)
    widget.setGraphicsEffect(eff)
    anim = QPropertyAnimation(eff, b'opacity')
    anim.setDuration(duration)
    anim.setStartValue(0.0)
    anim.setEndValue(1.0)
    anim.setEasingCurve(QEasingCurve.Type.OutCubic)
    anim.start()
    widget._fade_anim = anim  # garder une réf pour ne pas être collecté


def make_breadcrumb(*parts: str) -> QWidget:
    """Crée une barre de fil d'ariane : XPLORE › TÉLÉOP › RACE"""
    w = QWidget()
    lay = QHBoxLayout(w)
    lay.setContentsMargins(0, 0, 0, 0)
    lay.setSpacing(10)
    lay.addStretch(1)
    for i, part in enumerate(parts):
        lbl = QLabel(part.upper())
        lbl.setObjectName('breadcrumb_active' if i == len(parts) - 1 else 'breadcrumb')
        lay.addWidget(lbl)
        if i < len(parts) - 1:
            sep = QLabel('›')
            sep.setStyleSheet(f'color: {TEXT_MUTED}; font-size: 14px;')
            lay.addWidget(sep)
    lay.addStretch(1)
    return w


# ══════════════════════════════════════════════════════════════════════════════
# PAGE : MENU PRINCIPAL
# ══════════════════════════════════════════════════════════════════════════════

class MenuPage(QWidget):
    def __init__(self, on_autonomous, on_teleop, on_quit):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(80, 60, 80, 60)
        layout.setSpacing(28)

        layout.addStretch(2)

        # Logo
        layout.addWidget(make_logo(150), alignment=Qt.AlignmentFlag.AlignCenter)

        # Sous-titre
        subtitle = QLabel('ROVER  CONTROL')
        subtitle.setObjectName('subtitle')
        subtitle.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(subtitle)

        layout.addSpacing(40)

        # Boutons sobres par défaut, rouge uniquement au hover
        for text, slot, obj_name in [
            ('AUTONOME', on_autonomous, 'menu_btn'),
            ('TÉLÉOPÉRATION', on_teleop, 'menu_btn'),
            ('QUITTER', on_quit, 'menu_btn_quit'),
        ]:
            btn = QPushButton(text)
            btn.setObjectName(obj_name)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.clicked.connect(slot)
            layout.addWidget(btn, alignment=Qt.AlignmentFlag.AlignCenter)

        layout.addStretch(3)


# ══════════════════════════════════════════════════════════════════════════════
# PAGE : SOUS-MENU TÉLÉOP
# ══════════════════════════════════════════════════════════════════════════════

class TeleopMenuPage(QWidget):
    def __init__(self, on_race, on_arm, on_back):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(80, 60, 80, 60)
        layout.setSpacing(28)

        layout.addStretch(2)

        title = QLabel('TÉLÉOPÉRATION')
        title.setObjectName('title')
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        subtitle = QLabel('CHOIX DU SOUS-MODE')
        subtitle.setObjectName('subtitle')
        subtitle.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(subtitle)

        layout.addSpacing(40)

        for text, slot, obj_name in [
            ('RACE  ·  FPV', on_race, 'menu_btn'),
            ('BRAS  ·  RAMASSAGE', on_arm, 'menu_btn'),
        ]:
            btn = QPushButton(text)
            btn.setObjectName(obj_name)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.clicked.connect(slot)
            layout.addWidget(btn, alignment=Qt.AlignmentFlag.AlignCenter)

        layout.addSpacing(28)

        back = QPushButton('← MENU PRINCIPAL')
        back.setObjectName('ghost_btn')
        back.setCursor(Qt.CursorShape.PointingHandCursor)
        back.clicked.connect(on_back)
        layout.addWidget(back, alignment=Qt.AlignmentFlag.AlignCenter)

        layout.addStretch(3)


# ══════════════════════════════════════════════════════════════════════════════
# WIDGET : INDICATEUR DE TOUCHE
# ══════════════════════════════════════════════════════════════════════════════

class KeyIndicator(QLabel):
    """Carré qui s'allume quand la touche est pressée."""

    def __init__(self, key: str):
        super().__init__(key)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setFixedSize(60, 60)
        self.setActive(False)

    def setActive(self, active: bool):
        if active:
            style = (
                f'background-color: {PRIMARY};'
                f'color: {TEXT};'
                f'border: 1px solid {PRIMARY};'
                f'border-radius: 12px;'
                f'font-size: 17px;'
                f'font-weight: 800;'
                f'letter-spacing: 1px;'
            )
        else:
            style = (
                f'background-color: {BG_ELEVATED};'
                f'color: {TEXT_DIM};'
                f'border: 1px solid {BORDER};'
                f'border-radius: 12px;'
                f'font-size: 17px;'
                f'font-weight: 700;'
                f'letter-spacing: 1px;'
            )
        self.setStyleSheet(style)


# ══════════════════════════════════════════════════════════════════════════════
# PAGE : MODE RACE
# ══════════════════════════════════════════════════════════════════════════════

AXIS_TIMEOUT = 0.20

SPEED_LEVELS = {
    Qt.Key.Key_8: (0.33, '8'),
    Qt.Key.Key_9: (0.66, '9'),
    Qt.Key.Key_0: (1.0,  '0'),
}

# Mapping clavier → (linear, angular)
MOVEMENT_KEYS = {
    Qt.Key.Key_W: ( 1.0,  0.0),
    Qt.Key.Key_S: (-1.0,  0.0),
    Qt.Key.Key_A: ( 0.0,  1.0),
    Qt.Key.Key_D: ( 0.0, -1.0),
    Qt.Key.Key_Q: ( 1.0,  1.0),
    Qt.Key.Key_E: ( 1.0, -1.0),
    Qt.Key.Key_Y: (-1.0,  1.0),
    Qt.Key.Key_X: (-1.0, -1.0),
}


class RacePage(QWidget):
    def __init__(self, bridge: RosBridge, on_back):
        super().__init__()
        self.bridge = bridge
        self.on_back = on_back

        self.linear = 0.0
        self.angular = 0.0
        self.last_linear_t = 0.0
        self.last_angular_t = 0.0
        self.speed = 1.0
        self.active_keys: set = set()

        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        # ── Layout principal : commandes (gauche) + caméra (droite) ──
        root = QHBoxLayout(self)
        root.setContentsMargins(32, 28, 32, 28)
        root.setSpacing(28)

        root.addWidget(self._build_left_panel(), 0)
        root.addWidget(self._build_right_panel(), 1)

        # ── Connexion vidéo ──
        bridge.frame_ready.connect(self._on_frame)

        # ── Timer d'envoi cmd_vel à 10Hz ──
        self.cmd_timer = QTimer(self)
        self.cmd_timer.timeout.connect(self._send_cmd)
        self.cmd_timer.start(100)

    # ── Construction UI ──

    def _build_left_panel(self) -> QWidget:
        card = QFrame()
        card.setObjectName('card')
        card.setFixedWidth(380)

        layout = QVBoxLayout(card)
        layout.setContentsMargins(28, 28, 28, 28)
        layout.setSpacing(20)

        # En-tête : badge mode (avec dot animé)
        header = QHBoxLayout()
        header.setSpacing(8)
        badge_wrap = QFrame()
        badge_wrap.setStyleSheet(
            f'background-color: {PRIMARY}; border-radius: 14px; padding: 4px 14px;'
        )
        badge_lay = QHBoxLayout(badge_wrap)
        badge_lay.setContentsMargins(8, 4, 14, 4)
        badge_lay.setSpacing(8)
        badge_lay.addWidget(PulsingDot(TEXT, size=10))
        badge_text = QLabel('RACE')
        badge_text.setStyleSheet(
            f'color: {TEXT}; font-size: 11px; font-weight: 800; letter-spacing: 4px;'
        )
        badge_lay.addWidget(badge_text)
        header.addWidget(badge_wrap)
        header.addStretch(1)
        layout.addLayout(header)

        layout.addSpacing(4)

        # Vitesse
        layout.addWidget(make_section_label('Vitesse'))

        speed_row = QHBoxLayout()
        speed_row.setSpacing(10)
        self.speed_indicators = {}
        for key, (val, label) in SPEED_LEVELS.items():
            ind = KeyIndicator(label)
            self.speed_indicators[key] = ind
            speed_row.addWidget(ind)
        speed_row.addStretch(1)
        layout.addLayout(speed_row)

        self.speed_label = QLabel('100 %')
        self.speed_label.setStyleSheet(
            f'color: {ACCENT}; font-size: 24px; font-weight: 700; '
            f'letter-spacing: 2px; padding-top: 6px;'
        )
        layout.addWidget(self.speed_label)

        layout.addSpacing(16)

        # Mouvement (W/A/S/D + diagonales)
        layout.addWidget(make_section_label('Mouvement'))

        grid = QGridLayout()
        grid.setSpacing(8)
        grid.setContentsMargins(0, 0, 0, 0)
        self.move_indicators = {}

        layout_keys = [
            (Qt.Key.Key_Q, 0, 0, 'Q'),
            (Qt.Key.Key_W, 0, 1, 'W'),
            (Qt.Key.Key_E, 0, 2, 'E'),
            (Qt.Key.Key_A, 1, 0, 'A'),
            (Qt.Key.Key_S, 1, 1, 'S'),
            (Qt.Key.Key_D, 1, 2, 'D'),
            (Qt.Key.Key_Y, 2, 0, 'Y'),
            (Qt.Key.Key_X, 2, 2, 'X'),
        ]
        for qkey, r, c, label in layout_keys:
            ind = KeyIndicator(label)
            self.move_indicators[qkey] = ind
            grid.addWidget(ind, r, c)

        grid_holder = QHBoxLayout()
        grid_holder.addLayout(grid)
        grid_holder.addStretch(1)
        layout.addLayout(grid_holder)

        layout.addSpacing(16)

        # Aide — chaque raccourci sur sa ligne avec key + description
        layout.addWidget(make_section_label('Raccourcis'))
        shortcuts = [
            ('ESPACE', 'Stop moteurs'),
            ('8 / 9 / 0', 'Vitesse'),
            ('M', 'Retour menu'),
        ]
        for key_text, desc in shortcuts:
            row = QHBoxLayout()
            row.setSpacing(12)
            k = QLabel(key_text)
            k.setStyleSheet(
                f'color: {TEXT}; font-size: 11px; font-weight: 700; '
                f'letter-spacing: 2px; min-width: 80px;'
            )
            d = QLabel(desc)
            d.setStyleSheet(f'color: {TEXT_MUTED}; font-size: 12px;')
            row.addWidget(k)
            row.addWidget(d)
            row.addStretch(1)
            layout.addLayout(row)

        layout.addStretch(1)

        # Bouton retour (souris)
        back = QPushButton('← MENU PRINCIPAL')
        back.setObjectName('ghost_btn')
        back.setCursor(Qt.CursorShape.PointingHandCursor)
        back.clicked.connect(self.on_back)
        layout.addWidget(back)

        return card

    def _build_right_panel(self) -> QWidget:
        card = QFrame()
        card.setObjectName('card')

        layout = QVBoxLayout(card)
        layout.setContentsMargins(28, 28, 28, 28)
        layout.setSpacing(18)

        header = QHBoxLayout()
        title = QLabel('FLUX CAMÉRA  ·  FPV')
        title.setObjectName('section')
        header.addWidget(title)
        header.addStretch(1)
        self.fps_label = QLabel('— FPS')
        self.fps_label.setObjectName('status_value')
        header.addWidget(self.fps_label)
        layout.addLayout(header)

        # Vidéo
        self.video = QLabel()
        self.video.setObjectName('video')
        self.video.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video.setMinimumSize(640, 480)
        self.video.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.video.setText('En attente du flux caméra…')
        layout.addWidget(self.video, 1)

        # FPS calc
        self._frame_times = []

        return card

    # ── Slots ──

    def _on_frame(self, qimg: QImage):
        if not self.isVisible():
            return
        scaled = qimg.scaled(
            self.video.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.video.setPixmap(QPixmap.fromImage(scaled))

        now = time.time()
        self._frame_times.append(now)
        self._frame_times = [t for t in self._frame_times if now - t < 1.0]
        self.fps_label.setText(f'{len(self._frame_times)} FPS')

    # ── Cycle ROS2 cmd_vel ──

    def _send_cmd(self):
        now = time.time()
        if now - self.last_linear_t > AXIS_TIMEOUT:
            self.linear = 0.0
        if now - self.last_angular_t > AXIS_TIMEOUT:
            self.angular = 0.0
        self.bridge.publish_cmd(self.linear * self.speed, self.angular * self.speed)

    # ── Clavier ──

    def keyPressEvent(self, event: QKeyEvent):
        if event.isAutoRepeat():
            return
        key = event.key()
        now = time.time()

        if key == Qt.Key.Key_M:
            self.on_back()
            return

        if key == Qt.Key.Key_Space:
            self.linear = 0.0
            self.angular = 0.0
            self.last_linear_t = 0.0
            self.last_angular_t = 0.0
            return

        if key in SPEED_LEVELS:
            speed_val, _ = SPEED_LEVELS[key]
            self.speed = speed_val
            for k, ind in self.speed_indicators.items():
                ind.setActive(k == key)
            self.speed_label.setText(f'{int(speed_val*100)} %')
            return

        if key in MOVEMENT_KEYS:
            lin, ang = MOVEMENT_KEYS[key]
            if lin != 0.0:
                self.linear = lin
                self.last_linear_t = now
            if ang != 0.0:
                self.angular = ang
                self.last_angular_t = now
            self.active_keys.add(key)
            self._refresh_keys()

    def keyReleaseEvent(self, event: QKeyEvent):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in MOVEMENT_KEYS:
            self.active_keys.discard(key)
            self._refresh_keys()

    def _refresh_keys(self):
        for k, ind in self.move_indicators.items():
            ind.setActive(k in self.active_keys)

    # ── Cycle de vie ──

    def showEvent(self, event):
        super().showEvent(event)
        # Init affichage vitesse selon valeur actuelle
        for k, (val, _) in SPEED_LEVELS.items():
            self.speed_indicators[k].setActive(abs(val - self.speed) < 1e-3)
        self.setFocus()


# ══════════════════════════════════════════════════════════════════════════════
# PAGE : MODE AUTONOME
# ══════════════════════════════════════════════════════════════════════════════

class AutonomousPage(QWidget):
    def __init__(self, bridge: RosBridge, on_back):
        super().__init__()
        self.bridge = bridge
        self.on_back = on_back

        self._aruco_log = []  # historique des détections

        root = QVBoxLayout(self)
        root.setContentsMargins(40, 40, 40, 40)
        root.setSpacing(24)

        # En-tête : badge auto (cyan, dot pulse)
        header = QHBoxLayout()
        badge_wrap = QFrame()
        badge_wrap.setStyleSheet(
            f'background-color: {ACCENT}; border-radius: 14px;'
        )
        badge_lay = QHBoxLayout(badge_wrap)
        badge_lay.setContentsMargins(12, 4, 16, 4)
        badge_lay.setSpacing(8)
        badge_lay.addWidget(PulsingDot(BG_DEEP, size=10))
        badge_text = QLabel('AUTONOME')
        badge_text.setStyleSheet(
            f'color: {BG_DEEP}; font-size: 11px; font-weight: 800; letter-spacing: 4px;'
        )
        badge_lay.addWidget(badge_text)
        header.addWidget(badge_wrap)
        header.addStretch(1)
        back = QPushButton('← MENU PRINCIPAL')
        back.setObjectName('ghost_btn')
        back.setCursor(Qt.CursorShape.PointingHandCursor)
        back.clicked.connect(on_back)
        header.addWidget(back)
        root.addLayout(header)

        # Carte état rover
        state_card = QFrame()
        state_card.setObjectName('card')
        state_layout = QVBoxLayout(state_card)
        state_layout.setContentsMargins(28, 24, 28, 24)
        state_layout.setSpacing(10)

        state_layout.addWidget(make_section_label('État rover'))
        self.status_label = QLabel('En attente du status…')
        self.status_label.setObjectName('status_value')
        state_layout.addWidget(self.status_label)
        root.addWidget(state_card)

        # Carte détection ArUco — en grand
        aruco_card = QFrame()
        aruco_card.setObjectName('card')
        aruco_layout = QVBoxLayout(aruco_card)
        aruco_layout.setContentsMargins(32, 28, 32, 28)
        aruco_layout.setSpacing(18)

        aruco_layout.addWidget(make_section_label('Détection ArUco'))

        self.aruco_status = QLabel('○  AUCUN MARKER VISIBLE')
        self.aruco_status.setStyleSheet(
            f'color: {TEXT_MUTED}; font-size: 22px; '
            f'font-weight: 600; letter-spacing: 4px;'
        )
        # Effet d'opacité pour pulse quand détecté
        self._aruco_eff = QGraphicsOpacityEffect(self.aruco_status)
        self._aruco_eff.setOpacity(1.0)
        self.aruco_status.setGraphicsEffect(self._aruco_eff)
        self._aruco_anim = QPropertyAnimation(self._aruco_eff, b'opacity')
        self._aruco_anim.setDuration(1200)
        self._aruco_anim.setKeyValueAt(0.0, 1.0)
        self._aruco_anim.setKeyValueAt(0.5, 0.45)
        self._aruco_anim.setKeyValueAt(1.0, 1.0)
        self._aruco_anim.setLoopCount(-1)
        self._aruco_anim.setEasingCurve(QEasingCurve.Type.InOutSine)
        aruco_layout.addWidget(self.aruco_status)

        info_row = QHBoxLayout()
        info_row.setSpacing(36)
        self.aruco_id = QLabel('ID  —')
        self.aruco_pos = QLabel('POSITION  —')
        self.aruco_area = QLabel('AIRE  —')
        for w in (self.aruco_id, self.aruco_pos, self.aruco_area):
            w.setStyleSheet(
                f'color: {TEXT}; font-size: 13px; font-weight: 600; letter-spacing: 2px;'
            )
            info_row.addWidget(w)
        info_row.addStretch(1)
        aruco_layout.addLayout(info_row)

        aruco_layout.addSpacing(12)
        aruco_layout.addWidget(make_section_label('Historique'))

        self.aruco_history = QLabel('—')
        self.aruco_history.setObjectName('aruco_log')
        self.aruco_history.setAlignment(
            Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft
        )
        self.aruco_history.setMinimumHeight(180)
        self.aruco_history.setWordWrap(True)
        aruco_layout.addWidget(self.aruco_history, 1)

        root.addWidget(aruco_card, 1)

        # Connexions
        bridge.aruco_update.connect(self._on_aruco)
        bridge.status_update.connect(self._on_status)

        self._last_seen = False

    def _on_aruco(self, detected: bool, marker_id: int, cx: float, cy: float, area: float):
        if detected:
            self.aruco_status.setText(f'●  MARKER  ID {marker_id}  DÉTECTÉ')
            self.aruco_status.setStyleSheet(
                f'color: {ACCENT_2}; font-size: 22px; '
                f'font-weight: 800; letter-spacing: 4px;'
            )
            if self._aruco_anim.state() != QPropertyAnimation.State.Running:
                self._aruco_anim.start()
            self.aruco_id.setText(f'ID  {marker_id}')
            self.aruco_pos.setText(f'POSITION  ({cx:.0f}, {cy:.0f})')
            self.aruco_area.setText(f'AIRE  {area:.0f} px²')

            if not self._last_seen:
                ts = time.strftime('%H:%M:%S')
                self._aruco_log.insert(0, f'[{ts}]  ✓  Détecté ID={marker_id}  pos=({cx:.0f},{cy:.0f})')
                self._aruco_log = self._aruco_log[:12]
                self.aruco_history.setText('\n'.join(self._aruco_log))
            self._last_seen = True
        else:
            self.aruco_status.setText('○  AUCUN MARKER VISIBLE')
            self.aruco_status.setStyleSheet(
                f'color: {TEXT_MUTED}; font-size: 22px; '
                f'font-weight: 600; letter-spacing: 4px;'
            )
            if self._aruco_anim.state() == QPropertyAnimation.State.Running:
                self._aruco_anim.stop()
                self._aruco_eff.setOpacity(1.0)
            self.aruco_id.setText('ID  —')
            self.aruco_pos.setText('POSITION  —')
            self.aruco_area.setText('AIRE  —')

            if self._last_seen:
                ts = time.strftime('%H:%M:%S')
                self._aruco_log.insert(0, f'[{ts}]  ×  Marker perdu')
                self._aruco_log = self._aruco_log[:12]
                self.aruco_history.setText('\n'.join(self._aruco_log))
            self._last_seen = False

    def _on_status(self, status: str):
        self.status_label.setText(status)


# ══════════════════════════════════════════════════════════════════════════════
# PAGE : MODE BRAS
# ══════════════════════════════════════════════════════════════════════════════

class ArmPage(QWidget):
    def __init__(self, on_back):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(60, 60, 60, 60)
        layout.setSpacing(24)

        header = QHBoxLayout()
        badge = QLabel('BRAS')
        badge.setObjectName('mode_badge')
        header.addWidget(badge)
        header.addStretch(1)
        back = QPushButton('← MENU PRINCIPAL')
        back.setObjectName('ghost_btn')
        back.setCursor(Qt.CursorShape.PointingHandCursor)
        back.clicked.connect(on_back)
        header.addWidget(back)
        layout.addLayout(header)

        layout.addStretch(1)

        msg = QLabel('Module bras — à implémenter')
        msg.setObjectName('subtitle')
        msg.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(msg)

        layout.addStretch(2)


# ══════════════════════════════════════════════════════════════════════════════
# FENÊTRE PRINCIPALE
# ══════════════════════════════════════════════════════════════════════════════

class MainWindow(QMainWindow):
    def __init__(self, bridge: RosBridge):
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle('XPlore — Rover Control')
        self.resize(1320, 840)

        central = QWidget()
        central.setObjectName('root')
        self.setCentralWidget(central)

        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Stack des pages
        self.stack = QStackedWidget()
        layout.addWidget(self.stack, 1)

        # Footer
        footer = self._build_footer()
        layout.addWidget(footer)

        # Pages
        self.page_menu = MenuPage(
            on_autonomous=self.show_autonomous,
            on_teleop=self.show_teleop_menu,
            on_quit=self.close,
        )
        self.page_teleop = TeleopMenuPage(
            on_race=self.show_race,
            on_arm=self.show_arm,
            on_back=self.show_menu,
        )
        self.page_race = RacePage(bridge, on_back=self.show_menu)
        self.page_autonomous = AutonomousPage(bridge, on_back=self.show_menu)
        self.page_arm = ArmPage(on_back=self.show_menu)

        for page in (self.page_menu, self.page_teleop, self.page_race,
                     self.page_autonomous, self.page_arm):
            self.stack.addWidget(page)

        self.show_menu()

    def _build_footer(self) -> QWidget:
        f = QFrame()
        f.setObjectName('footer')
        f.setFixedHeight(40)
        lay = QHBoxLayout(f)
        lay.setContentsMargins(28, 0, 28, 0)
        lay.setSpacing(14)

        # Logo XPlore en mini
        if os.path.exists(LOGO_PATH):
            mini = QLabel()
            mini.setPixmap(QPixmap(LOGO_PATH).scaledToHeight(
                18, Qt.TransformationMode.SmoothTransformation))
            lay.addWidget(mini)
        else:
            lay.addWidget(QLabel('XPLORE'))

        sep = QLabel('·')
        sep.setStyleSheet(f'color: {TEXT_MUTED};')
        lay.addWidget(sep)

        version = QLabel('ROVER CONTROL  v0.1')
        version.setObjectName('footer_label')
        lay.addWidget(version)

        lay.addStretch(1)

        mode_lbl = QLabel('MODE')
        mode_lbl.setObjectName('footer_label')
        lay.addWidget(mode_lbl)
        self.footer_dot = PulsingDot(ACCENT, size=10)
        lay.addWidget(self.footer_dot)
        self.footer_mode = QLabel('IDLE')
        self.footer_mode.setObjectName('footer_value')
        lay.addWidget(self.footer_mode)

        return f

    # ── Navigation ──

    def _set_mode(self, mode: str):
        self.bridge.publish_mode(mode)
        self.footer_mode.setText(mode.upper())
        # Couleur du dot footer selon le mode
        color_map = {
            'idle': TEXT_MUTED,
            'race': PRIMARY,
            'autonomous': ACCENT_2,
            'arm': ACCENT,
        }
        self.footer_dot.set_color(color_map.get(mode, ACCENT), size=10)

    def show_menu(self):
        self._set_mode('idle')
        self.stack.setCurrentWidget(self.page_menu)

    def show_teleop_menu(self):
        self.stack.setCurrentWidget(self.page_teleop)

    def show_race(self):
        self._set_mode('race')
        self.stack.setCurrentWidget(self.page_race)
        self.page_race.setFocus()

    def show_autonomous(self):
        self._set_mode('autonomous')
        self.stack.setCurrentWidget(self.page_autonomous)

    def show_arm(self):
        self._set_mode('arm')
        self.stack.setCurrentWidget(self.page_arm)

    def closeEvent(self, event):
        self.bridge.publish_mode('idle')
        super().closeEvent(event)


# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    app.setStyleSheet(GLOBAL_QSS)

    bridge = RosBridge()

    # Spin ROS2 dans un thread séparé
    spin_thread = threading.Thread(target=rclpy.spin, args=(bridge.node,), daemon=True)
    spin_thread.start()

    window = MainWindow(bridge)
    window.show()

    exit_code = app.exec()

    bridge.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
