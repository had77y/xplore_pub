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

from PySide6.QtCore import Qt, QObject, Signal, QTimer, QSize
from PySide6.QtGui import QImage, QPixmap, QFont, QFontDatabase, QKeyEvent
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QStackedWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QFrame, QSizePolicy, QSpacerItem,
)


# ══════════════════════════════════════════════════════════════════════════════
# PALETTE XPLORE
# ══════════════════════════════════════════════════════════════════════════════

PRIMARY    = '#E53935'  # rouge identité
BACKGROUND = '#0D0D0D'  # fond
SURFACE    = '#1A1A1A'  # cartes / panneaux
TEXT       = '#FFFFFF'  # texte
TEXT_DIM   = '#8A8A8A'  # texte secondaire
ACCENT     = '#00AEEF'  # cyan accent
DIVIDER    = '#2A2A2A'

LOGO_PATH = os.path.join(os.path.dirname(__file__), 'xplore_logo.jpg')


# ══════════════════════════════════════════════════════════════════════════════
# QSS — STYLE GLOBAL
# ══════════════════════════════════════════════════════════════════════════════

GLOBAL_QSS = f"""
* {{
    color: {TEXT};
    font-family: "Helvetica Neue", "Inter", "Segoe UI", sans-serif;
}}

QMainWindow, QWidget#root {{
    background-color: {BACKGROUND};
}}

QLabel#title {{
    font-size: 32px;
    font-weight: 300;
    letter-spacing: 4px;
    color: {TEXT};
}}

QLabel#subtitle {{
    font-size: 14px;
    font-weight: 400;
    letter-spacing: 2px;
    color: {TEXT_DIM};
}}

QLabel#section {{
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 3px;
    color: {TEXT_DIM};
}}

QLabel#status_label {{
    font-size: 13px;
    color: {TEXT_DIM};
    letter-spacing: 1px;
}}

QLabel#status_value {{
    font-size: 13px;
    color: {ACCENT};
    font-weight: 600;
}}

QPushButton#menu_btn {{
    background-color: {SURFACE};
    border: 1px solid {DIVIDER};
    border-radius: 8px;
    padding: 24px 32px;
    font-size: 18px;
    font-weight: 500;
    letter-spacing: 2px;
    min-width: 280px;
    min-height: 80px;
}}
QPushButton#menu_btn:hover {{
    border: 1px solid {PRIMARY};
    background-color: #1F1F1F;
}}
QPushButton#menu_btn:pressed {{
    background-color: {PRIMARY};
    border-color: {PRIMARY};
}}

QPushButton#primary_btn {{
    background-color: {PRIMARY};
    border: none;
    border-radius: 6px;
    padding: 14px 28px;
    font-size: 14px;
    font-weight: 600;
    letter-spacing: 2px;
}}
QPushButton#primary_btn:hover {{
    background-color: #FF4A44;
}}

QPushButton#ghost_btn {{
    background-color: transparent;
    border: 1px solid {DIVIDER};
    border-radius: 6px;
    padding: 12px 24px;
    font-size: 13px;
    font-weight: 500;
    letter-spacing: 2px;
    color: {TEXT_DIM};
}}
QPushButton#ghost_btn:hover {{
    border-color: {TEXT};
    color: {TEXT};
}}

QFrame#card {{
    background-color: {SURFACE};
    border-radius: 12px;
    border: 1px solid {DIVIDER};
}}

QFrame#camera_frame {{
    background-color: #000000;
    border-radius: 8px;
    border: 1px solid {DIVIDER};
}}

QLabel#video {{
    background-color: #000000;
    border-radius: 8px;
}}

QLabel#mode_badge {{
    background-color: {PRIMARY};
    color: {TEXT};
    padding: 6px 14px;
    border-radius: 12px;
    font-size: 11px;
    font-weight: 700;
    letter-spacing: 3px;
}}

QLabel#aruco_log {{
    background-color: #050505;
    border: 1px solid {DIVIDER};
    border-radius: 6px;
    padding: 12px;
    font-family: "Menlo", "Consolas", monospace;
    font-size: 12px;
    color: {ACCENT};
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
# PAGE : MENU PRINCIPAL
# ══════════════════════════════════════════════════════════════════════════════

class MenuPage(QWidget):
    def __init__(self, on_autonomous, on_teleop, on_quit):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(60, 60, 60, 60)
        layout.setSpacing(40)

        layout.addStretch(1)
        layout.addWidget(make_logo(140), alignment=Qt.AlignmentFlag.AlignCenter)

        subtitle = QLabel('ROVER CONTROL')
        subtitle.setObjectName('subtitle')
        subtitle.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(subtitle)

        layout.addSpacing(20)

        for text, slot in [
            ('AUTONOME', on_autonomous),
            ('TÉLÉOPÉRATION', on_teleop),
            ('QUITTER', on_quit),
        ]:
            btn = QPushButton(text)
            btn.setObjectName('menu_btn')
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.clicked.connect(slot)
            layout.addWidget(btn, alignment=Qt.AlignmentFlag.AlignCenter)

        layout.addStretch(2)


# ══════════════════════════════════════════════════════════════════════════════
# PAGE : SOUS-MENU TÉLÉOP
# ══════════════════════════════════════════════════════════════════════════════

class TeleopMenuPage(QWidget):
    def __init__(self, on_race, on_arm, on_back):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(60, 60, 60, 60)
        layout.setSpacing(30)

        layout.addStretch(1)

        title = QLabel('TÉLÉOPÉRATION')
        title.setObjectName('title')
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        subtitle = QLabel('Choix du sous-mode')
        subtitle.setObjectName('subtitle')
        subtitle.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(subtitle)

        layout.addSpacing(20)

        for text, slot in [
            ('RACE  (FPV)', on_race),
            ('BRAS  (RAMASSAGE)', on_arm),
        ]:
            btn = QPushButton(text)
            btn.setObjectName('menu_btn')
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.clicked.connect(slot)
            layout.addWidget(btn, alignment=Qt.AlignmentFlag.AlignCenter)

        layout.addSpacing(20)

        back = QPushButton('← MENU PRINCIPAL')
        back.setObjectName('ghost_btn')
        back.setCursor(Qt.CursorShape.PointingHandCursor)
        back.clicked.connect(on_back)
        layout.addWidget(back, alignment=Qt.AlignmentFlag.AlignCenter)

        layout.addStretch(2)


# ══════════════════════════════════════════════════════════════════════════════
# WIDGET : INDICATEUR DE TOUCHE
# ══════════════════════════════════════════════════════════════════════════════

class KeyIndicator(QLabel):
    """Carré qui s'allume en rouge quand la touche est pressée."""

    def __init__(self, key: str):
        super().__init__(key)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setFixedSize(56, 56)
        self.setActive(False)

    def setActive(self, active: bool):
        if active:
            style = (
                f'background-color: {PRIMARY};'
                f'color: {TEXT};'
                f'border: 1px solid {PRIMARY};'
                f'border-radius: 8px;'
                f'font-size: 16px;'
                f'font-weight: 700;'
            )
        else:
            style = (
                f'background-color: {SURFACE};'
                f'color: {TEXT_DIM};'
                f'border: 1px solid {DIVIDER};'
                f'border-radius: 8px;'
                f'font-size: 16px;'
                f'font-weight: 600;'
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
        root.setContentsMargins(24, 24, 24, 24)
        root.setSpacing(24)

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
        card.setFixedWidth(360)

        layout = QVBoxLayout(card)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(16)

        # En-tête : badge mode
        header = QHBoxLayout()
        badge = QLabel('RACE')
        badge.setObjectName('mode_badge')
        header.addWidget(badge)
        header.addStretch(1)
        layout.addLayout(header)

        layout.addSpacing(8)

        # Vitesse
        layout.addWidget(make_section_label('Vitesse'))

        speed_row = QHBoxLayout()
        speed_row.setSpacing(8)
        self.speed_indicators = {}
        for key, (val, label) in SPEED_LEVELS.items():
            ind = KeyIndicator(label)
            self.speed_indicators[key] = ind
            speed_row.addWidget(ind)
        speed_row.addStretch(1)
        layout.addLayout(speed_row)

        self.speed_label = QLabel('Niveau : 0  (100%)')
        self.speed_label.setObjectName('status_label')
        layout.addWidget(self.speed_label)

        layout.addSpacing(12)

        # Mouvement (W/A/S/D + diagonales)
        layout.addWidget(make_section_label('Mouvement'))

        grid = QGridLayout()
        grid.setSpacing(6)
        grid.setContentsMargins(0, 0, 0, 0)
        self.move_indicators = {}

        # Layout en croix avec diagonales
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

        layout.addSpacing(12)

        # Aide
        layout.addWidget(make_section_label('Raccourcis'))
        help_text = QLabel(
            'Espace  →  Stop\n'
            '8 / 9 / 0  →  Vitesse\n'
            'M  →  Retour menu'
        )
        help_text.setObjectName('status_label')
        layout.addWidget(help_text)

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
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(16)

        header = QHBoxLayout()
        title = QLabel('FLUX CAMÉRA')
        title.setObjectName('section')
        header.addWidget(title)
        header.addStretch(1)
        self.fps_label = QLabel('— FPS')
        self.fps_label.setObjectName('status_label')
        header.addWidget(self.fps_label)
        layout.addLayout(header)

        # Vidéo
        self.video = QLabel()
        self.video.setObjectName('video')
        self.video.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video.setMinimumSize(640, 480)
        self.video.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.video.setText('En attente du flux caméra…')
        self.video.setStyleSheet(
            f'background-color: #000; color: {TEXT_DIM}; '
            f'border: 1px solid {DIVIDER}; border-radius: 8px; font-size: 14px;'
        )
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
            self.speed_label.setText(
                f'Niveau : {SPEED_LEVELS[key][1]}  ({int(speed_val*100)}%)'
            )
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

        # En-tête
        header = QHBoxLayout()
        badge = QLabel('AUTONOME')
        badge.setObjectName('mode_badge')
        header.addWidget(badge)
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
        state_layout.setContentsMargins(24, 24, 24, 24)
        state_layout.setSpacing(12)

        state_layout.addWidget(make_section_label('État rover'))
        self.status_label = QLabel('En attente de status...')
        self.status_label.setObjectName('status_value')
        state_layout.addWidget(self.status_label)
        root.addWidget(state_card)

        # Carte détection ArUco — en grand
        aruco_card = QFrame()
        aruco_card.setObjectName('card')
        aruco_layout = QVBoxLayout(aruco_card)
        aruco_layout.setContentsMargins(24, 24, 24, 24)
        aruco_layout.setSpacing(16)

        aruco_layout.addWidget(make_section_label('Détection ArUco'))

        self.aruco_status = QLabel('● AUCUN MARKER VISIBLE')
        self.aruco_status.setStyleSheet(
            f'color: {TEXT_DIM}; font-size: 18px; '
            f'font-weight: 600; letter-spacing: 2px;'
        )
        aruco_layout.addWidget(self.aruco_status)

        info_row = QHBoxLayout()
        self.aruco_id = QLabel('ID  —')
        self.aruco_pos = QLabel('POS  —')
        self.aruco_area = QLabel('AIRE  —')
        for w in (self.aruco_id, self.aruco_pos, self.aruco_area):
            w.setStyleSheet(f'color: {TEXT}; font-size: 14px; font-weight: 500;')
            info_row.addWidget(w)
            info_row.addSpacing(20)
        info_row.addStretch(1)
        aruco_layout.addLayout(info_row)

        aruco_layout.addSpacing(8)
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
            self.aruco_status.setText(f'● MARKER ID {marker_id} DÉTECTÉ')
            self.aruco_status.setStyleSheet(
                f'color: {PRIMARY}; font-size: 18px; '
                f'font-weight: 700; letter-spacing: 2px;'
            )
            self.aruco_id.setText(f'ID  {marker_id}')
            self.aruco_pos.setText(f'POS  ({cx:.0f}, {cy:.0f})')
            self.aruco_area.setText(f'AIRE  {area:.0f} px²')

            if not self._last_seen:
                ts = time.strftime('%H:%M:%S')
                self._aruco_log.insert(0, f'[{ts}]  Détecté ID={marker_id}  pos=({cx:.0f},{cy:.0f})')
                self._aruco_log = self._aruco_log[:12]
                self.aruco_history.setText('\n'.join(self._aruco_log))
            self._last_seen = True
        else:
            self.aruco_status.setText('● AUCUN MARKER VISIBLE')
            self.aruco_status.setStyleSheet(
                f'color: {TEXT_DIM}; font-size: 18px; '
                f'font-weight: 600; letter-spacing: 2px;'
            )
            self.aruco_id.setText('ID  —')
            self.aruco_pos.setText('POS  —')
            self.aruco_area.setText('AIRE  —')

            if self._last_seen:
                ts = time.strftime('%H:%M:%S')
                self._aruco_log.insert(0, f'[{ts}]  Perdu')
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
        self.resize(1280, 800)

        central = QWidget()
        central.setObjectName('root')
        self.setCentralWidget(central)

        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.stack = QStackedWidget()
        layout.addWidget(self.stack)

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

    # ── Navigation ──

    def show_menu(self):
        self.bridge.publish_mode('idle')
        self.stack.setCurrentWidget(self.page_menu)

    def show_teleop_menu(self):
        self.stack.setCurrentWidget(self.page_teleop)

    def show_race(self):
        self.bridge.publish_mode('race')
        self.stack.setCurrentWidget(self.page_race)
        self.page_race.setFocus()

    def show_autonomous(self):
        self.bridge.publish_mode('autonomous')
        self.stack.setCurrentWidget(self.page_autonomous)

    def show_arm(self):
        self.bridge.publish_mode('arm')
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
