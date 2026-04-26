# ──────────────────────────────────────────────────────────────────────────────
# rover_gui.py — Interface graphique de contrôle du rover XPlore
#
# Fonctionnalités :
#   - Menu principal cliquable à la souris (Autonome / Téléop / Quitter)
#   - Mode race : flux caméra à droite, panneau commandes à gauche, clavier WASD
#   - Mode autonome : carte navigation (placeholder) | ArUco + caméra + capteurs
#
# Topics publiés :
#   /rover/mode     (std_msgs/String)        — autonomous / race / arm / idle
#   /rover/cmd_vel  (geometry_msgs/Twist)    — uniquement en mode race
#
# Topics écoutés :
#   /camera/image_compressed (sensor_msgs/CompressedImage)
#   /aruco_detected          (std_msgs/Float32MultiArray)
#   /rover_status            (std_msgs/String)
#
# Topics à venir (placeholders UI prêts) :
#   /distances               Float32MultiArray [fl, fc, fr, l, r] cm  — US sensors
#   /imu                     sensor_msgs/Imu                          — IMU
# ──────────────────────────────────────────────────────────────────────────────

import math
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
    Qt, QObject, Signal, QTimer, QSize, QRectF, QPointF,
    QPropertyAnimation, QEasingCurve,
)
from PySide6.QtGui import (
    QImage, QPixmap, QFont, QFontDatabase, QKeyEvent,
    QPainter, QColor, QPen, QBrush,
)
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QStackedWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QFrame, QSizePolicy, QSpacerItem,
    QGraphicsOpacityEffect,
)


# ══════════════════════════════════════════════════════════════════════════════
# PALETTE XPLORE
# ══════════════════════════════════════════════════════════════════════════════

BG_DEEP     = '#0C1427'
BG_SURFACE  = '#121E36'
BG_ELEVATED = '#182440'

BORDER       = '#1E2E4A'
BORDER_HOVER = '#2E4268'

PRIMARY     = '#E53935'
PRIMARY_HOV = '#FF5A55'
ACCENT      = '#00AEEF'
ACCENT_2    = '#5AE8B5'

TEXT        = '#F3F6F9'
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

QFrame#glow_card {{
    background-color: {BG_SURFACE};
    border-radius: 18px;
    border: none;
}}

QLabel#video {{
    background-color: #050B17;
    border-radius: 12px;
    border: 1px solid {BORDER};
    color: {TEXT_MUTED};
}}

QLabel#mode_badge {{
    background-color: {PRIMARY};
    color: {TEXT};
    padding: 8px 18px;
    border-radius: 14px;
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 4px;
}}

QLabel#aruco_log {{
    background-color: rgba(5, 11, 23, 200);
    border: 1px solid {BORDER};
    border-radius: 10px;
    padding: 14px;
    font-family: "JetBrains Mono", "Menlo", "Consolas", monospace;
    font-size: 11px;
    color: {ACCENT_2};
    letter-spacing: 1px;
}}
"""


# ══════════════════════════════════════════════════════════════════════════════
# ROS2 BRIDGE
# ══════════════════════════════════════════════════════════════════════════════

VIDEO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class RosBridge(QObject):
    frame_ready   = Signal(QImage)
    aruco_update  = Signal(bool, int, float, float, float)
    status_update = Signal(str)

    def __init__(self):
        super().__init__()
        self._node = _RosNode(self)

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
    def __init__(self, bridge: RosBridge):
        super().__init__('rover_gui')
        self._bridge = bridge

        self.pub_mode = self.create_publisher(String, '/rover/mode', 10)
        self.pub_cmd  = self.create_publisher(Twist,  '/rover/cmd_vel', 10)

        self.create_subscription(
            CompressedImage, '/camera/image_compressed',
            self._on_image, VIDEO_QOS,
        )
        self.create_subscription(
            Float32MultiArray, '/aruco_detected', self._on_aruco, 10,
        )
        self.create_subscription(
            String, '/rover_status', self._on_status, 10,
        )

        # Placeholders à brancher quand les nodes seront prêts :
        #   /distances  Float32MultiArray [fl, fc, fr, l, r]  → USSensorsCard
        #   /imu        sensor_msgs/Imu                       → IMUCard

    def publish_mode(self, mode: str):
        msg = String(); msg.data = mode
        self.pub_mode.publish(msg)

    def publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = float(linear)
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
        self._bridge.aruco_update.emit(
            msg.data[0] > 0.5,
            int(msg.data[1]), msg.data[2], msg.data[3], msg.data[4],
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
# WIDGET : CARTE ANIMÉE — bordure respirante
# ══════════════════════════════════════════════════════════════════════════════

class GlowCard(QFrame):
    """QFrame dont la bordure respire lentement.
    glow_color : couleur cible RGB au pic (défaut cyan ACCENT).
    phase_offset : décalage pour que les cartes ne pulsent jamais ensemble.
    """

    # Cyan ACCENT par défaut
    _CYAN = (0, 174, 239)

    def __init__(self, phase_offset: float = 0.0, glow_color: tuple = _CYAN):
        super().__init__()
        self.setObjectName('glow_card')
        self._phase = phase_offset
        self._gr, self._gg, self._gb = glow_color
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(40)  # 25 fps

    def _tick(self):
        self._phase = (self._phase + 0.063) % (2 * math.pi)  # cycle ~4 s
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        t = (math.sin(self._phase) + 1) / 2  # 0..1

        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setBrush(Qt.BrushStyle.NoBrush)
        rect = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)

        # Halo extérieur — couleur très transparente, seulement au pic
        if t > 0.05:
            pen = QPen(QColor(self._gr, self._gg, self._gb, int(t * 30)))
            pen.setWidthF(3.5)
            p.setPen(pen)
            p.drawRoundedRect(rect, 17.5, 17.5)

        # Bordure principale — interpolation BORDER → glow_color
        a = t * 0.5
        pen = QPen(QColor(
            int(0x1E + (self._gr - 0x1E) * a),
            int(0x2E + (self._gg - 0x2E) * a),
            int(0x4A + (self._gb - 0x4A) * a),
        ))
        pen.setWidthF(1.0)
        p.setPen(pen)
        p.drawRoundedRect(rect, 17.5, 17.5)

        p.end()


class GlowButton(QPushButton):
    """QPushButton dont la bordure respire indépendamment."""

    def __init__(self, text: str, phase_offset: float = 0.0,
                 glow_color: tuple = (200, 35, 35), border_radius: float = 16.0):
        super().__init__(text)
        self._phase = phase_offset
        self._gr, self._gg, self._gb = glow_color
        self._br = border_radius - 0.5
        timer = QTimer(self)
        timer.timeout.connect(self._tick)
        timer.start(40)

    def _tick(self):
        self._phase = (self._phase + 0.063) % (2 * math.pi)
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        t = (math.sin(self._phase) + 1) / 2

        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setBrush(Qt.BrushStyle.NoBrush)
        rect = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)

        if t > 0.05:
            pen = QPen(QColor(self._gr, self._gg, self._gb, int(t * 30)))
            pen.setWidthF(3.5)
            p.setPen(pen)
            p.drawRoundedRect(rect, self._br, self._br)

        a = t * 0.5
        pen = QPen(QColor(
            int(0x1E + (self._gr - 0x1E) * a),
            int(0x2E + (self._gg - 0x2E) * a),
            int(0x4A + (self._gb - 0x4A) * a),
        ))
        pen.setWidthF(1.0)
        p.setPen(pen)
        p.drawRoundedRect(rect, self._br, self._br)
        p.end()


# ══════════════════════════════════════════════════════════════════════════════
# ANIMATIONS
# ══════════════════════════════════════════════════════════════════════════════

class PulsingDot(QLabel):
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
    eff = QGraphicsOpacityEffect(widget)
    widget.setGraphicsEffect(eff)
    anim = QPropertyAnimation(eff, b'opacity')
    anim.setDuration(duration)
    anim.setStartValue(0.0)
    anim.setEndValue(1.0)
    anim.setEasingCurve(QEasingCurve.Type.OutCubic)
    anim.start()
    widget._fade_anim = anim


def make_breadcrumb(*parts: str) -> QWidget:
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

_RED_GLOW = (200, 35, 35)   # bordeaux — couleur glow pages menu


class MenuPage(QWidget):
    def __init__(self, on_autonomous, on_teleop, on_quit):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(80, 60, 80, 60)
        layout.setSpacing(28)

        layout.addStretch(2)
        layout.addWidget(make_logo(220), alignment=Qt.AlignmentFlag.AlignCenter)

        subtitle = QLabel('ROVER  CONTROL')
        subtitle.setObjectName('subtitle')
        subtitle.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(subtitle)

        layout.addSpacing(40)

        for i, (text, slot, obj_name) in enumerate([
            ('AUTONOME',      on_autonomous, 'menu_btn'),
            ('TÉLÉOPÉRATION', on_teleop,     'menu_btn'),
            ('QUITTER',       on_quit,       'menu_btn_quit'),
        ]):
            btn = GlowButton(text, phase_offset=i * 2 * math.pi / 3, glow_color=_RED_GLOW)
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
        self._on_back = on_back
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

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

        for i, (text, slot, obj_name) in enumerate([
            ('RACE  ·  FPV',      on_race, 'menu_btn'),
            ('BRAS  ·  RAMASSAGE', on_arm,  'menu_btn'),
        ]):
            btn = GlowButton(text, phase_offset=i * math.pi, glow_color=_RED_GLOW)
            btn.setObjectName(obj_name)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.clicked.connect(slot)
            layout.addWidget(btn, alignment=Qt.AlignmentFlag.AlignCenter)

        layout.addSpacing(28)

        back_btn = QPushButton('← MENU PRINCIPAL')
        back_btn.setObjectName('ghost_btn')
        back_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        back_btn.clicked.connect(on_back)
        layout.addWidget(back_btn, alignment=Qt.AlignmentFlag.AlignCenter)

        layout.addStretch(3)

    def keyPressEvent(self, event: QKeyEvent):
        if event.key() == Qt.Key.Key_M:
            self._on_back()
        else:
            super().keyPressEvent(event)

    def showEvent(self, event):
        super().showEvent(event)
        self.setFocus()


# ══════════════════════════════════════════════════════════════════════════════
# WIDGET : INDICATEUR DE TOUCHE
# ══════════════════════════════════════════════════════════════════════════════

class KeyIndicator(QLabel):
    def __init__(self, key: str):
        super().__init__(key)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setFixedSize(60, 60)
        self.setActive(False)

    def setActive(self, active: bool):
        if active:
            style = (
                f'background-color: {PRIMARY}; color: {TEXT};'
                f'border: 1px solid {PRIMARY}; border-radius: 12px;'
                f'font-size: 17px; font-weight: 800; letter-spacing: 1px;'
            )
        else:
            style = (
                f'background-color: {BG_ELEVATED}; color: {TEXT_DIM};'
                f'border: 1px solid {BORDER}; border-radius: 12px;'
                f'font-size: 17px; font-weight: 700; letter-spacing: 1px;'
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

        root = QHBoxLayout(self)
        root.setContentsMargins(32, 28, 32, 28)
        root.setSpacing(28)

        root.addWidget(self._build_left_panel(), 0)
        root.addWidget(self._build_right_panel(), 1)

        bridge.frame_ready.connect(self._on_frame)

        self.cmd_timer = QTimer(self)
        self.cmd_timer.timeout.connect(self._send_cmd)
        self.cmd_timer.start(100)

    def _build_left_panel(self) -> QWidget:
        card = GlowCard(phase_offset=0.0)
        card.setFixedWidth(380)

        layout = QVBoxLayout(card)
        layout.setContentsMargins(28, 28, 28, 28)
        layout.setSpacing(20)

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

        self.speed_label = QLabel('Niveau : 0 (100%)')
        self.speed_label.setStyleSheet(
            f'color: {TEXT_DIM}; font-size: 14px; font-weight: 500; '
            f'letter-spacing: 1px; padding-top: 4px;'
        )
        layout.addWidget(self.speed_label)

        layout.addSpacing(16)
        layout.addWidget(make_section_label('Mouvement'))

        grid = QGridLayout()
        grid.setSpacing(8)
        grid.setContentsMargins(0, 0, 0, 0)
        self.move_indicators = {}

        layout_keys = [
            (Qt.Key.Key_Q, 0, 0, 'Q'), (Qt.Key.Key_W, 0, 1, 'W'), (Qt.Key.Key_E, 0, 2, 'E'),
            (Qt.Key.Key_A, 1, 0, 'A'), (Qt.Key.Key_S, 1, 1, 'S'), (Qt.Key.Key_D, 1, 2, 'D'),
            (Qt.Key.Key_Y, 2, 0, 'Y'),                              (Qt.Key.Key_X, 2, 2, 'X'),
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
        layout.addWidget(make_section_label('Raccourcis'))
        for key_text, desc in [('ESPACE', 'Stop moteurs'), ('8 / 9 / 0', 'Vitesse'), ('M', 'Retour menu')]:
            row = QHBoxLayout()
            row.setSpacing(12)
            k = QLabel(key_text)
            k.setStyleSheet(
                f'color: {TEXT}; font-size: 11px; font-weight: 700; letter-spacing: 2px; min-width: 80px;'
            )
            d = QLabel(desc)
            d.setStyleSheet(f'color: {TEXT_MUTED}; font-size: 12px;')
            row.addWidget(k)
            row.addWidget(d)
            row.addStretch(1)
            layout.addLayout(row)

        layout.addStretch(1)

        back = QPushButton('← TÉLÉOPÉRATION')
        back.setObjectName('ghost_btn')
        back.setCursor(Qt.CursorShape.PointingHandCursor)
        back.clicked.connect(self.on_back)
        layout.addWidget(back)

        return card

    def _build_right_panel(self) -> QWidget:
        card = GlowCard(phase_offset=math.pi)
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

        self.video = QLabel()
        self.video.setObjectName('video')
        self.video.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video.setMinimumSize(640, 480)
        self.video.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.video.setText('En attente du flux caméra…')
        layout.addWidget(self.video, 1)

        self._frame_times = []
        return card

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

    def _send_cmd(self):
        now = time.time()
        if now - self.last_linear_t > AXIS_TIMEOUT:
            self.linear = 0.0
        if now - self.last_angular_t > AXIS_TIMEOUT:
            self.angular = 0.0
        self.bridge.publish_cmd(self.linear * self.speed, self.angular * self.speed)

    def keyPressEvent(self, event: QKeyEvent):
        if event.isAutoRepeat():
            return
        key = event.key()
        now = time.time()

        if key == Qt.Key.Key_M:
            self.on_back(); return
        if key == Qt.Key.Key_Space:
            self.linear = self.angular = 0.0
            self.last_linear_t = self.last_angular_t = 0.0
            return
        if key in SPEED_LEVELS:
            speed_val, label = SPEED_LEVELS[key]
            self.speed = speed_val
            for k, ind in self.speed_indicators.items():
                ind.setActive(k == key)
            self.speed_label.setText(f'Niveau : {label} ({int(speed_val*100)}%)')
            return
        if key in MOVEMENT_KEYS:
            lin, ang = MOVEMENT_KEYS[key]
            if lin != 0.0:
                self.linear = lin; self.last_linear_t = now
            if ang != 0.0:
                self.angular = ang; self.last_angular_t = now
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

    def showEvent(self, event):
        super().showEvent(event)
        for k, (val, _) in SPEED_LEVELS.items():
            self.speed_indicators[k].setActive(abs(val - self.speed) < 1e-3)
        self.setFocus()


# ══════════════════════════════════════════════════════════════════════════════
# WIDGETS MODE AUTONOME
# ══════════════════════════════════════════════════════════════════════════════

class MapWidget(QWidget):
    """Carte de navigation — grille mise à jour en temps réel (placeholder)."""

    COLS = 20
    ROWS = 14

    def __init__(self):
        super().__init__()
        self.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent, False)
        self._robot = (10, 7)
        self._visited: set[tuple[int, int]] = {(10, 7)}
        self.setMinimumSize(200, 160)
        scan_timer = QTimer(self)
        scan_timer.timeout.connect(self.update)
        scan_timer.start(50)

    def update_position(self, col: int, row: int):
        self._robot = (col, row)
        self._visited.add((col, row))
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)

        margin = 16
        label_h = 28
        aw = self.width()  - 2 * margin
        ah = self.height() - label_h - margin

        cs = min(aw / self.COLS, ah / self.ROWS)
        gw = cs * self.COLS
        gh = cs * self.ROWS
        x0 = margin + (aw - gw) / 2
        y0 = label_h + (ah - gh) / 2

        # Section label
        p.setPen(QColor(TEXT_MUTED))
        f = QFont('Inter', 8)
        f.setWeight(QFont.Weight.Bold)
        f.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 2)
        p.setFont(f)
        p.drawText(margin, 18, 'NAVIGATION MAP')

        # Cellules
        for r in range(self.ROWS):
            for c in range(self.COLS):
                rect = QRectF(x0 + c * cs + 0.5, y0 + r * cs + 0.5, cs - 1, cs - 1)
                if (c, r) in self._visited:
                    p.fillRect(rect, QColor(0, 174, 239, 50))   # ACCENT teinté
                else:
                    p.fillRect(rect, QColor(BG_ELEVATED))

        # Lignes de grille
        p.setPen(QPen(QColor(BORDER), 0.5))
        for r in range(self.ROWS + 1):
            y = y0 + r * cs
            p.drawLine(QPointF(x0, y), QPointF(x0 + gw, y))
        for c in range(self.COLS + 1):
            x = x0 + c * cs
            p.drawLine(QPointF(x, y0), QPointF(x, y0 + gh))

        # Ligne de scan — balayage horizontal lent (sonar)
        scan_t = (time.time() % 6.0) / 6.0  # cycle 6 s
        scan_y = y0 + scan_t * gh
        scan_alpha = int(55 * math.sin(scan_t * math.pi))
        if scan_alpha > 0:
            p.setRenderHint(QPainter.RenderHint.Antialiasing, False)
            pen = QPen(QColor(0, 174, 239, scan_alpha))
            pen.setWidthF(1.5)
            p.setPen(pen)
            p.drawLine(QPointF(x0, scan_y), QPointF(x0 + gw, scan_y))

        # Marqueur rover
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rx, ry = self._robot
        cx = x0 + rx * cs + cs / 2
        cy = y0 + ry * cs + cs / 2
        radius = max(cs * 0.38, 3.0)
        p.setBrush(QBrush(QColor(PRIMARY)))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(cx, cy), radius, radius)

        p.end()


class _SensorCell(QFrame):
    """Cellule individuelle d'un capteur (label + valeur)."""

    def __init__(self, label: str):
        super().__init__()
        self.setStyleSheet(
            f'QFrame {{ background-color: {BG_ELEVATED}; border-radius: 8px; border: 1px solid {BORDER}; }}'
            f'QLabel {{ background: transparent; border: none; }}'
        )
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 5, 8, 5)
        lay.setSpacing(2)

        lbl = QLabel(label)
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet(f'color: {TEXT_MUTED}; font-size: 9px; font-weight: 700; letter-spacing: 2px;')

        self._val = QLabel('—')
        self._val.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._val.setStyleSheet(
            f'color: {TEXT_MUTED}; font-size: 12px; font-weight: 700; font-family: monospace; letter-spacing: 1px;'
        )

        lay.addWidget(lbl)
        lay.addWidget(self._val)

    def set_value(self, v: float | None):
        if v is None:
            self._val.setText('—')
            self._val.setStyleSheet(
                f'color: {TEXT_MUTED}; font-size: 12px; font-weight: 700; font-family: monospace; letter-spacing: 1px;'
            )
        else:
            color = PRIMARY if v < 30 else ACCENT
            self._val.setText(f'{v:.0f} cm' if v < 400 else '—')
            self._val.setStyleSheet(
                f'color: {color}; font-size: 12px; font-weight: 700; font-family: monospace; letter-spacing: 1px;'
            )


class USSensorsCard(GlowCard):
    """5 capteurs US — disposition spatiale FL/FC/FR | L/R.
    Les données arriveront via /distances quand le node US sera prêt.
    """

    def __init__(self, phase_offset: float = 0.0):
        super().__init__(phase_offset=phase_offset)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(14, 10, 14, 10)
        lay.setSpacing(7)

        hdr = QHBoxLayout()
        hdr.addWidget(make_section_label('Capteurs US'))
        placeholder = QLabel('— node à venir')
        placeholder.setStyleSheet(f'color: {TEXT_MUTED}; font-size: 9px; letter-spacing: 1px;')
        hdr.addStretch(1)
        hdr.addWidget(placeholder)
        lay.addLayout(hdr)

        # Rangée avant : FL FC FR
        front = QHBoxLayout()
        front.setSpacing(5)
        self._fl = _SensorCell('FL')
        self._fc = _SensorCell('FC')
        self._fr = _SensorCell('FR')
        for cell in (self._fl, self._fc, self._fr):
            front.addWidget(cell)
        lay.addLayout(front)

        # Rangée côtés : L  ◈  R
        sides = QHBoxLayout()
        sides.setSpacing(5)
        self._l = _SensorCell('L')
        dot = QLabel('◈')
        dot.setAlignment(Qt.AlignmentFlag.AlignCenter)
        dot.setStyleSheet(f'color: {BORDER_HOVER}; font-size: 16px;')
        self._r = _SensorCell('R')
        sides.addWidget(self._l)
        sides.addWidget(dot, 1)
        sides.addWidget(self._r)
        lay.addLayout(sides)

    def set_distances(self, fl: float, fc: float, fr: float, l: float, r: float):
        self._fl.set_value(fl)
        self._fc.set_value(fc)
        self._fr.set_value(fr)
        self._l.set_value(l)
        self._r.set_value(r)


class _IMUValue(QWidget):
    """Label + valeur empilés verticalement."""

    def __init__(self, label: str):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)

        lbl = QLabel(label)
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet(f'color: {TEXT_MUTED}; font-size: 9px; font-weight: 700; letter-spacing: 2px;')

        self._val = QLabel('—')
        self._val.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._val.setStyleSheet(
            f'color: {TEXT_MUTED}; font-size: 12px; font-weight: 700; font-family: monospace;'
        )

        lay.addWidget(lbl)
        lay.addWidget(self._val)

    def set_value(self, text: str):
        self._val.setText(text)
        self._val.setStyleSheet(
            f'color: {ACCENT_2}; font-size: 12px; font-weight: 700; font-family: monospace;'
        )


class IMUCard(GlowCard):
    """IMU — accélération x/y/z + angle yaw.
    Les données arriveront via /imu quand le node IMU sera prêt.
    """

    def __init__(self, phase_offset: float = 0.0):
        super().__init__(phase_offset=phase_offset)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(14, 10, 14, 10)
        lay.setSpacing(7)

        hdr = QHBoxLayout()
        hdr.addWidget(make_section_label('IMU'))
        placeholder = QLabel('— node à venir')
        placeholder.setStyleSheet(f'color: {TEXT_MUTED}; font-size: 9px; letter-spacing: 1px;')
        hdr.addStretch(1)
        hdr.addWidget(placeholder)
        lay.addLayout(hdr)

        row = QHBoxLayout()
        row.setSpacing(4)
        self._ax  = _IMUValue('Ax  m/s²')
        self._ay  = _IMUValue('Ay  m/s²')
        self._az  = _IMUValue('Az  m/s²')
        self._yaw = _IMUValue('Yaw  °')
        for w in (self._ax, self._ay, self._az, self._yaw):
            row.addWidget(w)
        lay.addLayout(row)

    def set_values(self, ax: float, ay: float, az: float, yaw: float):
        self._ax.set_value(f'{ax:+.2f}')
        self._ay.set_value(f'{ay:+.2f}')
        self._az.set_value(f'{az:+.2f}')
        self._yaw.set_value(f'{yaw:.1f}°')


class _CameraOverlay(QFrame):
    """Overlay flux caméra agrandi — positionné absolument sur AutonomousPage."""

    def __init__(self, parent: QWidget):
        super().__init__(parent)
        self.setObjectName('card_elevated')

        lay = QVBoxLayout(self)
        lay.setContentsMargins(16, 12, 16, 16)
        lay.setSpacing(8)

        hdr = QHBoxLayout()
        title = QLabel('FLUX CAMÉRA — CONDUITE AUTONOME')
        title.setObjectName('section')
        hdr.addWidget(title)
        hdr.addStretch(1)

        close_btn = QPushButton('✕')
        close_btn.setFixedSize(28, 28)
        close_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        close_btn.setStyleSheet(
            f'QPushButton {{ background: {BG_DEEP}; border: 1px solid {BORDER}; border-radius: 8px; '
            f'color: {TEXT_MUTED}; font-size: 13px; font-weight: 700; }}'
            f'QPushButton:hover {{ border-color: {PRIMARY}; color: {PRIMARY}; }}'
        )
        close_btn.clicked.connect(self.hide)
        hdr.addWidget(close_btn)
        lay.addLayout(hdr)

        self.video = QLabel()
        self.video.setObjectName('video')
        self.video.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video.setText('En attente du flux caméra…')
        lay.addWidget(self.video, 1)

        self.hide()

    def update_frame(self, qimg: QImage):
        if not self.isVisible():
            return
        scaled = qimg.scaled(
            self.video.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.video.setPixmap(QPixmap.fromImage(scaled))


class CameraThumb(GlowCard):
    """Miniature flux caméra — clic pour agrandir."""

    def __init__(self, on_expand, phase_offset: float = 0.0):
        super().__init__(phase_offset=phase_offset)
        self._on_expand = on_expand
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFixedWidth(220)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(4)

        hdr = QHBoxLayout()
        lbl = QLabel('CAMÉRA')
        lbl.setObjectName('section')
        hdr.addWidget(lbl)
        hdr.addStretch(1)
        hint = QLabel('⤢')
        hint.setStyleSheet(f'color: {TEXT_MUTED}; font-size: 14px;')
        hdr.addWidget(hint)
        lay.addLayout(hdr)

        self.video = QLabel()
        self.video.setObjectName('video')
        self.video.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video.setText('—')
        self.video.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        lay.addWidget(self.video, 1)

    def update_frame(self, qimg: QImage):
        scaled = qimg.scaled(
            self.video.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.video.setPixmap(QPixmap.fromImage(scaled))

    def mousePressEvent(self, event):
        self._on_expand()
        super().mousePressEvent(event)


# ══════════════════════════════════════════════════════════════════════════════
# PAGE : MODE AUTONOME
# ══════════════════════════════════════════════════════════════════════════════

class AutonomousPage(QWidget):
    def __init__(self, bridge: RosBridge, on_back):
        super().__init__()
        self.bridge = bridge
        self.on_back = on_back
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self._aruco_log = []
        self._last_seen = False

        root = QVBoxLayout(self)
        root.setContentsMargins(28, 20, 28, 20)
        root.setSpacing(16)

        # ── En-tête ──
        root.addLayout(self._build_header(on_back))

        # ── Corps : carte (gauche) | panneau capteurs (droite) ──
        body = QHBoxLayout()
        body.setSpacing(16)
        body.addWidget(self._build_left_panel(), 4)
        body.addWidget(self._build_right_panel(), 6)
        root.addLayout(body, 1)

        # ── Overlay caméra (hors layout, positionné absolument) ──
        self._cam_overlay = _CameraOverlay(self)

        # ── Connexions ROS ──
        bridge.frame_ready.connect(self._on_frame)
        bridge.aruco_update.connect(self._on_aruco)
        bridge.status_update.connect(self._on_status)

    # ── Construction ──

    def _build_header(self, on_back) -> QHBoxLayout:
        hdr = QHBoxLayout()
        hdr.setSpacing(12)

        badge_wrap = QFrame()
        badge_wrap.setStyleSheet(f'background-color: {ACCENT}; border-radius: 14px;')
        badge_lay = QHBoxLayout(badge_wrap)
        badge_lay.setContentsMargins(12, 4, 16, 4)
        badge_lay.setSpacing(8)
        badge_lay.addWidget(PulsingDot(BG_DEEP, size=10))
        badge_text = QLabel('AUTONOME')
        badge_text.setStyleSheet(
            f'color: {BG_DEEP}; font-size: 11px; font-weight: 800; letter-spacing: 4px;'
        )
        badge_lay.addWidget(badge_text)
        hdr.addWidget(badge_wrap)

        self.status_label = QLabel('En attente…')
        self.status_label.setObjectName('status_value')
        hdr.addSpacing(12)
        hdr.addWidget(self.status_label)

        hdr.addStretch(1)

        back = QPushButton('← MENU PRINCIPAL')
        back.setObjectName('ghost_btn')
        back.setCursor(Qt.CursorShape.PointingHandCursor)
        back.clicked.connect(on_back)
        hdr.addWidget(back)

        return hdr

    def _build_left_panel(self) -> GlowCard:
        card = GlowCard(phase_offset=0.0)
        lay = QVBoxLayout(card)
        lay.setContentsMargins(16, 16, 16, 16)
        lay.setSpacing(0)

        self.map_widget = MapWidget()
        self.map_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        lay.addWidget(self.map_widget, 1)

        return card

    def _build_right_panel(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)

        # ArUco
        lay.addWidget(self._build_aruco_card(), 1)

        # Rangée basse : caméra thumb | (US + IMU)
        bottom = QHBoxLayout()
        bottom.setSpacing(12)

        self._cam_thumb = CameraThumb(on_expand=self._expand_camera, phase_offset=2 * math.pi * 2 / 5)
        bottom.addWidget(self._cam_thumb)

        sensors = QVBoxLayout()
        sensors.setSpacing(10)
        self.us_card  = USSensorsCard(phase_offset=2 * math.pi * 3 / 5)
        self.imu_card = IMUCard(phase_offset=2 * math.pi * 4 / 5)
        sensors.addWidget(self.us_card, 1)
        sensors.addWidget(self.imu_card, 1)
        bottom.addLayout(sensors, 1)

        lay.addLayout(bottom, 1)
        return w

    def _build_aruco_card(self) -> GlowCard:
        card = GlowCard(phase_offset=2 * math.pi / 5)
        lay = QVBoxLayout(card)
        lay.setContentsMargins(24, 18, 24, 18)
        lay.setSpacing(12)

        lay.addWidget(make_section_label('Détection ArUco'))

        self.aruco_status = QLabel('○  AUCUN MARKER VISIBLE')
        self.aruco_status.setStyleSheet(
            f'color: {TEXT_MUTED}; font-size: 20px; font-weight: 600; letter-spacing: 4px;'
        )
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
        lay.addWidget(self.aruco_status)

        info_row = QHBoxLayout()
        info_row.setSpacing(28)
        self.aruco_id   = QLabel('ID  —')
        self.aruco_pos  = QLabel('POSITION  —')
        self.aruco_area = QLabel('AIRE  —')
        for lbl in (self.aruco_id, self.aruco_pos, self.aruco_area):
            lbl.setStyleSheet(
                f'color: {TEXT}; font-size: 12px; font-weight: 600; letter-spacing: 2px;'
            )
            info_row.addWidget(lbl)
        info_row.addStretch(1)
        lay.addLayout(info_row)

        lay.addWidget(make_section_label('Historique'))
        self.aruco_history = QLabel('—')
        self.aruco_history.setObjectName('aruco_log')
        self.aruco_history.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        self.aruco_history.setMinimumHeight(100)
        self.aruco_history.setWordWrap(True)
        lay.addWidget(self.aruco_history, 1)

        return card

    # ── Resize : overlay suit la taille de la page ──

    def resizeEvent(self, event):
        super().resizeEvent(event)
        m = 48
        self._cam_overlay.setGeometry(m, m, self.width() - 2 * m, self.height() - 2 * m)

    def _expand_camera(self):
        m = 48
        self._cam_overlay.setGeometry(m, m, self.width() - 2 * m, self.height() - 2 * m)
        self._cam_overlay.raise_()
        self._cam_overlay.show()

    # ── Slots ROS ──

    def _on_frame(self, qimg: QImage):
        if not self.isVisible():
            return
        self._cam_thumb.update_frame(qimg)
        self._cam_overlay.update_frame(qimg)

    def _on_aruco(self, detected: bool, marker_id: int, cx: float, cy: float, area: float):
        if detected:
            self.aruco_status.setText(f'●  MARKER  ID {marker_id}  DÉTECTÉ')
            self.aruco_status.setStyleSheet(
                f'color: {ACCENT_2}; font-size: 20px; font-weight: 800; letter-spacing: 4px;'
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
                f'color: {TEXT_MUTED}; font-size: 20px; font-weight: 600; letter-spacing: 4px;'
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

    def keyPressEvent(self, event: QKeyEvent):
        if event.key() == Qt.Key.Key_M:
            self.on_back()
        else:
            super().keyPressEvent(event)

    def showEvent(self, event):
        super().showEvent(event)
        self.setFocus()


# ══════════════════════════════════════════════════════════════════════════════
# PAGE : MODE BRAS
# ══════════════════════════════════════════════════════════════════════════════

class ArmPage(QWidget):
    def __init__(self, on_back):
        super().__init__()
        self._on_back = on_back
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(60, 60, 60, 60)
        layout.setSpacing(24)

        header = QHBoxLayout()
        badge = QLabel('BRAS')
        badge.setObjectName('mode_badge')
        header.addWidget(badge)
        header.addStretch(1)
        back = QPushButton('← TÉLÉOPÉRATION')
        back.setObjectName('ghost_btn')
        back.setCursor(Qt.CursorShape.PointingHandCursor)
        back.clicked.connect(self._on_back)
        header.addWidget(back)
        layout.addLayout(header)

        layout.addStretch(1)
        msg = QLabel('Module bras — à implémenter')
        msg.setObjectName('subtitle')
        msg.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(msg)
        layout.addStretch(2)

    def keyPressEvent(self, event: QKeyEvent):
        if event.key() == Qt.Key.Key_M:
            self._on_back()
        else:
            super().keyPressEvent(event)

    def showEvent(self, event):
        super().showEvent(event)
        self.setFocus()


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

        self.stack = QStackedWidget()
        layout.addWidget(self.stack, 1)
        layout.addWidget(self._build_footer())

        self.page_menu      = MenuPage(on_autonomous=self.show_autonomous, on_teleop=self.show_teleop_menu, on_quit=self.close)
        self.page_teleop    = TeleopMenuPage(on_race=self.show_race, on_arm=self.show_arm, on_back=self.show_menu)
        self.page_race      = RacePage(bridge, on_back=self.show_teleop_menu)
        self.page_autonomous = AutonomousPage(bridge, on_back=self.show_menu)
        self.page_arm       = ArmPage(on_back=self.show_teleop_menu)

        for page in (self.page_menu, self.page_teleop, self.page_race, self.page_autonomous, self.page_arm):
            self.stack.addWidget(page)

        self.show_menu()

    def _build_footer(self) -> QWidget:
        f = QFrame()
        f.setObjectName('footer')
        f.setFixedHeight(40)
        lay = QHBoxLayout(f)
        lay.setContentsMargins(28, 0, 28, 0)
        lay.setSpacing(14)

        if os.path.exists(LOGO_PATH):
            mini = QLabel()
            mini.setPixmap(QPixmap(LOGO_PATH).scaledToHeight(18, Qt.TransformationMode.SmoothTransformation))
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

    def _set_mode(self, mode: str):
        self.bridge.publish_mode(mode)
        self.footer_mode.setText(mode.upper())
        color_map = {'idle': TEXT_MUTED, 'race': PRIMARY, 'autonomous': ACCENT_2, 'arm': ACCENT}
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
