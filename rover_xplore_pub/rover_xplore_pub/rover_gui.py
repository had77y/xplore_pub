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

class _AdaptiveStack(QStackedWidget):
    """QStackedWidget avec minimum fixe — empêche la fenêtre de grandir quand une page a un grand contenu."""
    def minimumSizeHint(self):
        return QSize(400, 200)
    def sizeHint(self):
        return QSize(1320, 840)

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
# ROS2 BRIDGE — Pont entre ROS2 (thread C) et Qt (thread principal)
#
# PROBLÈME : ROS2 et Qt tournent dans des threads différents.
#   - Qt exige que TOUS les accès aux widgets se fassent dans le thread principal (GUI).
#   - Les callbacks ROS2 (sur réception d'un message réseau) s'exécutent dans un thread séparé.
#   - Modifier un widget depuis un thread non-GUI → crash ou comportement indéfini.
#
# SOLUTION : les Signaux Qt (Signal/emit/connect)
#   1. Le callback ROS2 appelle bridge.signal.emit(données)  ← thread ROS (sécurisé)
#   2. Qt met l'appel en file d'attente dans le thread GUI
#   3. Le slot connecté s'exécute dans le thread GUI → peut modifier les widgets
#
# SIGNAUX DISPONIBLES :
#   frame_ready(QImage)                       → reçu par les widgets caméra
#   aruco_update(found, id, cx, cy, area)     → reçu par AutonomousPage._on_aruco()
#   status_update(str)                        → reçu par AutonomousPage._on_status()
#
# ARCHITECTURE :
#   RosBridge   = façade publique (utilisée par les pages Qt)
#   _RosNode    = implémentation ROS2 interne (souscriptions, publications)
# ══════════════════════════════════════════════════════════════════════════════

VIDEO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # abandonne les vieux frames si réseau lent
    history=HistoryPolicy.KEEP_LAST,
    depth=1,  # ne garde que le frame le plus récent en file
)


class RosBridge(QObject):
    # Signaux Qt : déclarés comme attributs de classe (pas d'instance)
    # Chaque Signal définit les types des données qu'il transporte
    frame_ready   = Signal(QImage)                      # un nouveau frame caméra est prêt
    aruco_update  = Signal(bool, int, float, float, float)  # (found, id, cx, cy, area)
    status_update = Signal(str)                         # texte de statut depuis autonomous_node

    def __init__(self):
        super().__init__()
        self._node = _RosNode(self)  # crée le nœud ROS2 sous-jacent

    # Méthodes de publication : délèguent à _RosNode
    # Appelées depuis les pages Qt (thread GUI) → safe car publish() est thread-safe dans ROS2
    def publish_mode(self, mode: str):
        self._node.publish_mode(mode)

    def publish_cmd(self, linear: float, angular: float):
        self._node.publish_cmd(linear, angular)

    def publish_arm_cmd(self, z: float, y: float, pince: float,
                        speed: float, dump: bool = False, bin_dir: float = 0.0):
        self._node.publish_arm_cmd(z, y, pince, speed, dump, bin_dir)

    def destroy_node(self):
        self._node.destroy_node()

    @property
    def node(self):
        return self._node


class _RosNode(Node):
    """
    Nœud ROS2 interne — gère toutes les communications réseau.
    Ne touche jamais aux widgets Qt directement (thread safety).
    Utilise bridge.signal.emit() pour envoyer les données vers Qt.
    """
    def __init__(self, bridge: RosBridge):
        super().__init__('rover_gui')
        self._bridge = bridge  # référence vers RosBridge pour emit()

        # ── Publishers → vers le rover (RPi) via WiFi ──────────────────────────
        self.pub_mode = self.create_publisher(String,            '/rover/mode',    10)
        self.pub_cmd  = self.create_publisher(Twist,             '/rover/cmd_vel', 10)
        self.pub_arm  = self.create_publisher(Float32MultiArray, '/rover/arm_cmd', 10)

        # ── Subscribers ← depuis le rover (RPi) via WiFi ──────────────────────
        # /camera/image_compressed : flux JPEG depuis camera_node (RPi)
        # QoS BEST_EFFORT + depth=1 = toujours le frame le plus récent, jamais de lag
        self.create_subscription(
            CompressedImage, '/camera/image_compressed',
            self._on_image, VIDEO_QOS,
        )
        # /aruco_detected : résultat de détection ArUco depuis aruco_node (RPi)
        self.create_subscription(
            Float32MultiArray, '/aruco_detected', self._on_aruco, 10,
        )
        # /rover_status : texte d'état depuis autonomous_node (RPi)
        self.create_subscription(
            String, '/rover_status', self._on_status, 10,
        )

        # Placeholders à brancher quand les nodes seront prêts :
        #   /distances  Float32MultiArray [fl, fc, fr, l, r]  → USSensorsCard
        #   /imu        sensor_msgs/Imu  accel(x,y,z) + gyro(x,y,z)  → IMUCard

    def publish_mode(self, mode: str):
        """Publie le mode sur /rover/mode → reçu par tous les nodes RPi."""
        msg = String(); msg.data = mode
        self.pub_mode.publish(msg)

    def publish_cmd(self, linear: float, angular: float):
        """Publie un Twist sur /rover/cmd_vel → reçu par motor_controller_node (RPi)."""
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.pub_cmd.publish(msg)

    def publish_arm_cmd(self, z: float, y: float, pince: float,
                        speed: float, dump: bool = False, bin_dir: float = 0.0):
        """
        Publie une commande bras sur /rover/arm_cmd → reçu par arm_node (RPi, à créer).
        Format : [z, y, pince, speed, dump(0/1), bin_dir]
        """
        msg = Float32MultiArray()
        msg.data = [
            float(z), float(y), float(pince),
            float(speed),
            1.0 if dump else 0.0,
            float(bin_dir),
        ]
        self.pub_arm.publish(msg)

    def _on_image(self, msg: CompressedImage):
        """
        Callback appelé dans le thread ROS2 à chaque frame reçu de camera_node (RPi).
        Décode le JPEG (bytes ROS2 → numpy BGR → numpy RGB → QImage)
        puis émet le signal frame_ready → Qt affiche dans le thread GUI.
        BGR→RGB : OpenCV stocke en BGR, Qt attend du RGB.
        .copy() : nécessaire car QImage partage le buffer numpy sinon (dangereux en multi-thread).
        """
        buf  = np.frombuffer(msg.data, dtype=np.uint8)
        bgr  = cv2.imdecode(buf, cv2.IMREAD_COLOR)  # décompresse le JPEG
        if bgr is None:
            return
        rgb  = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w, _ = rgb.shape
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888).copy()
        self._bridge.frame_ready.emit(qimg)  # → thread GUI → CameraThumb / _CameraOverlay

    def _on_aruco(self, msg: Float32MultiArray):
        """
        Callback appelé dans le thread ROS2 à chaque message /aruco_detected.
        Reçoit [found, id, cx, cy, area] depuis aruco_node (RPi).
        Émet aruco_update → thread GUI → AutonomousPage._on_aruco()
        """
        if len(msg.data) < 5:
            return
        self._bridge.aruco_update.emit(
            msg.data[0] > 0.5,       # found : True/False
            int(msg.data[1]),         # id
            msg.data[2],              # cx (pixels)
            msg.data[3],              # cy (pixels)
            msg.data[4],              # area (pixels²)
        )

    def _on_status(self, msg: String):
        """
        Callback appelé dans le thread ROS2 à chaque message /rover_status.
        Reçoit une string depuis autonomous_node (RPi) ex: "state=SEARCH pos=(1.2,0.5)..."
        Émet status_update → thread GUI → AutonomousPage._on_status()
        """
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

        back_btn = GlowButton('← MENU PRINCIPAL', glow_color=(0, 174, 239), border_radius=10)
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
        self.setFixedSize(38, 38)
        self.setActive(False)

    def setActive(self, active: bool):
        if active:
            style = (
                f'background-color: {PRIMARY}; color: {TEXT};'
                f'border: 1px solid {PRIMARY}; border-radius: 9px;'
                f'font-size: 14px; font-weight: 800; letter-spacing: 1px;'
            )
        else:
            style = (
                f'background-color: {BG_ELEVATED}; color: {TEXT_DIM};'
                f'border: 1px solid {BORDER}; border-radius: 9px;'
                f'font-size: 14px; font-weight: 700; letter-spacing: 1px;'
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
    Qt.Key.Key_A: ( 0.0,  8.0),   # rotation gauche — angular×8 pour avoir ±100% PWM (entraxe 25cm)
    Qt.Key.Key_D: ( 0.0, -8.0),   # rotation droite
    Qt.Key.Key_Q: ( 1.0,  1.0),   # arc avant-gauche
    Qt.Key.Key_E: ( 1.0, -1.0),   # arc avant-droite
    Qt.Key.Key_Y: (-1.0, -1.0),   # arc arrière-gauche (angular négatif = roue droite plus rapide)
    Qt.Key.Key_X: (-1.0,  1.0),   # arc arrière-droite
}


class RoverVisualWidget(QWidget):
    """Vue top-down simplifiée du rover — 4 roues colorées avec vitesse."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._fl = self._fr = self._bl = self._br = 0.0
        self.setFixedSize(170, 130)

    def update_speeds(self, fl: float, fr: float, bl: float, br: float):
        self._fl, self._fr, self._bl, self._br = fl, fr, bl, br
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        W, H = self.width(), self.height()
        BODY_W, BODY_H = 50, 80
        WHEEL_W, WHEEL_H = 14, 26
        GAP = 6

        bx = (W - BODY_W) // 2
        by = (H - BODY_H) // 2

        # Corps
        p.setBrush(QBrush(QColor('#1A2540')))
        p.setPen(QPen(QColor(BORDER), 1.5))
        p.drawRoundedRect(bx, by, BODY_W, BODY_H, 6, 6)

        # Flèche avant dans le corps
        p.setPen(QPen(QColor(TEXT_MUTED), 1.5))
        ax = bx + BODY_W // 2
        ay = by + BODY_H // 2
        p.drawLine(ax, ay + 12, ax, ay - 10)
        p.drawLine(ax, ay - 10, ax - 6, ay - 2)
        p.drawLine(ax, ay - 10, ax + 6, ay - 2)

        # Roues et texte vitesse
        wx_l = bx - GAP - WHEEL_W
        wx_r = bx + BODY_W + GAP

        font = p.font()
        font.setPixelSize(10)
        font.setBold(True)
        p.setFont(font)

        for speed, wx, wy, is_left in [
            (self._fl, wx_l, by + 8,                True),   # FL
            (self._fr, wx_r, by + 8,                False),  # FR
            (self._bl, wx_l, by + BODY_H - 8 - WHEEL_H, True),   # BL
            (self._br, wx_r, by + BODY_H - 8 - WHEEL_H, False),  # BR
        ]:
            col = (QColor(ACCENT_2) if speed > 0.02
                   else QColor(PRIMARY) if speed < -0.02
                   else QColor(BORDER))
            p.setBrush(QBrush(col))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawRoundedRect(wx, wy, WHEEL_W, WHEEL_H, 3, 3)

            pct = int(round(abs(speed) * 100))
            text = '0' if pct == 0 else (f'+{pct}' if speed > 0 else f'-{pct}')
            p.setPen(QPen(QColor(TEXT_MUTED) if pct == 0 else col))
            if is_left:
                p.drawText(QRectF(0, wy, wx_l - 2, WHEEL_H),
                           Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, text)
            else:
                p.drawText(QRectF(wx + WHEEL_W + 4, wy, W - wx - WHEEL_W - 4, WHEEL_H),
                           Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, text)

        p.end()


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

        self._rover_visual = RoverVisualWidget()

        root = QHBoxLayout(self)
        root.setContentsMargins(32, 28, 32, 28)
        root.setSpacing(28)

        root.addWidget(self._build_left_panel(), 0)
        root.addWidget(self._build_right_panel(), 1)

        bridge.frame_ready.connect(self._on_frame)

        self.cmd_timer = QTimer(self)
        self.cmd_timer.timeout.connect(self._send_cmd)
        self.cmd_timer.start(100)

    def _make_sep(self) -> QFrame:
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet(f'background: {BORDER}; border: none; max-height: 1px;')
        return sep

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

        layout.addWidget(self._make_sep())
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

        layout.addWidget(self._make_sep())
        layout.addWidget(make_section_label('Mouvement rover'))

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
        grid_holder.addSpacing(12)
        grid_holder.addWidget(self._rover_visual)
        grid_holder.addStretch(1)
        layout.addLayout(grid_holder)

        layout.addWidget(self._make_sep())
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

        back = GlowButton('← TÉLÉOPÉRATION', glow_color=(0, 174, 239), border_radius=10)
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
        self.video.setMinimumSize(320, 240)
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
        if self.active_keys:
            linear = sum(MOVEMENT_KEYS[k][0] for k in self.active_keys)
            angular = sum(MOVEMENT_KEYS[k][1] for k in self.active_keys)
            self.linear  = max(-1.0, min(1.0, linear))
            self.angular = max(-1.0, min(1.0, angular))
        else:
            self.linear = self.angular = 0.0
        lin = self.linear * self.speed
        ang = self.angular * self.speed
        self.bridge.publish_cmd(lin, ang)
        fl = bl = max(-1.0, min(1.0, lin - ang * 0.125))
        fr = br = max(-1.0, min(1.0, lin + ang * 0.125))
        self._rover_visual.update_speeds(fl, fr, bl, br)

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
    """
    Carte de navigation interactive — grille 12×8 représentant le terrain de compétition.

    RÔLE DOUBLE :
      1. Simulation locale (côté PC) : visualise l'algorithme BFS de navigation
         avant de l'implémenter sur le rover réel.
      2. Visualisation temps réel (futur) : quand autonomous_node (RPi) publiera
         /rover/grid_state et /rover/grid_pos, MapWidget les affichera via
         update_grid() et update_position().

    GRILLE :
      12 lignes × 8 colonnes = 96 cases
      Bordure de 1 case tout autour (CELL_BORDER) = zone interdite
      Zone navigable intérieure : 10×6 cases = 80cm × 80cm chacune
      → terrain réel 8m × 5m (8/0.8=10, 5/0.8=6... puis arrondi avec bordures)

    ÉTATS DES CASES :
      UNDISCOVERED (0) = bleu foncé — le rover ne sait pas si c'est libre ou non
      FREE         (1) = bleu      — le rover est passé par là, c'est libre
      OBSTACLE     (2) = rouge     — un obstacle a été détecté ici
      AMBUSH       (3) = noir      — cul-de-sac vécu : le rover y est allé et
                                    toutes les voisines étaient bloquées → à éviter
      CELL_BORDER  (4) = noir      — bordure de la grille (hors terrain)

    ALGORITHME BFS (_bfs_to_best_undiscovered) :
      À chaque tick de navigation (_nav_step), le rover cherche la meilleure case
      UNDISCOVERED à explorer. "Meilleure" = distance Chebyshev minimale vers la cible.
      Distance de Chebyshev = max(|dr|, |dc|) — permet les diagonales.
      Le BFS traverse uniquement les cases FREE (déjà visitées) pour atteindre
      une case UNDISCOVERED adjacente.

    PRIORITÉ (_priority) :
      Calculée par _recalc_priorities() : pour chaque case (r,c),
      priority[r][c] = max(|r - target_r|, |c - target_c|)
      → 0 au centre de la cible, augmente en s'éloignant
      → BFS choisit toujours la case UNDISCOVERED avec la priorité la plus faible
      → le rover explore en spirale vers la cible

    GESTION DES CUL-DE-SAC (AMBUSH) :
      Si BFS ne trouve aucune case UNDISCOVERED atteignable depuis la position courante
      → la case courante est marquée AMBUSH
      → on remonte d'un cran dans path_stack (backtracking)
    """

    ROWS = 12
    COLS = 8

    UNDISCOVERED = 0
    FREE         = 1
    OBSTACLE     = 2
    AMBUSH       = 3
    CELL_BORDER  = 4

    _LEGEND_ITEMS = [
        (0, 'NON DÉCOUVERT'),
        (1, 'LIBRE'),
        (2, 'OBSTACLE'),
        (3, 'EMBUSCADE'),
        (4, 'BORDURE'),
    ]

    def __init__(self):
        super().__init__()
        self.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent, False)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

        self._fill = {
            self.UNDISCOVERED: QColor('#283C5A'),
            self.FREE:         QColor('#0D4A70'),
            self.OBSTACLE:     QColor('#9B1C1C'),
            self.AMBUSH:       QColor('#111118'),
            self.CELL_BORDER:  QColor('#080810'),
        }

        # Positions par défaut (coordonnées paddées, row, col)
        self._target    = (1, 1)
        self._start     = (self.ROWS - 2, self.COLS - 2)
        self._robot_pos = self._start

        self._manual_obstacles = set()
        self._reset_grid()
        self._recalc_priorities()

        # Mode de clic actif : 'target' | 'start' | 'obstacle' | None
        self._click_mode = None
        # État de navigation : 'ready' | 'running' | 'done'
        self._nav_state  = 'ready'

        # Rects buttons (mis à jour dans paintEvent, lus dans mousePressEvent)
        self._btn_cible    = QRectF()
        self._btn_depart   = QRectF()
        self._btn_obstacle = QRectF()
        self._btn_start    = QRectF()
        self._grid_x0 = 0.0
        self._grid_y0 = 0.0
        self._grid_cs = 1.0

        # Navigation
        self._path_stack    = []
        self._transit_path  = []
        self._ambush_streak = 0
        self._nav_timer = QTimer(self)
        self._nav_timer.timeout.connect(self._nav_step)

        self.setMinimumSize(200, 380)
        t = QTimer(self)
        t.timeout.connect(self.update)
        t.start(50)

    # ── Grille / priorités ────────────────────────────────────────────

    def _reset_grid(self):
        self._cells = [
            [self.CELL_BORDER if (r == 0 or r == self.ROWS - 1 or c == 0 or c == self.COLS - 1)
             else self.UNDISCOVERED
             for c in range(self.COLS)]
            for r in range(self.ROWS)
        ]
        for r, c in getattr(self, '_manual_obstacles', set()):
            self._cells[r][c] = self.OBSTACLE

    def _recalc_priorities(self):
        tr, tc = self._target
        self._priority = [
            [max(abs(r - tr), abs(c - tc)) for c in range(self.COLS)]
            for r in range(self.ROWS)
        ]

    # ── Navigation simulée ────────────────────────────────────────────

    def _start_navigation(self):
        """
        Lance la simulation de navigation : remet la grille à zéro et démarre le timer.
        Appelée quand l'utilisateur clique le bouton DÉPART dans MapWidget.
        Le timer _nav_timer appelle _nav_step() toutes les 500ms (un pas de navigation).
        """
        self._reset_grid()
        self._robot_pos  = self._start         # robot part de la position de départ
        self._path_stack = [self._start]       # pile de backtracking (historique des positions)
        self._transit_path = []                # chemin de transit via cases FREE (vide au départ)
        sr, sc = self._start
        self._cells[sr][sc] = self.FREE        # marque la case de départ comme visitée
        self._ambush_streak  = 0
        self._nav_state      = 'running'
        self._nav_timer.start(500)             # un pas toutes les 500ms
        self.update()

    def _stop_navigation(self):
        """Arrête la simulation et remet la grille à l'état initial."""
        self._nav_timer.stop()
        self._nav_state    = 'ready'
        self._transit_path = []
        self._reset_grid()
        self._robot_pos    = self._start
        self.update()

    def _bfs_to_best_undiscovered(self, sr, sc):
        """
        BFS (Breadth-First Search) depuis la position (sr, sc).

        OBJECTIF : trouver la case UNDISCOVERED la plus proche de la CIBLE
                   qui soit atteignable en traversant uniquement des cases FREE.

        POURQUOI BFS et pas A* ou Dijkstra ?
          BFS garantit le chemin le plus court en nombre de cases (toutes les cases
          ont le même "coût" de traversée). Ici on ne cherche pas le chemin le plus
          court vers la cible finale, mais vers la prochaine case à explorer.
          BFS est plus simple et suffisamment rapide sur une grille 12×8.

        POURQUOI "via FREE" seulement ?
          On ne peut pas traverser une case UNDISCOVERED pour en atteindre une autre
          (on ne sait pas si elle est libre). On doit d'abord la visiter.
          Exception : si une UNDISCOVERED est directement voisine → on peut l'atteindre.

        DIAGONALES :
          Les 8 directions sont autorisées (y compris diagonales).
          Condition supplémentaire pour les diagonales : les deux cases adjacentes
          (dans les directions cardinales) ne doivent pas être bloquées.
          Exemple : pour aller en (r-1, c-1), les cases (r-1, c) ET (r, c-1)
          ne doivent pas être OBSTACLE/AMBUSH/BORDER (évite de "passer dans les coins").

        RETOURNE :
          Liste de cases [(sr,sc), ..., (best_r, best_c)] = chemin complet
          ou [] si aucune case UNDISCOVERED n'est atteignable (cul-de-sac total).

        PRIORITÉ :
          Parmi toutes les cases UNDISCOVERED atteignables, on choisit celle avec
          la priorité (distance Chebyshev vers la cible) la plus faible.
          → le rover explore toujours en direction de la cible en premier.
        """
        from collections import deque
        _blocked = (self.CELL_BORDER, self.OBSTACLE, self.AMBUSH)
        # 8 directions : N, S, O, E + 4 diagonales
        dirs = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

        parent   = {(sr, sc): None}  # pour reconstruire le chemin final
        queue    = deque([(sr, sc)])
        best_priority = float('inf')  # on cherche la priorité MINIMALE (plus proche cible)
        best_end = None               # case UNDISCOVERED la plus intéressante trouvée

        while queue:
            r, c = queue.popleft()
            for dr, dc in dirs:
                nr, nc = r + dr, c + dc
                # Vérification des limites de la grille
                if not (0 <= nr < self.ROWS and 0 <= nc < self.COLS):
                    continue
                cell = self._cells[nr][nc]
                # Cases bloquées → on ne peut pas passer
                if cell in _blocked:
                    continue
                # Vérification diagonale : pas de passage dans les coins
                if abs(dr) == 1 and abs(dc) == 1:
                    if self._cells[r][nc] in _blocked or self._cells[nr][c] in _blocked:
                        continue
                # Déjà visité par ce BFS → ignorer
                if (nr, nc) in parent:
                    continue
                parent[(nr, nc)] = (r, c)  # mémorise d'où on vient (pour reconstruire)

                if cell == self.UNDISCOVERED:
                    # Case candidate ! Vérifier si c'est la meilleure (priorité min)
                    p = self._priority[nr][nc]
                    if p < best_priority:
                        best_priority = p
                        best_end      = (nr, nc)
                    # On N'ajoute PAS à la queue : on ne traverse pas les UNDISCOVERED
                else:
                    # Case FREE → on peut traverser → continuer le BFS à partir d'ici
                    queue.append((nr, nc))

        if best_end is None:
            return []  # cul-de-sac total : aucune case UNDISCOVERED atteignable

        # Reconstruction du chemin en remontant le dict parent
        path = []
        cur  = best_end
        while cur is not None:
            path.append(cur)
            cur = parent[cur]
        path.reverse()  # on avait construit à l'envers (de la fin vers le début)
        return path      # → [(sr,sc), case1, case2, ..., best_end]

    def _nav_step(self):
        """
        Un pas de navigation — appelé toutes les 500ms par le timer.
        Implémente l'algorithme de navigation BFS avec backtracking.

        TROIS CAS POSSIBLES à chaque tick :

        1. La cible est atteinte → fin de simulation
        2. Transit en cours (self._transit_path non vide) → avancer d'une case sur le chemin
           Le transit se produit quand la meilleure UNDISCOVERED n'est pas directement voisine
           mais accessible via un couloir de cases FREE déjà visitées.
        3. Pas de transit → lancer un BFS pour trouver le prochain mouvement :
           a. Aucune case UNDISCOVERED atteignable → cul-de-sac → AMBUSH + backtrack
           b. Case UNDISCOVERED directement voisine → déplacement classique
           c. Case UNDISCOVERED accessible via transit → stocker le chemin intermédiaire
        """
        cr, cc = self._robot_pos

        # Cas 1 : mission accomplie
        if (cr, cc) == self._target:
            self._nav_state = 'done'
            self._nav_timer.stop()
            self.update()
            return

        # Cas 2 : transit en cours → suivre le chemin stocké case par case
        if self._transit_path:
            nr, nc = self._transit_path.pop(0)  # prochaine case du chemin
            if self._cells[nr][nc] == self.UNDISCOVERED:
                # On vient de "découvrir" cette case en transit → marquer FREE
                self._cells[nr][nc] = self.FREE
                self._ambush_streak  = 0
            self._robot_pos = (nr, nc)
            self.update()
            return

        # Cas 3 : calculer le prochain mouvement via BFS
        path = self._bfs_to_best_undiscovered(cr, cc)

        if not path:
            # CAS 3a : cul-de-sac — aucune case UNDISCOVERED atteignable depuis ici
            # → marquer la case courante AMBUSH (à éviter à l'avenir)
            # → backtrack : revenir à la case précédente dans path_stack
            self._cells[cr][cc]  = self.AMBUSH
            self._ambush_streak += 1
            if self._path_stack:
                self._robot_pos = self._path_stack.pop()  # recule d'un pas

        elif len(path) == 2:
            # CAS 3b : la case UNDISCOVERED est directement voisine (chemin = [courant, voisin])
            nr, nc = path[1]
            self._cells[nr][nc] = self.FREE          # découverte de la case
            self._path_stack.append((cr, cc))        # mémorise la position courante pour backtrack
            self._robot_pos      = (nr, nc)
            self._ambush_streak  = 0

        else:
            # CAS 3c : la case UNDISCOVERED est atteinte via un transit de cases FREE
            # path = [courant, free1, free2, ..., undiscovered]
            # Stocker le chemin complet et avancer d'un pas
            self._path_stack.append((cr, cc))
            self._transit_path = path[1:]            # tout le chemin sauf la case courante
            nr, nc = self._transit_path.pop(0)       # premier pas du transit
            self._robot_pos = (nr, nc)

        self.update()

    # ── Interface publique ────────────────────────────────────────────

    def update_grid(self, cells: list):
        self._cells = cells
        self.update()

    def update_position(self, col: int, row: int):
        self._robot_pos = (row, col)
        self.update()

    # ── Événements souris ─────────────────────────────────────────────

    def mousePressEvent(self, event):
        px = event.position().x()
        py = event.position().y()

        if self._btn_start.contains(px, py):
            if self._nav_state == 'running':
                self._stop_navigation()
            else:
                self._start_navigation()
            return

        if self._btn_cible.contains(px, py):
            self._click_mode = 'target' if self._click_mode != 'target' else None
            self.update()
            return

        if self._btn_depart.contains(px, py):
            self._click_mode = 'start' if self._click_mode != 'start' else None
            self.update()
            return

        if self._btn_obstacle.contains(px, py):
            self._click_mode = 'obstacle' if self._click_mode != 'obstacle' else None
            self.update()
            return

        if self._nav_state != 'running' and self._click_mode is not None:
            cs = self._grid_cs
            col = int((px - self._grid_x0) / cs)
            row = int((py - self._grid_y0) / cs)
            if 1 <= row <= self.ROWS - 2 and 1 <= col <= self.COLS - 2:
                if self._click_mode == 'obstacle':
                    if (row, col) not in (self._target, self._start):
                        if (row, col) in self._manual_obstacles:
                            self._manual_obstacles.discard((row, col))
                            self._cells[row][col] = self.UNDISCOVERED
                        else:
                            self._manual_obstacles.add((row, col))
                            self._cells[row][col] = self.OBSTACLE
                    # mode sticky : ne pas réinitialiser _click_mode
                elif self._click_mode == 'target':
                    self._target = (row, col)
                    self._click_mode = None
                    self._recalc_priorities()
                    self._reset_grid()
                    self._nav_state = 'ready'
                else:
                    self._start = (row, col)
                    self._robot_pos = (row, col)
                    self._click_mode = None
                    self._recalc_priorities()
                    self._reset_grid()
                    self._nav_state = 'ready'
                self.update()

    # ── Rendu ─────────────────────────────────────────────────────────

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)

        LEGEND_H = 58
        LABEL_H  = 26
        BTNS_H   = 24
        START_H  = 30
        GAP      = 5
        margin   = 12

        aw = self.width()  - 2 * margin
        ah = self.height() - LABEL_H - BTNS_H - START_H - LEGEND_H - 3 * GAP - margin
        cs = min(aw / self.COLS, ah / self.ROWS)
        gw = cs * self.COLS
        gh = cs * self.ROWS
        x0 = margin + (aw - gw) / 2
        y0 = LABEL_H + BTNS_H + GAP + (ah - gh) / 2

        self._grid_x0 = x0
        self._grid_y0 = y0
        self._grid_cs = cs

        # Titre
        p.setPen(QColor(TEXT_MUTED))
        f = QFont('Inter', 8)
        f.setWeight(QFont.Weight.Bold)
        f.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 2)
        p.setFont(f)
        p.drawText(margin, LABEL_H - 4, 'NAVIGATION MAP')

        # Boutons mode (ligne sous le titre)
        btn_w = 62
        btn_h = 20
        by    = LABEL_H + 2
        self._btn_obstacle = QRectF(self.width() - margin - 3 * btn_w - 10, by, btn_w, btn_h)
        self._btn_cible    = QRectF(self.width() - margin - 2 * btn_w - 5,  by, btn_w, btn_h)
        self._btn_depart   = QRectF(self.width() - margin - btn_w,           by, btn_w, btn_h)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        self._draw_mode_btn(p, self._btn_obstacle, 'OBSTACLE', self._click_mode == 'obstacle', QColor('#9B1C1C'))
        self._draw_mode_btn(p, self._btn_cible,    'CIBLE',    self._click_mode == 'target',   QColor(ACCENT_2))
        self._draw_mode_btn(p, self._btn_depart,   'DÉPART',   self._click_mode == 'start',    QColor(ACCENT))
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)

        # Cellules
        p.setPen(Qt.PenStyle.NoPen)
        for r in range(self.ROWS):
            for c in range(self.COLS):
                state = self._cells[r][c]
                rx = x0 + c * cs
                ry = y0 + r * cs
                p.setBrush(QBrush(self._fill[state]))
                p.drawRect(QRectF(rx, ry, cs, cs))
                if state == self.CELL_BORDER and cs >= 6:
                    p.setPen(QPen(QColor('#2A2A3C'), 0.8))
                    p.drawLine(QPointF(rx + 2, ry + 2), QPointF(rx + cs - 2, ry + cs - 2))
                    p.drawLine(QPointF(rx + cs - 2, ry + 2), QPointF(rx + 2, ry + cs - 2))
                    p.setPen(Qt.PenStyle.NoPen)

        # Lignes grandes cases
        p.setPen(QPen(QColor(BORDER), 1.0))
        for r in range(self.ROWS + 1):
            p.drawLine(QPointF(x0, y0 + r * cs), QPointF(x0 + gw, y0 + r * cs))
        for c in range(self.COLS + 1):
            p.drawLine(QPointF(x0 + c * cs, y0), QPointF(x0 + c * cs, y0 + gh))

        # Lignes mini-cases
        mini_col = QColor(BORDER)
        mini_col.setAlpha(90)
        p.setPen(QPen(mini_col, 0.4))
        for r in range(1, self.ROWS - 1):
            for c in range(1, self.COLS - 1):
                rx = x0 + c * cs
                ry = y0 + r * cs
                hcs = cs / 2
                p.drawLine(QPointF(rx, ry + hcs), QPointF(rx + cs, ry + hcs))
                p.drawLine(QPointF(rx + hcs, ry), QPointF(rx + hcs, ry + cs))

        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)

        # Marqueurs T et S
        tr, tc = self._target
        sr, sc = self._start
        self._draw_marker(p, x0, y0, cs, tc, tr, 'T', QColor(ACCENT_2))
        self._draw_marker(p, x0, y0, cs, sc, sr, 'S', QColor(ACCENT))

        # Rover
        rr, rc = self._robot_pos
        cx = x0 + rc * cs + cs / 2
        cy = y0 + rr * cs + cs / 2
        r_rad = max(cs * 0.28, 3.0)
        p.setBrush(QBrush(QColor(PRIMARY)))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(cx, cy), r_rad, r_rad)

        # Bouton START / STOP
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        start_y = y0 + gh + GAP
        self._btn_start = QRectF(x0, start_y, gw, START_H - 2)
        self._draw_start_btn(p, self._btn_start)

        # Légende
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        legend_y = int(start_y) + START_H + GAP - 4
        self._draw_legend(p, margin, legend_y, int(self.width() - 2 * margin))
        p.end()

    def _draw_mode_btn(self, p: QPainter, rect: QRectF, label: str, active: bool, color: QColor):
        bg = QColor(color)
        bg.setAlpha(80 if active else 25)
        p.setBrush(QBrush(bg))
        p.setPen(QPen(color, 1.0))
        p.drawRoundedRect(rect, 3, 3)
        p.setPen(color if active else QColor(TEXT_MUTED))
        f = QFont('Inter', 7)
        f.setWeight(QFont.Weight.Bold)
        f.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 1)
        p.setFont(f)
        p.drawText(rect, Qt.AlignmentFlag.AlignCenter, label)

    def _draw_start_btn(self, p: QPainter, rect: QRectF):
        if self._nav_state == 'running':
            label, color = '■  STOP',  QColor(PRIMARY)
        elif self._nav_state == 'done':
            label, color = '↺  RESET', QColor(ACCENT)
        else:
            label, color = '▶  START', QColor(ACCENT_2)
        bg = QColor(color)
        bg.setAlpha(35)
        p.setBrush(QBrush(bg))
        p.setPen(QPen(color, 1.2))
        p.drawRoundedRect(rect, 4, 4)
        p.setPen(color)
        f = QFont('Inter', 9)
        f.setWeight(QFont.Weight.Bold)
        f.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 2)
        p.setFont(f)
        p.drawText(rect, Qt.AlignmentFlag.AlignCenter, label)

    def _draw_marker(self, p: QPainter, x0, y0, cs, col, row, label, color):
        cx = x0 + col * cs + cs / 2
        cy = y0 + row * cs + cs / 2
        hs = cs * 0.28
        p.setPen(QPen(color, 1.5))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRect(QRectF(cx - hs, cy - hs, hs * 2, hs * 2))
        p.setPen(color)
        p.setFont(QFont('Inter', max(int(cs * 0.25), 5)))
        p.drawText(QRectF(cx - hs, cy - hs, hs * 2, hs * 2),
                   Qt.AlignmentFlag.AlignCenter, label)

    def _draw_legend(self, p: QPainter, x: int, y: int, w: int):
        BOX   = 12
        GAP   = 6
        COL_W = w // 3
        p.setFont(QFont('Inter', 8))
        for i, (state, label) in enumerate(self._LEGEND_ITEMS):
            col_i = i % 3
            row_i = i // 3
            ix = x + col_i * COL_W
            iy = y + row_i * 20
            p.setBrush(QBrush(self._fill[state]))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawRect(QRectF(ix, iy, BOX, BOX))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(QColor(BORDER_HOVER), 1.0))
            p.drawRect(QRectF(ix, iy, BOX, BOX))
            if state == self.CELL_BORDER:
                p.setPen(QPen(QColor('#3A3A50'), 0.8))
                p.drawLine(QPointF(ix + 2, iy + 2), QPointF(ix + BOX - 2, iy + BOX - 2))
                p.drawLine(QPointF(ix + BOX - 2, iy + 2), QPointF(ix + 2, iy + BOX - 2))
            p.setPen(QColor(TEXT_DIM))
            p.drawText(ix + BOX + GAP, iy + BOX - 1, label)


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


class _IMUSensorRow(QWidget):
    """Label axe (X/Y/Z) à gauche + valeur numérique alignée à droite."""

    def __init__(self, label: str):
        super().__init__()
        lay = QHBoxLayout(self)
        lay.setContentsMargins(4, 3, 4, 3)
        lay.setSpacing(6)

        lbl = QLabel(label)
        lbl.setFixedWidth(14)
        lbl.setStyleSheet(
            f'color: {TEXT_MUTED}; font-size: 11px; font-weight: 600; font-family: monospace;'
        )
        self._val = QLabel('—')
        self._val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self._val.setStyleSheet(
            f'color: {TEXT_MUTED}; font-size: 14px; font-weight: 700; font-family: monospace;'
        )
        lay.addWidget(lbl)
        lay.addWidget(self._val, 1)

    def set_value(self, val: float):
        self._val.setText(f'{val:+.3f}')
        self._val.setStyleSheet(
            f'color: {ACCENT}; font-size: 14px; font-weight: 700; font-family: monospace;'
        )

    def reset(self):
        self._val.setText('—')
        self._val.setStyleSheet(
            f'color: {TEXT_MUTED}; font-size: 14px; font-weight: 700; font-family: monospace;'
        )


class _IMUSensorSection(QFrame):
    """Bloc capteur IMU : titre [unité] + axes X/Y/Z + ligne magnitude."""

    def __init__(self, title: str, unit: str):
        super().__init__()
        self.setStyleSheet(
            f'QFrame {{ background-color: {BG_ELEVATED}; border-radius: 10px;'
            f' border: 1px solid {BORDER}; }}'
            f'QLabel {{ background: transparent; border: none; }}'
        )
        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(4)

        hdr = QHBoxLayout()
        hdr.setSpacing(4)
        title_lbl = QLabel(title)
        title_lbl.setStyleSheet(
            f'color: {TEXT}; font-size: 10px; font-weight: 700; letter-spacing: 2px;'
        )
        unit_lbl = QLabel(f'[{unit}]')
        unit_lbl.setStyleSheet(f'color: {TEXT_MUTED}; font-size: 9px; font-weight: 500;')
        hdr.addWidget(title_lbl)
        hdr.addWidget(unit_lbl)
        hdr.addStretch(1)
        lay.addLayout(hdr)

        self._x = _IMUSensorRow('X')
        self._y = _IMUSensorRow('Y')
        self._z = _IMUSensorRow('Z')
        for row in (self._x, self._y, self._z):
            lay.addWidget(row)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet(f'background: {BORDER}; border: none; max-height: 1px;')
        lay.addWidget(sep)

        self._mag = QLabel('≈ —')
        self._mag.setAlignment(Qt.AlignmentFlag.AlignRight)
        self._mag.setStyleSheet(
            f'color: {TEXT_MUTED}; font-size: 9px; font-family: monospace; letter-spacing: 1px;'
        )
        lay.addWidget(self._mag)

    def set_values(self, x: float, y: float, z: float, unit_suffix: str = ''):
        mag = math.sqrt(x * x + y * y + z * z)
        self._x.set_value(x)
        self._y.set_value(y)
        self._z.set_value(z)
        self._mag.setText(f'≈ {mag:.3f}{(" " + unit_suffix) if unit_suffix else ""}')


class IMUCard(GlowCard):
    """Accélération (m/s²) + Gyroscope (°/s) — données via /imu."""

    def __init__(self, phase_offset: float = 0.0):
        super().__init__(phase_offset=phase_offset)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(14, 10, 14, 10)
        lay.setSpacing(8)

        hdr = QHBoxLayout()
        hdr.addWidget(make_section_label('IMU'))
        placeholder = QLabel('— node à venir')
        placeholder.setStyleSheet(f'color: {TEXT_MUTED}; font-size: 9px; letter-spacing: 1px;')
        hdr.addStretch(1)
        hdr.addWidget(placeholder)
        lay.addLayout(hdr)

        sensors_row = QHBoxLayout()
        sensors_row.setSpacing(8)
        self._accel = _IMUSensorSection('ACCÉLÉRATION', 'm/s²')
        self._gyro  = _IMUSensorSection('GYROSCOPE', '°/s')
        sensors_row.addWidget(self._accel)
        sensors_row.addWidget(self._gyro)
        lay.addLayout(sensors_row)

    def set_values(self, ax: float, ay: float, az: float,
                   gx: float, gy: float, gz: float):
        self._accel.set_values(ax, ay, az, 'm/s²')
        self._gyro.set_values(gx, gy, gz, '°/s')


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

        back = GlowButton('← MENU PRINCIPAL', glow_color=(0, 174, 239), border_radius=10)
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
# CONSTANTES MODE BRAS
# ══════════════════════════════════════════════════════════════════════════════

ARM_SPEED_LEVELS = {
    Qt.Key.Key_6: (0.10, '6'),
    Qt.Key.Key_7: (0.25, '7'),
    Qt.Key.Key_8: (0.50, '8'),
    Qt.Key.Key_9: (0.75, '9'),
    Qt.Key.Key_0: (1.00, '0'),
}

ARM_AXIS_TIMEOUT = 0.20

# key → (z_dir, y_dir, pince_dir, bin_dir)
ARM_MOVE_KEYS = {
    Qt.Key.Key_O:    ( 1.0,  0.0,  0.0,  0.0),   # Z monter
    Qt.Key.Key_K:    (-1.0,  0.0,  0.0,  0.0),   # Z descendre
    Qt.Key.Key_P:    ( 0.0,  1.0,  0.0,  0.0),   # Y pivot +
    Qt.Key.Key_I:    ( 0.0, -1.0,  0.0,  0.0),   # Y dépivot
    Qt.Key.Key_J:    ( 0.0,  0.0,  1.0,  0.0),   # Pince ouvrir
    Qt.Key.Key_L:    ( 0.0,  0.0, -1.0,  0.0),   # Pince fermer
    Qt.Key.Key_Up:   ( 0.0,  0.0,  0.0,  1.0),   # Benne monter
    Qt.Key.Key_Down: ( 0.0,  0.0,  0.0, -1.0),   # Benne descendre
}

_AMBER = (255, 140, 0)


# ══════════════════════════════════════════════════════════════════════════════
# PAGE : MODE BRAS
# ══════════════════════════════════════════════════════════════════════════════

class ArmPage(QWidget):
    def __init__(self, bridge: RosBridge, on_back):
        super().__init__()
        self.bridge = bridge
        self._on_back = on_back
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        # ── Rover motion state ──
        self.linear = 0.0
        self.angular = 0.0
        self.last_linear_t = 0.0
        self.last_angular_t = 0.0
        self.active_rover_keys: set = set()

        # ── Arm state ──
        self.arm_z = 0.0
        self.arm_y = 0.0
        self.arm_pince = 0.0
        self.bin_dir = 0.0
        self.last_z_t = 0.0
        self.last_y_t = 0.0
        self.last_pince_t = 0.0
        self.last_bin_t = 0.0
        self.active_arm_keys: set = set()

        # ── Angles accumulés des servos [-100..100] ──
        # Le GUI accumule et envoie la valeur absolue directement à arm_node.
        # SERVO_STEP = unités ajoutées par tick (100ms) à speed=1.0
        self._SERVO_STEP = 3.0
        self._s1_angle  = 0.0   # flip/unflip  (arm_y)
        self._s23_angle = 0.0   # open/close   (arm_pince)
        self._s4_angle  = 0.0   # benne        (bin_dir)

        # ── Vitesse partagée rover + bras ──
        self.speed = 0.50   # défaut niveau 8
        self._dump_pending = False

        self._rover_visual = RoverVisualWidget()

        root = QHBoxLayout(self)
        root.setContentsMargins(32, 28, 32, 28)
        root.setSpacing(28)
        root.addWidget(self._build_left_panel(), 0)
        root.addWidget(self._build_right_panel(), 1)

        bridge.frame_ready.connect(self._on_frame)

        self.cmd_timer = QTimer(self)
        self.cmd_timer.timeout.connect(self._send_cmds)
        self.cmd_timer.start(100)

    # ── Construction panneau gauche ────────────────────────────────────────────

    def _build_left_panel(self) -> QWidget:
        card = GlowCard(phase_offset=math.pi / 3, glow_color=(0, 174, 239))
        card.setFixedWidth(390)

        lay = QVBoxLayout(card)
        lay.setContentsMargins(26, 22, 26, 22)
        lay.setSpacing(14)

        # Badge BRAS
        hdr = QHBoxLayout()
        badge_wrap = QFrame()
        badge_wrap.setStyleSheet(f'background-color: {ACCENT}; border-radius: 14px;')
        badge_lay = QHBoxLayout(badge_wrap)
        badge_lay.setContentsMargins(12, 4, 16, 4)
        badge_lay.setSpacing(8)
        badge_lay.addWidget(PulsingDot(BG_DEEP, size=10))
        badge_txt = QLabel('BRAS')
        badge_txt.setStyleSheet(
            f'color: {BG_DEEP}; font-size: 11px; font-weight: 800; letter-spacing: 4px;'
        )
        badge_lay.addWidget(badge_txt)
        hdr.addWidget(badge_wrap)
        hdr.addStretch(1)
        lay.addLayout(hdr)

        # Vitesse
        lay.addWidget(make_section_label('Vitesse'))
        speed_row = QHBoxLayout()
        speed_row.setSpacing(8)
        speed_row.addStretch(1)
        self.speed_indicators = {}
        for key, (val, label) in ARM_SPEED_LEVELS.items():
            ind = KeyIndicator(label)
            self.speed_indicators[key] = ind
            speed_row.addWidget(ind)
        speed_row.addStretch(1)
        lay.addLayout(speed_row)

        self.speed_label = QLabel('Niveau : 8  (50%)')
        self.speed_label.setStyleSheet(
            f'color: {TEXT_DIM}; font-size: 13px; font-weight: 500; letter-spacing: 1px;'
        )
        lay.addWidget(self.speed_label)

        lay.addWidget(self._make_sep())

        # Mouvement rover
        lay.addWidget(make_section_label('Mouvement rover'))
        grid = QGridLayout()
        grid.setSpacing(7)
        grid.setContentsMargins(0, 0, 0, 0)
        self.move_indicators = {}
        for qkey, r, c, lbl in [
            (Qt.Key.Key_Q, 0, 0, 'Q'), (Qt.Key.Key_W, 0, 1, 'W'), (Qt.Key.Key_E, 0, 2, 'E'),
            (Qt.Key.Key_A, 1, 0, 'A'), (Qt.Key.Key_S, 1, 1, 'S'), (Qt.Key.Key_D, 1, 2, 'D'),
            (Qt.Key.Key_Y, 2, 0, 'Y'),                              (Qt.Key.Key_X, 2, 2, 'X'),
        ]:
            ind = KeyIndicator(lbl)
            self.move_indicators[qkey] = ind
            grid.addWidget(ind, r, c)
        grid_holder = QHBoxLayout()
        grid_holder.addLayout(grid)
        grid_holder.addSpacing(12)
        grid_holder.addWidget(self._rover_visual)
        grid_holder.addStretch(1)
        lay.addLayout(grid_holder)

        lay.addWidget(self._make_sep())

        # Contrôle bras
        lay.addWidget(make_section_label('Contrôle bras'))
        lay.addLayout(self._build_arm_controls())

        lay.addWidget(self._make_sep())

        # Bouton VIDAGE
        lay.addWidget(self._build_dump_button())

        lay.addStretch(1)

        back = GlowButton('← TÉLÉOPÉRATION', glow_color=(0, 174, 239), border_radius=10)
        back.setObjectName('ghost_btn')
        back.setCursor(Qt.CursorShape.PointingHandCursor)
        back.clicked.connect(self._on_back)
        lay.addWidget(back)

        return card

    def _make_sep(self) -> QFrame:
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet(f'background: {BORDER}; border: none; max-height: 1px;')
        return sep

    def _build_arm_controls(self) -> QGridLayout:
        """Grille 4 axes × 2 touches + angle + bouton reset (servos uniquement)."""
        g = QGridLayout()
        g.setSpacing(6)
        g.setContentsMargins(0, 0, 0, 0)
        g.setColumnMinimumWidth(0, 92)
        g.setColumnStretch(3, 1)

        self.arm_indicators = {}
        self._axis_labels: dict[str, QLabel] = {}

        # attr → angle accumulé correspondant (None = stepper, pas de reset)
        _SERVO_ANGLE_ATTR = {
            'arm_pince': '_s23_angle',
            'arm_y':     '_s1_angle',
            'bin_dir':   '_s4_angle',
        }

        axes = [
            ('UP/DOWN',    Qt.Key.Key_O,    'O', Qt.Key.Key_K,    'K', 'arm_z',    '↑', '↓'),
            ('CLOSE/OPEN', Qt.Key.Key_J,    'J', Qt.Key.Key_L,    'L', 'arm_pince','◁', '▷'),
            ('FLIP/UNFLIP',Qt.Key.Key_I,    'I', Qt.Key.Key_P,    'P', 'arm_y',    '↺', '↻'),
            ('BENNE',      Qt.Key.Key_Up,   '↑', Qt.Key.Key_Down, '↓', 'bin_dir', '↑', '↓'),
        ]
        for row_i, (axis_name, k_pos, l_pos, k_neg, l_neg, attr, sym_p, sym_n) in enumerate(axes):
            lbl = QLabel(axis_name)
            lbl.setFixedWidth(90)
            lbl.setStyleSheet(
                f'color: {TEXT_MUTED}; font-size: 9px; font-weight: 700; letter-spacing: 2px;'
            )
            g.addWidget(lbl, row_i, 0)

            ind_pos = KeyIndicator(l_pos)
            self.arm_indicators[k_pos] = ind_pos
            g.addWidget(ind_pos, row_i, 1)

            ind_neg = KeyIndicator(l_neg)
            self.arm_indicators[k_neg] = ind_neg
            g.addWidget(ind_neg, row_i, 2)

            status = QLabel('—')
            status.setStyleSheet(
                f'color: {TEXT_MUTED}; font-size: 20px; font-weight: 700; padding-left: 10px;'
            )
            self._axis_labels[attr] = status
            g.addWidget(status, row_i, 3)

            if attr in _SERVO_ANGLE_ATTR:
                angle_attr = _SERVO_ANGLE_ATTR[attr]
                reset_btn = QPushButton('↺ 0')
                reset_btn.setFixedSize(48, 28)
                reset_btn.setCursor(Qt.CursorShape.PointingHandCursor)
                reset_btn.setStyleSheet(
                    f'QPushButton {{'
                    f'  background-color: #1A1A1A; border: 1px solid {BORDER};'
                    f'  border-radius: 8px; font-size: 10px; font-weight: 700;'
                    f'  color: {TEXT_MUTED};'
                    f'}}'
                    f'QPushButton:hover {{'
                    f'  border-color: {PRIMARY}; color: {PRIMARY};'
                    f'}}'
                    f'QPushButton:pressed {{'
                    f'  background-color: {PRIMARY}; color: {BG_DEEP};'
                    f'}}'
                )
                reset_btn.clicked.connect(lambda _, a=angle_attr: self._reset_servo_angle(a))
                g.addWidget(reset_btn, row_i, 4)

        return g

    def _reset_servo_angle(self, angle_attr: str):
        setattr(self, angle_attr, 0.0)
        self._update_arm_status()

    def _build_dump_button(self) -> QPushButton:
        self._dump_btn = QPushButton('⬇  POSITION DE VIDAGE')
        self._dump_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._dump_btn.setFixedHeight(52)
        self._dump_btn.setStyleSheet(
            f'QPushButton {{'
            f'  background-color: #1E1000;'
            f'  border: 1px solid #FF8C00;'
            f'  border-radius: 14px;'
            f'  font-size: 12px; font-weight: 700; letter-spacing: 3px;'
            f'  color: #FF8C00;'
            f'}}'
            f'QPushButton:hover {{'
            f'  background-color: #2E1800;'
            f'  border-color: #FFA500; color: #FFA500;'
            f'}}'
            f'QPushButton:pressed {{'
            f'  background-color: #FF8C00; color: {BG_DEEP};'
            f'}}'
            f'QPushButton:disabled {{'
            f'  background-color: #0E0800;'
            f'  border-color: #553300; color: #553300;'
            f'}}'
        )
        self._dump_btn.clicked.connect(self._on_dump)
        return self._dump_btn

    # ── Construction panneau droit ─────────────────────────────────────────────

    def _build_right_panel(self) -> QWidget:
        card = GlowCard(phase_offset=math.pi + math.pi / 3, glow_color=(0, 174, 239))
        lay = QVBoxLayout(card)
        lay.setContentsMargins(28, 28, 28, 28)
        lay.setSpacing(18)

        hdr = QHBoxLayout()
        title = QLabel('FLUX CAMÉRA  ·  BRAS')
        title.setObjectName('section')
        hdr.addWidget(title)
        hdr.addStretch(1)
        self.fps_label = QLabel('— FPS')
        self.fps_label.setObjectName('status_value')
        hdr.addWidget(self.fps_label)
        lay.addLayout(hdr)

        self.video = QLabel()
        self.video.setObjectName('video')
        self.video.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video.setMinimumSize(320, 240)
        self.video.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.video.setText('En attente du flux caméra…')
        lay.addWidget(self.video, 1)

        self._frame_times: list[float] = []
        return card

    # ── Slots ─────────────────────────────────────────────────────────────────

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

    def _send_cmds(self):
        if self.active_rover_keys:
            linear = sum(MOVEMENT_KEYS[k][0] for k in self.active_rover_keys)
            angular = sum(MOVEMENT_KEYS[k][1] for k in self.active_rover_keys)
            self.linear  = max(-1.0, min(1.0, linear))
            self.angular = max(-1.0, min(1.0, angular))
        else:
            self.linear = self.angular = 0.0

        if self.active_arm_keys:
            for key in self.active_arm_keys:
                z_dir, y_dir, pince_dir, bin_d = ARM_MOVE_KEYS[key]
                if z_dir != 0.0:
                    self.arm_z = z_dir
                if y_dir != 0.0:
                    self.arm_y = y_dir
                    self._s1_angle = max(-100.0, min(100.0,
                        self._s1_angle + y_dir * self.speed * self._SERVO_STEP))
                if pince_dir != 0.0:
                    self.arm_pince = pince_dir
                    self._s23_angle = max(-100.0, min(100.0,
                        self._s23_angle + pince_dir * self.speed * self._SERVO_STEP))
                if bin_d != 0.0:
                    self.bin_dir = bin_d
                    self._s4_angle = max(-100.0, min(100.0,
                        self._s4_angle + bin_d * self.speed * self._SERVO_STEP))
        else:
            self.arm_z = self.arm_y = self.arm_pince = self.bin_dir = 0.0
        self._refresh_arm_keys()

        if self._dump_pending:
            self._s4_angle = 100.0

        lin = self.linear * self.speed
        ang = self.angular * self.speed
        self.bridge.publish_cmd(lin, ang)
        fl = bl = max(-1.0, min(1.0, lin - ang * 0.125))
        fr = br = max(-1.0, min(1.0, lin + ang * 0.125))
        self._rover_visual.update_speeds(fl, fr, bl, br)
        self.bridge.publish_arm_cmd(
            self.arm_z, self._s1_angle, self._s23_angle,
            self.speed,
            dump=self._dump_pending,
            bin_dir=self._s4_angle,
        )
        if self._dump_pending:
            self._dump_pending = False

    def _on_dump(self):
        self._dump_pending = True
        self._dump_btn.setEnabled(False)
        self._dump_btn.setText('⏳  EN COURS…')
        QTimer.singleShot(2000, self._reset_dump_btn)

    def _reset_dump_btn(self):
        self._dump_btn.setEnabled(True)
        self._dump_btn.setText('⬇  POSITION DE VIDAGE')

    # ── Keyboard ──────────────────────────────────────────────────────────────

    def keyPressEvent(self, event: QKeyEvent):
        key = event.key()
        now = time.time()

        # Navigation
        if not event.isAutoRepeat():
            if key == Qt.Key.Key_M:
                self._on_back()
                return
            if key == Qt.Key.Key_Space:
                self.linear = self.angular = 0.0
                self.last_linear_t = self.last_angular_t = 0.0
                self.arm_z = self.arm_y = self.arm_pince = self.bin_dir = 0.0
                self.last_z_t = self.last_y_t = self.last_pince_t = self.last_bin_t = 0.0
                self._s1_angle = self._s23_angle = self._s4_angle = 0.0
                self.active_rover_keys.clear()
                self.active_arm_keys.clear()
                self._refresh_rover_keys()
                self._refresh_arm_keys()
                return
            if key in ARM_SPEED_LEVELS:
                speed_val, label = ARM_SPEED_LEVELS[key]
                self.speed = speed_val
                for k, ind in self.speed_indicators.items():
                    ind.setActive(k == key)
                self.speed_label.setText(f'Niveau : {label}  ({int(speed_val * 100)}%)')
                return

        # Bras + benne : auto-repeat autorisé pour mouvement continu
        if key in ARM_MOVE_KEYS:
            z_dir, y_dir, pince_dir, bin_d = ARM_MOVE_KEYS[key]
            if z_dir != 0.0:
                self.arm_z = z_dir
                self.last_z_t = now
            if y_dir != 0.0:
                self.arm_y = y_dir
                self.last_y_t = now
            if pince_dir != 0.0:
                self.arm_pince = pince_dir
                self.last_pince_t = now
            if bin_d != 0.0:
                self.bin_dir = bin_d
                self.last_bin_t = now
            self.active_arm_keys.add(key)
            self._refresh_arm_keys()
            return

        # Rover : auto-repeat filtré
        if event.isAutoRepeat():
            return
        if key in MOVEMENT_KEYS:
            lin, ang = MOVEMENT_KEYS[key]
            if lin != 0.0:
                self.linear = lin
                self.last_linear_t = now
            if ang != 0.0:
                self.angular = ang
                self.last_angular_t = now
            self.active_rover_keys.add(key)
            self._refresh_rover_keys()

    def keyReleaseEvent(self, event: QKeyEvent):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in ARM_MOVE_KEYS:
            z_dir, y_dir, pince_dir, bin_d = ARM_MOVE_KEYS[key]
            if z_dir != 0.0:
                self.arm_z = 0.0
                self.last_z_t = 0.0
            if y_dir != 0.0:
                self.arm_y = 0.0
                self.last_y_t = 0.0
            if pince_dir != 0.0:
                self.arm_pince = 0.0
                self.last_pince_t = 0.0
            if bin_d != 0.0:
                self.bin_dir = 0.0
                self.last_bin_t = 0.0
            self.active_arm_keys.discard(key)
            self._refresh_arm_keys()
        elif key in MOVEMENT_KEYS:
            self.active_rover_keys.discard(key)
            self._refresh_rover_keys()

    def _refresh_rover_keys(self):
        for k, ind in self.move_indicators.items():
            ind.setActive(k in self.active_rover_keys)

    def _refresh_arm_keys(self):
        for k, ind in self.arm_indicators.items():
            ind.setActive(k in self.active_arm_keys)
        self._update_arm_status()

    def _update_arm_status(self):
        # Stepper (arm_z) : affiche la direction (flèche)
        lbl_z = self._axis_labels['arm_z']
        if self.arm_z > 0.0:
            lbl_z.setText('↑')
            lbl_z.setStyleSheet(f'color: {ACCENT_2}; font-size: 20px; font-weight: 700; padding-left: 10px;')
        elif self.arm_z < 0.0:
            lbl_z.setText('↓')
            lbl_z.setStyleSheet(f'color: {PRIMARY}; font-size: 20px; font-weight: 700; padding-left: 10px;')
        else:
            lbl_z.setText('—')
            lbl_z.setStyleSheet(f'color: {TEXT_MUTED}; font-size: 20px; font-weight: 700; padding-left: 10px;')

        # Servos : affiche l'angle absolu accumulé [-100..100]
        servo_pairs = [
            ('arm_y',     self._s1_angle),
            ('arm_pince', self._s23_angle),
            ('bin_dir',   self._s4_angle),
        ]
        for attr, angle in servo_pairs:
            lbl = self._axis_labels[attr]
            text = f'{angle:+.0f}'
            if angle > 0.0:
                color = ACCENT_2
            elif angle < 0.0:
                color = PRIMARY
            else:
                color = TEXT_MUTED
            lbl.setText(text)
            lbl.setStyleSheet(
                f'color: {color}; font-size: 18px; font-weight: 700; padding-left: 10px;'
            )

    def showEvent(self, event):
        super().showEvent(event)
        for k, ind in self.speed_indicators.items():
            val, _ = ARM_SPEED_LEVELS[k]
            ind.setActive(abs(val - self.speed) < 1e-3)
        self.setFocus()


# ══════════════════════════════════════════════════════════════════════════════
# FENÊTRE PRINCIPALE
# ══════════════════════════════════════════════════════════════════════════════

class MainWindow(QMainWindow):
    def __init__(self, bridge: RosBridge):
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle('XPlore — Rover Control')
        screen = QApplication.primaryScreen().availableGeometry()
        w = min(1320, screen.width())
        h = min(840, screen.height())
        self.resize(w, h)
        self.setMaximumHeight(h)

        central = QWidget()
        central.setObjectName('root')
        self.setCentralWidget(central)

        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.stack = _AdaptiveStack()
        layout.addWidget(self.stack, 1)
        layout.addWidget(self._build_footer())

        self.page_menu      = MenuPage(on_autonomous=self.show_autonomous, on_teleop=self.show_teleop_menu, on_quit=self.close)
        self.page_teleop    = TeleopMenuPage(on_race=self.show_race, on_arm=self.show_arm, on_back=self.show_menu)
        self.page_race      = RacePage(bridge, on_back=self.show_teleop_menu)
        self.page_autonomous = AutonomousPage(bridge, on_back=self.show_menu)
        self.page_arm       = ArmPage(bridge, on_back=self.show_teleop_menu)

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
