# ══════════════════════════════════════════════════════════════════════════════
# controller_node.py — PC (terminal, sans GUI)
#
# RÔLE : interface clavier en terminal pour contrôler le rover.
#         C'est l'alternative légère à rover_gui.py (pas de fenêtre graphique).
#         L'opérateur navigue dans des menus et utilise WASD pour piloter.
#
# PLACE DANS LE SYSTÈME :
#
#   [PC] controller_node ← CE FICHIER (l'opérateur appuie sur les touches)
#         ├─ publie /rover/mode    → [RPi] mode_manager_node, camera_node,
#         │                                aruco_node, motor_controller_node
#         └─ publie /rover/cmd_vel → [RPi] motor_controller_node → Arduino → roues
#
# TOPICS PUBLIÉS :
#   /rover/mode    (std_msgs/String)
#     → "autonomous" quand l'opérateur appuie sur A au menu principal
#     → "race"       quand l'opérateur choisit Race en téléop
#     → "arm"        quand l'opérateur choisit Bras en téléop
#     → "idle"       quand l'opérateur appuie M pour revenir au menu
#
#   /rover/cmd_vel (geometry_msgs/Twist)
#     → publié toutes les 100ms SEULEMENT en mode race
#     → linear.x  = vitesse avant/arrière (appliquée par W/S)
#     → angular.z = vitesse de rotation   (appliquée par A/D)
#     → le speed multiplicateur (8/9/0) est appliqué ici avant publication
#
# ÉTATS DE L'APPLICATION (machine à états interne) :
#   menu_main   → menu principal (choisir autonome ou téléop)
#   menu_teleop → sous-menu téléop (choisir race ou bras)
#   autonomous  → mode autonome actif (une seule touche : M pour revenir)
#   race        → téléop FPV (WASD + touches de vitesse)
#   arm         → mode bras (contrôles à implémenter)
#
# THREADING :
#   ROS2 (spin) tourne dans un thread séparé (daemon thread).
#   Le thread principal gère la lecture clavier.
#   Un Lock protège les variables partagées (linear, angular, speed).
#
# MÉCANISME HOLD-TO-MOVE :
#   Les axes (linear, angular) sont mis à zéro automatiquement
#   si aucune touche n'est pressée depuis AXIS_TIMEOUT=200ms.
#   En pratique : maintenir W enfoncé pour avancer (key repeat OS ~30ms).
#   Relâcher W → 200ms plus tard → linear=0 → le rover s'arrête.
# ══════════════════════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys
import tty
import termios
import threading
import time
import select
import subprocess

# Timeout en secondes : si aucune touche d'axe n'est pressée depuis ce délai → axe = 0
# Key repeat OS ≈ 30ms → une touche maintenue envoie des events toutes les 30ms
# AXIS_TIMEOUT = 200ms > 30ms : garde le mouvement tant que la touche est enfoncée
AXIS_TIMEOUT = 0.20

# Multiplicateurs de vitesse (appliqués aux valeurs Twist avant publication)
SPEED_LEVELS = {
    '8': 0.33,  # lent
    '9': 0.66,  # moyen
    '0': 1.0,   # pleine vitesse
}

# Noms des états de l'application
STATE_MENU_MAIN   = 'menu_main'
STATE_MENU_TELEOP = 'menu_teleop'
STATE_AUTONOMOUS  = 'autonomous'
STATE_RACE        = 'race'
STATE_ARM         = 'arm'


class ControllerNode(Node):
    """
    Nœud ROS2 responsable de la publication des commandes vers le rover.
    La logique clavier est dans la fonction main() — ce node gère uniquement
    la publication ROS2 et l'état des axes.
    """

    def __init__(self):
        super().__init__('controller_node')

        # ── Publishers ────────────────────────────────────────────────────────
        # /rover/mode : envoyé quand l'état de l'application change (menu → race, etc.)
        self.mode_publisher = self.create_publisher(String, '/rover/mode', 10)
        # /rover/cmd_vel : envoyé toutes les 100ms en mode race
        self.cmd_publisher  = self.create_publisher(Twist,  '/rover/cmd_vel', 10)

        # ── État courant ──────────────────────────────────────────────────────
        self.state   = STATE_MENU_MAIN  # l'application démarre au menu principal

        # Valeurs des axes (normalisées : -1.0 à +1.0)
        # Multipliées par self.speed avant publication
        self.linear  = 0.0
        self.angular = 0.0

        # Timestamps de la dernière pression de touche par axe
        # Utilisés par le mécanisme AXIS_TIMEOUT pour remettre l'axe à 0
        self.last_linear_time  = 0.0
        self.last_angular_time = 0.0

        self.speed = 1.0  # multiplicateur de vitesse (0.33 / 0.66 / 1.0)

        # Lock pour protéger les variables partagées entre le thread clavier et le thread ROS
        self.lock = threading.Lock()

        # Sous-processus du video_viewer_node (lance une fenêtre de prévisualisation caméra)
        self._viewer_proc = None

        # Timer ROS2 : publie /rover/cmd_vel toutes les 100ms en mode race
        # Appelé dans le thread ROS (spin), pas dans le thread clavier
        self.timer = self.create_timer(0.1, self.send_command)

        self.get_logger().info('controller_node démarré')

    # ── Publication des commandes motrices ────────────────────────────────────

    def send_command(self):
        """
        Appelée toutes les 100ms par le timer ROS2 (dans le thread spin).
        Publie le Twist courant sur /rover/cmd_vel, SEULEMENT en mode race.
        Applique AXIS_TIMEOUT : si une touche n'a pas été pressée depuis 200ms → axe = 0.
        Le multiplicateur de vitesse (self.speed) est appliqué ici.
        """
        if self.state != STATE_RACE:
            return  # ne publie rien hors mode race

        now = time.time()
        msg = Twist()
        with self.lock:
            # Expiration automatique des axes : "relâcher" une touche
            if now - self.last_linear_time > AXIS_TIMEOUT:
                self.linear = 0.0
            if now - self.last_angular_time > AXIS_TIMEOUT:
                self.angular = 0.0
            msg.linear.x  = self.linear  * self.speed  # ← multiplicateur vitesse
            msg.angular.z = self.angular * self.speed
        self.cmd_publisher.publish(msg)
        # → reçu par motor_controller_node (RPi) → cinématique différentielle → Arduino

    def set_axes(self, linear=None, angular=None):
        """
        Met à jour un ou deux axes et enregistre le timestamp.
        Appelée depuis le thread clavier quand une touche est détectée.
        Le Lock garantit qu'on ne lit pas des valeurs à moitié mises à jour
        pendant que send_command() les utilise dans l'autre thread.
        """
        now = time.time()
        with self.lock:
            if linear is not None:
                self.linear            = linear
                self.last_linear_time  = now
            if angular is not None:
                self.angular           = angular
                self.last_angular_time = now

    def set_speed(self, speed):
        """Met à jour le multiplicateur de vitesse (thread-safe)."""
        with self.lock:
            self.speed = speed

    def stop_motors(self):
        """
        Remet les deux axes à 0 et expire les timestamps.
        Appelée quand l'opérateur appuie sur Espace ou quitte un mode.
        Les timestamps à 0 font expirer le timeout immédiatement → Twist(0,0) publié.
        """
        with self.lock:
            self.linear            = 0.0
            self.angular           = 0.0
            self.last_linear_time  = 0.0
            self.last_angular_time = 0.0

    # ── Publication du mode ───────────────────────────────────────────────────

    def publish_mode(self, mode):
        """
        Publie le mode sur /rover/mode.
        Reçu par : mode_manager_node, camera_node, aruco_node, motor_controller_node
        sur le RPi (via WiFi → DDS).
        """
        msg      = String()
        msg.data = mode
        self.mode_publisher.publish(msg)

    # ── Viewer FPV ────────────────────────────────────────────────────────────

    def start_viewer(self):
        """
        Lance video_viewer_node dans un sous-processus (fenêtre OpenCV séparée).
        Vérifie d'abord si le viewer n'est pas déjà en cours (poll() None = en vie).
        """
        if self._viewer_proc is None or self._viewer_proc.poll() is not None:
            self._viewer_proc = subprocess.Popen(
                ['ros2', 'run', 'rover_xplore_pub', 'video_viewer_node']
            )

    def stop_viewer(self):
        """Termine le sous-processus viewer si il tourne."""
        if self._viewer_proc and self._viewer_proc.poll() is None:
            self._viewer_proc.terminate()
            self._viewer_proc = None


# ── Fonctions d'affichage terminal ────────────────────────────────────────────

def clear():
    """Efface le terminal (séquence ANSI : effacer écran + curseur en haut)."""
    print('\033[2J\033[H', end='', flush=True)

def print_menu_main():
    clear()
    print('═' * 44)
    print('   XPLORE ROVER — Menu principal')
    print('═' * 44)
    print('   A   →  Mode autonome')
    print('   T   →  Mode commandé (téléop)')
    print('   P   →  Quitter')
    print('═' * 44 + '\n')

def print_menu_teleop():
    clear()
    print('═' * 44)
    print('   XPLORE ROVER — Mode commandé')
    print('═' * 44)
    print('   R   →  Race  (FPV)')
    print('   B   →  Bras  (ramassage)')
    print('   M   →  Retour menu principal')
    print('═' * 44 + '\n')

def print_autonomous():
    clear()
    print('═' * 44)
    print('   XPLORE ROVER — Mode autonome')
    print('═' * 44)
    print('   Le rover navigue seul.')
    print('   M   →  Retour menu principal')
    print('═' * 44 + '\n')

def print_race(speed):
    clear()
    print('═' * 44)
    print('   XPLORE ROVER — Race (FPV)')
    print('═' * 44)
    print('   W          →  avancer')
    print('   S          →  reculer')
    print('   A          →  pivot gauche')
    print('   D          →  pivot droite')
    print('   Q          →  arc avant gauche')
    print('   E          →  arc avant droite')
    print('   Y          →  arc arrière gauche')
    print('   X          →  arc arrière droite')
    print('   8 / 9 / 0  →  vitesse lente / moyenne / pleine')
    print('   Espace     →  stop moteurs')
    print('   M          →  retour menu')
    print('═' * 44)
    print_speed(speed)

def print_arm():
    clear()
    print('═' * 44)
    print('   XPLORE ROVER — Bras (ramassage)')
    print('═' * 44)
    print('   (contrôles bras à implémenter)')
    print('   M   →  Retour menu principal')
    print('═' * 44 + '\n')

def print_speed(speed):
    """Affiche une barre de vitesse sur la même ligne (overwrite avec \r)."""
    level = {0.33: '8', 0.66: '9', 1.0: '0'}.get(speed, '?')
    bar   = '█' * int(speed * 9)
    print(f'\r   Vitesse : [{bar:<9}] niveau {level}   ', end='', flush=True)


# ── Lecture clavier non-bloquante ─────────────────────────────────────────────

def get_key(settings, timeout=0.05):
    """
    Lit une touche clavier sans bloquer plus de `timeout` secondes.
    Utilise le mode "raw" du terminal : chaque touche est lue immédiatement
    sans attendre Entrée.

    settings : paramètres terminal originaux (pour restaurer après)
    timeout  : délai max d'attente en secondes (0.05 = 50ms)
    Retourne : la touche pressée (str) ou None si timeout
    """
    tty.setraw(sys.stdin.fileno())  # mode raw : pas d'écho, pas d'attente Entrée
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            # Touche spéciale (flèches) : code ESC + 2 caractères supplémentaires
            key += sys.stdin.read(2)
    else:
        key = None  # timeout atteint sans touche
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # restaure le terminal
    return key


# ── Point d'entrée ────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    # Thread daemon : le thread spin ROS2 ne bloque pas la fermeture du programme
    # quand le thread principal se termine (Ctrl+C ou 'P')
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Sauvegarde des paramètres terminal originaux pour les restaurer à la fin
    settings = termios.tcgetattr(sys.stdin)
    print_menu_main()

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.05)
            if key is None:
                continue  # timeout → pas de touche → recommencer

            state = node.state

            # ── Menu principal ────────────────────────────────────────────────
            if state == STATE_MENU_MAIN:
                if key.lower() == 'a':
                    node.state = STATE_AUTONOMOUS
                    node.publish_mode('autonomous')
                    # → RPi : mode_manager_node log "MODE AUTONOME"
                    #         camera_node active le flux
                    #         aruco_node s'abonne à /camera/image_raw
                    print_autonomous()

                elif key.lower() == 't':
                    node.state = STATE_MENU_TELEOP
                    print_menu_teleop()

                elif key.lower() == 'p':
                    print('\nAu revoir.')
                    break

            # ── Sous-menu téléop ──────────────────────────────────────────────
            elif state == STATE_MENU_TELEOP:
                if key.lower() == 'r':
                    node.state = STATE_RACE
                    node.publish_mode('race')
                    # → RPi : motor_controller_node autorise /rover/cmd_vel
                    #         camera_node active le flux JPEG vers le PC
                    node.start_viewer()  # lance video_viewer_node (fenêtre caméra)
                    print_race(node.speed)

                elif key.lower() == 'b':
                    node.state = STATE_ARM
                    node.publish_mode('arm')
                    # → RPi : mode_manager_node log "MODE BRAS"
                    #         camera_node active le flux JPEG (pour voir ce qu'on attrape)
                    print_arm()

                elif key.lower() == 'm':
                    node.state = STATE_MENU_MAIN
                    print_menu_main()

            # ── Mode autonome ─────────────────────────────────────────────────
            elif state == STATE_AUTONOMOUS:
                if key.lower() == 'm':
                    node.state = STATE_MENU_MAIN
                    node.publish_mode('idle')
                    # → RPi : tous les actionneurs s'arrêtent
                    print_menu_main()

            # ── Mode race (téléopération FPV) ─────────────────────────────────
            elif state == STATE_RACE:
                if key.lower() == 'm':
                    node.stop_motors()  # stoppe les moteurs avant de changer de mode
                    node.stop_viewer()  # ferme la fenêtre caméra
                    node.state = STATE_MENU_MAIN
                    node.publish_mode('idle')
                    print_menu_main()
                elif key == ' ':
                    node.stop_motors()  # Espace = stop d'urgence
                # Commandes de déplacement
                # set_axes() met à jour linear/angular + le timestamp AXIS_TIMEOUT
                # send_command() (timer 100ms) publiera ces valeurs × speed sur /rover/cmd_vel
                elif key.lower() == 'w': node.set_axes(linear= 1.0)
                elif key.lower() == 's': node.set_axes(linear=-1.0)
                elif key.lower() == 'a': node.set_axes(angular= 1.0)
                elif key.lower() == 'd': node.set_axes(angular=-1.0)
                elif key.lower() == 'q': node.set_axes(linear= 1.0, angular= 1.0)  # arc avant gauche
                elif key.lower() == 'e': node.set_axes(linear= 1.0, angular=-1.0)  # arc avant droite
                elif key.lower() == 'y': node.set_axes(linear=-1.0, angular= 1.0)  # arc arrière gauche
                elif key.lower() == 'x': node.set_axes(linear=-1.0, angular=-1.0)  # arc arrière droite
                elif key in SPEED_LEVELS:
                    node.set_speed(SPEED_LEVELS[key])
                    print_speed(SPEED_LEVELS[key])

            # ── Mode bras ─────────────────────────────────────────────────────
            elif state == STATE_ARM:
                if key.lower() == 'm':
                    node.state = STATE_MENU_MAIN
                    node.publish_mode('idle')
                    print_menu_main()

    finally:
        # Nettoyage dans TOUS les cas (Ctrl+C, exception, quitter normalement)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # restaure le terminal
        node.stop_motors()
        node.stop_viewer()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
