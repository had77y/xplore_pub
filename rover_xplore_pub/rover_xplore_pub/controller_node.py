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


AXIS_TIMEOUT = 0.20

SPEED_LEVELS = {
    '8': 0.33,
    '9': 0.66,
    '0': 1.0,
}

STATE_MENU_MAIN   = 'menu_main'
STATE_MENU_TELEOP = 'menu_teleop'
STATE_AUTONOMOUS  = 'autonomous'
STATE_RACE        = 'race'
STATE_ARM         = 'arm'


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        self.mode_publisher = self.create_publisher(String, '/rover/mode', 10)
        self.cmd_publisher  = self.create_publisher(Twist, '/rover/cmd_vel', 10)

        self.state   = STATE_MENU_MAIN
        self.linear  = 0.0
        self.angular = 0.0
        self.last_linear_time  = 0.0
        self.last_angular_time = 0.0
        self.speed   = 1.0

        self.lock = threading.Lock()

        self.timer = self.create_timer(0.1, self.send_command)

        self.get_logger().info('controller_node démarré')

    # ── Publication Twist (uniquement en mode race) ──────────────────────────

    def send_command(self):
        if self.state != STATE_RACE:
            return
        now = time.time()
        msg = Twist()
        with self.lock:
            if now - self.last_linear_time > AXIS_TIMEOUT:
                self.linear = 0.0
            if now - self.last_angular_time > AXIS_TIMEOUT:
                self.angular = 0.0
            msg.linear.x  = self.linear  * self.speed
            msg.angular.z = self.angular * self.speed
        self.cmd_publisher.publish(msg)

    def set_axes(self, linear=None, angular=None):
        now = time.time()
        with self.lock:
            if linear is not None:
                self.linear = linear
                self.last_linear_time = now
            if angular is not None:
                self.angular = angular
                self.last_angular_time = now

    def set_speed(self, speed):
        with self.lock:
            self.speed = speed

    def stop_motors(self):
        with self.lock:
            self.linear = 0.0
            self.angular = 0.0
            self.last_linear_time  = 0.0
            self.last_angular_time = 0.0

    # ── Publication mode ─────────────────────────────────────────────────────

    def publish_mode(self, mode):
        msg = String()
        msg.data = mode
        self.mode_publisher.publish(msg)


# ── Affichage ─────────────────────────────────────────────────────────────────

def clear():
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
    level = {0.33: '8', 0.66: '9', 1.0: '0'}.get(speed, '?')
    bar = '█' * int(speed * 9)
    print(f'\r   Vitesse : [{bar:<9}] niveau {level}   ', end='', flush=True)


# ── Lecture clavier ───────────────────────────────────────────────────────────

def get_key(settings, timeout=0.05):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
    else:
        key = None
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# ── Main ──────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    settings = termios.tcgetattr(sys.stdin)
    print_menu_main()

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.05)
            if key is None:
                continue

            state = node.state

            # ── Menu principal ────────────────────────────────────────────
            if state == STATE_MENU_MAIN:
                if key.lower() == 'a':
                    node.state = STATE_AUTONOMOUS
                    node.publish_mode('autonomous')
                    print_autonomous()
                elif key.lower() == 't':
                    node.state = STATE_MENU_TELEOP
                    print_menu_teleop()
                elif key.lower() == 'p':
                    print('\nAu revoir.')
                    break

            # ── Sous-menu téléop ──────────────────────────────────────────
            elif state == STATE_MENU_TELEOP:
                if key.lower() == 'r':
                    node.state = STATE_RACE
                    node.publish_mode('race')
                    print_race(node.speed)
                elif key.lower() == 'b':
                    node.state = STATE_ARM
                    node.publish_mode('arm')
                    print_arm()
                elif key.lower() == 'm':
                    node.state = STATE_MENU_MAIN
                    print_menu_main()

            # ── Autonome ──────────────────────────────────────────────────
            elif state == STATE_AUTONOMOUS:
                if key.lower() == 'm':
                    node.state = STATE_MENU_MAIN
                    node.publish_mode('idle')
                    print_menu_main()

            # ── Race ──────────────────────────────────────────────────────
            elif state == STATE_RACE:
                if key.lower() == 'm':
                    node.stop_motors()
                    node.state = STATE_MENU_MAIN
                    node.publish_mode('idle')
                    print_menu_main()
                elif key == ' ':
                    node.stop_motors()
                elif key.lower() == 'w':
                    node.set_axes(linear=1.0)
                elif key.lower() == 's':
                    node.set_axes(linear=-1.0)
                elif key.lower() == 'a':
                    node.set_axes(angular=1.0)
                elif key.lower() == 'd':
                    node.set_axes(angular=-1.0)
                elif key.lower() == 'q':
                    node.set_axes(linear=1.0, angular=1.0)
                elif key.lower() == 'e':
                    node.set_axes(linear=1.0, angular=-1.0)
                elif key.lower() == 'y':
                    node.set_axes(linear=-1.0, angular=1.0)
                elif key.lower() == 'x':
                    node.set_axes(linear=-1.0, angular=-1.0)
                elif key in SPEED_LEVELS:
                    node.set_speed(SPEED_LEVELS[key])
                    print_speed(SPEED_LEVELS[key])

            # ── Bras ──────────────────────────────────────────────────────
            elif state == STATE_ARM:
                if key.lower() == 'm':
                    node.state = STATE_MENU_MAIN
                    node.publish_mode('idle')
                    print_menu_main()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
