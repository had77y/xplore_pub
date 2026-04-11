import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import threading
import time
import select


AXIS_TIMEOUT = 0.20

SPEED_LEVELS = {
    '1': 0.33,
    '2': 0.66,
    '3': 1.0,
}


class TeleopNode(Node):
    """
    Nœud côté PC — téléopération du rover via le clavier.
    Comportement hold-to-move : maintenir la touche = actif, relâcher = stop.
    linear.x  = avant/arrière
    angular.z = rotation gauche/droite
    """

    def __init__(self):
        super().__init__('teleop_node')

        self.publisher = self.create_publisher(Twist, '/rover/cmd_vel', 10)

        self.linear = 0.0
        self.angular = 0.0
        self.last_linear_time = 0.0
        self.last_angular_time = 0.0
        self.speed = 1.0

        self.lock = threading.Lock()

        self.timer = self.create_timer(0.1, self.send_command)

        self.get_logger().info('teleop_node démarré')

    def send_command(self):
        now = time.time()
        msg = Twist()
        with self.lock:
            if now - self.last_linear_time > AXIS_TIMEOUT:
                self.linear = 0.0
            if now - self.last_angular_time > AXIS_TIMEOUT:
                self.angular = 0.0
            msg.linear.x = self.linear * self.speed
            msg.angular.z = self.angular * self.speed
        self.publisher.publish(msg)

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

    def stop(self):
        with self.lock:
            self.linear = 0.0
            self.angular = 0.0
            self.last_linear_time = 0.0
            self.last_angular_time = 0.0


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


def print_status(speed):
    level = {0.33: '1', 0.66: '2', 1.0: '3'}.get(speed, '?')
    bar = '█' * int(speed * 9)
    print(f'\r   Vitesse : [{bar:<9}] niveau {level}   ', end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    settings = termios.tcgetattr(sys.stdin)

    print('\n' + '═' * 48)
    print('   XPLORE ROVER — Téléopération')
    print('═' * 48)
    print('   W            →  avancer')
    print('   S            →  reculer')
    print('   A            →  pivot gauche')
    print('   D            →  pivot droite')
    print('   Q            →  arc avant gauche')
    print('   E            →  arc avant droite')
    print('   Y            →  arc arrière gauche')
    print('   X            →  arc arrière droite')
    print('   1 / 2 / 3    →  vitesse lente / moyenne / pleine')
    print('   Espace       →  stop immédiat')
    print('   P            →  quitter')
    print('═' * 48)
    print_status(node.speed)

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.05)

            if key is None:
                continue
            elif key.lower() == 'w':     # avancer
                node.set_axes(linear=1.0)
            elif key.lower() == 's':     # reculer
                node.set_axes(linear=-1.0)
            elif key.lower() == 'a':     # pivot gauche
                node.set_axes(angular=1.0)
            elif key.lower() == 'd':     # pivot droite
                node.set_axes(angular=-1.0)
            elif key.lower() == 'q':     # arc avant gauche
                node.set_axes(linear=1.0, angular=1.0)
            elif key.lower() == 'e':     # arc avant droite
                node.set_axes(linear=1.0, angular=-1.0)
            elif key.lower() == 'y':     # arc arrière gauche
                node.set_axes(linear=-1.0, angular=1.0)
            elif key.lower() == 'x':     # arc arrière droite
                node.set_axes(linear=-1.0, angular=-1.0)
            elif key in SPEED_LEVELS:
                node.set_speed(SPEED_LEVELS[key])
                print_status(SPEED_LEVELS[key])
            elif key == ' ':             # stop
                node.stop()
            elif key.lower() == 'p':
                node.stop()
                print('\n\nArrêt téléopération.')
                break

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
