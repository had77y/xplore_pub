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
            msg.linear.x = self.linear
            msg.angular.z = self.angular
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


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    settings = termios.tcgetattr(sys.stdin)

    print('\n' + '═' * 48)
    print('   XPLORE ROVER — Téléopération')
    print('═' * 48)
    print('   ↑            →  avancer')
    print('   ↓            →  reculer')
    print('   ←            →  pivot gauche')
    print('   →            →  pivot droite')
    print('   W            →  arc avant gauche')
    print('   E            →  arc avant droite')
    print('   A            →  arc arrière gauche')
    print('   D            →  arc arrière droite')
    print('   Espace       →  stop immédiat')
    print('   Q            →  quitter')
    print('═' * 48 + '\n')

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.05)

            if key is None:
                continue
            elif key == '\x1b[A':        # ↑
                node.set_axes(linear=1.0)
            elif key == '\x1b[B':        # ↓
                node.set_axes(linear=-1.0)
            elif key == '\x1b[D':        # ←
                node.set_axes(angular=1.0)
            elif key == '\x1b[C':        # →
                node.set_axes(angular=-1.0)
            elif key.lower() == 'w':     # arc avant gauche
                node.set_axes(linear=1.0, angular=1.0)
            elif key.lower() == 'e':     # arc avant droite
                node.set_axes(linear=1.0, angular=-1.0)
            elif key.lower() == 'a':     # arc arrière gauche
                node.set_axes(linear=-1.0, angular=1.0)
            elif key.lower() == 'd':     # arc arrière droite
                node.set_axes(linear=-1.0, angular=-1.0)
            elif key == ' ':             # stop
                node.stop()
            elif key.lower() == 'q':
                node.stop()
                print('\nArrêt téléopération.')
                break

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
