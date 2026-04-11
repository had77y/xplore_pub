import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import threading
import time
import select


# Délai avant qu'un axe soit remis à zéro si aucune répétition reçue (secondes)
AXIS_TIMEOUT = 0.20


class TeleopNode(Node):
    """
    Nœud côté PC — téléopération du rover via les flèches du clavier.
    Comportement hold-to-move : maintenir la touche = avance, relâcher = stop.
    Chaque axe (linear / angular) a son propre timeout indépendant.
    linear.x  = avant/arrière
    angular.z = rotation gauche/droite
    """

    def __init__(self):
        super().__init__('teleop_node')

        self.publisher = self.create_publisher(Twist, '/rover/cmd_vel', 10)

        self.linear = 0.0
        self.angular = 0.0

        # Timestamp de la dernière touche reçue par axe
        self.last_linear_time = 0.0
        self.last_angular_time = 0.0

        self.lock = threading.Lock()

        # Envoie les commandes toutes les 100ms et gère le timeout des axes
        self.timer = self.create_timer(0.1, self.send_command)

        self.get_logger().info('teleop_node démarré')

    def send_command(self):
        """Publie Twist. Remet un axe à 0 si aucune répétition reçue depuis AXIS_TIMEOUT."""
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

    def set_linear(self, value):
        with self.lock:
            self.linear = value
            self.last_linear_time = time.time()

    def set_angular(self, value):
        with self.lock:
            self.angular = value
            self.last_angular_time = time.time()

    def stop(self):
        with self.lock:
            self.linear = 0.0
            self.angular = 0.0
            self.last_linear_time = 0.0
            self.last_angular_time = 0.0


def read_key_sequence(settings):
    """Lit une séquence clavier (non-bloquant). Retourne None si rien disponible."""
    rlist, _, _ = select.select([sys.stdin], [], [], 0)
    if not rlist:
        return None
    key = sys.stdin.read(1)
    if key == '\x1b':
        key += sys.stdin.read(2)  # flèche = \x1b[A/B/C/D
    return key


def get_all_keys(settings, timeout=0.05):
    """
    Attend jusqu'à `timeout` secondes la première touche,
    puis draine toutes les touches disponibles dans le buffer.
    Retourne une liste (vide si aucune touche).
    """
    tty.setraw(sys.stdin.fileno())
    keys = []

    # Attendre la première touche
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
        keys.append(key)

        # Drainer le reste du buffer (non-bloquant)
        while True:
            key = read_key_sequence(settings)
            if key is None:
                break
            keys.append(key)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return keys


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    # spin ROS dans un thread séparé — le thread principal gère le clavier
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    settings = termios.tcgetattr(sys.stdin)

    print('\n' + '═' * 42)
    print('   XPLORE ROVER — Téléopération')
    print('═' * 42)
    print('   ↑  (maintenir)  →  avancer')
    print('   ↓  (maintenir)  →  reculer')
    print('   ←  (maintenir)  →  tourner gauche')
    print('   →  (maintenir)  →  tourner droite')
    print('   ↑ + ←           →  virage gauche en arc')
    print('   ←  seul          →  pivot sur place')
    print('   Espace           →  stop immédiat')
    print('   Q                →  quitter')
    print('═' * 42 + '\n')

    try:
        while rclpy.ok():
            keys = get_all_keys(settings, timeout=0.05)

            for key in keys:
                if key == '\x1b[A':    # flèche haut
                    node.set_linear(1.0)
                elif key == '\x1b[B':    # flèche bas
                    node.set_linear(-1.0)
                elif key == '\x1b[C':    # flèche droite
                    node.set_angular(-1.0)
                elif key == '\x1b[D':    # flèche gauche
                    node.set_angular(1.0)
                elif key == ' ':         # espace = stop immédiat
                    node.stop()
                elif key.lower() == 'q':
                    node.stop()
                    print('\nArrêt téléopération.')
                    rclpy.shutdown()
                    break

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
