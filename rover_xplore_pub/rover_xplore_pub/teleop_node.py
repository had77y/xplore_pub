import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import threading


class TeleopNode(Node):
    """
    Nœud côté PC — téléopération du rover via les flèches du clavier.
    Publie des commandes Twist sur /rover/cmd_vel toutes les 100ms.
    linear.x  = avant/arrière
    angular.z = rotation gauche/droite
    """

    def __init__(self):
        super().__init__('teleop_node')

        self.publisher = self.create_publisher(Twist, '/rover/cmd_vel', 10)

        # État courant des commandes
        self.linear = 0.0
        self.angular = 0.0
        self.lock = threading.Lock()

        # Envoie les commandes en continu toutes les 100ms
        self.timer = self.create_timer(0.1, self.send_command)

        self.get_logger().info('teleop_node démarré')

    def send_command(self):
        """Publie la commande Twist avec l'état actuel."""
        msg = Twist()
        with self.lock:
            msg.linear.x = self.linear
            msg.angular.z = self.angular
        self.publisher.publish(msg)

    def set_velocity(self, linear=None, angular=None):
        """Met à jour les valeurs de vitesse de façon thread-safe."""
        with self.lock:
            if linear is not None:
                self.linear = linear
            if angular is not None:
                self.angular = angular

    def stop(self):
        """Stoppe le rover — remet tout à zéro."""
        with self.lock:
            self.linear = 0.0
            self.angular = 0.0


def get_key(settings):
    """Lit une touche clavier (gère les flèches = séquences de 3 caractères)."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == '\x1b':
        key += sys.stdin.read(2)  # flèche = 3 caractères : \x1b[A/B/C/D
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


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
    print('   ↑        →  avancer')
    print('   ↓        →  reculer')
    print('   ←        →  tourner gauche')
    print('   →        →  tourner droite')
    print('   Espace   →  stop')
    print('   Q        →  quitter')
    print('═' * 42)
    print('   ↑ puis ← = virage gauche en arc')
    print('   ← seul   = pivot sur place')
    print('═' * 42 + '\n')

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key == '\x1b[A':      # flèche haut
                node.set_velocity(linear=1.0)
            elif key == '\x1b[B':    # flèche bas
                node.set_velocity(linear=-1.0)
            elif key == '\x1b[C':    # flèche droite
                node.set_velocity(angular=-1.0)
            elif key == '\x1b[D':    # flèche gauche
                node.set_velocity(angular=1.0)
            elif key == ' ':         # espace = stop
                node.stop()
                print('[STOP]')
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
