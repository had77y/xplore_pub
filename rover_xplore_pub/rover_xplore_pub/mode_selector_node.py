import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import tty
import termios
import threading


class ModeSelectorNode(Node):
    """
    Nœud côté PC — sélection du mode de fonctionnement du rover.
    L'opérateur choisit le mode avant le démarrage du défi.
    Publie le mode choisi sur /rover/mode.
    """

    def __init__(self):
        super().__init__('mode_selector_node')

        self.publisher = self.create_publisher(String, '/rover/mode', 10)
        self.current_mode = None

        self.get_logger().info('mode_selector_node démarré')

    def publish_mode(self, mode):
        """Publie le mode choisi sur /rover/mode."""
        msg = String()
        msg.data = mode
        self.publisher.publish(msg)
        self.current_mode = mode
        self.get_logger().info(f'[MODE ENVOYÉ] → {mode}')


def get_key(settings):
    """Lit une touche clavier sans attendre Entrée (mode raw terminal)."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = ModeSelectorNode()

    # spin ROS dans un thread séparé — le thread principal gère le clavier
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Sauvegarde les paramètres du terminal pour les restaurer à la fin
    settings = termios.tcgetattr(sys.stdin)

    print('\n' + '═' * 42)
    print('   XPLORE ROVER — Sélection du mode')
    print('═' * 42)
    print('   A  →  mode AUTONOME')
    print('   C  →  mode COMMANDÉ (téléop)')
    print('   Q  →  quitter')
    print('═' * 42 + '\n')

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key.lower() == 'a':
                node.publish_mode('autonomous')
            elif key.lower() == 'c':
                node.publish_mode('teleop')
            elif key.lower() == 'q':
                print('\nArrêt du sélecteur de mode.')
                break

    finally:
        # Restaure le terminal dans tous les cas (même en cas d'erreur)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
