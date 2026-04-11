import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
from pynput import keyboard


class TeleopNode(Node):
    """
    Nœud côté PC — téléopération du rover via les flèches du clavier.
    Comportement hold-to-move : press = actif, release = stop.
    Gère plusieurs touches simultanées (ex: avancer + tourner).
    linear.x  = avant/arrière
    angular.z = rotation gauche/droite
    """

    def __init__(self):
        super().__init__('teleop_node')

        self.publisher = self.create_publisher(Twist, '/rover/cmd_vel', 10)

        # Touches actuellement enfoncées
        self.keys_pressed = set()
        self.lock = threading.Lock()
        self.running = True

        # Envoie les commandes toutes les 100ms
        self.timer = self.create_timer(0.1, self.send_command)

        self.get_logger().info('teleop_node démarré')

    def send_command(self):
        """Publie Twist selon les touches actuellement enfoncées."""
        msg = Twist()
        with self.lock:
            if keyboard.Key.up in self.keys_pressed:
                msg.linear.x += 1.0
            if keyboard.Key.down in self.keys_pressed:
                msg.linear.x -= 1.0
            if keyboard.Key.left in self.keys_pressed:
                msg.angular.z += 1.0
            if keyboard.Key.right in self.keys_pressed:
                msg.angular.z -= 1.0
        self.publisher.publish(msg)

    def on_press(self, key):
        if key == keyboard.Key.space:
            with self.lock:
                self.keys_pressed.clear()
            return
        if key == keyboard.KeyCode.from_char('q'):
            self.running = False
            return False  # arrête le listener
        with self.lock:
            self.keys_pressed.add(key)

    def on_release(self, key):
        with self.lock:
            self.keys_pressed.discard(key)

    def stop(self):
        with self.lock:
            self.keys_pressed.clear()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    # spin ROS dans un thread séparé
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

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

    with keyboard.Listener(on_press=node.on_press, on_release=node.on_release) as listener:
        listener.join()

    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
