# ══════════════════════════════════════════════════════════════════════════════
# mode_selector_node.py — PC (terminal, version simplifiée)
#
# RÔLE : version minimale du sélecteur de mode — seulement deux choix :
#         autonome ou commandé. Pas de contrôle clavier en temps réel.
#
# DIFFÉRENCE AVEC controller_node.py :
#   controller_node.py  = interface complète (menus, WASD, vitesse, viewer)
#   mode_selector_node  = version réduite, uniquement pour choisir le mode de départ
#                         Utile pour les tests rapides sans lancer toute la GUI.
#
# PLACE DANS LE SYSTÈME :
#
#   [PC] mode_selector_node ← CE FICHIER (l'opérateur appuie A ou C)
#         └─ publie /rover/mode → [RPi] mode_manager_node, camera_node,
#                                         aruco_node, motor_controller_node
#
# TOPIC PUBLIÉ :
#   /rover/mode (std_msgs/String) :
#     "autonomous" → si l'opérateur appuie A
#     "teleop"     → si l'opérateur appuie C (note : motor_controller_node attend "race")
#
# THREADING :
#   ROS2 spin() tourne dans un thread daemon séparé.
#   Le thread principal bloque sur la lecture clavier (une touche à la fois).
#   Pas de timeout clavier ici : get_key() attend indéfiniment.
# ══════════════════════════════════════════════════════════════════════════════

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

        # Publie sur /rover/mode → reçu par tous les nodes RPi qui écoutent ce topic
        self.publisher    = self.create_publisher(String, '/rover/mode', 10)
        self.current_mode = None  # mémorise le mode courant pour le logging

        self.get_logger().info('mode_selector_node démarré')

    def publish_mode(self, mode):
        """
        Publie le mode sur /rover/mode et le mémorise.
        Ce message traverse le WiFi via ROS2 DDS et est reçu par le RPi.
        """
        msg      = String()
        msg.data = mode
        self.publisher.publish(msg)
        self.current_mode = mode
        self.get_logger().info(f'[MODE ENVOYÉ] → {mode}')


def get_key(settings):
    """
    Lit UNE touche clavier en mode raw (sans Entrée).
    BLOQUANT : attend indéfiniment jusqu'à ce qu'une touche soit pressée.
    (Contrairement à controller_node.py qui a un timeout de 50ms.)
    settings : paramètres terminal originaux pour la restauration.
    """
    tty.setraw(sys.stdin.fileno())  # mode raw : chaque touche est lue immédiatement
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # restaure le terminal
    return key


def main(args=None):
    rclpy.init(args=args)
    node = ModeSelectorNode()

    # Thread daemon : ROS2 spin() dans un thread séparé pour ne pas bloquer le clavier
    # daemon=True : le thread se termine automatiquement quand le programme principal s'arrête
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Sauvegarde des paramètres terminal (pour les restaurer au finally)
    settings = termios.tcgetattr(sys.stdin)

    print('\n' + '═' * 42)
    print('   XPLORE ROVER — Sélection du mode')
    print('═' * 42)
    print('   A  →  mode AUTONOME   (publie "autonomous" sur /rover/mode)')
    print('   C  →  mode COMMANDÉ   (publie "teleop" sur /rover/mode)')
    print('   Q  →  quitter')
    print('═' * 42 + '\n')

    try:
        while rclpy.ok():
            key = get_key(settings)  # bloque ici jusqu'à une touche

            if key.lower() == 'a':
                # → RPi : mode_manager_node log "MODE AUTONOME"
                #         camera_node active le flux
                #         aruco_node s'abonne à /camera/image_raw
                node.publish_mode('autonomous')

            elif key.lower() == 'c':
                # Note : motor_controller_node attend "race", pas "teleop"
                # Ce node est une ancienne version — controller_node.py est plus complet
                node.publish_mode('teleop')

            elif key.lower() == 'q':
                print('\nArrêt du sélecteur de mode.')
                break

    finally:
        # Restaure TOUJOURS le terminal (même en cas d'exception ou Ctrl+C)
        # Sans ça, le terminal resterait en mode raw et serait inutilisable
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
