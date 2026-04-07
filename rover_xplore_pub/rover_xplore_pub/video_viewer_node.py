# --- Imports ---
# rclpy est la bibliothèque Python pour ROS 2
import rclpy
# Node est la classe de base pour créer un nœud ROS 2
from rclpy.node import Node
# Image est le type de message ROS 2 standard pour transmettre des images
from sensor_msgs.msg import Image
# CvBridge permet de convertir les messages ROS Image en images OpenCV
from cv_bridge import CvBridge
# cv2 est OpenCV, utilisé pour afficher les images dans une fenêtre
import cv2
# threading permet d'exécuter du code en parallèle sur plusieurs threads
import threading


class VideoViewerNode(Node):
    """
    Nœud ROS 2 qui reçoit le flux vidéo de la caméra du rover.
    La réception des images (ROS) et l'affichage (OpenCV) tournent
    dans des threads séparés pour éviter les conflits.
    """

    def __init__(self):
        # Initialise le nœud ROS 2 avec le nom 'video_viewer_node'
        super().__init__('video_viewer_node')

        # Crée le pont ROS ↔ OpenCV pour convertir les messages Image
        self.bridge = CvBridge()

        # Stocke la dernière image reçue (None au départ, avant la 1ère frame)
        self.latest_frame = None

        # Verrou (mutex) pour protéger l'accès à latest_frame entre les threads.
        # Sans ça, le thread ROS pourrait écrire une image pendant que le thread
        # principal est en train de la lire → corruption de données.
        self.lock = threading.Lock()

        # S'abonne au topic '/camera/image_raw' où la caméra publie ses images
        # - Image              : type de message attendu
        # - '/camera/image_raw': nom du topic ROS 2 à écouter
        # - self.image_callback: fonction appelée à chaque nouvelle image reçue
        # - 10                 : taille de la file d'attente (queue size)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('video_viewer_node démarré — en attente de frames...')

    def image_callback(self, msg):
        """
        Appelée automatiquement par ROS 2 (dans le thread ROS) à chaque nouvelle image.
        Convertit le message ROS en image OpenCV et la stocke pour l'affichage.
        """
        # Convertit le message ROS Image en tableau numpy au format BGR (standard OpenCV)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Prend le verrou avant d'écrire dans latest_frame pour éviter
        # qu'un autre thread lise une image à moitié écrite
        with self.lock:
            self.latest_frame = frame


def main(args=None):
    """
    Point d'entrée principal.

    Architecture à deux threads :
      - Thread ROS  : gère la réception des messages (spin) → s'exécute en arrière-plan
      - Thread principal : gère l'affichage OpenCV → doit rester sur le thread principal
        car les fenêtres graphiques (GUI) ne sont pas thread-safe sur la plupart des OS
    """
    # Initialise le système ROS 2
    rclpy.init(args=args)

    # Crée le nœud
    node = VideoViewerNode()

    # Lance rclpy.spin() dans un thread séparé (daemon=True : s'arrête automatiquement
    # quand le programme principal se termine, sans besoin de join explicite)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Boucle d'affichage principale — tourne tant que ROS 2 est actif
    while rclpy.ok():
        # Récupère la dernière frame de façon thread-safe (verrou)
        with node.lock:
            frame = node.latest_frame

        # Affiche la frame seulement si on en a reçu au moins une
        if frame is not None:
            cv2.imshow('Rover XPlore — FPV', frame)

        # Attend 1ms pour les événements clavier.
        # Si l'utilisateur appuie sur 'q', on sort de la boucle
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Nettoyage final : ferme la fenêtre OpenCV, détruit le nœud, arrête ROS 2
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


# Ce bloc s'exécute uniquement si le fichier est lancé directement (pas importé comme module)
if __name__ == '__main__':
    main()
