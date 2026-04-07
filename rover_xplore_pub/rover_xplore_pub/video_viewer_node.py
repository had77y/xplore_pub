# --- Imports ---
# rclpy est la bibliothèque Python pour ROS 2 (Robot Operating System)
import rclpy
# Node est la classe de base pour créer un nœud ROS 2
from rclpy.node import Node
# Image est le type de message ROS 2 standard pour transmettre des images
from sensor_msgs.msg import Image
# CvBridge permet de convertir les messages ROS Image en images OpenCV (et vice versa)
from cv_bridge import CvBridge
# cv2 est OpenCV, la bibliothèque de vision par ordinateur utilisée pour afficher les images
import cv2


class VideoViewerNode(Node):
    """
    Nœud ROS 2 qui s'abonne au flux vidéo de la caméra du rover
    et l'affiche en temps réel dans une fenêtre (vue FPV).
    """

    def __init__(self):
        # Initialise le nœud ROS 2 avec le nom 'video_viewer_node'
        super().__init__('video_viewer_node')

        # Crée un pont entre les messages ROS Image et les images OpenCV
        self.bridge = CvBridge()

        # S'abonne au topic '/camera/image_raw' où la caméra publie ses images
        # - Image       : type de message attendu
        # - '/camera/image_raw' : nom du topic ROS 2 à écouter
        # - self.image_callback : fonction appelée à chaque nouvelle image reçue
        # - 10          : taille de la file d'attente (queue size)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Log dans la console pour confirmer que le nœud est bien démarré
        self.get_logger().info('video_viewer_node démarré — en attente de frames...')

    def image_callback(self, msg):
        """
        Appelée automatiquement par ROS 2 à chaque nouvelle image reçue sur le topic.
        Convertit le message ROS en image OpenCV et l'affiche dans une fenêtre.
        """
        # Convertit le message ROS Image en tableau numpy (format BGR utilisé par OpenCV)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Affiche l'image dans une fenêtre intitulée 'Rover XPlore — FPV'
        cv2.imshow('Rover XPlore — FPV', frame)

        # Vérifie si l'utilisateur appuie sur la touche 'q' pour quitter
        # cv2.waitKey(1) attend 1ms pour un événement clavier
        # & 0xFF permet d'isoler le code ASCII de la touche pressée
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('Fermeture du viewer...')
            # Ferme toutes les fenêtres OpenCV
            cv2.destroyAllWindows()
            # Arrête proprement le système ROS 2
            rclpy.shutdown()

    def destroy_node(self):
        """
        Appelée automatiquement quand le nœud est détruit (ex: Ctrl+C).
        S'assure que toutes les fenêtres OpenCV sont bien fermées avant de quitter.
        """
        cv2.destroyAllWindows()
        # Appelle la méthode destroy_node() de la classe parente pour le nettoyage ROS
        super().destroy_node()


def main(args=None):
    """
    Point d'entrée principal du programme.
    Initialise ROS 2, crée le nœud, le fait tourner en boucle,
    puis nettoie proprement à la fermeture.
    """
    # Initialise le système ROS 2 (doit être appelé avant tout)
    rclpy.init(args=args)

    # Crée une instance du nœud VideoViewerNode
    node = VideoViewerNode()

    # Lance la boucle événementielle ROS 2 (bloque ici jusqu'à Ctrl+C ou rclpy.shutdown())
    rclpy.spin(node)

    # Nettoyage : détruit le nœud et libère les ressources
    node.destroy_node()

    # Arrête proprement ROS 2
    rclpy.shutdown()


# Ce bloc s'exécute uniquement si le fichier est lancé directement (pas importé comme module)
if __name__ == '__main__':
    main()
