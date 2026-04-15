import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

WINDOW_TITLE = 'Rover XPlore — FPV'


class VideoViewerNode(Node):
    def __init__(self):
        super().__init__('video_viewer_node')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.lock = threading.Lock()
        self.create_subscription(Image, '/camera/image_raw', self._image_callback, 10)
        self.get_logger().info('video_viewer_node démarré — en attente de frames...')

    def _image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.lock:
            self.latest_frame = frame


def main(args=None):
    rclpy.init(args=args)
    node = VideoViewerNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)

    while rclpy.ok():
        with node.lock:
            frame = node.latest_frame

        if frame is not None:
            cv2.imshow(WINDOW_TITLE, frame)
        else:
            # Fenêtre vide avec message d'attente
            import numpy as np
            placeholder = np.zeros((480, 640, 3), dtype='uint8')
            cv2.putText(
                placeholder, 'En attente du flux camera...',
                (80, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2
            )
            cv2.imshow(WINDOW_TITLE, placeholder)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
