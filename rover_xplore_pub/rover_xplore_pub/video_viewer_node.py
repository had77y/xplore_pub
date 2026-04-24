import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

WINDOW_TITLE = 'Rover XPlore — FPV'

VIDEO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class VideoViewerNode(Node):
    def __init__(self):
        super().__init__('video_viewer_node')
        self.latest_frame = None
        self.lock = threading.Lock()
        self.create_subscription(
            CompressedImage, '/camera/image_compressed', self._image_callback, VIDEO_QOS
        )
        self.get_logger().info('video_viewer_node démarré — en attente de frames...')

    def _image_callback(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is not None:
            with self.lock:
                self.latest_frame = frame


def main(args=None):
    rclpy.init(args=args)
    node = VideoViewerNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    placeholder = np.zeros((480, 640, 3), dtype='uint8')
    cv2.putText(
        placeholder, 'En attente du flux camera...',
        (80, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2
    )

    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)

    while rclpy.ok():
        with node.lock:
            frame = node.latest_frame

        cv2.imshow(WINDOW_TITLE, frame if frame is not None else placeholder)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
