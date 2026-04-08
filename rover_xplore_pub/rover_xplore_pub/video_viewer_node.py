import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

class VideoViewerNode(Node):
    def __init__(self):
        super().__init__('video_viewer_node')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.lock = threading.Lock()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('video_viewer_node démarré — en attente de frames...')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.lock:
            self.latest_frame = frame

def main(args=None):
    rclpy.init(args=args)
    node = VideoViewerNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    cv2.namedWindow('Rover XPlore — FPV', cv2.WINDOW_NORMAL)

    while rclpy.ok():
        with node.lock:
            frame = node.latest_frame

        if frame is not None:
            cv2.imshow('Rover XPlore — FPV', frame)
        else:
            cv2.waitKey(100)
            continue

        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()