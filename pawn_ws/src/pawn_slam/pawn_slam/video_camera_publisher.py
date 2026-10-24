import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class VideoCameraPublisher(Node):

    def __init__(self):
        super().__init__('video_camera_publisher')

        self.declare_parameter('drone_id', 1)
        drone = self.get_parameter('drone_id').value
        if drone < 1:
            raise ValueError('drone_id must be >= 1')
        self.camera_frame = f'drone_{drone}/camera_optical_frame'

        self.declare_parameter('video_path', '')
        self.declare_parameter(
            'image_topic', ''
        )

        video_path = self.get_parameter(
            'video_path'
        ).value

        image_topic = self.get_parameter(
            'image_topic'
        ).value or f'/drone_{drone}/camera/image_raw'

        if not video_path:
            raise ValueError('video_path is required')

        self.cap = cv2.VideoCapture(video_path)

        if not self.cap.isOpened():
            raise RuntimeError(
                f'Cannot open video: {video_path}'
            )

        fps = self.cap.get(cv2.CAP_PROP_FPS)

        if fps <= 0:
            fps = 30.0

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            Image,
            image_topic,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(
            1.0 / fps,
            self.publish_frame
        )

        self.frame_count = 0

        self.get_logger().info(
            f'Video opened: {video_path} | FPS: {fps:.2f}'
        )

    def publish_frame(self):

        success, frame = self.cap.read()

        if not success:
            self.get_logger().info(
                f'Video finished: {self.frame_count} frames'
            )
            self.timer.cancel()
            self.cap.release()
            return

        gray = cv2.cvtColor(
            frame, cv2.COLOR_BGR2GRAY
        )

        msg = self.bridge.cv2_to_imgmsg(
            gray,
            encoding='mono8'
        )

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame

        self.publisher.publish(msg)

        self.frame_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = VideoCameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
