import csv
import os

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class EurocCameraPublisher(Node):

    def __init__(self):
        super().__init__('euroc_camera_publisher')

        self.declare_parameter('drone_id', 1)
        drone = self.get_parameter('drone_id').value
        if drone < 1:
            raise ValueError('drone_id must be >= 1')
        self.camera_frame = f'drone_{drone}/camera_optical_frame'

        self.declare_parameter(
            'dataset_path',
            ''
        )

        self.declare_parameter('fps', 20.0)
        self.declare_parameter('image_topic', '')

        dataset_path = self.get_parameter(
            'dataset_path'
        ).value

        fps = self.get_parameter('fps').value

        if fps <= 0:
            raise ValueError('fps must be positive')

        csv_path = os.path.join(
            dataset_path, 'mav0/cam0/data.csv'
        )

        self.image_dir = os.path.join(
            dataset_path, 'mav0/cam0/data'
        )

        self.frames = []

        with open(csv_path, newline='') as file:
            reader = csv.reader(file)

            for row in reader:
                if not row or row[0].startswith('#'):
                    continue

                timestamp = int(row[0])
                filename = row[1].strip()

                self.frames.append((timestamp, filename))

        image_topic = (self.get_parameter('image_topic').value
                       or f'/drone_{drone}/camera/image_raw')
        self.bridge = CvBridge()

        self.image_pub = self.create_publisher(
            Image,
            image_topic,
            10
        )

        self.index = 0

        self.timer = self.create_timer(
            1.0 / fps,
            self.publish_frame
        )

        self.get_logger().info(
            f'Loaded {len(self.frames)} camera frames'
        )

    def publish_frame(self):

        if self.index >= len(self.frames):
            self.timer.cancel()
            self.get_logger().info('EuRoC playback completed!')
            return

        timestamp, filename = self.frames[self.index]

        image_path = os.path.join(
            self.image_dir, filename
        )

        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

        if image is None:
            self.get_logger().warn(
                f'Cannot read image: {image_path}'
            )
            self.index += 1
            return

        msg = self.bridge.cv2_to_imgmsg(
            image,
            encoding='mono8'
        )

        # EuRoC timestamp: nanoseconds
        msg.header.stamp.sec = timestamp // 1_000_000_000
        msg.header.stamp.nanosec = timestamp % 1_000_000_000

        msg.header.frame_id = self.camera_frame

        self.image_pub.publish(msg)

        self.index += 1


def main(args=None):

    rclpy.init(args=args)

    node = EurocCameraPublisher()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
