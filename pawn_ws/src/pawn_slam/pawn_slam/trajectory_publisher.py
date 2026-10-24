from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node


class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')

        default_file = ''
        self.declare_parameter('drone_id', 1)
        drone = self.get_parameter('drone_id').value
        if drone < 1:
            raise ValueError('drone_id must be >= 1')
        self.local_frame = f'drone_{drone}/slam_map'

        self.declare_parameter('trajectory_file', default_file)

        filename = self.get_parameter(
            'trajectory_file'
        ).value

        self.poses = []

        with open(filename, 'r') as file:
            for line in file:
                values = line.strip().split()

                if len(values) != 8:
                    continue

                # timestamp x y z qx qy qz qw
                x, y, z, qx, qy, qz, qw = map(
                    float, values[1:]
                )

                self.poses.append(
                    (x, y, z, qx, qy, qz, qw)
                )

        self.index = 0

        self.pose_pub = self.create_publisher(
            PoseStamped, f'/drone_{drone}/slam/pose', 10
        )

        self.path_pub = self.create_publisher(
            Path, f'/drone_{drone}/path', 10
        )

        self.path = Path()
        self.path.header.frame_id = self.local_frame

        # EuRoC kamera frekansı: yaklaşık 20 Hz
        self.timer = self.create_timer(0.05, self.publish_pose)

        self.get_logger().info(
            f'Loaded {len(self.poses)} poses'
        )

    def publish_pose(self):

        if self.index >= len(self.poses):
            self.timer.cancel()
            self.get_logger().info('Trajectory completed!')
            return

        x, y, z, qx, qy, qz, qw = self.poses[self.index]

        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.local_frame

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.pose_pub.publish(msg)

        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(msg)

        self.path_pub.publish(self.path)

        self.index += 1


def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
