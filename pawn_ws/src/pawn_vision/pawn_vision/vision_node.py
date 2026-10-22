"""
PAWN VISION ROS 2 NODE (vision_node.py)
=======================================
Official ROS 2 Humble/Iron/Rolling Node for the PAWN Vision Subsystem.
Strictly adheres to `ARCHITECTURE_AND_RULES.md`:
  - Uses relative topics (`camera/image_raw`, `vision/detections_2d`, `vision/hud_annotated`)
    so multi-drone launch (`/drone_1`, `/drone_2`) works without topic collisions.
  - Copies exact `header.stamp` from input image to all output messages for SLAM raycasting.
  - Runs Frame Multiplexing scheduler for real-time 30-40 FPS inference on ground station.
"""

import sys
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

# Ensure package directory is on python path
pkg_dir = Path(__file__).resolve().parent
if str(pkg_dir) not in sys.path:
    sys.path.insert(0, str(pkg_dir))

from sar_video_pipeline import SARVideoPipeline

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image, CompressedImage
    from std_msgs.msg import Header
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

try:
    from pawn_interfaces.msg import Detection2D
    INTERFACES_AVAILABLE = True
except ImportError:
    INTERFACES_AVAILABLE = False


class PawnVisionNode(Node if ROS2_AVAILABLE else object):
    def __init__(self) -> None:
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS 2 (rclpy / sensor_msgs) is not installed in the current environment.")

        super().__init__("pawn_vision_node")

        # Declare parameters
        self.declare_parameter("drone_id", 1)
        self.declare_parameter("mode", "multiplexed")
        self.declare_parameter("conf_threshold", 0.25)

        self.drone_id = self.get_parameter("drone_id").get_parameter_value().integer_value
        self.mode = self.get_parameter("mode").get_parameter_value().string_value

        self.get_logger().info(f"[PAWN VISION] Initializing Vision Node for Drone ID: {self.drone_id} (Mode: {self.mode})")

        # Initialize SAR Video Engine
        self.pipeline = SARVideoPipeline(mode=self.mode)
        self.bridge = CvBridge()
        self.frame_count = 0

        # Sensor data QoS (Best effort for video streaming)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 1. Subscriber: Relative topic 'camera/image_raw' (Namespaced automatically by ROS 2 launch)
        self.sub_image = self.create_subscription(
            Image,
            "camera/image_raw",
            self.image_callback,
            sensor_qos,
        )

        # 2. Publisher: HUD Annotated Image for PySide6 GCS Viewport
        self.pub_hud = self.create_publisher(
            Image,
            "vision/hud_annotated",
            10,
        )

        # 3. Publisher: 2D Detections for SLAM Raycaster
        if INTERFACES_AVAILABLE:
            self.pub_detections = self.create_publisher(
                Detection2D,
                "vision/detections_2d",
                10,
            )
        else:
            self.get_logger().warn("[PAWN VISION] 'pawn_interfaces' not found; 'vision/detections_2d' publisher disabled until interfaces are built.")
            self.pub_detections = None

        self.get_logger().info(f"[PAWN VISION] Node fully online. Subscribed to 'camera/image_raw'.")

    def image_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV: {e}")
            return

        h_img, w_img = cv_image.shape[:2]

        # Process frame via Multiplexed SAR Perception Engine
        annotated_bgr, report = self.pipeline.process_frame(cv_image, self.frame_count)
        self.frame_count += 1

        # Publish HUD Annotated Image (Preserving exact input timestamp!)
        try:
            hud_msg = self.bridge.cv2_to_imgmsg(annotated_bgr, encoding="bgr8")
            hud_msg.header = msg.header  # STRICT CONTRACT: Preserve camera timestamp
            self.pub_hud.publish(hud_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish HUD image: {e}")

        # Publish Detection2D messages if targets found and interfaces available
        if self.pub_detections is not None and INTERFACES_AVAILABLE:
            all_dets = (
                self.pipeline.cache["human_animal"]
                + self.pipeline.cache["structural"]
            )
            for det in all_dets:
                cat = det.get("category", "")
                cname = det.get("class_name", "")
                
                # Map strictly to contract: "HUMAN", "CRACK", or "FALL_RISK"
                if cat == "human" or "human" in cname:
                    hazard_type = "FALL_RISK" if det.get("status") == "YERDE_YATIYOR_KRITIK" else "HUMAN"
                else:
                    hazard_type = "CRACK"

                bbox = det.get("bbox_xyxy", [0, 0, 0, 0])
                bx1, by1, bx2, by2 = bbox
                bw = max(1.0, bx2 - bx1)
                bh = max(1.0, by2 - by1)
                cx = (bx1 + bx2) / 2.0
                cy = (by1 + by2) / 2.0

                pin = det.get("slam_pin_2d", {})
                norm_u = pin.get("norm_u", cx / max(1.0, w_img))
                norm_v = pin.get("norm_v", cy / max(1.0, h_img))

                det_msg = Detection2D()
                det_msg.header = msg.header  # STRICT CONTRACT: Match source frame stamp
                det_msg.drone_id = self.drone_id
                det_msg.hazard_type = hazard_type
                det_msg.confidence = float(det.get("confidence", 0.8))
                det_msg.severity_score = float(det.get("severity_score_100", 50.0) / 10.0)
                det_msg.bbox_xywh = [float(cx / w_img), float(cy / h_img), float(bw / w_img), float(bh / h_img)]
                det_msg.raycast_uv = [float(norm_u), float(norm_v)]

                self.pub_detections.publish(det_msg)


def main(args: Any = None) -> None:
    if not ROS2_AVAILABLE:
        print("[ERROR] rclpy is required to run the vision_node ROS 2 executable.")
        print("        To run standalone video streaming without ROS 2, use:")
        print("        python3 -m pawn_vision.sar_video_pipeline --source 0")
        sys.exit(1)

    rclpy.init(args=args)
    node = PawnVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[PAWN VISION] Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
