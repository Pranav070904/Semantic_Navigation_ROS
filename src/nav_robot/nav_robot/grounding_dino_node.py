import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import State

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import torch
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
from PIL import Image as PILImage
import time


class GroundingDinoNode(LifecycleNode):
    def __init__(self):
        super().__init__('grounding_dino_node')

        self.confirmation_frames = 3
        self.pending_label = None
        self.pending_count = 0
        self.frame_count = 0
        self.process_every_n = 3
        self.box_threshold = 0.6
        self.target = 'chair.'
        self.scene_objects = (
            "chair. table. sofa. bed. refrigerator. sink. counter. "
            "microwave. door. cabinet. cupboard. window. tv. desk. lamp."
        )

        self.bridge = CvBridge()
        self.latest_frame = None  # keep the most recent frame for on-demand scene scan

        self.model = None
        self.processor = None
        self.device = None
        self.sub = None
        self.target_sub = None
        self.scan_request_sub = None
        self.pub = None
        self.box_pub = None
        self.scene_pub = None

        self.get_logger().info('Grounding DINO node constructed (unconfigured)')

    # ---------------- LIFECYCLE ----------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring — loading Grounding DINO model...')

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        model_id = 'IDEA-Research/grounding-dino-tiny'

        try:
            self.processor = AutoProcessor.from_pretrained(model_id)
            self.model = AutoModelForZeroShotObjectDetection.from_pretrained(
                model_id).to(self.device)
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            return TransitionCallbackReturn.FAILURE

        self.pub = self.create_publisher(Image, '/camera/annotated', 10)
        self.box_pub = self.create_publisher(Twist, '/detected_box', 10)
        self.scene_pub = self.create_publisher(String, '/scene_objects', 10)

        self.target_sub = self.create_subscription(
            String, '/search_target', self.target_callback, 10)

        # NEW — explicit scan request, replaces internal timeout
        self.scan_request_sub = self.create_subscription(
            String, '/scan_request', self.scan_request_callback, 10)

        self.get_logger().info(f'Configured. Using device: {self.device}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating — starting detection')
        self.sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating — stopping detection')
        if self.sub is not None:
            self.destroy_subscription(self.sub)
            self.sub = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up')
        for sub in [self.sub, self.target_sub, self.scan_request_sub]:
            if sub is not None:
                self.destroy_subscription(sub)
        self.sub = None
        self.target_sub = None
        self.scan_request_sub = None
        self.model = None
        self.processor = None
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # ---------------- DETECTION ----------------

    def scan_scene(self, frame):
        pil_image = PILImage.fromarray(frame)
        inputs = self.processor(
            images=pil_image, text=self.scene_objects, return_tensors='pt'
        ).to(self.device)

        with torch.no_grad():
            outputs = self.model(**inputs)

        results = self.processor.post_process_grounded_object_detection(
            outputs, threshold=0.35, target_sizes=[pil_image.size[::-1]]
        )

        labels = results[0]['text_labels']
        scores = results[0]['scores']
        seen = list(set([
            label for label, score in zip(labels, scores) if score > 0.35
        ]))
        return seen

    def scan_request_callback(self, msg):
        """Called explicitly by local_search_node when its pattern is exhausted."""
        self.get_logger().info('Scan requested by local_search_node')

        if self.latest_frame is None:
            self.get_logger().warn('No frame available for scene scan')
            scene_msg = String()
            scene_msg.data = 'nothing recognizable'
            self.scene_pub.publish(scene_msg)
            return

        scene_objects = self.scan_scene(self.latest_frame)
        scene_msg = String()
        scene_msg.data = (
            ', '.join(scene_objects) if scene_objects else 'nothing recognizable'
        )
        self.scene_pub.publish(scene_msg)
        self.get_logger().info(f'Scene objects published: {scene_msg.data}')

    def image_callback(self, msg):
        self.frame_count += 1

        frame = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        self.latest_frame = frame

        if self.frame_count % self.process_every_n != 0:
            return

        pil_image = PILImage.fromarray(frame)

        inputs = self.processor(
            images=pil_image, text=self.target, return_tensors='pt'
        ).to(self.device)

        with torch.no_grad():
            outputs = self.model(**inputs)

        results = self.processor.post_process_grounded_object_detection(
            outputs, threshold=self.box_threshold, target_sizes=[pil_image.size[::-1]]
        )

        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        boxes = results[0]['boxes']
        scores = results[0]['scores']
        labels = results[0]['text_labels']

        if len(boxes) > 0:
            for box, score, label in zip(boxes, scores, labels):
                if score < 0.6:
                    continue
                x1, y1, x2, y2 = [int(v) for v in box.tolist()]
                cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame_bgr, f'{label} {score:.2f}',
                    (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2
                )

            best_idx = scores.argmax()
            best_label = labels[best_idx]
            best_score = scores[best_idx]

           
            if best_label == self.pending_label:
                self.pending_count += 1
            else:
                self.pending_label = best_label
                self.pending_count = 1

            
            if self.pending_count >= self.confirmation_frames and best_score > 0.65:
                box = boxes[best_idx]
                x1, y1, x2, y2 = box.tolist()
                cx = (x1 + x2) / 2
                area = (x2 - x1) * (y2 - y1)

                box_msg = Twist()
                box_msg.linear.x = cx
                box_msg.linear.y = area
                self.box_pub.publish(box_msg)
            else:
                self.get_logger().info(
                    f'Pending confirmation: {best_label} ({self.pending_count}/{self.confirmation_frames}, score = {best_score:.2f})',
                    throttle_duration_sec=1.0)
        else:
           
            self.pending_label = None
            self.pending_count = 0

        annotated_msg = self.bridge.cv2_to_imgmsg(
            cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB), 'rgb8')
        self.pub.publish(annotated_msg)
    def target_callback(self, msg):
        self.target = msg.data
        if not self.target.endswith('.'):
            self.target += '.'
        self.get_logger().info(f'New target: {self.target}')


def main(args=None):
    rclpy.init(args=args)
    node = GroundingDinoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()