import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import State

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty
from cv_bridge import CvBridge
import cv2
import torch
import clip
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
from PIL import Image as PILImage
import time
import json
import re


class GroundingDinoNode(LifecycleNode):
    def __init__(self):
        super().__init__('grounding_dino_node')
        # swap without rebuilding, e.g. -p model:=IDEA-Research/grounding-dino-base
        self.declare_parameter('model', 'IDEA-Research/grounding-dino-tiny')

        self.confirmation_frames = 1
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

       
        self.clip_device = 'cpu'
        self.clip_model = None
        self.clip_preprocess = None
        
        self.clip_min_margin = 0.03
        self.similar_objects = []  # LLM's guess at confusable objects; set via /similar_objects
        self.similar_objects_sub = None

        
        self.clip_stall_limit = 6
        self.clip_stall_label = None
        self.clip_stall_count = 0

       
        self.confident_reject_margin = 0.25
        
        self.stall_tie_gap = 0.05
        self.crop_padding = 0.4      # fraction of box size added on each side for context
        self.clip_prompt = 'a photo of a {}'
        
        self.same_category_sim = 0.88
        self.reject_limit = 3
        self.reject_streak = 0
        self.verify_cooldown = 10.0       # seconds to stop pausing after repeated rejects
        self.verify_cooldown_until = 0.0


    # ---------------- LIFECYCLE ----------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        model_id = self.get_parameter('model').get_parameter_value().string_value

        try:
            self.processor = AutoProcessor.from_pretrained(model_id)
            self.model = AutoModelForZeroShotObjectDetection.from_pretrained(
                model_id).to(self.device)
        except Exception as e:
            self.get_logger().error(f'Failed to load {model_id}: {e}')
            return TransitionCallbackReturn.FAILURE

        try:
            self.clip_model, self.clip_preprocess = clip.load(
                'ViT-B/32', device=self.clip_device)
        except Exception as e:
            self.get_logger().error(f'Failed to load CLIP: {e}')
            return TransitionCallbackReturn.FAILURE

        self.pub = self.create_publisher(Image, '/camera/annotated', 10)
        self.box_pub = self.create_publisher(Twist, '/detected_box', 10)
        self.scene_pub = self.create_publisher(String, '/scene_objects', 10)
        # fires the instant a raw box clears threshold, before the slower CLIP check -
        # stops the robot moving away from the target during that verification window
        self.pause_pub = self.create_publisher(Empty, '/detection_pause', 10)

        self.target_sub = self.create_subscription(
            String, '/search_target', self.target_callback, 10)
        self.similar_objects_sub = self.create_subscription(
            String, '/similar_objects', self.similar_objects_callback, 10)

        # NEW — explicit scan request, replaces internal timeout
        self.scan_request_sub = self.create_subscription(
            String, '/scan_request', self.scan_request_callback, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        if self.sub is not None:
            self.destroy_subscription(self.sub)
            self.sub = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        for sub in [self.sub, self.target_sub, self.scan_request_sub, self.similar_objects_sub]:
            if sub is not None:
                self.destroy_subscription(sub)
        self.sub = None
        self.target_sub = None
        self.scan_request_sub = None
        self.similar_objects_sub = None
        self.model = None
        self.processor = None
        self.clip_model = None
        self.clip_preprocess = None
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

    def drop_same_category(self, target_label, alternatives):
        """Backstop for the LLM ignoring the different-category rule. A distractor that is
        really another name for the target (tv/monitor, coffee maker/espresso machine)
        beats the target in the lineup and makes us reject the correct object. Threshold
        is deliberately conservative: text similarity does not cleanly separate
        same-category pairs from legitimately different ones, so only drop the obvious."""
        if not alternatives:
            return alternatives

        with torch.no_grad():
            emb = self.clip_model.encode_text(clip.tokenize(
                [self.clip_prompt.format(w) for w in [target_label] + alternatives]
            ).to(self.clip_device))
        emb = emb / emb.norm(dim=-1, keepdim=True)
        sims = (emb[1:] @ emb[0]).tolist()

        kept = []
        for word, sim in zip(alternatives, sims):
            if sim >= self.same_category_sim:
                self.get_logger().info(
                    f'Dropping "{word}" from the lineup - too close to '
                    f'"{target_label}" ({sim:.2f})')
            else:
                kept.append(word)
        return kept

    def clip_verify(self, frame, box, target_label):
        """Crop DINO's box and ask CLIP to pick the target out of a lineup of the other
        known room objects. Rejects the box if CLIP disagrees or isn't confident."""
        target_label = target_label.strip().rstrip('.')
        x1, y1, x2, y2 = [max(0, int(v)) for v in box]
        if x2 <= x1 or y2 <= y1:
            return False

        
        h, w = frame.shape[:2]
        pad_x = int((x2 - x1) * self.crop_padding)
        pad_y = int((y2 - y1) * self.crop_padding)
        crop = PILImage.fromarray(frame[max(0, y1 - pad_y):min(h, y2 + pad_y),
                                        max(0, x1 - pad_x):min(w, x2 + pad_x)])

        if self.similar_objects:
            # LLM-supplied: objects it expects the detector could confuse with this
            # specific target, rather than every object the room might contain.
            similar_objects = [
                w.strip().rstrip('.') for w in self.similar_objects
                if w.strip() and w.strip().rstrip('.') != target_label
            ]
        else:
            similar_objects = [
                w.strip().rstrip('.') for w in self.scene_objects.split('.')
                if w.strip() and w.strip().rstrip('.') != target_label
            ]
        similar_objects = self.drop_same_category(target_label, similar_objects)
        candidates = [target_label] + similar_objects
        if len(candidates) < 2:
            # nothing to disambiguate against - fall back to the generic room-object list
            candidates = [target_label] + self.drop_same_category(target_label, [
                w.strip().rstrip('.') for w in self.scene_objects.split('.')
                if w.strip() and w.strip().rstrip('.') != target_label
            ])

        image = self.clip_preprocess(crop).unsqueeze(0).to(self.clip_device)
        
        text = clip.tokenize(
            [self.clip_prompt.format(c) for c in candidates]).to(self.clip_device)

        with torch.no_grad():
            logits_per_image, _ = self.clip_model(image, text)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()[0]

        order = probs.argsort()[::-1]
        best_idx, runner_up_idx = order[0], order[1]
        best_label, best_prob = candidates[best_idx], probs[best_idx]
        margin = best_prob - probs[runner_up_idx]

       
        agrees = best_label == target_label and margin >= self.clip_min_margin

       
        target_gap = best_prob - probs[0]
        override = False

        
        confident_reject = not agrees and target_gap >= self.confident_reject_margin
        tied_with_target = not agrees and target_gap <= self.stall_tie_gap

        stall_key = (target_label, best_label)
        if agrees or stall_key != self.clip_stall_label:
            self.clip_stall_label = stall_key
            self.clip_stall_count = 0

        if agrees:
            self.reject_streak = 0
            self.verify_cooldown_until = 0.0
        elif confident_reject:
            self.reject_streak += 1
            if self.reject_streak >= self.reject_limit:
                
                self.verify_cooldown_until = time.time() + self.verify_cooldown
                self.reject_streak = 0
        elif tied_with_target:
            self.clip_stall_count += 1
            if self.clip_stall_count >= self.clip_stall_limit:
                agrees = True
                override = True
                self.clip_stall_count = 0

        verdict = 'CONFIRMED'
        if override:
            verdict = f'CONFIRMED (override: tied with "{target_label}" x{self.clip_stall_limit})'
        elif not agrees:
            verdict = 'REJECTED'

        self.get_logger().info(
            f'CLIP check: best={best_label} ({best_prob:.2f}), '
            f'target={target_label} ({probs[0]:.2f}), '
            f'margin={margin:.2f} -> {verdict}')
        return agrees

    def verification_on_cooldown(self):
        return time.time() < self.verify_cooldown_until

    def scan_request_callback(self, msg):
        """Called explicitly by local_search_node when its pattern is exhausted."""

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
                if score < self.box_threshold:
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

            # a label CLIP has already confidently rejected enough times is a known false
            # positive - keep annotating it, but don't stop the robot or re-run CLIP on it
            if self.verification_on_cooldown():
                self.pending_label = None
                self.pending_count = 0
                annotated_msg = self.bridge.cv2_to_imgmsg(
                    cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB), 'rgb8')
                self.pub.publish(annotated_msg)
                return

            
            self.pause_pub.publish(Empty())

            if best_label == self.pending_label:
                self.pending_count += 1
            else:
                self.pending_label = best_label
                self.pending_count = 1

            
            
            if self.pending_count >= self.confirmation_frames and best_score >= self.box_threshold:
                box = boxes[best_idx]
                x1, y1, x2, y2 = box.tolist()

                if self.clip_verify(frame, (x1, y1, x2, y2), best_label):
                    cx = (x1 + x2) / 2
                    area = (x2 - x1) * (y2 - y1)

                    box_msg = Twist()
                    box_msg.linear.x = cx
                    box_msg.linear.y = area
                    box_msg.linear.z = x2 - x1   # width, for the approach sanity check
                    self.box_pub.publish(box_msg)
                else:
                    
                    self.pending_label = None
                    self.pending_count = 0
        else:
           
            self.pending_label = None
            self.pending_count = 0

        annotated_msg = self.bridge.cv2_to_imgmsg(
            cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB), 'rgb8')
        self.pub.publish(annotated_msg)
    def target_callback(self, msg):
        
        self.target = re.sub(r'^(the|a|an)\s+', '', msg.data.strip(), flags=re.IGNORECASE)
        if not self.target.endswith('.'):
            self.target += '.'
        self.clip_stall_label = None
        self.clip_stall_count = 0
        self.reject_streak = 0      # a new target makes old rejections irrelevant
        self.verify_cooldown_until = 0.0

    def similar_objects_callback(self, msg):
        try:
            self.similar_objects = [str(w) for w in json.loads(msg.data)]
        except (json.JSONDecodeError, TypeError):
            self.similar_objects = []


def main(args=None):
    rclpy.init(args=args)
    node = GroundingDinoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()