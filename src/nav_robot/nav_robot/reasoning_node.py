import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dotenv import load_dotenv, find_dotenv

# Dynamically search for .env in current workspace or user home directory
load_dotenv(find_dotenv(usecwd=True))
load_dotenv(os.path.expanduser('~/.env'))


SYSTEM_PROMPT = """
You are a reasoning engine for a mobile robot navigating an indoor house.

The robot can perform these actions:
- GO_TO_ROOM: navigate to a named room
- SEARCH_FOR: search for a specific object or container in current location
- TASK_COMPLETE: the goal has been achieved
- TASK_FAILED: the goal cannot be achieved after exhausting all options

The house has these rooms: living room, kitchen, bedroom, dining room, study room.

Always respond with a single JSON object:
{
  "action": "SEARCH_FOR",
  "target": "cabinet",
  "reasoning": "cups are often stored inside cabinets",
  "possible_misclassifications": [
    {"object": "fridge", "misclassified_as": "door", "reason": "large tall vertical surface"},
    {"object": "cabinet", "misclassified_as": "cupboard", "reason": "similar wooden box structure"},
    {"object": "sofa", "misclassified_as": "bed", "reason": "similar cushioned rectangular shape"},
    {"object": "microwave", "misclassified_as": "oven", "reason": "box with front glass door"},
    {"object": "sink", "misclassified_as": "counter", "reason": "integrated into counter surface"},
    {"object": "window", "misclassified_as": "mirror", "reason": "reflective rectangular frame"},
    {"object": "tv", "misclassified_as": "monitor", "reason": "flat screen display"},
    {"object": "chair", "misclassified_as": "stool", "reason": "four-legged seating object"},
    {"object": "desk", "misclassified_as": "table", "reason": "flat horizontal wooden surface"},
    {"object": "lamp", "misclassified_as": "light_fixture", "reason": "illuminated vertical object"}
  ]
}

Rules:
- Give ONE action at a time
- When told you have ARRIVED at a room, always respond with SEARCH_FOR for the goal object
- If the goal is not found but you see a container (cabinet, cupboard, drawer, shelf, box),
  respond with SEARCH_FOR targeting that container before moving rooms
- Only use GO_TO_ROOM when there are no useful clues in the current location
- Never repeat a room you have already visited
- Only use TASK_FAILED after checking all likely rooms and containers
- ALWAYS include AT LEAST 10 items in the possible_misclassifications array listing household objects in the target or surrounding rooms that visual sensors might misclassify (e.g., fridge as door, sofa as bed, microwave as oven, cabinet as cupboard, sink as counter, window as mirror, tv as monitor, chair as stool, desk as table, lamp as light fixture).
- Be concise in reasoning, one sentence max
"""

class ReasoningNode(Node):
    def __init__(self):
        super().__init__("reasoning_node")
        api_key = os.getenv('GEMINI_API_KEY')

        if not api_key:
            self.get_logger().error("No API KEY Found")
            return

        self.client = genai.Client(api_key=api_key)
        self.history = []
        self.goal = None

        # REMOVED: self.goal_sub — no more independent /goal subscription
        # This eliminates the discovery-race condition entirely.
        # Orchestrator is now the sole trigger, via /reasoning_feedback.

        self.feed_back_sub = self.create_subscription(
            String, '/reasoning_feedback', self.feedback_callback, 10
        )
        self.action_pub = self.create_publisher(
            String, '/reasoning_action', 10
        )
        self.misclassification_pub = self.create_publisher(
            String, '/possible_misclassifications', 10
        )

        self.get_logger().info("Reasoning Node Ready. Waiting for Goal")

    def feedback_callback(self, msg):
        feedback = msg.data
        self.get_logger().info(f'[DEBUG] feedback_callback ENTERED: {feedback!r}')

        if feedback.startswith('NEW_GOAL:'):
            self.goal = feedback.replace('NEW_GOAL:', '').strip()
            self.get_logger().info(f'New goal received: {self.goal}')
            self.history = []  # reset for new goal
            self.reason(f'I need to find: {self.goal}. Where should I start looking?')
        else:
            self.get_logger().info(f'Feedback received: {feedback}')
            self.reason(feedback)

    def reason(self, message):
        try:
            self.history.append({
                "role": "user",
                "parts": [{"text": message}]
            })
            self.get_logger().info("API CALLED")
            response = self.client.models.generate_content(
                model='gemini-3.6-flash',
                contents=self.history,
                config=types.GenerateContentConfig(
                    system_instruction=SYSTEM_PROMPT,
                )
            )
            self.get_logger().info("API returned")

            raw = response.text.strip()
            self.get_logger().info(f'LLM response: {raw}')

            self.history.append({
                "role": "model",
                "parts": [{"text": raw}]
            })

            if '```' in raw:
                raw = raw.split('```')[1]
                if raw.startswith('json'):
                    raw = raw[4:]

            action = json.loads(raw.strip())

            action_msg = String()
            action_msg.data = json.dumps(action)
            self.action_pub.publish(action_msg)

            misclass_list = action.get('possible_misclassifications', [])
            misclass_msg = String()
            misclass_msg.data = json.dumps(misclass_list)
            self.misclassification_pub.publish(misclass_msg)

            self.get_logger().info(
                f'Action: {action["action"]} → {action.get("target", "")} | {action["reasoning"]}')
            self.get_logger().info(
                f'Possible Misclassifications Published: {misclass_msg.data}')

        except Exception as e:
            self.get_logger().error(f'Reasoning failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = ReasoningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()