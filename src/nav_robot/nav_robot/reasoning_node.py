import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dotenv import load_dotenv
import os
import json


from google import genai
from google.genai import types


load_dotenv('/home/pranav/nav_ws/.env')


SYSTEM_PROMPT = """
You are a reasoning engine for a mobile robot navigating an indoor house.

The robot can perform exactly these actions:
- GO_TO_ROOM: navigate to a named room
- SEARCH_FOR: search for a specific object or container in the current location
- TASK_COMPLETE: the goal has been achieved
- TASK_FAILED: the goal cannot be achieved after exhausting all options

VALID_ROOMS = ["living room", "kitchen", "bedroom", "study room", "workout room"]
These are the ONLY rooms that exist. Never invent a room name, and "target" for
GO_TO_ROOM must be copied EXACTLY (same spelling, same spaces) from VALID_ROOMS.

You must respond with EXACTLY ONE JSON object and NOTHING else - no markdown fences,
no commentary before or after it. The object has EXACTLY these five keys, always, for
every action type:
{
  "action": <one of the four action strings above>,
  "target": <string, see per-action rules below - never omit this key>,
  "goal_object": <string, see below - never omit this key>,
  "reasoning": <one sentence, max ~20 words>,
  "alternatives": <array of strings, see per-action rules below - never omit this key>
}

"goal_object" is the OVERALL object the user asked us to find, as a bare noun with no
leading article - e.g. for the request "find the coffee cup" it is "coffee cup", NOT
"find the coffee cup" and NOT "the coffee cup". It stays the SAME in every response for
a given goal, even while "target" changes to containers you want searched along the way.

Per-action rules for "target" and "alternatives":

- GO_TO_ROOM: "target" = the exact room name, copied from VALID_ROOMS.
  "alternatives" = REQUIRED, never empty. The robot starts searching for the overall goal
  object the instant it arrives, before you are asked again - so list exactly 4-5 OTHER
  objects for the GOAL object, following the SAME different-category rules given under
  SEARCH_FOR below. Never a synonym or same-category variant of the goal.

- SEARCH_FOR: "target" = the single object or container to search for, as a bare noun
  with NO leading article (e.g. "tv", NOT "the tv" or "a tv").
  "alternatives" = REQUIRED, never empty. Exactly 4-5 other objects that might plausibly
  be found in the same room and could be confused with the target from a distance. Alternatives
  should be OBJECTS ONLY and NEVER a ROOM.

  CRITICAL - every alternative must be from a DIFFERENT CATEGORY of object than the
  target. A verifier compares the target against this list, so anything in the same
  category wins and the CORRECT object gets rejected. Apply this test to each one:
    * Is it another kind of the same thing? -> EXCLUDE
    * Would a person use the same generic word for both? -> EXCLUDE
    * Is one a type/brand/variant of the other? -> EXCLUDE

  BANNED as alternatives for each other (same category):
    tv / monitor / television / screen / display / projector screen
    coffee maker / espresso machine / coffee machine
    sofa / couch / loveseat / settee
    cup / mug / glass / tumbler
    bookshelf / shelf / bookcase / dresser / cabinet / cupboard / wardrobe
    plate / dish / saucer / bowl
  These lists are examples of the PATTERN, not the whole rule - apply the same reasoning
  to any target.

  GOOD alternatives for "tv": painting, lamp, window, plant, door.
  BAD alternatives for "tv": monitor, screen, projector screen, television.
  GOOD alternatives for "coffee maker": sink, toaster, chair, bin, plant.
  BAD alternatives for "coffee maker": espresso machine, kettle, coffee machine.

  Pick ordinary, clearly different pieces of furniture or household objects. It is fine
  if they are not especially confusable - a weak distractor is far better than one so
  close the verifier cannot separate it from the target.

- TASK_COMPLETE: "target" = "" (empty string). "alternatives" = [].

- TASK_FAILED: "target" = "" (empty string). "alternatives" = [].

Other rules:
- Give ONE action at a time.
- When told you have ARRIVED at a room, always respond with SEARCH_FOR for the goal object.
- If the goal is not found but you see a container (cabinet, cupboard, drawer, shelf, box),
  respond with SEARCH_FOR targeting that container before moving rooms.
- Only use GO_TO_ROOM when there are no useful clues in the current location.
- Never repeat a room you have already visited.
- Only use TASK_FAILED after checking all likely rooms and containers.

Example turns:
{"action": "GO_TO_ROOM", "target": "kitchen", "goal_object": "mug", "reasoning": "mugs are usually kept in the kitchen", "alternatives": ["sink", "chair", "toaster", "plant"]}
{"action": "SEARCH_FOR", "target": "cabinet", "goal_object": "mug", "reasoning": "cups are often stored inside cabinets", "alternatives": ["chair", "window", "plant", "bin"]}
{"action": "TASK_COMPLETE", "target": "", "goal_object": "mug", "reasoning": "the mug was found and confirmed", "alternatives": []}
"""

class ReasoningNode(Node):
    def __init__(self):
        super().__init__("reasoning_node")
        self.declare_parameter('model', 'gemini-3.6-flash')
        api_key = os.getenv('GEMINI_API_KEY')

        if not api_key:
            self.get_logger().error("No API KEY Found")
            return

        self.client = genai.Client(api_key=api_key)
        self.history = []
        self.goal = None



        self.feed_back_sub = self.create_subscription(
            String, '/reasoning_feedback', self.feedback_callback, 10
        )
        self.action_pub = self.create_publisher(
            String, '/reasoning_action', 10
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
            # read per-call so `ros2 param set` switches models live, without a restart
            model = self.get_parameter('model').get_parameter_value().string_value
            self.get_logger().info(f'API CALLED (model={model})')
            response = self.client.models.generate_content(
                model=model,
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

            self.get_logger().info(
                f'Action: {action["action"]} → {action.get("target", "")} | {action["reasoning"]}')

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