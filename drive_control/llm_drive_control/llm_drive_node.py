# Should be under ASIMOV>src>controller>controller
import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from controller import llama_cpp_script 


REQUIRED_KEYS = {"action", "direction", "speed", "duration_s"}
ALLOWED_DIRECTIONS = {"forward", "backward", "left", "right", "stop"}


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class LlmDriveNode(Node):
    """
    Lean LLM-drive node:
      - reads terminal prompts
      - calls llama_cpp_script.generate_text_from_prompt(...)
      - parses + validates JSON
      - publishes JSON to /motor_llm_cmd
      - publishes Twist keepalive to /cmd_vel (prevents motor_driver watchdog exit)
    """

    def __init__(self):
        super().__init__("llm_drive_node")

        # ---- hard-coded "integration contract" ----
        self.JSON_TOPIC = "/motor_llm_cmd"
        self.CMD_VEL_TOPIC = "/cmd_vel"

        # motor_driver watchdog compatibility: must publish regularly
        self.PUBLISH_HZ = 10.0

        # hard-coded safety limits
        self.MAX_SPEED = 1.0
        self.MAX_DURATION_S = 10.0

        # LLM settings (match the script defaults)
        self.LLM_MAX_TOKENS = 80
        self.LLM_TEMPERATURE = 0.3
        self.LLM_TOP_P = 0.5

        self.json_pub = self.create_publisher(String, self.JSON_TOPIC, 10)
        self.cmd_pub = self.create_publisher(Twist, self.CMD_VEL_TOPIC, 10)

        # active command state
        self._lock = threading.Lock()
        self._active_twist = Twist()  # default stop
        self._end_time = 0.0          # epoch seconds; 0 means idle

        # keepalive publisher loop
        self.create_timer(1.0 / self.PUBLISH_HZ, self.publish_loop)

        # terminal input loop in background
        threading.Thread(target=self.terminal_loop, daemon=True).start()

        self.get_logger().info(f"Publishing JSON on {self.JSON_TOPIC}")
        self.get_logger().info(f"Publishing Twist on {self.CMD_VEL_TOPIC} @ {self.PUBLISH_HZ} Hz (keepalive)")
        self.get_logger().info("Type a prompt (or 'quit').")

    def terminal_loop(self):
        while rclpy.ok():
            try:
                user_in = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not user_in:
                continue
            if user_in.lower() in ("q", "quit", "exit"):
                break

            self.handle_prompt(user_in)

    def handle_prompt(self, user_text: str):
        # Hard-coded contract instruction (no delimiter tricks)
        prompt = f"Return ONLY a single-line JSON object. {user_text}\n"

        try:
            out = llama_cpp_script.generate_text_from_prompt(
                prompt,
                max_tokens=self.LLM_MAX_TOKENS,
                temperature=self.LLM_TEMPERATURE,
                top_p=self.LLM_TOP_P,
                echo=False,
                stop=None,
            )

            raw = out["choices"][0]["text"].strip()
            cmd = json.loads(raw)
            self.validate_and_sanitize(cmd)

            # publish the structured command JSON for logging/debug
            self.publish_json(cmd)

            # schedule motion
            twist, duration = self.cmd_to_twist(cmd)
            with self._lock:
                self._active_twist = twist
                self._end_time = time.time() + duration

            self.get_logger().info(
                f"Exec {duration:.2f}s: dir={cmd['direction']} speed={cmd['speed']}"
            )

        except Exception as e:
            self.get_logger().error(f"Rejected: {e}")

    def validate_and_sanitize(self, cmd: dict):
        if not isinstance(cmd, dict):
            raise ValueError("Command is not a JSON object.")

        missing = REQUIRED_KEYS - set(cmd.keys())
        if missing:
            raise ValueError(f"Missing keys: {sorted(missing)}")

        if cmd["action"] != "drive":
            raise ValueError(f"Unsupported action: {cmd['action']}")

        if cmd["direction"] not in ALLOWED_DIRECTIONS:
            raise ValueError(f"Unsupported direction: {cmd['direction']}")

        cmd["speed"] = clamp(float(cmd["speed"]), 0.0, self.MAX_SPEED)
        cmd["duration_s"] = clamp(float(cmd["duration_s"]), 0.0, self.MAX_DURATION_S)

    def cmd_to_twist(self, cmd: dict):
        t = Twist()
        d = cmd["direction"]
        s = float(cmd["speed"])
        dur = float(cmd["duration_s"])

        # motor_driver computes wheel speeds from linear.x and angular.z
        if d == "forward":
            t.linear.x = +s
        elif d == "backward":
            t.linear.x = -s
        elif d == "left":
            t.angular.z = +s
        elif d == "right":
            t.angular.z = -s
        elif d == "stop":
            dur = 0.0  # keepalive loop will publish stop

        return t, dur

    def publish_json(self, cmd: dict):
        msg = String()
        msg.data = json.dumps(cmd, separators=(",", ":"))
        self.json_pub.publish(msg)
        self.get_logger().info(f"/motor_llm_cmd: {msg.data}")

    def publish_loop(self):
        now = time.time()
        with self._lock:
            if now < self._end_time:
                t = self._active_twist
            else:
                t = Twist()  # keepalive stop

        self.cmd_pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = LlmDriveNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
