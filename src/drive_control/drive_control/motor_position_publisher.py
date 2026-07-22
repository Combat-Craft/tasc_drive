import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class MotorPositionPublisher(Node):
    def __init__(self) -> None:
        super().__init__('motor_position_publisher')
        self.publisher = self.create_publisher(JointState, '/rover/drive/motor_positions', 10)
        self.subscription = self.create_subscription(
            String,
            '/rover/drive/motor_telemetry',
            self.telemetry_callback,
            10,
        )

    def telemetry_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Malformed motor telemetry JSON received')
            return

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()

        for motor in payload.get('motors', []):
            if not motor.get('attached', False):
                continue
            joint_state.name.append(motor.get('name', 'unknown_motor'))
            joint_state.position.append(float(motor.get('position', 0.0)))
            joint_state.velocity.append(float(motor.get('velocity', 0.0)))

        self.publisher.publish(joint_state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorPositionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
