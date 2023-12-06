import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pyfirmata2 import Arduino

class FanControlNode(Node):

    def __init__(self):
        super().__init__("fan_control_node")

        self.control_subscriber = self.create_subscription(
            String,
            'control/motor',
            self.move_callback,
            1)

        self.arduino = Arduino('/dev/ttyACM0')

        self.TESTPIN = self.arduino.get_pin('d:13:o')

        self.get_logger().info("Start driver node")
        self.get_logger().info("Start driver node")
    
    def move_callback(self, msg: String):

        self.get_logger().info(f"msg: {msg.data}")

        # TODO: Gradually speed up and down
        # TODO: Move in different directions
        if msg.data == 'forward':
            self.TESTPIN.write(True)
        else:
            self.TESTPIN.write(False)


def main(args=None):
    rclpy.init(args=args)
    node = FanControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
