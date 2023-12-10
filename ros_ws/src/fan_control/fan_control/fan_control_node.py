import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pyfirmata2 import Arduino

class FanControlNode(Node):

    def __init__(self):
        super().__init__("fan_control_node")

        self.control_subscriber = self.create_subscription(
            String,
            'control/fan',
            self.fan_callback,
            1)

        PORT = Arduino.AUTODETECT
        self.board = Arduino(PORT)

        self.fan_pin = self.board.get_pin('d:7:o')
        self.fan_pin.write(False)

        self.get_logger().info("Start fan node")
    
    def fan_callback(self, msg: String):

        self.get_logger().info(f"msg: {msg.data}")

        # TODO: Gradually speed up and down
        # TODO: Move in different directions
        if msg.data == 'on':
            self.fan_pin.write(True)
        else:
            self.fan_pin.write(False)


def main(args=None):
    rclpy.init(args=args)
    node = FanControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
