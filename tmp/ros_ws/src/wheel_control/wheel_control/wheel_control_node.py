import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32, String

from pyfirmata2 import Arduino
from threading import Thread, Lock, Event
from collections import deque
import time

class WheelControlNode(Node):

    def __init__(self):
        super().__init__("wheel_control_node")

        # TODO: should be able to switch between best effort and reliable
        # Use best effort QoS reliability policy
        custom_qos_profile = qos_profile_sensor_data

        # custom_qos_profile = QoSProfile(
        #     depth=10,
        #     reliability=QoSReliabilityPolicy.RELIABLE)

        # TODO: use ROS2 parameters for PID parameters

        self.Kp = 0.8
        self.Ki = 0.00001
        self.Kd = 7.0

        self.lock = Lock()
        self.event = Event()
        self.move_thread = Thread(target=self.move_task)

        self.us_front_sub = self.create_subscription(
            Float32,
            'sensor/us_front',
            self.us_front_callback,
            custom_qos_profile)
        self.us_sidefront_sub = self.create_subscription(
            Float32,
            'sensor/us_sidefront',
            self.us_sidefront_callback,
            custom_qos_profile)
        self.us_siderear_sub = self.create_subscription(
            Float32,
            'sensor/us_siderear',
            self.us_siderear_callback,
            custom_qos_profile)

        # Setup Arduino board
        PORT = Arduino.AUTODETECT
        self.board = Arduino(PORT)

        # Setup DC motor PWM pin
        self.pwm_1 = self.board.get_pin('d:5:p')
        self.pwm_2 = self.board.get_pin('d:6:p')
        self.pwm_3 = self.board.get_pin('d:10:p')
        self.pwm_4 = self.board.get_pin('d:11:p')

        # Set speed
        # TODO: should be able to change speed from UI, Use server client.
        self.speed = float(100) / 100.0

        self.get_logger().info("Start driver node")

    def start_measure(self):
        if self.measure_thread.is_alive() is False:
            self.event.clear()
            self.measure_thread.start()
            self.get_logger().info("Start angle thread")
        else:
            self.get_logger().warning("Thread is already running! stop it first.")
    
    def stop_measure(self):
        if self.measure_thread.is_alive():
            self.event.set()
            self.get_logger().info("Stop angle thread")

    def move_task(self):
        # while self.
        pass
    
    

    def move_callback(self, msg: String):

        self.get_logger().info(f"Move Direction: {msg.data}")

        # TODO: Gradually speed up and down
        # TODO: Move in different directions (check pin)

        # TODO: Direction and spining angle will be adjusted using IMU gyro
        # Move forward
        if msg.data == 'forward':
            self.pwm_1.write(self.speed)
            self.pwm_2.write(self.speed)
            self.pwm_3.write(self.speed)
            self.pwm_4.write(self.speed)
        # Move backward
        elif msg.data == 'backward':
            self.pwm_1.write(self.speed)
            self.pwm_2.write(self.speed)
            self.pwm_3.write(self.speed)
            self.pwm_4.write(self.speed)
        # Move left
        elif msg.data == 'left':
            self.pwm_1.write(self.speed)
            self.pwm_2.write(self.speed)
            self.pwm_3.write(self.speed)
            self.pwm_4.write(self.speed)
        # Move right
        elif msg.data == 'right':
            self.pwm_1.write(self.speed)
            self.pwm_2.write(self.speed)
            self.pwm_3.write(self.speed)
            self.pwm_4.write(self.speed)
        # Stop
        elif msg.data == 'stop':
            self.pwm_1.write(0)
            self.pwm_2.write(0)
            self.pwm_3.write(0)
            self.pwm_4.write(0)


def main(args=None):
    rclpy.init(args=args)

    try:
        executor = MultiThreadedExecutor()
        node = WheelControlNode()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
