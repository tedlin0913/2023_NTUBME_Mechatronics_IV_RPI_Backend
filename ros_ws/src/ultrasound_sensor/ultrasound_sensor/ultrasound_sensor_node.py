import rclpy
from rclpy.node import Node

from rclpy import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32

from pyfirmata2 import Arduino
from threading import Thread, Lock, Event
from collections import deque
import time
# import RPi.GPIO as GPIO


class UltrasoundNode(Node):
    def __init__(self):
        super().__init__('ultrasound_sensor_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('trig', 0),
                ('echo', 0),
                ('target_distance', 8),
                ('topic_name', ''),
            ]
        )
        self.trig_pin = self.get_parameter('trig').get_parameter_value().integer_value
        self.echo_pin = self.get_parameter('echo').get_parameter_value().integer_value
        self.topic = self.get_parameter('topic_name').get_parameter_value().string_value
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value

        self.get_logger().info(f"Trig: {self.trig_pin} Echo: {self.echo_pin}")
        self.get_logger().info(f"Topic: {self.topic}")
        self.get_logger().info(f"Distance: {self.target_distance}")
        # TODO: should be able to switch between best effort and reliable
        # Use best effort QoS reliability policy
        custom_qos_profile = qos_profile_sensor_data
        
        PORT =  Arduino.AUTODETECT
        self.board = Arduino(PORT)


        self.buffer = deque(maxlen=10)

        self.lock = Lock()
        self.event = Event()
        self.measure_thread = Thread(target=self.measure_task)

        # self.us1d_publisher = self.create_publisher(
        #     Float32,
        #     'sensor/us1d',
        #     custom_qos_profile)

        # self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("Start ultrasound node")

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

    def measure_task(self):
        # Trigger pulse
        with self.lock: 
            self.board.digital[self.trig_pin].write(1)
            time.sleep(0.00001)
            self.board.digital[self.trig_pin].write(0)

            # Wait for the pulse to be sent
            while self.board.digital[self.echo_pin].read() == 0:
                pulse_start = time.time()

            # Wait for the pulse to return
            while self.board.digital[self.echo_pin].read() == 1:
                pulse_end = time.time()

        # Calculate distance
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Speed of sound is 343 meters per second (17150 = 343 * 100 / 2)
        self.buffer.append(distance)
        time.sleep(0.2)

    def timer_callback(self):
        self.buffer.pop()
        pass


def main(args=None):
    rclpy.init(args=args)

    try:
        executor = MultiThreadedExecutor()
        node = UltrasoundNode()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()



        # # custom_qos_profile = QoSProfile(
        # #     depth=10,
        # #     reliability=QoSReliabilityPolicy.RELIABLE)
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setwarnings(False)

        # self.GPIO_TRIGGER = trigger_pin
        # self.GPIO_ECHO    = echo_pin

        # GPIO.setup(self.GPIO_TRIGGER,GPIO.OUT)  # Trigger
        # GPIO.setup(self.GPIO_ECHO,GPIO.IN)      # Echo
        
        # GPIO.output(self.GPIO_TRIGGER, False)


        # # TODO: set pin as ros2 parameters
        # # self.sensor = DistanceSensor(echo=echo_pin, trigger=trigger_pin)

        # # US sensor 1 distance publisher


        # GPIO.output(self.GPIO_TRIGGER, True)
        # time.sleep(0.00001)
        # GPIO.output(self.GPIO_TRIGGER, False)
        # self.start = time.time()
        # self.stop = time.time()

        # while GPIO.input(self.GPIO_ECHO)==0:
        #     self.start = time.time()

        # while GPIO.input(self.GPIO_ECHO)==1:
        #     self.stop = time.time()

        # # Calculate pulse length
        # elapsed = self.stop-self.start

        # # Distance pulse travelled in that time is time
        # # multiplied by the speed of sound (cm/s)
        # distancet = elapsed * 34300

        # # That was the distance there and back so halve the value
        # distance = float(distancet) / 2.0

        # self.get_logger().info(f"Distance: {distance}")