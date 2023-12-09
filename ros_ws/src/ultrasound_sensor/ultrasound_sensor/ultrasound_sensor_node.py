import rclpy
from rclpy.node import Node

from rclpy import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Float32

from pyfirmata2 import Arduino
from threading import Thread, Lock, Event
from collections import deque
import time
import RPi.GPIO as GPIO


class UltrasoundNode(Node):
    def __init__(self, 
                 sensor_cbgroup, 
                 sender_cbgroup):
        super().__init__('ultrasound_sensor_node')

        # TODO: Maybe set this from UI
        # TODO: Start and stop measure from UI
        self.declare_parameters(
            namespace='',
            parameters=[
                ('trig', Parameter.Type.INTEGER),
                ('echo', Parameter.Type.INTEGER),
                ('target_distance', Parameter.Type.DOUBLE),
                ('topic_name', Parameter.Type.STRING),
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
        
        # PORT =  Arduino.AUTODETECT
        # self.board = Arduino(PORT)

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self.GPIO_TRIGGER = self.trig_pin
        self.GPIO_ECHO    = self.echo_pin

        GPIO.setup(self.GPIO_TRIGGER,GPIO.OUT)  # Trigger
        GPIO.setup(self.GPIO_ECHO,GPIO.IN)      # Echo
        
        GPIO.output(self.GPIO_TRIGGER, False)


        self.buffer = deque(maxlen=10)

        # self.lock = Lock()
        # self.event = Event()
        # self.measure_thread = Thread(target=self.measure_task)
        

        self.us_pub = self.create_publisher(
            Float32,
            self.topic,
            custom_qos_profile)
        self.sensor_timer = self.create_timer(0.4, 
                                              self.sensor_callback, 
                                              callback_group=sensor_cbgroup)
        self.timer = self.create_timer(0.5, 
                                       self.sender_callback,
                                       callback_group=sender_cbgroup)
        self.get_logger().info("Start ultrasound node")
        # self.measure_thread.start()

    # def start_measure(self):
    #     if self.measure_thread.is_alive() is False:
    #         self.event.clear()
    #         self.measure_thread.start()
    #         self.get_logger().info("Start angle thread")
    #     else:
    #         self.get_logger().warning("Thread is already running! stop it first.")
    
    # def stop_measure(self):
    #     if self.measure_thread.is_alive():
    #         self.event.set()
    #         self.get_logger().info("Stop angle thread")

    def sensor_callback(self):

        pulse_start = 0.0
        pulse_end = 0.0
        elapsed = 0.0
        distance = 0.0
        time_out_duration = 0.05
        is_time_out = False
        get_pulse = False
        
        # while True:
        GPIO.output(self.GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)
        
        pulse_start = time.time()
        pulse_end = time.time()
        start_time = time.time()
        while GPIO.input(self.GPIO_ECHO)==0:
            pulse_start = time.time()
            if time.time() - start_time > time_out_duration:
                is_time_out = True
                break
            # self.get_logger().info(f"+++++")
        if not is_time_out:
            start_time = time.time()
            while GPIO.input(self.GPIO_ECHO)==1:
                pulse_end  = time.time()
                get_pulse = True
                if time.time() - start_time > time_out_duration:
                    is_time_out = True
                    break
            # self.get_logger().info(f"=====")
        # # pulse_end = time.time()
        # while time.time() - start_time < time_out:
        #     if GPIO.input(self.GPIO_ECHO)==0:
        #         pulse_start = time.time()
        #         break
        
        # start_time = time.time()
        # while time.time() - start_time < time_out:
        #     if GPIO.input(self.GPIO_ECHO)==1:
        #         pulse_end = time.time()
        #         get_pulse = True
        #         break
        if get_pulse:
            elapsed = pulse_end - pulse_start

            # That was the distance there and back so halve the value
            distance =  elapsed * 34300 / 2.0
            # self.get_logger().info(f"Distance: {distance:.2f}")
            self.buffer.append(distance)
        # if get_pulse:
        #     # Calculate pulse length
        #     elapsed = pulse_end - pulse_start

        #     # That was the distance there and back so halve the value
        #     distance =  elapsed * 34300 / 2.0
        #     self.get_logger().info(f"Distance: {distance:.2f}")
        #     self.buffer.append(distance)
        
        # time.sleep(0.5)
    
    # def echo_callback(self):
        
    #     pass

    def sender_callback(self):
        if self.buffer:
            distance = self.buffer.pop()
            # self.get_logger().info(f"Distance: {distance}")
            us_data = Float32()
            us_data.data = distance
            
            self.us_pub.publish(us_data)
        else: 
            # self.get_logger().info(f"Out of Data.")
            pass


def main(args=None):
    rclpy.init(args=args)

    try:
        sensor_cbgroup = MutuallyExclusiveCallbackGroup()
        sender_cbgroup = MutuallyExclusiveCallbackGroup()
        executor = MultiThreadedExecutor()
        node = UltrasoundNode(sensor_cbgroup=sensor_cbgroup, 
                              sender_cbgroup=sender_cbgroup)
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