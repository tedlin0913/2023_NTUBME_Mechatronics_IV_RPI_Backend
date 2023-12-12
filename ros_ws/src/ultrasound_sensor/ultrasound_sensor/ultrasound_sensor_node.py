import rclpy
from rclpy.node import Node

from rclpy import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

# from pyfirmata2 import Arduino
from threading import Thread, Lock, Event
from collections import deque
import time
import RPi.GPIO as GPIO
from gpiozero import DistanceSensor


class UltrasoundNode(Node):
    def __init__(self, 
                 srv_cbgroup, 
                 sender_cbgroup):
        super().__init__('ultrasound_sensor_node')

        # TODO: Maybe set this from UI
        # TODO: Start and stop measure from UI
        self.declare_parameters(
            namespace='',
            parameters=[
                ('trig', Parameter.Type.INTEGER),
                ('echo', Parameter.Type.INTEGER),
                # ('target_distance', Parameter.Type.DOUBLE),
                ('topic_name', Parameter.Type.STRING),
            ]
        )
        self.trig_pin = self.get_parameter('trig').get_parameter_value().integer_value
        self.echo_pin = self.get_parameter('echo').get_parameter_value().integer_value
        self.topic = self.get_parameter('topic_name').get_parameter_value().string_value
        # self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value

        self.get_logger().info(f"Trig: {self.trig_pin} Echo: {self.echo_pin}")
        self.get_logger().info(f"Topic: {self.topic}")
        # self.get_logger().info(f"Distance: {self.target_distance}")
        
        # TODO: should be able to switch between best effort and reliable
        # Use best effort QoS reliability policy
        custom_qos_profile = qos_profile_sensor_data
        service_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE)
        
        self.sensor = DistanceSensor(echo=self.echo_pin, 
                                     trigger=self.trig_pin,
                                     max_distance=0.6)


        self.buffer = deque(maxlen=3)

        self.lock = Lock()
        self.measure_thread = Thread(target=self.measure_task)
        
        self.us_pub = self.create_publisher(
            Float32,
            self.topic,
            custom_qos_profile,
            callback_group=sender_cbgroup)
        
        self.timer = self.create_timer(0.5, 
                                       self.sender_callback)
        # self.ready_client = self.create_client(Trigger, 
        #                                         f"{self.topic}_ready",
        #                                         qos_profile=service_qos,
        #                                         callback_group=srv_cbgroup)
        # self.req = Trigger.Request()
        self.measure_thread.start()
        self.get_logger().info("Start ultrasound node")
    
    def measure_task(self):
        while True:
            distance = self.sensor.distance * 100
            self.buffer.append(distance)
            time.sleep(0.1)

    def sender_callback(self):
        if self.buffer:
            distance = self.buffer.pop()
            self.get_logger().info(f"Distance: {distance}")
            us_data = Float32()
            us_data.data = distance
            
            self.us_pub.publish(us_data)
        else: 
            # self.get_logger().info(f"Out of Data.")
            pass


def main(args=None):
    rclpy.init(args=args)

    try:
        srv_cbgroup = MutuallyExclusiveCallbackGroup()
        sender_cbgroup = MutuallyExclusiveCallbackGroup()
        executor = MultiThreadedExecutor()
        node = UltrasoundNode(srv_cbgroup=srv_cbgroup, 
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

