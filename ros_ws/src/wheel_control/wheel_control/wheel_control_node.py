import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Float32, String

from pyfirmata2 import Arduino
from threading import Thread, Lock, Event
from collections import deque
import time

class WheelControlNode(Node):

    def __init__(self,
                 car_cbgroup,
                 us_front_cbgroup,
                 us_sidefront_cbgroup,
                 us_siderear_cbgroup,
                 imu_cbgroup):
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

        self.car_control_sub = self.create_subscription(
            msg_type=String,
            topic='control/motor',
            callback=self.control_car_callback,
            qos_profile=custom_qos_profile,
            callback_group=car_cbgroup)
        
        # Sensors
        self.us_front_sub = self.create_subscription(
            msg_type=Float32,
            topic='sensor/us_front',
            callback=self.us_front_callback,
            qos_profile=custom_qos_profile,
            callback_group=us_front_cbgroup)
        self.us_sidefront_sub = self.create_subscription(
            msg_type=Float32,
            topic='sensor/us_sidefront',
            callback=self.us_sidefront_callback,
            qos_profile=custom_qos_profile,
            callback_group=us_sidefront_cbgroup)
        self.us_siderear_sub = self.create_subscription(
            msg_type=Float32,
            topic='sensor/us_siderear',
            callback=self.us_siderear_callback,
            qos_profile=custom_qos_profile,
            callback_group=us_siderear_cbgroup)
        self.imu_yaw = self.create_subscription(
            msg_type=Float32,
            topic='sensor/imu',
            callback=self.imu_callback,
            qos_profile=custom_qos_profile,
            callback_group=imu_cbgroup)
        
        self.us_front = 0.0
        self.us_sidefront = 0.0
        self.us_siderear = 0.0
        self.imu_yaw = 0.0

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
    
    def control_car_callback(self, msg:String):
        self.get_logger().info(f"USF: {self.us_front}\tUSSF: {self.us_sidefront}\tUSFR: {self.us_siderear}")
        
        
        pass
    
    def us_front_callback(self, msg:Float32):
        with self.lock:
            self.us_front = msg.data
    
    def us_sidefront_callback(self, msg:Float32):
        with self.lock:
            self.us_sidefront = msg.data

    def us_siderear_callback(self, msg:Float32):
        with self.lock:
            self.us_siderear = msg.data

    def imu_callback(self, msg:Float32):
        with self.lock:
            self.imu_yaw = msg.data
        self.get_logger().info(f"IMU: {self.imu_yaw}")

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
        car_cbgroup = MutuallyExclusiveCallbackGroup()
        us_front_cbgroup = MutuallyExclusiveCallbackGroup()
        us_sidefront_cbgroup = MutuallyExclusiveCallbackGroup()
        us_siderear_cbgroup = MutuallyExclusiveCallbackGroup()
        imu_cbgroup = MutuallyExclusiveCallbackGroup()
        executor = MultiThreadedExecutor()
        node = WheelControlNode(
            car_cbgroup=car_cbgroup,
            us_front_cbgroup=us_front_cbgroup,
            us_sidefront_cbgroup=us_sidefront_cbgroup,
            us_siderear_cbgroup=us_siderear_cbgroup,
            imu_cbgroup=imu_cbgroup
        )
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
