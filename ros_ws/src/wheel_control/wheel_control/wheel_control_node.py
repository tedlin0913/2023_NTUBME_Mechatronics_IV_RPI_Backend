import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

from pyfirmata2 import Arduino
from threading import Thread, Lock, Event
from collections import deque
import time

from .sensor_subscirber import SensorSubscriber
from .move_thread import *

# class PIDController:
#     def __init__(self, kp, ki, kd):
#         self.kp = kp
#         self.ki = ki
#         self.kd = kd
#         self.prev_error = 0
#         self.integral = 0

#     def compute(self, setpoint, current_value, dt):
#         error = setpoint - current_value
#         self.integral += error * dt
#         derivative = (error - self.prev_error) / dt
#         output = self.kp * error + self.ki * self.integral + self.kd * derivative
#         self.prev_error = error
#         return output



class WheelControlNode(Node):

    def __init__(self):
        super().__init__("wheel_control_node")

        

        # ============================
        # Quality of service settings
        # ============================
        # TODO: should be able to switch between best effort and reliable
        # Use best effort QoS reliability policy
        self.sensor_qos = qos_profile_sensor_data
        self.service_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE)
        

        self.register_sensors()
        self.register_moves()
        
        
        #=========================
        # PID control parameters
        #=========================
        # TODO: use ROS2 parameters for PID parameters
        self.wall_kp = 1.8
        self.wall_ki = 0.001
        self.wall_kd = 7.0
        self.wall_last_error = 0.0
        self.wall_integral = 0.0
        
        self.parallel_kp = 0.8
        self.parallel_ki = 0.001
        self.parallel_kd = 7.0
        self.parallel_last_error = 0.0
        self.parallel_integral = 0.0

        #=======================
        # Car control threads 
        #=======================
        self.lock = Lock()
        self.stop_now_flag = False
        self.break_now_flag = False
        self.commands_queue = deque()
        # self.executing_command = False
        self.move_thread = Thread(target=self.move_task)
        # Auto control route #1
        
        # Sensor availble dictionary
        self.sensor_available_dict = {
            "imu": False,
            "us_front": False,
            "us_sidefront_right": False,
            "us_siderear_right": False,
            "us_sidefront_left": False,
            "us_siderear_left": False
        }
        
        
        self.car_remote_control_sub = self.create_subscription(
            msg_type=String,
            topic='control/motor',
            callback=self.control_car_callback,
            qos_profile=self.sensor_data_qos_profile,
            callback_group=self.car_control_cbgroup)

        # Setup Arduino board
        PORT = Arduino.AUTODETECT
        self.board = Arduino(PORT)

        # Setup DC motor PWM pin
        self.m1p1= self.board.get_pin('d:5:p')
        self.m1p2 = self.board.get_pin('d:6:p')
        self.m2p1 = self.board.get_pin('d:10:p')
        self.m2p2 = self.board.get_pin('d:11:p')

        # Set speed
        # TODO: should be able to change speed from UI, Use server client.
        self.speed = float(100) / 100.0

        self.get_logger().info("Start driver node")

    def register_sensors(self):
        # Sensors
        self.imu_sub = SensorSubscriber(self, 
                                        "imu", 
                                        "sensor/imu", 
                                        self.sensor_qos, 
                                        self.service_qos)
        self.us_front_sub = SensorSubscriber(self, 
                                             "us_front", 
                                             "sensor/us_front",
                                             self.sensor_qos, 
                                             self.service_qos)
        self.us_sidefront_right_sub = SensorSubscriber(self, 
                                             "us_sidefront_right", 
                                             "sensor/us_sidefront_right",
                                             self.sensor_qos, 
                                             self.service_qos)
        self.us_siderear_right_sub = SensorSubscriber(self, 
                                             "us_siderear_right", 
                                             "sensor/us_siderear_right",
                                             self.sensor_qos, 
                                             self.service_qos)

    def register_moves(self):
        self.command_dispatcher = {
            "straight_forward": self.straight_forward,
            "straight_backward": self.straight_backward,
            "straight_approach_target": self.straight_approach_target,
            "along_wall": self.along_wall,
            "align_wall": self.align_wall,
            "align_center": self.align_center,
            "align_corner": self.align_corner,
            "spin_degree": self.spin_degree,
            "stop": self.stop,
        }

    # =============================
    # CAR CONTROL 
    # =============================
    def stop(self):
        try:
            self.m1p1.write(0)
            self.m1p2.write(0)
            self.m2p1.write(0)
            self.m2p2.write(0)
        except Exception as e:
            self.get_logger().warning(f"Error: {e}")
        finally:
            return True

    def straight_forward(self) -> bool:
        try:
            return False
        except Exception as e:
            self.get_logger().warning(f"")
            return True
            

    def straight_backward(self):
        # TODO: move straight backward
        pass

    def along_wall(self, 
                   right_base_speed:float,
                   left_base_speed:float,
                   target_distance:float=8.0, 
                   backward:float=0.0):
        
        # adjust distance error
        right_sidefront_distance = self.us_sidefront_right_sub.get_data()
        right_siderear_distance = self.us_siderear_right_sub.get_data()
        wall_error = (right_sidefront_distance + right_siderear_distance)/2.0 - target_distance
        self.wall_integral += wall_error
        wall_derivative = wall_error - self.wall_last_error
        wall_output = self.wall_kp * wall_error + self.wall_ki * self.wall_integral + self.wall_kd * wall_derivative
        
        # adjust parallel error
        parallel_error = right_sidefront_distance - right_siderear_distance
        self.parallel_integral += parallel_error
        derivative = parallel_error - self.parallel_last_error
        parallel_output = self.parallel_kd * parallel_error + \
            self.parallel_ki * self.parallel_integral + \
                self.parallel_kp * derivative
        right_speed = min(max(right_base_speed - parallel_output + wall_output, 0), 140)
        left_speed = min(max(left_base_speed + parallel_output - wall_output, 0), 140)
        
        if backward == 0.0:
            self.m1p1.write(right_speed)
            self.m1p2.write(0)
            self.m2p1.write(left_speed)
            self.m2p2.write(0)
        else:
            self.m1p1.write()
            self.m1p2.write(right_speed)
            self.m2p1.write()
            self.m2p2.write(left_speed)

    def straight_approach_target(self, target: float):
        # TODO: move straight to approach front target
        pass

    def spin_degree(self, degree: float, clockwise=1):
        # TODO: spin a degree
        pass

    def align_wall(self, adjustment_speed:float=110, error:float=0.01):
        # TODO: adjust to align wall
        right_sidefront_distance = self.us_sidefront_right_sub.get_data()
        right_siderear_distance = self.us_siderear_right_sub.get_data()
        duty_cycle = adjustment_speed / 255.0

        if right_sidefront_distance - right_siderear_distance > error:
            self.m1p1.write(0)
            self.m1p2.write(duty_cycle)
            self.m2p1.write(duty_cycle)
            self.m2p2.write(0)
        elif right_sidefront_distance - right_siderear_distance < -error:
            self.m1p1.write(duty_cycle)
            self.m1p2.write(0)
            self.m2p1.write(0)
            self.m2p2.write(duty_cycle)
        else:
            self.m1p1.write(0)
            self.m1p2.write(0)
            self.m2p1.write(0)
            self.m2p2.write(0)
            return True
        return False

    def align_corner(self, adjustment_speed:float=110, error:float=0.01):
        # TODO: adjust to align corner
        # 
        front_distance = self.us_front_sub.get_data()
        right_sidefront_distance = self.us_sidefront_right_sub.get_data()
        right_siderear_distance = self.us_siderear_right_sub.get_data()
        duty_cycle = adjustment_speed / 255.0

        pass

    def align_center(self):
        # TODO: adjust to align center of the test field

        pass

    def auto_1_task(self):
        # while self.
        pass

    def get_command(self):
        return self.current_command

    def control_car_callback(self, msg:String):
        # TODO: This should be a service and after setting the command a 
        # message is send back to the remote control device to notify the user.
        
        commands_string = msg.data
        commands = commands_string.split(',')
        # clean all old commands
        self.commands_queue.clear()
        for element in commands:
            # Split each element by colons
            parts = element.split(':')
            # Extract the key
            command = parts[0]
            args = []
            for arg in parts[1].split(';'):
                try:
                    arg = float(arg)
                    args.append(arg)
                except ValueError:
                    pass
            self.commands_queue.appendleft([command, args])
    
    def set_start_pose(self, yaw_angle):
        # TODO: set initial position
        pass

    def move_callback(self, msg: String):


        self.get_logger().info(f"Move Direction: {msg.data}")

        # TODO: Gradually speed up and down
        # TODO: Move in different directions (check pin)

        # TODO: In corner => open vacuum
        # TODO: detect danger corner => don't open vacuum
        # TODO: detect center => open vacuum
        # TODO: Start => move straight
        # TODO: Turning  


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
            pass
    
    def all_senosr_ready(self):
        pass

    
    def move_task(self):
        is_idle = True
        current_command = "stop"
        command_args = []
        
        while not self.break_now_flag:
            if is_idle:
                if len(self.commands_queue) == 0:
                    self.commands_queue.appendleft(["stop", []])
                else:
                    # get a command out of queue
                    command = self.commands_queue.pop()
                    current_command = command[0]
                    command_args = command[1]
                    # stop idle state
                    is_idle = False
            else:
                command_function = self.command_dispatcher.get(current_command)
                is_finished = command_function(*command_args)
                if is_finished:
                    is_idle = True
            

            
            pass

# class MoveThread(Thread):
#     def __init__(self, 
#                  node: WheelControlNode, 
#                  command: str,
#                  command_dispatcher: dict, 
#                  stop_flag: bool,
#                  *args):
#         super().__init__()

#         self.command = command
#         self.node = node
#         self.stop_flag = stop_flag
#         self.command_dispatcher = command_dispatcher
    
#     def default_command(self):
#         self.stop_flag = False
#         pass
    
#     def run(self):
#         while not self.stop_flag:
#             self.command_dispatcher.get(self.command, self.default_command)
#             pass  # do something



    

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
