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
import math

from .sensor_subscirber import SensorSubscriber

class PIDController:
    def __init__(self, kp, ki, kd, dt=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.prev_error = 0.0
        self.integral = 0.0
    
    def set_kp(self, kp):
        self.kp = kp

    def set_ki(self, ki):
        self.ki = ki

    def set_kd(self, kd):
        self.kd = kd
    
    def set_dt(self, dt):
        self.dt = dt
    
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0


    def compute(self, target, current_value):
        error = current_value - target
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output



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
        
        #=========================
        # PID control parameters
        #=========================
        # TODO: use ROS2 parameters for PID parameters
        self.side_distance_PID = PIDController(kp=0.7, ki=0.001, kd=7.0)
        self.parallel_PID = PIDController(kp=0.9, ki=0.001, kd=7.0)
        self.front_PID = PIDController(kp=0.9, ki=0.001, kd=7.0)
        self.spin_PID = PIDController(kp=0.8, ki=0.001, kd=7.0)

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

        #=======================
        # Car control parameters
        #=======================
        # Spin
        self.initial_pose = 0.0
        self.target_pose = 0.0
        self.start_spin = False

        
        # Sensor availble dictionary
        self.available_sensor_list = []
        
        car_control_cbgroup = MutuallyExclusiveCallbackGroup()
        self.car_remote_control_sub = self.create_subscription(
            msg_type=String,
            topic='control/motor',
            callback=self.control_car_callback,
            qos_profile=self.sensor_qos,
            callback_group=car_control_cbgroup)

        # Setup Arduino board
        PORT = Arduino.AUTODETECT
        self.board = Arduino(PORT)

        # Setup DC motor PWM pin
        self.m1p1= self.board.get_pin('d:5:p')
        self.m1p2 = self.board.get_pin('d:6:p')
        self.m2p1 = self.board.get_pin('d:9:p')
        self.m2p2 = self.board.get_pin('d:10:p')

        self.register_sensors()
        self.register_moves()  

        self.move_thread.start()
        self.get_logger().info("Start driver node")

    def register_sensors(self):
        # Sensors
        self.imu_sub = SensorSubscriber(self, 
                                        "imu", 
                                        "sensor/imu", 
                                        self.sensor_qos, 
                                        self.service_qos)
        self.available_sensor_list.append(self.imu_sub)
        self.us_front_sub = SensorSubscriber(self, 
                                             "us_front", 
                                             "sensor/us_front",
                                             self.sensor_qos, 
                                             self.service_qos)
        self.available_sensor_list.append(self.us_front_sub)
        self.us_sidefront_right_sub = SensorSubscriber(self, 
                                             "us_sidefront_right", 
                                             "sensor/us_sidefront_right",
                                             self.sensor_qos, 
                                             self.service_qos)
        self.available_sensor_list.append(self.us_sidefront_right_sub)
        self.us_siderear_right_sub = SensorSubscriber(self, 
                                             "us_siderear_right", 
                                             "sensor/us_siderear_right",
                                             self.sensor_qos, 
                                             self.service_qos)
        self.available_sensor_list.append(self.us_siderear_right_sub)

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
                   right_base_speed:float=95,
                   left_base_speed:float=90,
                   front_target:float=10.0,
                   target_distance:float=8.0, 
                   backward:float=0.0):
        try:
                # Get sensor data
            front_distance = self.us_front_sub.get_data()
            right_sidefront_distance = self.us_sidefront_right_sub.get_data()
            right_siderear_distance = self.us_siderear_right_sub.get_data()
            if (front_distance is None) or (right_sidefront_distance is None) or (right_siderear_distance is None):
                self.get_logger().info(f"Data is None: {front_distance}, {right_sidefront_distance}, {right_siderear_distance}") 
                return False
            
            # self.get_logger().info(f"Front distance: {front_distance}")
            # reach end point => stop 
            if front_distance <= front_target:
                self.m1p1.write(0)
                self.m1p2.write(0)
                self.m2p1.write(0)
                self.m2p2.write(0)
                self.side_distance_PID.reset()
                self.parallel_PID.reset()
                return True

            # adjust distance error
            # Forward: + => away from wall => left +, right -
            # Backward: + => same as forward
            side_distance = (right_sidefront_distance + right_siderear_distance) / 2.0
            wall_output = self.side_distance_PID.compute(target_distance, side_distance)
            
            # adjust parallel error
            # Forward: + => front farther than rear => left +, right -
            # Backward: + => front farther than rear => left -, right + 
            parallel_error = right_sidefront_distance - right_siderear_distance
            parallel_output = self.parallel_PID.compute(0, parallel_error)
            
            self.get_logger().info(f"P: {parallel_output}, W: {wall_output}")
            # Go forward
            if backward == 0.0:
                self.get_logger().info(f"Go forward")
                right_speed = min(max(right_base_speed - parallel_output - wall_output, right_base_speed), 140)
                left_speed = min(max(left_base_speed + parallel_output + wall_output, left_base_speed), 140)
                self.get_logger().info(f"R: {right_speed}, L:{left_speed}")
                right_duty_cycle = float(right_speed) / 255.0
                left_duty_cycle = float(left_speed) / 255.0
                self.m1p1.write(right_duty_cycle)
                self.m1p2.write(0)
                self.m2p1.write(left_duty_cycle)
                self.m2p2.write(0)
            # Go backward
            else:
                right_speed = min(max(right_base_speed + parallel_output - wall_output, 0), right_base_speed)
                left_speed = min(max(left_base_speed - parallel_output + wall_output, 0), left_base_speed)
                right_duty_cycle = float(right_speed) / 255.0
                left_duty_cycle = float(left_speed) / 255.0
                self.m1p1.write(0)
                self.m1p2.write(right_duty_cycle)
                self.m2p1.write(0)
                self.m2p2.write(left_duty_cycle)
            return False
        except Exception as e:
            self.get_logger().warning(f"[Along Wall] Error:{e}")
            return True
        # end try
        
            
    def straight_approach_target(self, 
                                right_base_speed:float=110,
                                left_base_speed:float=110,
                                side_target_distance:float=15,
                                front_target: float=20,
                                front_target_error:float=1):
        try:
            front_distance = self.us_front_sub.get_data()
            right_sidefront_distance = self.us_sidefront_right_sub.get_data()
            right_siderear_distance = self.us_siderear_right_sub.get_data()

            current_pose = self.imu_sub.get_data()
            current_pose = (current_pose + 360) % 360
            # set initial pose only at start
            if not self.start_spin:
                self.set_target_pose(current_pose)
                with self.lock:
                    self.start_spin = True
            else:
                pose_diff = (self.target_pose - current_pose + 360) % 360
                # normalize angle to 180 -> -180
                pose_diff = (pose_diff + 180) % 360 - 180
                spin_output = self.spin_PID.compute(0, abs(pose_diff))
                # TODO: test and check actual spinning direction???
                if pose_diff >= 0:
                    right_speed = right_base_speed
                    left_speed = min(max(left_base_speed - spin_output, 90.0), 140.0)
                else:
                    right_speed = right_base_speed
                    left_speed = min(max(left_base_speed + spin_output, 90.0), 140.0)
                
                # adjust distance error
                # Forward: + => away from wall => left +, right -
                # Backward: + => same as forward
                
                if not (right_sidefront_distance == 60.0 or right_siderear_distance == 60.0):
                    side_distance = (right_sidefront_distance + right_siderear_distance) / 2.0
                    dist_diff = side_distance - side_target_distance
                    wall_output = self.side_distance_PID.compute(0, abs(dist_diff))
                    if dist_diff > 0:
                        left_speed = min(max(left_speed + wall_output, 90.0), 140.0)
                    elif dist_diff < 0:
                        left_speed = min(max(left_speed - wall_output, 90.0), 140.0)
                
                if not (front_distance == 60.0):
                    front_diff = front_distance - front_target
                    front_output = self.front_PID.compute(0, abs(front_diff))
                    front_output = 40.0 / float(front_output)
                    right_speed = min(max(right_speed - front_output, 80.0), 140.0)
                    left_speed = min(max(left_speed - front_output, 80.0), 140.0)
                    right_duty_cycle = right_speed / 255.0
                    left_duty_cycle = left_speed / 255.0
                    if front_diff > front_target_error:
                        self.m1p1.write(right_duty_cycle)
                        self.m1p2.write(0)
                        self.m2p1.write(left_duty_cycle)
                        self.m2p2.write(0)
                    elif front_diff < -front_target_error:
                        self.m1p1.write(0)
                        self.m1p2.write(right_duty_cycle)
                        self.m2p1.write(0)
                        self.m2p2.write(left_duty_cycle)
                    else:
                        self.m1p1.write(0)
                        self.m1p2.write(0)
                        self.m2p1.write(0)
                        self.m2p2.write(0)
                        self.spin_PID.reset()
                        self.front_PID.reset()
                        self.side_distance_PID.reset()
                        return True

                right_duty_cycle = right_speed / 255.0
                left_duty_cycle = left_speed / 255.0
                self.m1p1.write(right_duty_cycle)
                self.m1p2.write(0)
                self.m2p1.write(left_duty_cycle)
                self.m2p2.write(0)
            return False
        except Exception as e:
            self.get_logger().warning(f"Error:{e}")
            return True
        # end try
        
    def set_initial_pose(self, pose):
        with self.lock:
            # normalize to 0 -> 360
            pose = (pose + 360) % 360
            self.initial_pose = pose
        pass

    def set_target_pose(self, pose):
        with self.lock:
            # normalize to 0 -> 360
            pose = (pose + 360) % 360
            self.target_pose = pose
        pass

    def spin_degree(self, degree: float, adjustment_speed:float=90, error:float=0.01):
        
        try:
            # degree should be smaller than 180 to avoid ambiguity
            # if degree > 0 => clockwise
            current_pose = self.imu_sub.get_data()
            current_pose = (current_pose + 360) % 360
            
            # set initial pose only at start
            if not self.start_spin:
                self.set_initial_pose(current_pose)
                target_pose = (self.initial_pose + degree + 360) % 360
                self.set_target_pose(target_pose)
                with self.lock:
                    self.start_spin = True
            else:
                # check if get to target
                if abs(current_pose - self.target_pose) < error:
                    self.m1p1.write(0)
                    self.m1p2.write(0)
                    self.m2p1.write(0)
                    self.m2p2.write(0)
                    with self.lock:
                        self.start_spin = False
                    self.spin_PID.reset()
                    return True
                # clockwise and counterclockwise
                pose_diff = (math.copysign(1, degree) * (self.target_pose - current_pose)) % 360
                spin_output = self.spin_PID.compute(0, pose_diff)
                duty_cycle = adjustment_speed + spin_output / 255.0
                # TODO: test and check actual spinning direction???
                if degree > 0:
                    self.m1p1.write(0)
                    self.m1p2.write(duty_cycle)
                    self.m2p1.write(duty_cycle)
                    self.m2p2.write(0)
                else:
                    self.m1p1.write(duty_cycle)
                    self.m1p2.write(0)
                    self.m2p1.write(0)
                    self.m2p2.write(duty_cycle)
            return False 
        except Exception as e:
            self.get_logger().warning(f"Error:{e}")
            self.spin_PID.reset()
            return True
        # end try
        

    def align_wall(self, adjustment_speed:float=110, error:float=0.1):
        # TODO: adjust to align wall
        try:
            # degree should be smaller than 180 to avoid ambiguity
            # if degree > 0 => clockwise
            # start_pose = self.imu_sub.get_data()
            # start_pose = (start_pose + 360) % 360
            
            right_sidefront_distance = self.us_sidefront_right_sub.get_data()
            right_siderear_distance = self.us_siderear_right_sub.get_data()
            
            diff = right_sidefront_distance - right_siderear_distance
            output = self.parallel_PID.compute(error, abs(diff))
            new_speed = min(max(adjustment_speed - output, 90), 110.0)
            if (right_sidefront_distance == 60.0 or right_siderear_distance == 60.0):
                new_speed = 110
            duty_cycle = float(new_speed) / 255.0
            self.get_logger().info(f"DC: {duty_cycle}")
            # TODO: test and see if correct
            if diff < -error or \
                right_sidefront_distance == 60.0 or \
                    right_siderear_distance == 60.0:
                self.m1p1.write(duty_cycle)
                self.m1p2.write(0)
                self.m2p1.write(0)
                self.m2p2.write(0)
            elif diff > error:
                self.m1p1.write(0)
                self.m1p2.write(duty_cycle)
                self.m2p1.write(0)
                self.m2p2.write(0)
            else:
                self.get_logger().info(f"Debug distance:{right_sidefront_distance},{right_siderear_distance}")
                self.m1p1.write(0)
                self.m1p2.write(0)
                self.m2p1.write(0)
                self.m2p2.write(0)
                return True
            time.sleep(0.3)
            self.m1p1.write(0)
            self.m1p2.write(0)
            self.m2p1.write(0)
            self.m2p2.write(0)
            time.sleep(0.5)
            # # adjust back if go too far
            # end_pose = self.imu_sub.get_data()
            # end_pose = (end_pose + 360) % 360
            # degree_diff = (end_pose - start_pose + 360) % 360
            # if degree_diff > 180:
            #     degree_diff = degree_diff - 360
            # if degree_diff > 30:

            
            return False 
        except Exception as e:
            self.get_logger().warning(f"[Align Wall] Error:{e}")
            return True
        # end try
        

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
        self.add_commands(commands_string)
        
    
    def add_commands(self, commands_string:str):
        # clean all old commands
        self.commands_queue.clear()
        commands = commands_string.split(',')
        self.get_logger().info(f"Commands: {commands}")
        for element in commands:
            # Split each element by colons
            parts = element.split(':')

            # Extract the key
            command = parts[0]
            args = parts[1].split(';')
            self.get_logger().info(f"Args: {args}")
            float_args = []
            for arg in args:
                
                try:
                    arg = float(arg)
                    float_args.append(arg)
                except ValueError:
                    pass
            self.commands_queue.appendleft([command, float_args])
        pass
    
    def all_sensor_ready(self):
        ready_list = []
        for sensor in self.available_sensor_list:
            ready_list.append(sensor.status())
        return all(ready_list)

    
    def move_task(self):
        is_idle = True
        current_command = "stop"
        command_args = []
        sensors_ready = False
        # dummy test comands
        # along_wall:95;95;15;15;0,
        self.add_commands("align_wall:110;1,straight_approach_target:110;110;15;20;1")
        time.sleep(1)
        
        while not self.break_now_flag:
            if not sensors_ready:
                sensors_ready = self.all_sensor_ready()
                # self.get_logger().warning(f"Sensor not ready")
                continue
            # else: 

            
            if is_idle:
                if len(self.commands_queue) == 0:
                    self.commands_queue.appendleft(["stop", []])
                else:
                    # get a command out of queue
                    command = self.commands_queue.pop()
                    current_command = command[0]
                    command_args = command[1]
                    self.get_logger().info(f"#### Start Command: {current_command} ####")
                    # stop idle state
                    is_idle = False
            else:
                command_function = self.command_dispatcher.get(current_command)
                is_finished = command_function(*command_args)
                if is_finished:
                    self.get_logger().info(f"#### Command Finished!! ####")
                    is_idle = True
                    time.sleep(0.5)
        # time.sleep(0.5)
        # # Stop the motors!
        # self.m1p1.write(0)
        # self.m1p2.write(0)
        # self.m2p1.write(0)
        # self.m2p2.write(0)
    

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
