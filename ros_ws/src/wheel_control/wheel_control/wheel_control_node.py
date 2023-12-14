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

from .sensor_subscriber import SensorSubscriber
from .remote_subscriber import RemoteSubscriber

class PIDController:
    def __init__(self, kp, ki, kd, dt=1):
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


    def compute(self, current_value, target):
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
        self.side_distance_PID = PIDController(kp=0.8, ki=0, kd=20, dt=1)
        self.parallel_PID = PIDController(kp=1.2, ki=0, kd=20, dt=1)
        self.front_PID = PIDController(kp=0.9, ki=0.001, kd=7.0)
        self.spin_PID = PIDController(kp=3.0, ki=0, kd=200, dt=1)

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
        self.start_approach = False
        self.min_speed = 100.0
        self.max_speed = 200.0
        self.us_max_range = 80.0

        
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

        self.fan_pin = self.board.get_pin('d:7:o')
        # Setup DC motor PWM pin
        # RIGHT
        self.m1p1= self.board.get_pin('d:5:p')
        self.m1p2 = self.board.get_pin('d:6:p')
        # LEFT
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
            "dummy_test": self.dummy_test,
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
                   right_base_speed:float=110,
                   left_base_speed:float=110,
                   front_target:float=25.0,
                   side_target:float=15.0):
        try:
            
                # Get sensor data
            front_distance = self.us_front_sub.get_data()
            right_sidefront_distance = self.us_sidefront_right_sub.get_data()
            right_siderear_distance = self.us_siderear_right_sub.get_data()
            # if (front_distance is None) or (right_sidefront_distance is None) or (right_siderear_distance is None):
            #     self.get_logger().info(f"Data is None: {front_distance}, {right_sidefront_distance}, {right_siderear_distance}") 
            #     return False
            
            # self.get_logger().info(f"Front distance: {front_distance}")
            # reach end point => stop 
            if front_distance < front_target:
                self.m1p1.write(0)
                self.m1p2.write(0)
                self.m2p1.write(0)
                self.m2p2.write(0)
                time.sleep(0.3)
                self.side_distance_PID.reset()
                self.parallel_PID.reset()
                self.start_approach = False
                return True
            
            current_pose = self.imu_sub.get_data()
            current_pose = (current_pose + 360) % 360
            # set initial pose only at start
            # if not self.start_spin:
            #     self.set_target_pose(current_pose)
            #     with self.lock:
            #         self.start_spin = True
            # else:
            #     pass

            if ((right_sidefront_distance == self.us_max_range) or (right_siderear_distance == self.us_max_range)):
                finish_align_wall = False
                while not finish_align_wall:
                    finish_align_wall = self.align_wall(right_base_speed, 1)
                time.sleep(0.5)


            #     self.m1p1.write(0)
            #     self.m1p2.write(0)
            #     self.m2p1.write(0)
            #     self.m2p2.write(0)
            #     time.sleep(0.5)
                
                
                    # self.start_approach = True

            #     right_base_speed = right_base_speed - 10
            #     left_base_speed = left_base_speed - 10
                
            
            # if front_distance < 30.0 and self.start_approach:
            #     # time.sleep(0.1)
            #     self.m1p1.write(0)
            #     self.m1p2.write(0)
            #     self.m2p1.write(0)
            #     self.m2p2.write(0)
            #     self.side_distance_PID.reset()
            #     self.parallel_PID.reset()
            #     time.sleep(0.1)
            #     finish_align_wall = False
            #     while not finish_align_wall:
            #         finish_align_wall = self.align_wall(ajustment_speed, ajustment_error)
                # self.start_approach = True
            
            # adjust distance error
            # Forward: + => away from wall => left +, right -
            # Backward: + => same as forward
            side_distance = (right_sidefront_distance + right_siderear_distance) / 2.0
            # side_error = side_distance - side_target
            
            wall_output = self.side_distance_PID.compute(side_distance, side_target)
            # wall_output = math.copysign(1.0, side_error) * wall_output 
            # adjust parallel error
            # Forward: + => front farther than rear => left +, right -
            # Backward: + => front farther than rear => left -, right + 
            # parallel_error = right_sidefront_distance - right_siderear_distance
            parallel_output = self.parallel_PID.compute(right_sidefront_distance, right_siderear_distance)
            # parallel_output = math.copysign(1.0, parallel_error) * parallel_output
            
            
            # final_output = wall_output + parallel_output
            # if abs(side_error) > error:
            #     # if far away from the wall we want it to correct to close to the wall asap
            #     final_output = 0.9 * wall_output + 0.1 * parallel_output
            #     self.get_logger().info(f">>>ERROR: FO: {final_output}")
            # else:
            #     final_output = 0.1 * wall_output + 0.9 * parallel_output
            #     self.get_logger().info(f"<<<ERROR: FO: {final_output}")


            
            # self.get_logger().info(f"SE: {side_error} PE: {parallel_error}")
            right_speed = min(max(right_base_speed - parallel_output + wall_output, self.min_speed), self.max_speed)
            left_speed =  min(max(left_base_speed + parallel_output, self.min_speed), self.max_speed)
            
            
            # Go forward
            # if backward == 0.0:
            self.get_logger().info(f"PO: {parallel_output}, WO: {wall_output}")
            # right_speed = min(max(right_base_speed + parallel_output + wall_output, right_base_speed), 140)
            
            self.get_logger().info(f"RS: {right_speed}, LS:{left_speed}")
            right_duty_cycle = float(right_speed) / 255.0
            left_duty_cycle = float(left_speed) / 255.0
            
            self.m1p1.write(right_duty_cycle)
            self.m1p2.write(0)
            self.m2p1.write(left_duty_cycle)
            self.m2p2.write(0)
            # time.sleep(0.5)
            
            
            # # Go backward
            # else:
            #     right_speed = min(max(right_base_speed + parallel_output - wall_output, 0), right_base_speed)
            #     left_speed = min(max(left_base_speed - parallel_output + wall_output, 0), left_base_speed)
            #     right_duty_cycle = float(right_speed) / 255.0
            #     left_duty_cycle = float(left_speed) / 255.0
            #     self.m1p1.write(0)
            #     self.m1p2.write(right_duty_cycle)
            #     self.m2p1.write(0)
            #     self.m2p2.write(left_duty_cycle)
           
            return False
        except Exception as e:
            self.get_logger().warning(f"[Along Wall] Error:{e}")
            self.start_approach = False
            return True
        # end try
    
    # def approach_wall(self):
    #     side_distance = (right_sidefront_distance + right_siderear_distance) / 2.0
    #     wall_output = self.side_distance_PID.compute(side_target, side_distance)
            
    #     pass
        
            
    def straight_approach_target(self, 
                                right_base_speed:float=110,
                                left_base_speed:float=110,
                                side_target_distance:float=30,
                                front_target: float=40):
        try:
            front_distance = self.us_front_sub.get_data()
            right_sidefront_distance = self.us_sidefront_right_sub.get_data()
            right_siderear_distance = self.us_siderear_right_sub.get_data()

            current_pose = self.imu_sub.get_data()
            current_pose = (current_pose + 360) % 360

            if front_distance < front_target:
                self.m1p1.write(0)
                self.m1p2.write(0)
                self.m2p1.write(0)
                self.m2p2.write(0)
                time.sleep(0.5)
                self.spin_PID.reset()
                self.front_PID.reset()
                self.side_distance_PID.reset()
                return True
            # set initial pose only at start
            if not self.start_spin:
                self.set_target_pose(current_pose)
                with self.lock:
                    self.start_spin = True
            else:
                
                pose_diff = (current_pose - self.target_pose + 360) % 360
                if pose_diff > 180:
                    pose_diff = pose_diff - 360
                # normalize angle to 180 -> -180
                # pose_diff = (pose_diff + 180) % 360 - 180
                spin_output = self.spin_PID.compute(abs(pose_diff), 0)
                self.get_logger().info(f"PD: {pose_diff} PO: {spin_output}")
                # TODO: test and check actual spinning direction???
                if pose_diff > 0:
                    left_speed = min(max(left_base_speed - spin_output, self.min_speed), self.max_speed)
                    right_speed = min(max(right_base_speed + spin_output, self.min_speed), self.max_speed)
                    self.get_logger().info(f">0")   
                else:
                    left_speed = min(max(left_base_speed + spin_output, self.min_speed), self.max_speed)
                    right_speed = min(max(right_base_speed - spin_output, self.min_speed), self.max_speed)
                    self.get_logger().info(f"<0") 
                # # adjust distance error
                # # Forward: + => away from wall => left +, right -
                # # Backward: + => same as forward
                
                # if not (right_sidefront_distance == 60.0 or right_siderear_distance == 60.0):
                #     side_distance = (right_sidefront_distance + right_siderear_distance) / 2.0
                #     
                side_distance = (right_sidefront_distance + right_siderear_distance) / 2.0
                if side_distance < side_target_distance:
                    dist_diff = side_distance - side_target_distance
                    wall_output = self.side_distance_PID.compute(abs(dist_diff),0)
                    # if dist_diff > 0:
                    left_speed = min(max(left_speed + wall_output, self.min_speed), self.max_speed)
                    right_speed = right_base_speed
                    
                    # elif dist_diff < 0:
                    #     left_speed = min(max(left_speed - wall_output, self.min_speed), self.max_speed)
                    # return False

                right_duty_cycle = float(right_speed) / 255.0
                left_duty_cycle = float(left_speed) / 255.0
                self.m1p1.write(right_duty_cycle)
                self.m1p2.write(0)
                self.m2p1.write(left_duty_cycle)
                self.m2p2.write(0)
                
                # if front_distance < front_target + 20:
                #     self.m1p1.write(0)
                #     self.m1p2.write(0)
                #     self.m2p1.write(0)
                #     self.m2p2.write(0)
                #     time.sleep(0.5)
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
                #target_pose = current_pose + degree 
                if degree < 0:
                    target_pose - 360
                self.set_target_pose(target_pose)
                self.get_logger().info(f"T: {self.target_pose} I: {self.initial_pose} C: {current_pose}")
                with self.lock:
                    self.start_spin = True
            else:
                # check if get to target
                self.get_logger().info(f"{self.target_pose - current_pose}")
                if abs(self.target_pose - current_pose) < error:
                    self.m1p1.write(0)
                    self.m1p2.write(0)
                    self.m2p1.write(0)
                    self.m2p2.write(0)
                    with self.lock:
                        self.start_spin = False
                    self.spin_PID.reset()
                    return True
                # clockwise and counterclockwise
                # pose_diff = ((self.target_pose - current_pose) + 180) % 360 - 180
                # spin_output = self.spin_PID.compute(abs(pose_diff), 0)
                # speed = min(max(adjustment_speed - spin_output, self.min_speed), adjustment_speed)
                # self.get_logger().info(f"Diff: {pose_diff} Speed: {speed} SO:{spin_output}")
                duty_cycle = float(adjustment_speed) / 255.0
                # TODO: test and check actual spinning direction???
                if self.target_pose - current_pose < 0:
                    self.m1p1.write(duty_cycle)
                    self.m1p2.write(0)
                    self.m2p1.write(0)
                    self.m2p2.write(duty_cycle)
                else:
                    self.m1p1.write(0)
                    self.m1p2.write(duty_cycle)
                    self.m2p1.write(duty_cycle)
                    self.m2p2.write(0)
                time.sleep(0.1)
                self.m1p1.write(0)
                self.m1p2.write(0)
                self.m2p1.write(0)
                self.m2p2.write(0)
                time.sleep(0.1)
            return False 
        except Exception as e:
            self.get_logger().warning(f"Error:{e}")
            self.spin_PID.reset()
            with self.lock:
                self.start_spin = False
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
            output = self.parallel_PID.compute(abs(diff), 0)
            new_speed = min(max(adjustment_speed - output, self.min_speed), self.max_speed)
            if (right_sidefront_distance == self.us_max_range or right_siderear_distance == self.us_max_range):
                new_speed = 110
            duty_cycle = float(new_speed) / 255.0
            self.get_logger().info(f"DC: {duty_cycle}")
            # TODO: test and see if correct
            if diff < -error:
                self.m1p1.write(duty_cycle)
                self.m1p2.write(0)
                self.m2p1.write(0)
                self.m2p2.write(duty_cycle)
                time.sleep(0.1)
            elif diff > error:
                self.m1p1.write(0)
                self.m1p2.write(duty_cycle)
                self.m2p1.write(duty_cycle)
                self.m2p2.write(0)
                time.sleep(0.1)
            elif right_sidefront_distance == self.us_max_range and \
                    right_siderear_distance == self.us_max_range:
                self.m1p1.write(duty_cycle)
                self.m1p2.write(0)
                self.m2p1.write(0)
                self.m2p2.write(duty_cycle)
                time.sleep(0.3)
            elif abs(diff) < error:
                self.get_logger().info(f"Debug distance:{right_sidefront_distance},{right_siderear_distance}")
                self.m1p1.write(0)
                self.m1p2.write(0)
                self.m2p1.write(0)
                self.m2p2.write(0)
                self.parallel_PID.reset()
                return True
            
            self.m1p1.write(0)
            self.m1p2.write(0)
            self.m2p1.write(0)
            self.m2p2.write(0)
            time.sleep(0.2)
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
        

    def dummy_test(self, adjustment_speed:float=110, error:float=0.01):
        duty_cycle = float(adjustment_speed) / 255.0
        self.m1p1.write(duty_cycle)
        self.m1p2.write(0)
        self.m2p1.write(0)
        self.m2p2.write(0)

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
        # along_wall:100;100;110;0.5;10;8;2,
        # spin_degree
        # align_wall:110;0.5,straight_approach_target:110;110;30;40;1
        # "spin_degree:-90;110;5,align_wall:110;0.5,spin_degree:-90;110;5,align_wall:110;0.5"
        # "along_wall:100;110;110;1;20;25;2"along_wall:110;110;20;40
        # dummy_test:110;0
        # straight_approach_target:110;110;30;40
        self.add_commands("spin_degree:315;110;1")
        time.sleep(20)


        
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
