import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32

from threading import Thread, Lock, Event
from collections import deque
import math
import smbus2
import time
from decimal import Decimal
import numpy as np

from .MPU6050 import MPU6050

I2C_BUS = 1
DEVICE_ADDRESS = 0x68
# The offsets are different for each device and should be changed
# accordingly using a calibration procedure
X_ACC_OFFSET = 0
Y_ACC_OFFSET = 0
Z_ACC_OFFSET = 0
X_GYR_OFFSET = 0
Y_GYR_OFFSET = 0
Z_GYR_OFFSET = 0
ENABLE_DEBUG = True


class IMUSensorNode(Node):
    def __init__(self):
        super().__init__('imu_sensor_node')

        custom_qos_profile = qos_profile_sensor_data
        self.buffer = deque(maxlen=10)

        # self.test = self.get_parameter('test').value
        # self.get_logger().info(f"Test: {self.test}")

        self.lock = Lock()
        self.event = Event()
        self.angle_thread = Thread(target=self.calc_angle_task)
        self.mpu = MPU6050(I2C_BUS, 
                      DEVICE_ADDRESS, 
                      X_ACC_OFFSET, 
                      Y_ACC_OFFSET,
                      Z_ACC_OFFSET, 
                      X_GYR_OFFSET, 
                      Y_GYR_OFFSET, 
                      Z_GYR_OFFSET,
                      ENABLE_DEBUG)
        
        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)
        mpu_int_status = self.mpu.get_int_status()
        self.get_logger().info(f"MPU Status: {hex(mpu_int_status)}")
        self.packet_size = self.mpu.DMP_get_FIFO_packet_size()
        self.get_logger().info(f"Packet size: {self.packet_size}")
        FIFO_count = self.mpu.get_FIFO_count()
        self.get_logger().info(f"FIFO count: {FIFO_count}")

        self.imu_data = Float32()
        self.imu_publisher = self.create_publisher(
            Float32, 
            'sensor/imu', 
            custom_qos_profile)
        self.timer = self.create_timer(0.1, self.pub_imu_data)
        self.get_logger().info("Initilize IMU Node")
        self.start_measure()


    def start_measure(self):
        if self.angle_thread.is_alive() is False:
            self.event.clear()
            self.angle_thread.start()
            self.get_logger().info("Start angle thread")
        else:
            self.get_logger().warning("Thread is already running! stop it first.")
    
    def stop_measure(self):
        if self.angle_thread.is_alive():
            self.event.set()
            self.get_logger().info("Stop angle thread")

    # def normalize_yaw(self, yaw, reference_yaw):
    #     """
    #     Normalize yaw angle to be in the range between -180 and 180,
    #     with a new reference point as the new 0.

    #     Parameters:
    #     - yaw: Yaw angle to be normalized.
    #     - reference_yaw: New reference point for yaw=0.

    #     Returns:
    #     - Normalized yaw angle.
    #     """
    #     normalized_yaw = (Decimal(yaw) - Decimal(reference_yaw) + Decimal(180.0)) % Decimal(360) - Decimal(180.0)
    #     return float(normalized_yaw)
    
        
    def calc_angle_task(self) -> None:
        self.get_logger().info(f"Wait 20s to stabilize.")
        time.sleep(30)
        self.get_logger().info(f"Finish stabilize.")
        FIFO_buffer = [0]*64
        count = 0
        
        spike_filter = deque([0] * 3, maxlen=3)
        
        cal_head_array = np.zeros(200)
        cal_tail_array = np.zeros(200)
        # yaw_offset = 0.0
        cal_head_rpyz = 0.0
        cal_tail_rpyz = 0.0
        cal_gain = 0.0
        # Start calibration process
        while True: 
            try: 
                FIFO_count = self.mpu.get_FIFO_count()
                mpu_int_status = self.mpu.get_int_status()

                # If overflow is detected by status or fifo count we want to reset
                if (FIFO_count == 1024) or (mpu_int_status & 0x10):
                    # with self.lock:
                    self.mpu.reset_FIFO()
                    # self.get_logger().warning(f"overflow!")
                
                # Check if fifo data is ready
                elif (mpu_int_status & 0x02):
                    # Wait until packet_size number of bytes are ready for reading, default
                    # is 42 bytes
                    while FIFO_count < self.packet_size:
                        FIFO_count = self.mpu.get_FIFO_count()
                    FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)
                    accel = self.mpu.DMP_get_acceleration_int16(FIFO_buffer)
                    quat = self.mpu.DMP_get_quaternion_int16(FIFO_buffer)
                    grav = self.mpu.DMP_get_gravity(quat)
                    
                    rpy = self.mpu.DMP_get_euler_roll_pitch_yaw(quat.get_normalized(), grav)
                    # self.get_logger().info(f"X:{rpy.x} Y:{rpy.y} Z:{rpy.z}")
                    # thresholding to remove spikes
                    spike_filter.append(rpy.z)
                    if count > 2:
                        diff_pre = abs(spike_filter[0] - spike_filter[1])
                        diff_post = abs(spike_filter[2] - spike_filter[1])
                        if (diff_pre > 5) and (diff_post > 5):
                            rpy.z = spike_filter[2]
                        else: 
                            rpy.z = spike_filter[1]

                    # if (pre_rpy is not None):
                    #     diff_r = rpy.x - pre_rpy.x
                    #     diff_p = rpy.y - pre_rpy.y
                    #     diff_y = rpy.z - pre_rpy.z
                    #     # pre_rpy = rpy
                    #     if (abs(diff_r) > 10.0) or \
                    #         (abs(diff_p) > 10.0) or \
                    #         (abs(diff_y) > 50.0 and abs(diff_y) < 340):
                    #         rpy = pre_rpy
                    #         # Have to reset pre_rpy
                    
                    spike_filter[1]
                    # Calibration
                    if count <= 1000: 
                        if count < 200:
                            cal_head_array[count] = rpy.z
                        # if count % 1 == 0:
                        elif count > 799 and count < 1000:
                            i = count % 200
                            cal_tail_array[i] = rpy.z
                        elif count == 1000:
                            h_vals, h_counts = np.unique(cal_head_array, return_counts=True)
                            h_index = np.argmax(h_counts)
                            most_head = h_vals[h_index]
                            t_vals, t_counts = np.unique(cal_tail_array, return_counts=True)
                            t_index = np.argmax(t_counts)
                            most_tail = t_vals[t_index]
                            # avg_head = cal_head_rpyz / 100.0
                            # avg_tail = cal_tail_rpyz / 100.0
                            # yaw_offset = avg_tail
                            # self.get_logger().info(f"Head: {most_head:.2f} Tail:{most_tail:.2f}")
                            
                            cal_gain = abs(most_head - most_tail) / 850.0
                            # self.get_logger().info(f"Gain: {cal_gain}")
                    else: 
                        angle_compansate = cal_gain * count
                        rpyz_new = rpy.z + angle_compansate
                        # rpyz_new = self.normalize_yaw(rpyz_new, yaw_offset)
                        # self.get_logger().info(f"X:{grav.x} Y:{grav.y} Z:{grav.z}")
                        # self.get_logger().info(f"Y:{rpy.z:.2f} YC:{rpyz_new:.2f} C:{angle_compansate:.5f}")
                        self.buffer.append(rpyz_new)
                    
                    count += 1
                # time.sleep(0.01)
            except Exception as exc:
                self.get_logger().warning(f"Exception: {exc}")

    def pub_imu_data(self):
        try:
            
            yaw = self.buffer.pop()
            self.imu_data.data = yaw
            self.imu_publisher.publish(self.imu_data)
            # self.get_logger().info(f"Yaw: {yaw:.2f}")
        except IndexError:
            pass
            # self.get_logger().warning(f"No data left")
        
         

def main(args=None):
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor()
        node = IMUSensorNode()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# # self.radToDeg = 57.2957786
# # https://github.com/TKJElectronics/KalmanFilter/blob/master/Kalman.cpp

# class KalmanAngle:
#     def __init__(self):
#         self.QAngle = 0.001
#         self.QBias = 0.003
#         self.RMeasure = 0.03
#         self.angle = 0.0
#         self.bias = 0.0
#         self.rate = 0.0
#         # This is a 2x2 matrix
#         self.P=[[0.0,0.0],[0.0,0.0]]
#         self.lock = Lock()

#     # The angle should be in degrees and the rate should be in degrees per second 
#     # and the delta time in seconds
#     def getAngle(self ,newAngle, newRate, dt):

#         with self.lock:
#             # Step 1:
#             # Update xhat - Project the state ahead
#             # Discrete Kalman filter time update equations - Time Update ("Predict")
#             self.rate = newRate - self.bias    #new_rate is the latest Gyro measurement
#             self.angle += dt * self.rate

#             # Step 2:
#             # Update estimation error covariance - Project the error covariance ahead
#             self.P[0][0] += dt * (dt*self.P[1][1] -self.P[0][1] - self.P[1][0] + self.QAngle)
#             self.P[0][1] -= dt * self.P[1][1]
#             self.P[1][0] -= dt * self.P[1][1]
#             self.P[1][1] += self.QBias * dt

#             # Step 3: Innovation

#             # Calculate angle and bias - Update estimate with measurement zk (newAngle)
#             y = newAngle - self.angle

#             #Step 4: Innovation covariance
#             # Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
#             # Calculate Kalman gain - Compute the Kalman gain
#             s = self.P[0][0] + self.RMeasure

#             #Step 5:    Kalman Gain
#             K=[0.0,0.0]
#             K[0] = self.P[0][0] / float(s)
#             K[1] = self.P[1][0] / float(s)

#             #Step 6: Update the Angle
#             self.angle += K[0] * y
#             self.bias  += K[1] * y

#             #Step 7: Calculate estimation error covariance - Update the error covariance
#             P00Temp = self.P[0][0]
#             P01Temp = self.P[0][1]

#             self.P[0][0] -= K[0] * P00Temp
#             self.P[0][1] -= K[0] * P01Temp
#             self.P[1][0] -= K[1] * P00Temp
#             self.P[1][1] -= K[1] * P01Temp

#             return self.angle
    
#     # Used to set angle, this should be set as the starting angle
#     def setAngle(self, angle):
#         with self.lock:
#             self.angle = angle

#     # Return the unbiased rate
#     def getRate(self):
#         return self.rate

#     # ============================
#     # Kalman filter parameters
#     # ============================

#     def setQAngle(self, QAngle):
#         with self.lock:
#             self.QAngle = QAngle

#     def setQBias(self, QBias):
#         with self.lock:
#             self.QBias = QBias

#     def setRMeasure(self, RMeasure):
#         with self.lock:
#             self.RMeasure = RMeasure

#     def getQAngle(self):
#         return self.QAngle

#     def getQBias(self):
#         return self.QBias

#     def getRMeasure(self):
#         return self.RMeasure


         


