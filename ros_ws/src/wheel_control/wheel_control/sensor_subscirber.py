from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import Float32
from collections import deque

# TODO: can create as a base class, and extends to different sensor types.
# overriding sensor data type
class SensorSubscriber:
    def __init__(self, 
                 node: Node, 
                 sensor_name:str, 
                 topic:str, 
                 data_qos, 
                 srv_qos):
        self.is_ready = False
        self.sensor_data = deque(maxlen=3)
        self.name = sensor_name
        self.node = node
        self.callback_group = MutuallyExclusiveCallbackGroup()
        
        self.sub = self.node.create_subscription(
            msg_type=Float32,
            topic=topic,
            callback=self.data_callback,
            qos_profile=data_qos,
            callback_group=self.callback_group
        )
        self.ready_service = self.node.create_service(
            Trigger, 
            f"{topic}_ready", 
            self.ready_callback,
            qos_profile=srv_qos,
            callback_group=self.callback_group
        )
        
    
    def get_data(self):
        if self.is_ready:
            if len(self.sensor_data) > 0:
                return self.sensor_data.pop()
            else: 
                return None
        else: 
            return None
    
    def status(self):
        return self.is_ready
         
    def ready_callback(self):
        self.node.get_logger().info(f"{self.name} is ready!")
        self.is_ready = True
        return Trigger.Response(success=True)


    def data_callback(self, msg: Float32):
        sensor_data = msg.data
        self.sensor_data.append(sensor_data)
        pass

