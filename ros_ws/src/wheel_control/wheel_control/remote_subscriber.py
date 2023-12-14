
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import String
from collections import deque

class RemoteSubscriber:
    def __init__(self, 
                 node: Node, 
                 sensor_name:str, 
                 topic:str,  
                 srv_qos):
        self.is_ready = False
        self.sensor_data = deque(maxlen=3)
        self.name = sensor_name
        self.node = node
        self.prev_data = 0.0
        self.callback_group = MutuallyExclusiveCallbackGroup()
        # self.srv_cbgroup = MutuallyExclusiveCallbackGroup()
        
        self.sub = self.node.create_subscription(
            msg_type=String,
            topic=topic,
            callback=self.data_callback,
            qos_profile=srv_qos,
            callback_group=self.callback_group
        )
        
    
    def get_data(self):
        if self.is_ready:
            if len(self.sensor_data) > 0:
                self.prev_data = self.sensor_data.pop()      
                return self.prev_data
            else: 
                return self.prev_data
        else: 
            return None
    
    def status(self):
        return self.is_ready
         
    # def ready_callback(self):
    #     self.node.get_logger().info(f"{self.name} is ready!")
    #     self.is_ready = True
    #     return Trigger.Response(success=True)


    def data_callback(self, msg: String):
        self.node.get_logger().info(f"{self.name} is ready!")
        sensor_data = msg.data
        # self.node.get_logger().info(f"[{self.name}] Data: {sensor_data}")
        self.sensor_data.append(sensor_data)
        pass
