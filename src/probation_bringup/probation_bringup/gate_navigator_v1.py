#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from mavros_msgs.srv import SetMode
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from vision_msgs.msg import BoundingBoxArray

class GateNavigator(Node):

    def __init__(self):
        super().__init__('gate_navigator')
        self.set_mode_client = self.create_client(SetMode, "mavros/set_mode")
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_mode service not available, waiting again...')
        self.req = SetMode.Request()
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.vel_pub = self.create_publisher(Twist, "mavros/setpoint_velocity/cmd_vel_unstamped", qos_profile)
        self.create_timer(0.05, self.timer_callback)
        
        # It seems that a relative alt of about -1.8 is roughly the middle of the gate
        self.alt_sub = self.create_subscription(Float64, "mavros/global_position/rel_alt", self.alt_listener_callback, 10)
        self.rel_alt = 0
        
        self.camera_sub = self.create_subscription(BoundingBoxArray, "main_camera/detection/bounding_boxes", self.camera_listener_callback, 10)
        self.bounding_boxes = []
        self.gate_count = 0
        self.gate_x = 0
        self.gate_y = 0
        self.gate_w = 0
        self.gate_h = 0
        self.prev_w = 0
        
        self.heading_sub = self.create_subscription(Float64, "mavros/global_position/compass_hdg", self.heading_listener_callback, 10)
        self.heading = 0
        
    def send_request(self):
        self.req.base_mode = 0
        self.req.custom_mode = "GUIDED"
        return self.set_mode_client.call_async(self.req)
    
    def timer_callback(self):
        vel_msg = Twist()
        
        # Descend to depth of 1.8m
        vel_msg.linear.z = -1.0 * (self.rel_alt + 1.8)
        
        # Spin while descending to detect gate, proportional to gate_count
        if self.gate_count < 5:
            vel_msg.angular.z = 0.1 * (5 - self.gate_count)
        else:
            if self.gate_w > self.prev_w - 0.1:
                # Align to middle of gate
                vel_msg.angular.z = 0.4 * (0.5 - self.gate_x)
                
                if 90 < self.heading < 270:
                    vel_msg.linear.y = 0.4 * (1 - self.heading/180)
                else:
                    vel_msg.linear.y = -0.4 * (1 - self.heading/180)
                
                # Track toward gate, faster if more width available
                vel_msg.linear.x = 1.0 * self.gate_w
                
                # Keep track of width
                self.prev_w = self.gate_w
                
            else: # Too close for camera to see whole gate
                if 90 < self.heading < 270:
                    vel_msg.linear.y = 0.4 * (1 - self.heading/180)
                    vel_msg.angular.z = 0.4 * (1 - self.heading/180)
                else:
                    vel_msg.linear.y = -0.4 * (1 - self.heading/180)
                    vel_msg.linear.z = -0.4 * (1 - self.heading/180)
                
                # Commit to motion
                vel_msg.linear.x = 1.0
                    
        self.vel_pub.publish(vel_msg)
        self.get_logger().info(f"Up/Down Vel = {vel_msg.linear.z}, Angular Vel = {vel_msg.angular.z}, Side Vel = {vel_msg.linear.y}")
        
    def alt_listener_callback(self, alt):
        self.rel_alt = alt.data
        self.get_logger().info(f"REL_ALT = {self.rel_alt}")
        
    def camera_listener_callback(self, array):
        self.bounding_boxes = array.bounding_boxes
        
        # Because camera is unreliable, we want to make sure we have detected
        # the gate multiple times before we can confirm it is in fact the gate
        for i in range(len(self.bounding_boxes)):
            if self.bounding_boxes[i].label_id == 3:
                if self.bounding_boxes[i].conf == 1.0:
                    self.gate_count += 1
                    self.gate_x = self.bounding_boxes[i].x
                    self.gate_y = self.bounding_boxes[i].y
                    self.gate_w = self.bounding_boxes[i].w
                    self.gate_h = self.bounding_boxes[i].h
                    
    def heading_listener_callback(self, heading):
        self.heading = heading.data
        self.get_logger().info(f"HEADING = {self.heading}")
             
        
def main():
    rclpy.init()

    node = GateNavigator()
    
    # Set to GUIDED mode
    future = node.send_request()
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    if response is not None and response.mode_sent:
        node.get_logger().info("Entered GUIDED mode successfully")
    else:
        node.get_logger().warn("Failed to enter GUIDED mode")
    
    # Spin the node for everything else
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
