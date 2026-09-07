#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

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

        self.vel_pub = self.create_publisher(Twist, "mavros/setpoint_velocity/cmd_vel_unstamped", 10)
        self.create_timer(0.05, self.timer_callback)

        # It seems that a relative alt of about -1.8 is roughly the middle of the gate
        self.alt_sub = self.create_subscription(Float64, "mavros/global_position/rel_alt", self.alt_listener_callback, 10)
        self.rel_alt = 0

        self.camera_sub = self.create_subscription(BoundingBoxArray, "main_camera/detection/bounding_boxes", self.camera_listener_callback, 10)
        self.bounding_boxes = []
        self.gate_count = 0
        self.gate_x = 0
        self.gate_w = 0
        self.gate_in_view = False

        self.heading_sub = self.create_subscription(Float64, "mavros/global_position/compass_hdg", self.heading_listener_callback, 10)
        self.heading = 0

        self.gate_found = False
        self.guide_state = 0
        self.straight = False
        self.aligned = False

        self.gate_found_ticks = 0
        self.aligned_ticks = 0
        self.GATE_FOUND_CONFIRM = 6
        self.ALIGN_CONFIRM = 6

        self.obstacle_in_view = False
        self.obstacle_size = 0.0
        self.obstacle_x = 0.0
        self.avoidance_state = 0  # 0=normal, 1=avoiding
        self.avoid_direction = 0.0

    def send_request(self):
        self.req.base_mode = 0
        self.req.custom_mode = "GUIDED"
        return self.set_mode_client.call_async(self.req)

    def timer_callback(self):
        vel_msg = Twist()

        OBSTACLE_TRIGGER = 0.03   # obstacle width that triggers avoidance

        if self.avoidance_state == 0:
            if self.obstacle_in_view and 0.1 < self.obstacle_x < 0.9 and self.obstacle_size > OBSTACLE_TRIGGER:
                self.avoidance_state = 1
                # Choose a direction to avoid it with
                if self.obstacle_x < 0.5: # I.e. left side of bounding box
                    self.avoid_direction = -0.5 # We want to go right
                else:
                    self.avoid_direction = 0.5
                self.get_logger().warn(f"AVOIDING OBSTACLE, DIR={self.avoid_direction}")

        if self.avoidance_state == 1:
            vel_msg.linear.x = -0.3 # Back up slightly
            vel_msg.linear.y = self.avoid_direction * 2.0 # Go left or right to avoid
            vel_msg.linear.z = 0.0
            vel_msg.angular.z = 0.0
            if not self.obstacle_in_view or self.obstacle_x < 0.1 or self.obstacle_x > 0.9:
                self.avoidance_state = 0  # Resume normal gate-seeking
                self.gate_count = 0
                self.gate_in_view = False
                self.get_logger().info("OBSTACLE AVOIDED")
            self.vel_pub.publish(vel_msg)
            return

        # Descend to depth of 1.8m
        vel_msg.linear.z = -1.0 * (self.rel_alt + 1.8)

        # Spin while descending to detect gate
        if self.gate_count < 20 and not self.gate_in_view:
            vel_msg.angular.z = 0.5
        else:
            # Align to middle of gate
            vel_msg.angular.z = 1.0 * (0.5 - self.gate_x)

            # Confirm we have found theg ate over multiple ticks, resolves intertia issues
            if 0.45 < self.gate_x < 0.55:
                self.gate_found_ticks += 1
                if self.gate_found_ticks > self.GATE_FOUND_CONFIRM and not self.gate_found:
                    self.gate_found = True
                    if 90 < self.heading < 180:
                        # We need to move laterally left and rotate clockwise for heading +180
                        self.guide_state = 1
                    elif 180 < self.heading < 270:
                        # We need to move laterally right and rotate anticlockwise for heading +180
                        self.guide_state = 2
                    elif 0 < self.heading < 90:
                        # We need to move laterally right and rotate anticlockwise for heading 0
                        self.guide_state = 3
                    else:
                        # We need to move laterally left and rotate clockwise for heading 0
                        self.guide_state = 4
            else:
                self.gate_found_ticks = 0

            if self.gate_found == True:
                if self.guide_state == 1 or self.guide_state == 2:
                    vel_msg.angular.z = -2.0 * (1 - self.heading/180)
                    if 175 < self.heading < 185:
                        self.straight = True
                else:
                    if self.heading <= 180:
                        adjusted_heading = self.heading
                    else:
                        adjusted_heading = self.heading - 360 # Must wrap around because we use /180
                    vel_msg.angular.z = 2.0 * (adjusted_heading/180)
                    if self.heading < 5 or self.heading > 355:
                        self.straight = True

            if self.straight == True:
                if self.guide_state == 1:
                    if self.gate_in_view:
                        vel_msg.linear.y = 2.0 * (0.5 - self.gate_x)
                    else:
                        vel_msg.linear.y = 2.0
                elif self.guide_state == 2:
                    if self.gate_in_view:
                        vel_msg.linear.y = 2.0 * (0.5 - self.gate_x)
                    else:
                        vel_msg.linear.y = -2.0
                elif self.guide_state == 3:
                    if self.gate_in_view:
                        vel_msg.linear.y = 2.0 * (0.5 - self.gate_x)
                    else:
                        vel_msg.linear.y = -2.0
                else:
                    if self.gate_in_view:
                        vel_msg.linear.y = 2.0 * (0.5 - self.gate_x)
                    else:
                        vel_msg.linear.y = 2.0

                # Confirm we are aligned with gate over multiple ticks, handles intertia issues
                if self.gate_in_view and 0.45 < self.gate_x < 0.55:
                    self.aligned_ticks += 1
                    if self.aligned_ticks > self.ALIGN_CONFIRM and not self.aligned:
                        self.aligned = True
                else:
                    self.aligned_ticks = 0

            # Go thru gate
            if self.aligned == True:
                vel_msg.linear.x = 0.5
                if self.gate_in_view:
                    vel_msg.linear.y = 1.0 * (0.5 - self.gate_x) # Correct lateral
                    vel_msg.angular.z = 1.0 * (0.5 - self.gate_x) # Correct angle
                else:
                    vel_msg.linear.y = 0.0
                    if self.guide_state == 1 or self.guide_state == 2:
                        vel_msg.angular.z = -0.5 * (1 - self.heading/180) # Otherwise just head straight perpendicular to gate
                    else:
                        if self.heading <= 180:
                            adjusted_heading = self.heading
                        else:
                            adjusted_heading = self.heading - 360 # Must wrap around because we use /180
                        vel_msg.angular.z = 0.5 * (adjusted_heading/180)

        self.vel_pub.publish(vel_msg)

        self.get_logger().info(
            f"gate_x={self.gate_x:.3f} w={self.gate_w:.3f} in_view={self.gate_in_view} "
            f"found={self.gate_found}({self.gate_found_ticks}) straight={self.straight} "
            f"aligned={self.aligned}({self.aligned_ticks})"
        )

    def alt_listener_callback(self, alt):
        self.rel_alt = alt.data

    def camera_listener_callback(self, array):
        self.bounding_boxes = array.bounding_boxes

        self.obstacle_in_view = False
        self.obstacle_size = 0.0
        
        gate_miss_count = 0
        GATE_MISS_TOLERANCE = 3  # allow a few bounding_box misses before declaring gate lost
        gate_detected_this_tick = False

        # Because camera is unreliable, we want to make sure we have detected
        # the gate multiple times before we can confirm it is in fact the gate
        for i in range(len(self.bounding_boxes)):
            if self.bounding_boxes[i].label_id == 3:
                if self.bounding_boxes[i].conf >= 0.9:
                    self.gate_count += 1
                    self.gate_x = self.bounding_boxes[i].x
                    self.gate_w = self.bounding_boxes[i].w
                    gate_detected_this_tick = True

            elif self.bounding_boxes[i].label_id == 4:
                if self.bounding_boxes[i].conf >= 0.7:
                    self.obstacle_in_view = True
                    self.obstacle_size = self.bounding_boxes[i].w
                    self.obstacle_x = self.bounding_boxes[i].x
                    
        if gate_detected_this_tick:
            gate_miss_count = 0
            if self.gate_w > 0.15:
                self.gate_in_view = True # We only want to see the whole gate, not just a supporting pole
        else:
            gate_miss_count += 1
            if gate_miss_count > GATE_MISS_TOLERANCE:
                self.gate_in_view = False

    def heading_listener_callback(self, heading):
        self.heading = heading.data


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