#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32, Float64MultiArray 
from geometry_msgs.msg import TwistStamped  # [ADDED] was missing

class FinalProject(Node):
    def __init__(self):
        super().__init__('Final_Project')
        self.get_logger().info('Final Project node has been initialized.')

        # --- LINE FOLLOWING PARAMETERS ---
        
        # might need to tune these values 
        # dead band
        self.dead_band = 7 # pixels
        # p
        self.kp = 0.001
        # i
        self.ki = 0.000002
        self.integral = 0.0 # m/s
        # d
        self.kd = 0.000008
        self.derivative = 0.0
        self.lasterror = 0.0

        # [ADDED] target line index (pixels); TODO: set to the real centre value
        self.desired = 320
        # [ADDED] initial speeds so cases 1 and 2 don't hit an AttributeError
        self.linear_speed = 0.05
        self.angular_speed = 0.0
        # [ADDED] clamp for the integral term (anti-windup); tune as needed
        self.integral_limit = 5000.0

        # [ADDED] publisher for velocity commands; TODO: confirm topic name for your robot
        self.cmd_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)

        # --- SUBSCRIBERS ---

        # line_idx for line-following
        self.color_sub = self.create_subscription(
            UInt32, "line_idx", self.camera_callback, 1
        )
        
        # mean RGB of the image for localization
        self.rgb_sub = self.create_subscription(
            Float64MultiArray, "mean_img_rgb", self.rgb_callback, 1
        )
        # latest RGB reading (None until first message)
        self.mean_rgb = None


    # --- CALLBACKS ---

    def rgb_callback(self, msg):
        """Callback to store the latest mean RGB"""
        # maybe convert it to a int or string representing each color
        self.mean_rgb = list(msg.data)

    def camera_callback(self, msg):
        """Callback for line index."""
        # access the line_idx using msg.data

        self.get_logger().info("\ncamera callback")

        # CHECK FOR LOCALIZATION
        if self.should_localize():
            # TODO: bayesian localization actions (part 1?)
            # also need to go straight for a bit to get back on the white line
            return  # [ADDED] fixes the empty if-body; skips line following while localizing

        # LINE FOLLOWING
        self.get_logger().info("line following")

        actual = msg.data
        error = self.desired - actual # update error  [ADDED] self.desired instead of undefined desired

        #self.get_logger().info("check 2")
     
        self.follow_the_line(error)

    # --- LINE FOLLOWING ---

    def follow_the_line(self, error): 
        twist_stamped = TwistStamped()
        # [ADDED] timestamp for the stamped message
        twist_stamped.header.stamp = self.get_clock().now().to_msg()

        # integral
        self.integral = self.integral + error
        # [ADDED] anti-windup clamp
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

        # derivative
        # [ADDED] swapped order: compute derivative BEFORE updating lasterror (was always 0)
        self.derivative = error - self.lasterror
        self.lasterror = error

        # pid
        correction = self.kp*error + self.ki*self.integral + self.kd*self.derivative

        self.get_logger().info(str(error))
        self.get_logger().info(str(correction))


        # correction
        if error > self.dead_band/2: # landmark left of robot
            self.angular_speed = correction
            #self.linear_speed = 0.01
            twist_stamped.twist.angular.z = self.angular_speed
            twist_stamped.twist.linear.x = self.linear_speed
            self.get_logger().info("case 1, landmark on left")

        elif error < -self.dead_band/2: # landmark right of robot
            self.angular_speed = correction
            #self.linear_speed = 0.01
            twist_stamped.twist.angular.z = self.angular_speed
            twist_stamped.twist.linear.x = self.linear_speed
            self.get_logger().info("case 2, landmark on right")

        else:
            self.angular_speed = 0.0
            self.linear_speed = 0.05
            twist_stamped.twist.angular.z = self.angular_speed
            twist_stamped.twist.linear.x = self.linear_speed
            self.get_logger().info("case 3")

        # drive
        
        self.cmd_pub.publish(twist_stamped)
        # [ADDED] removed self._state_elapsed += self._period (attributes never defined)


    # --- LOCALIZATION ---

    # this one might be unecessary, a simple check directly in camera_callback might be enough
    def should_localize(self):
        """Return True if a bayesian localization step is needed."""
        if self.mean_rgb is None:
            return False
        # TODO: e.g. check if the RGB matches a colour marker
        return False


def main():
    rclpy.init()
    node = FinalProject()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == '__main__':
    main()
