# Subscriber to /goal_pose topic -> converts to data sent to cmd_vel topic for movement

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Odometry
from math import atan2, pi
import numpy as np
#from tf.transformations import euler_from_quaternion
from enum import Enum

home = Point()
home.x = -1.0
home.y = -1.0

class MovementStatus(Enum):
    MOVING = 1
    FINISHED = 2

goal_x = 0.0
goal_y = 0.0

movement_status = MovementStatus.FINISHED

def euler_from_quaternion(x, y, z, w):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class NavigateToPose(Node):
    def __init__(self):
        super().__init__('navigate_to_pose')

        self.subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.subscriber_callback,
            10
        )

        self.go_to_position_tictactoe_subscriber = self.create_subscription(
            Point,
            '/go_to_position_tictactoe',
            self.go_to_position_tictactoe_callback,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.currentOdom = self.create_subscription(
            Odometry,
            '/odom',
            self.calculate_movement_to_goal_pose,
            10
        )

    def subscriber_callback(self, msg):
        global goal_x
        global goal_y

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

    def go_to_position_tictactoe_callback(self, msg):
        #ros2 topic pub /go_to_position_tictactoe geometry_msgs/Point "{x: 0, y: 1}"
        x = msg.x
        y = msg.y

        # Testirano na empty world launch iz gazeba pa zato te coord

        #        + (x)
        #        |
        # + < -- | --  > - (y)  
        #        |
        #        -
        # (robot postavljen na shit način - gleda prema xpos, desno je yneg)

        #      |      |
        #  3,1 | 3,0  | 3,-1  <- 0,0 - 1,0 - 2,0 (x,y)
        # -----|------|-------
        #  2,1 | 2,0  | 2,-1  <- 0,1 - 1,1 - 2,1
        # -----|------|-------
        #  1,1 | 1,0  | 1,-1  <- 0,2 - 1,2 - 2,2
        #      |      |
        # ~~~~~~~0,0~~~~~~~~~ <- inital robot pos 

        global goal_x
        global goal_y

        # mapiranje:
        if (x == 0 and y == 0):
            goal_x = 3.0
            goal_y = 1.0
        elif (x == 1 and y == 0):
            goal_x = 3.0
            goal_y = 0.0
        elif (x == 2 and y == 0):
            goal_x = 3.0
            goal_y = -1.0
        elif (x == 0 and y == 1):
            goal_x = 2.0
            goal_y = 1.0
        elif (x == 1 and y == 1):
            goal_x = 2.0
            goal_y = 0.0
        elif (x == 2 and y == 1):
            goal_x = 2.0
            goal_y = -1.0
        elif (x == 0 and y == 2):
            goal_x = 1.0
            goal_y = 1.0
        elif (x == 1 and y == 2):
            goal_x = 1.0
            goal_y = 0.0
        elif (x == 2 and y == 2):
            goal_x = 1.0
            goal_y = -1.0
        elif (x == -1 and y == -1):
          #home 0,0
          print(f'Going home')
          goal_x = 0.0
          goal_y = 0.0
        
        print(f'Going to x: {goal_x}, y: {goal_y}')
        print(f'Mapped from tictactoe coods x: {x}, y: {y}')

    def calculate_movement_to_goal_pose(self, msg):
        moveMsg = Twist()

        moveMsg.linear.x, moveMsg.angular.z = self.calculate_movement(msg)

        self.publisher.publish(moveMsg)
    
    def calculate_movement(self, msg):
        global movement_status

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation

        inc_x = goal_x - x
        inc_y = goal_y - y
        angle_goal = atan2(inc_y, inc_x)

        t3 = +2.0 * (rot_q.w * rot_q.z + rot_q.y * rot_q.x)
        t4 = +1.0 - 2.0 * (rot_q.y **2 + rot_q.z ** 2)
        theta = atan2(t3, t4)

        if abs(angle_goal - theta) > 0.5:
            if abs(inc_x) <= 0.04 and abs(inc_y) <= 0.04:
                linearX = 0.0
                angularZ = 0.0
                if (movement_status != MovementStatus.FINISHED):
                  movement_status = MovementStatus.FINISHED
                  print(movement_status)
                  # TODO: Spin and go home!
                  self.go_to_position_tictactoe_callback(home)
            else:
                linearX = 0.0
                angularZ = 0.4 * (angle_goal - theta)
                if (movement_status != MovementStatus.MOVING):
                  movement_status = MovementStatus.MOVING
                  print(movement_status)
        else:    
            linearX = 0.3
            angularZ = 0.0
            if (movement_status != MovementStatus.MOVING):
                  movement_status = MovementStatus.MOVING
                  print(movement_status)

        return linearX, angularZ


def main(args=None):
    rclpy.init(args=args)

    navigate_to_pose = NavigateToPose()

    rclpy.spin(navigate_to_pose)

    rclpy.shutdown()

if __name__ == "__main__":
    main()