#!/usr/bin/env python3
# Node recieves average distance measurements from 4 directions around robots and then 
# publishes a integer value which stands for a decision state (Finite-State-Machine)
#
# 1 = forward
# 2 = turn right ????until front is free or 90°??? Which is better?
# 3 = turn left 
# 4 = drive around end of wall
# 0 = stop
#
# The robot always tries to move forward to explore, then turn right and if that doesn't work turn left.
# If left and right are blocked, stop the robot

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from acs6121_team24_2025.msg import DecisionState
from acs6121_team24_2025.msg import ObstacleDistance
from nav_msgs.msg import Odometry 

from acs6121_team24_2025_modules.tb3_tools import quaternion_to_euler
from math import sqrt, pow, pi


class Decision(Node):

    def __init__(self):
        super().__init__("decision_node")
        
        self.dec_msg = DecisionState() 

        self.first_message = False        
        self.x = 0.0; self.y = 0.0; self.theta_z = 0.0
        self.xref = 0.0; self.yref = 0.0; self.theta_zref = 0.0
        self.yaw = 0.0 
        self.displacement = 0.0
        self.at_wall = False

        self.dis_front = 0.5        # Change initial values if threshold changes
        self.dis_right = 0.5
        self.dis_left = 0.5
        self.dis_fleft = 0.5
        self.dis_fright = 0.5
        
        self.dec_pub = self.create_publisher(
            msg_type=DecisionState,
            topic="decision_state",
            qos_profile=10,
        )

        self.lidar_sub = self.create_subscription(
            msg_type=ObstacleDistance,
            topic="obs_detection",
            callback=self.dec_callback,
            qos_profile=10,
        )

        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic="odom",
            callback=self.odom_callback,
            qos_profile=10,
        )

        ctrl_rate = 10 # hz
        self.timer = self.create_timer(
            timer_period_sec=1/ctrl_rate,
            callback=self.timer_callback,
        )

        self.shutdown = False

        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised."
        )

    def on_shutdown(self):
        print("Stopping the robot...")
        self.dec_pub.publish(DecisionState())
        self.shutdown = True

    def dec_callback(self, msg_data: ObstacleDistance):
        self.dis_front = msg_data.front
        self.dis_left = msg_data.left
        self.dis_right = msg_data.right
        self.dis_fleft = msg_data.fleft
        self.dis_fright = msg_data.fright

    def odom_callback(self, msg_data: Odometry):
        pose = msg_data.pose.pose 

        (roll, pitch, yaw) = quaternion_to_euler(pose.orientation) 

        self.x = pose.position.x 
        self.y = pose.position.y
        self.theta_z = yaw # abs(yaw) makes mistakes in detection!!

        if not self.first_message: 
            self.first_message = True
            self.xref = self.x
            self.yref = self.y
            self.theta_zref = self.theta_z        

    def timer_callback(self):
        # here is where the code to implement the robot decision making process goes. 
        # Use the measured distances to create a logica how the robot should behave
        front_threshold = 0.48
        side_threshold = 0.22
        right_threshold = 0.2
        left_threshold = 0.2

        front_free = self.dis_front > front_threshold
        fleft_free = self.dis_fleft > side_threshold
        fright_free = self.dis_fright > side_threshold
        right_free = self.dis_right > right_threshold
        left_free = self.dis_left > left_threshold

        self.displacement = sqrt(pow(self.x-self.xref,2)+pow(self.y-self.yref,2))

        if self.dis_front == float("nan") or self.dis_front == 0.0:
            decision_state = 0 # stop the robot if front is not free
            angular_error = 0.0
        
        elif self.displacement > 1.5 or self.at_wall:
            if not front_free and  self.dis_fright < 0.7:
                #if not self.dis_fleft > 1.2 :#and self.dis_fright < self.dis_right:
                    decision_state = 3  # Turn left
                    angular_error = 0.0  # No need for PID correction here
                # else:
                #     decision_state = 1  # Move forward
                #     error = (right_threshold - self.dis_right) # positive if too far, negative if too close
                #     angular_error = error  # P-control term (or compute full PID here if needed)
            elif self.dis_right > 0.4 and self.dis_fright > 0.6:
                # Right wall gone → turn right to reacquire
                decision_state = 4
                angular_error = 0.0
            else:
                decision_state = 1  # Move forward
                error = right_threshold - self.dis_right # positive if too far, negative if too close
                angular_error = error  # P-control term (or compute full PID here if needed)
            # if not front_free:
            #         self.dec_msg.decision_state = 3 	# Turn left
            # # Doesnt work like that
            # elif self.dis_right > 1:
            #                 self.dec_msg.decision_state = 4 	# Around edge
            # else:
            #         if self.dis_right > wall_upper_bound: 
            #             self.dec_msg.decision_state = 2 # turn right
            #         elif self.dis_right < wall_lower_bound: self.dec_msg.decision_state = 3 # Turn left
            #         else: self.dec_msg.decision_state = 1 	# straight

            self.at_wall = True
        
        else:
            if front_free and not fleft_free : #m Threshold
                decision_state = 2 # turn right
                angular_error = 0.0
            elif front_free and not fright_free : #m
                decision_state = 3 # turn left
                angular_error = 0.0
            elif front_free:
                decision_state = 1 # straight
                angular_error = 0.0
            else:
                decision_state = 3 # turn left
                angular_error = 0.0
            


        # # First check if left is NOT free and front free, if YES turn to right 
        # # Second check if right is NOT free and front free, if YES turn left
        # # Third check if front is free, if yes move straight
        # # ELSE : 
        #         # Check if left free, turn left
        #         # Check if right free, turn right
        #         # Else STOP

        # elif front_free and not left_free : #m Threshold
        #     self.dec_msg.decision_state = 2 # turn right
        # elif front_free and not right_free : #m
        #     self.dec_msg.decision_state = 3 # turn left
        # elif front_free:
        #     self.dec_msg.decision_state = 1 # straight      
        # else:
        #     if left_free:
        #         self.dec_msg.decision_state = 3 # turn left
        #     elif right_free:
        #         self.dec_msg.decision_state = 2 # turn right
        #     else:
        #        self.dec_msg.decision_state = 0 # stop         
        
        self.dec_msg.decision_state = decision_state
        self.dec_msg.angular_error = angular_error
        # Output to terminal
        self.get_logger().info(
            f"Current Decision state: {self.dec_msg.decision_state,self.at_wall,angular_error}",
            throttle_duration_sec = 1,
        )
        # publish whatever velocity command has been set above:
        self.dec_pub.publish(self.dec_msg)

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    decision_node = Decision()
    try:
        rclpy.spin(decision_node)
    except KeyboardInterrupt:
        print(
            f"{decision_node.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally:
        decision_node.on_shutdown()
        while not decision_node.shutdown:
            continue
        decision_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()