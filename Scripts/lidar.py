#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions 

from sensor_msgs.msg import LaserScan 
from acs6121_team24_2025.msg import ObstacleDistance

import numpy as np 

class Lidar(Node): 

    def __init__(self): 
        super().__init__("lidar_subscriber")

        self.lidar_msg = ObstacleDistance()

        self.lidar_sub = self.create_subscription(
            msg_type=LaserScan,
            topic="/scan",
            callback=self.lidar_callback,
            qos_profile=10,
        ) 

        self.lidar_pub = self.create_publisher(
            msg_type=ObstacleDistance,
            topic="obs_detection",
            qos_profile=10,
        )

        self.get_logger().info(f"The '{self.get_name()}' node is initialised.")

    # Compute average distance of next obstacle in 4 directions of the robot
    def lidar_callback(self, scan_data: LaserScan): 
        # define the angle for directions in degree
        mid_left_20_deg = scan_data.ranges[0:31]
        mid_right_20_deg = scan_data.ranges[-30:] 
        front = np.array(mid_left_20_deg + mid_right_20_deg) 

        fleft = np.array(scan_data.ranges[21:51])
        fright = np.array(scan_data.ranges[-50:-20])

        left = np.array(scan_data.ranges[90:100])

        right = np.array(scan_data.ranges[-100:-90])

        # compuute average distance of the 5 smallest distance measurements of the 
        # valid data (too far and too close measurements get deleted)
        front_valid_data = front[(front != float("inf")) & (front != 0.0)] 
        if np.shape(front_valid_data)[0] > 0: 
            front_valid_data_mins = np.sort(front_valid_data)[:5]
            front_single_point_average = front_valid_data_mins.mean() 
        else:
            front_single_point_average = float("nan") 
      

        fright_valid_data = fright[(fright != float("inf")) & (fright != 0.0)] 
        if np.shape(fright_valid_data)[0] > 0: 
            fright_valid_data_mins = np.sort(fright_valid_data)[:5]
            fright_single_point_average = fright_valid_data_mins.mean() 
        else:
            fright_single_point_average = float("nan") 

        fleft_valid_data = fleft[(fleft != float("inf")) & (fleft != 0.0)] 
        if np.shape(fleft_valid_data)[0] > 0: 
            fleft_valid_data_mins = np.sort(fleft_valid_data)[:5]
            fleft_single_point_average = fleft_valid_data_mins.mean()  
        else:
            fleft_single_point_average = float("nan") 

        right_valid_data = right[(right != float("inf")) & (right != 0.0)] 
        if np.shape(right_valid_data)[0] > 0: 
            right_valid_data_mins = np.sort(right_valid_data)[:5]
            right_single_point_average = right_valid_data_mins.mean()  
        else:
            right_single_point_average = float("nan") 
        
        left_valid_data = left[(left != float("inf")) & (left != 0.0)] 
        if np.shape(left_valid_data)[0] > 0: 
            left_valid_data_mins = np.sort(left_valid_data)[:5]
            left_single_point_average = left_valid_data_mins.mean()  
        else:
            left_single_point_average = float("nan") 

        # Write distance values into lidar_msg
        self.lidar_msg.front = float(front_single_point_average)
        self.lidar_msg.fright = float(fright_single_point_average)
        self.lidar_msg.fleft = float(fleft_single_point_average)
        self.lidar_msg.right = float(right_single_point_average)
        self.lidar_msg.left = float(left_single_point_average)

        self.get_logger().info(
            f"LiDAR front: {front_single_point_average:.2f} m, fright: {fright_single_point_average:.2f}m, fleft: {fleft_single_point_average:.2f} m, right: {right_single_point_average:.2f} m, left {left_single_point_average: .2f}",
            throttle_duration_sec = 1,
        ) 

        self.lidar_pub.publish(self.lidar_msg)

        

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = Lidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutdown request (Ctrl+C) detected...")
    finally:
        node.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()