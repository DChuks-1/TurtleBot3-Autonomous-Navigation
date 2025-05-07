#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SaveMap
import os

class MapSaverClient(Node):
    def __init__(self):
        super().__init__('map_saver_client')

        self.filename = f"{out_dir}/{base}"
        
        self.cli = self.create_client(SaveMap, 'map_saver/save_map')
        self.maps_dir = os.path.join(
            os.getcwd(), 'maps'
        )
        os.makedirs(self.maps_dir, exist_ok=True)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for map_saver...')
        self.timer = self.create_timer(30.0, self.save)

    def save(self):
        req = SaveMap.Request()
        req.map_url = os.path.join(self.maps_dir, 'explore_map')
        self.get_logger().info('Saving map...')
        self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = MapSaverClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()