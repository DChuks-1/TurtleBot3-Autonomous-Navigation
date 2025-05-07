#!/usr/bin/env python3
"""
ROS2 node for autonomous exploration with A* path planning.
Transitions from reactive wall-following to deliberative path planning
as required in the assignment.
"""
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from acs6121_team24_2025.msg import DecisionState
from acs6121_team24_2025_modules.tb3_tools import quaternion_to_euler

import numpy as np
import math
import heapq
import time
import random
from enum import Enum


class ExplorationState(Enum):
    INITIALIZE    = 0
    PLAN          = 1
    FOLLOW_PATH   = 2
    REACT         = 3
    WALL_FOLLOW   = 4
    STOP          = 5


class Exploration(Node):
    def __init__(self):
        super().__init__('exploration')
        # Robot/map parameters

        self.map_data = None
        self.map_width = self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin = None

        # Path planning
        self.path = []
        self.current_waypoint_idx = 0
        self.path_timeout = 30.0
        self.last_plan_time = 0.0
        self.waypoint_reached_distance = 0.15

        # Motion params
        self.linear_velocity  = 0.2
        self.angular_velocity = 0.7
        self.max_angular_velocity = 1.0

        # Obstacle thresholds
        self.front_obstacle_threshold = 0.45
        self.side_obstacle_threshold  = 0.25

        # Exploration state
        self.state = ExplorationState.INITIALIZE
        self.pose = None
        self.orientation = 0.0
        self.start_time = time.time()
        self.max_exploration_time = 300.0
        self.stuck_counter = 0
        self.stuck_threshold = 10
        self.exploration_complete = False

        # LiDAR readings
        self.front_dist       = float('inf')
        self.front_left_dist  = float('inf')
        self.front_right_dist = float('inf')
        self.left_dist        = float('inf')
        self.right_dist       = float('inf')
        
        # Decision State
        self.decision_state = 0
        self.create_subscription(
            DecisionState,
            'decision_state',
            self._decision_callback,
            10
        )

        # Frontier
        self.frontiers = []
        self.visited_goals = set()
        self.min_frontier_size = 3
        self.frontier_cluster_distance = 5
        self.explore_closest_frontier = True

        # ROS publishers/subscribers
        self.cmd_vel_pub  = self.create_publisher(Twist, 'cmd_vel', 10)
        self.decision_pub = self.create_publisher(DecisionState, 'decision_state', 10)
        self.path_pub     = self.create_publisher(MarkerArray, 'path_markers', 10)
        self.frontier_pub = self.create_publisher(MarkerArray, 'frontier_markers', 10)

        self.map_sub  = self.create_subscription(OccupancyGrid, 'map',  self.map_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan,      'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,       'odom', self.odom_callback, 10)

        # Timers
        self.create_timer(0.1, self.control_loop)
        self.create_timer(1.0, self.planning_loop)
        self.create_timer(0.5, self.publish_visualizations)

        self.get_logger().info("Exploration node started")

    def map_callback(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        if self.state == ExplorationState.INITIALIZE and self.pose is not None:
            self.state = ExplorationState.PLAN
            self.get_logger().info("Map received, switching to PLAN")

    def scan_callback(self, msg):
        ranges = np.nan_to_num(np.array(msg.ranges),
                               nan=msg.range_max,
                               posinf=msg.range_max,
                               neginf=0.0)
        n = len(ranges)
        front  = np.concatenate((ranges[0:31], ranges[-30:]))
        fl     = ranges[n//2-60:n//2-20]
        fr     = ranges[n//2+20:n//2+60]
        left   = ranges[n//2+80:n//2+100]
        right  = ranges[n//2-100:n//2-80]
        def avg5(sector):
            valid = sector[(sector>0)&(sector<msg.range_max)]
            if len(valid)>=5:
                return float(np.sort(valid)[:5].mean())
            elif len(valid)>0:
                return float(valid.mean())
            else:
                return msg.range_max
        self.front_dist       = avg5(front)
        self.front_left_dist  = avg5(fl)
        self.front_right_dist = avg5(fr)
        self.left_dist        = avg5(left)
        self.right_dist       = avg5(right)

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        _,_,yaw = quaternion_to_euler(self.pose.orientation)
        self.orientation = yaw
        
    def _decision_callback(self, msg):
        # msg.state {0: stop, 1:forward, 2:turn right, 3: turn left, 4: around}
        self.decision_state = msg.state

    def planning_loop(self):
        if self.map_data is None or self.pose is None:
            return
        if time.time() - self.start_time > self.max_exploration_time:
            self.exploration_complete = True
            self.state = ExplorationState.STOP
            return
        if self.state == ExplorationState.PLAN:
            self.plan_path()
        elif self.state == ExplorationState.FOLLOW_PATH:
            if not self.path or (time.time() - self.last_plan_time)>self.path_timeout:
                self.state = ExplorationState.PLAN
        elif self.state == ExplorationState.REACT:
            if self.front_dist > self.front_obstacle_threshold:
                self.state = ExplorationState.FOLLOW_PATH

    def plan_path(self):
        # Detect frontiers
        raw = []
        for y in range(self.map_height):
            for x in range(self.map_width):
                if self.map_data[y,x]==0:
                    for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                        nx,ny=x+dx,y+dy
                        if 0<=nx<self.map_width and 0<=ny<self.map_height and self.map_data[ny,nx]==-1:
                            raw.append((x,y)); break
        # cluster
        clusters=[]; rem=set(raw)
        while rem:
            seed=rem.pop(); cl=[seed]; queue=[seed]
            while queue:
                ux,uy=queue.pop(0)
                for dx in range(-self.frontier_cluster_distance, self.frontier_cluster_distance+1):
                    for dy in range(-self.frontier_cluster_distance, self.frontier_cluster_distance+1):
                        nb=(ux+dx, uy+dy)
                        if nb in rem:
                            rem.remove(nb); cl.append(nb); queue.append(nb)
            clusters.append(cl)
        self.frontiers=[c for c in clusters if len(c)>=self.min_frontier_size]
        if not self.frontiers:
            self.exploration_complete=True
            self.state=ExplorationState.STOP
            return
        # select
        robot_cell=self.world_to_grid(self.pose.position.x, self.pose.position.y)
        choices=[]
        for cl in self.frontiers:
            cx=sum(p[0] for p in cl)//len(cl)
            cy=sum(p[1] for p in cl)//len(cl)
            if (cx,cy) not in self.visited_goals:
                choices.append((cx,cy,len(cl)))
        if not choices:
            self.state=ExplorationState.WALL_FOLLOW
            return
        if self.explore_closest_frontier:
            choices.sort(key=lambda t: (t[0]-robot_cell[0])**2+(t[1]-robot_cell[1])**2)
        else:
            choices.sort(key=lambda t: t[2], reverse=True)
        goal=choices[0][:2]
        self.visited_goals.add(goal)
        # A*
        start=robot_cell; target=goal
        dirs=[(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
        h=lambda a,b: abs(a[0]-b[0])+abs(a[1]-b[1])
        open_set=[(h(start,target),0,start,None)]
        came,cost={}, {start:0}
        while open_set:
            _,g,cur,pr=heapq.heappop(open_set)
            if cur in came: continue
            came[cur]=pr
            if cur==target: break
            for dx,dy in dirs:
                nb=(cur[0]+dx,cur[1]+dy)
                if not (0<=nb[0]<self.map_width and 0<=nb[1]<self.map_height): continue
                idx=nb[1]*self.map_width+nb[0]
                if self.map_data[nb[1],nb[0]]>50: continue
                step=math.sqrt(2) if abs(dx)+abs(dy)==2 else 1
                ng=g+step
                if ng<cost.get(nb,1e9):
                    cost[nb]=ng
                    heapq.heappush(open_set,(ng+h(nb,target),ng,cur))
        # reconstruct
        path=[]; node=target
        while node in came:
            path.append(node); node=came[node]
        self.path=[self.grid_to_world(x,y) for x,y in path[::-1]]
        self.current_waypoint_idx=0
        self.last_plan_time=time.time()
        self.state=ExplorationState.FOLLOW_PATH
        self.get_logger().info(f"Planned {len(self.path)} waypoints")

    def control_loop(self):
        if self.exploration_complete or self.pose is None:
            self.cmd_vel_pub.publish(Twist())
            return
        twist=Twist()
        if self.state==ExplorationState.FOLLOW_PATH:
            if self.front_dist<self.front_obstacle_threshold:
                self.state=ExplorationState.REACT
            else:
                self._follow_path(twist)
        elif self.state==ExplorationState.REACT:
            self._react_via_fsm(twist)
        elif self.state==ExplorationState.WALL_FOLLOW:
            self._follow_wall(twist)
        self.cmd_vel_pub.publish(twist)
        self._publish_decision_state()
        
    def _react_via_fsm(self, twist: Twist):
        # 1 = forward, 2 = turn right, 3 = turn left, 4 = skirt wall, 0 = stop
        ds = self.decision_state
        if ds == 1:
            twist.linear.x = self.linear_velocity
            twist.angular.z = 0.0
        elif ds == 2:
            twist.linear.x  = 0.0
            twist.angular.z = -self.angular_velocity
        elif ds == 3:
            twist.linear.x  = 0.0
            twist.angular.z =  self.angular_velocity
        elif ds == 4:
            # “around wall” behavior, e.g. small forward + turn—
            twist.linear.x  = self.linear_velocity * 0.5
            twist.angular.z =  self.angular_velocity * 0.5
        else:
            twist.linear.x  = 0.0
            twist.angular.z = 0.0

    def _follow_path(self, twist):
        xw,yw=self.path[self.current_waypoint_idx]
        dx=xw-self.pose.position.x; dy=yw-self.pose.position.y
        dist=math.hypot(dx,dy)
        if dist<self.waypoint_reached_distance:
            self.current_waypoint_idx+=1
            if self.current_waypoint_idx>=len(self.path):
                self.state=ExplorationState.PLAN
                return
            xw,yw=self.path[self.current_waypoint_idx]
            dx=xw-self.pose.position.x; dy=yw-self.pose.position.y
        targ=math.atan2(dy,dx)
        err=((targ-self.orientation+math.pi)%(2*math.pi))-math.pi
        if abs(err)>0.1:
            twist.angular.z=self.angular_velocity*math.copysign(1,err)
        else:
            twist.linear.x=self.linear_velocity

    def _avoid_obstacle(self, twist):
        twist.linear.x=0.0
        twist.angular.z = self.angular_velocity if self.left_dist>self.right_dist else -self.angular_velocity
        if self.front_dist>self.front_obstacle_threshold:
            self.state=ExplorationState.FOLLOW_PATH

    def _follow_wall(self, twist):
        err = self.side_obstacle_threshold - self.right_dist
        twist.linear.x=self.linear_velocity
        twist.angular.z = max(-self.angular_velocity, min(self.angular_velocity, 3.0*err))
        if time.time()-self.last_plan_time>10.0:
            self.state=ExplorationState.PLAN

    def _publish_decision_state(self):
        msg=DecisionState()
        sm={ExplorationState.STOP:0,
            ExplorationState.FOLLOW_PATH:1,
            ExplorationState.REACT:2,
            ExplorationState.WALL_FOLLOW:4}
        msg.state=sm.get(self.state,0)
        self.decision_pub.publish(msg)

    def publish_visualizations(self):
        # path markers
        if self.path:
            ma=MarkerArray()
            line=Marker(); line.header.frame_id='map'; line.header.stamp=self.get_clock().now().to_msg()
            line.ns='path'; line.id=0; line.type=Marker.LINE_STRIP; line.action=Marker.ADD
            line.scale.x=0.03; line.color=ColorRGBA(r=0,g=1,b=0,a=1)
            for x,y in self.path:
                line.points.append(Point(x=x,y=y,z=0.05))
            ma.markers.append(line)
            self.path_pub.publish(ma)
        # frontier markers
        if self.frontiers:
            ma=MarkerArray()
            d=Marker(); d.header.frame_id='map'; d.header.stamp=self.get_clock().now().to_msg()
            d.ns='frontiers'; d.id=0; d.action=Marker.DELETEALL
            ma.markers.append(d)
            for idx,cl in enumerate(self.frontiers,1):
                pts=Marker(); pts.header.frame_id='map'; pts.header.stamp=self.get_clock().now().to_msg()
                pts.ns='frontiers'; pts.id=idx; pts.type=Marker.POINTS; pts.action=Marker.ADD
                pts.scale.x=self.map_resolution*0.5; pts.scale.y=self.map_resolution*0.5
                pts.color=ColorRGBA(r=1,g=0,b=0,a=1)
                for cx,cy in cl:
                    wx=self.map_origin.position.x+(cx+0.5)*self.map_resolution
                    wy=self.map_origin.position.y+(cy+0.5)*self.map_resolution
                    pts.points.append(Point(x=wx,y=wy,z=0.05))
                ma.markers.append(pts)
            self.frontier_pub.publish(ma)

    def normalize_angle(self, a):
        while a>math.pi: a-=2*math.pi
        while a<-math.pi: a+=2*math.pi
        return a

    def world_to_grid(self,x,y):
        gx=int((x-self.map_origin.position.x)/self.map_resolution)
        gy=int((y-self.map_origin.position.y)/self.map_resolution)
        return (max(0,min(self.map_width-1,gx)), max(0,min(self.map_height-1,gy)))

    def grid_to_world(self,gx,gy):
        x=self.map_origin.position.x+(gx+0.5)*self.map_resolution
        y=self.map_origin.position.y+(gy+0.5)*self.map_resolution
        return (x,y)

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node=Exploration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
