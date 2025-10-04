#!/usr/bin/env python3

"""
Waypoint Collector Node for Robot Trajectory Generator

This node subscribes to clicked points from RViz and collects them as waypoints
for trajectory generation.

Topics:
- Subscribe: /clicked_point (geometry_msgs/PointStamped) - from RViz "Publish Point" tool
- Publish: /waypoints (nav_msgs/Path) - collected waypoints as a path
- Publish: /waypoint_markers (visualization_msgs/MarkerArray) - visualization in RViz

Services:
- /clear_waypoints (std_srvs/Trigger) - clear all collected waypoints
- /save_waypoints (std_srvs/Trigger) - save waypoints to file (future implementation)

How to use:
1. Launch this node
2. In RViz, add "Path" display and subscribe to /waypoints
3. In RViz, add "MarkerArray" display and subscribe to /waypoint_markers
4. Click "Publish Point" button in RViz toolbar
5. Click on the map/grid to add waypoints
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger


class WaypointCollector(Node):
    def __init__(self):
        super().__init__('waypoint_collector')
        
        # Parameters
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('max_waypoints', 100)
        self.declare_parameter('default_z', 0.0)
        
        self.frame_id = self.get_parameter('frame_id').value
        self.max_waypoints = self.get_parameter('max_waypoints').value
        self.default_z = self.get_parameter('default_z').value
        
        # Waypoint storage
        self.waypoints = []  # List of (x, y, z) tuples
        
        # Subscribers
        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/waypoints', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        
        # Services
        self.clear_srv = self.create_service(
            Trigger,
            '/clear_waypoints',
            self.clear_waypoints_callback
        )
        
        # Timer for periodic publishing
        self.timer = self.create_timer(0.5, self.publish_waypoints)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Waypoint Collector Node Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Instructions:')
        self.get_logger().info('  1. Open RViz')
        self.get_logger().info('  2. Click "Publish Point" tool in toolbar')
        self.get_logger().info('  3. Click on the grid/map to add waypoints')
        self.get_logger().info('')
        self.get_logger().info('RViz Displays to add:')
        self.get_logger().info('  - Path: topic=/waypoints')
        self.get_logger().info('  - MarkerArray: topic=/waypoint_markers')
        self.get_logger().info('')
        self.get_logger().info(f'Subscribed to: /clicked_point')
        self.get_logger().info(f'Publishing to: /waypoints, /waypoint_markers')
        self.get_logger().info(f'Max waypoints: {self.max_waypoints}')
        self.get_logger().info('=' * 60)
    
    def clicked_point_callback(self, msg: PointStamped):
        """
        Callback for clicked points from RViz
        """
        # Check if we've reached max waypoints
        if len(self.waypoints) >= self.max_waypoints:
            self.get_logger().warn(
                f'Maximum waypoints ({self.max_waypoints}) reached! '
                f'Call /clear_waypoints service to reset.'
            )
            return
        
        # Extract point coordinates
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z if msg.point.z != 0.0 else self.default_z
        
        # Add to waypoints list
        self.waypoints.append((x, y, z))
        
        self.get_logger().info(
            f'Waypoint {len(self.waypoints)} added: '
            f'({x:.3f}, {y:.3f}, {z:.3f})'
        )
        
        # Publish immediately
        self.publish_waypoints()
    
    def publish_waypoints(self):
        """
        Publish waypoints as Path and MarkerArray for visualization
        """
        if not self.waypoints:
            return
        
        # Create and publish Path message
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y, z in self.waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0  # Default orientation
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        
        # Create and publish markers
        self.publish_markers()
    
    def publish_markers(self):
        """
        Create visualization markers for waypoints
        """
        marker_array = MarkerArray()
        
        # Create sphere markers for each waypoint
        for i, (x, y, z) in enumerate(self.waypoints):
            # Sphere marker
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            
            # Color gradient from green to red
            if len(self.waypoints) == 1:
                marker.color.r = 0.0
                marker.color.g = 1.0
            else:
                ratio = i / (len(self.waypoints) - 1)
                marker.color.r = ratio
                marker.color.g = 1.0 - ratio
            marker.color.b = 0.0
            marker.color.a = 0.9
            
            marker_array.markers.append(marker)
            
            # Text marker for waypoint number
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "waypoint_numbers"
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = z + 0.25
            
            text_marker.scale.z = 0.12
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = str(i + 1)
            
            marker_array.markers.append(text_marker)
        
        # Create line strip connecting waypoints
        if len(self.waypoints) > 1:
            line_marker = Marker()
            line_marker.header.frame_id = self.frame_id
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "waypoint_path"
            line_marker.id = 10000
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            
            line_marker.scale.x = 0.05  # Line width
            line_marker.color.r = 0.0
            line_marker.color.g = 0.5
            line_marker.color.b = 1.0
            line_marker.color.a = 0.7
            
            for x, y, z in self.waypoints:
                point = Point()
                point.x = x
                point.y = y
                point.z = z
                line_marker.points.append(point)
            
            marker_array.markers.append(line_marker)
        
        self.markers_pub.publish(marker_array)
    
    def clear_waypoints_callback(self, request, response):
        """
        Service callback to clear all waypoints
        """
        num_waypoints = len(self.waypoints)
        self.waypoints.clear()
        
        # Publish delete all markers
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.markers_pub.publish(marker_array)
        
        response.success = True
        response.message = f'Cleared {num_waypoints} waypoints'
        self.get_logger().info(response.message)
        
        return response
    
    def get_waypoints(self):
        """
        Return the list of collected waypoints
        """
        return self.waypoints.copy()


def main(args=None):
    rclpy.init(args=args)
    
    node = WaypointCollector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Waypoint Collector...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
