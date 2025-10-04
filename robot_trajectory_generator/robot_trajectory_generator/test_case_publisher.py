#!/usr/bin/env python3

"""
Test Case Publisher for Waypoint Collector

This node provides predefined test cases that can be selected by number
and automatically published as waypoints to the waypoint_collector node.

Usage:
    ros2 run robot_trajectory_generator test_case_publisher
    
Then follow the prompts to select a test case by number.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import numpy as np
import time


class TestCasePublisher(Node):
    def __init__(self):
        super().__init__('test_case_publisher')
        
        # Publisher for clicked points
        self.point_pub = self.create_publisher(PointStamped, '/clicked_point', 10)
        
        # Give ROS 2 time to set up
        time.sleep(0.5)
        
        self.get_logger().info('Test Case Publisher Node Started')
        self.get_logger().info('This will publish waypoints to /clicked_point topic')
        
        # Define all test cases
        self.test_cases = {
            1: ("Square", self.square_waypoints()),
            2: ("Triangle", self.triangle_waypoints()),
            3: ("Circle", self.circle_waypoints()),
            4: ("Figure-8", self.figure8_waypoints()),
            5: ("Zigzag", self.zigzag_waypoints()),
            6: ("Star", self.star_waypoints()),
            7: ("U-Turn", self.U_turn_waypoints()),
            8: ("Slalom", self.slalom_waypoints()),
            9: ("Parking", self.parking_maneuver()),
            10: ("Corridor", self.narrow_corridor()),
            11: ("Roundabout", self.roundabout()),
            12: ("Chicane", self.chicane()),
            13: ("Warehouse", self.warehouse_path()),
            14: ("Obstacle", self.obstacle_avoidance()),
            15: ("Line", self.line_waypoints()),
            16: ("Rhombus", self.rhombus_waypoints()),
            17: ("S-Curve", self.s_curve_waypoints()),
            18: ("Spiral", self.spiral_waypoints()),
            19: ("Hexagon", self.hexagon_waypoints()),
            20: ("Custom (Manual Entry)", []),  # Special case for manual entry
        }
    
    # ==================== WAYPOINT GENERATORS ====================
    
    def line_waypoints(self, n=5):
        return [(float(i), 0.0) for i in range(n)]
    
    def square_waypoints(self, size=5):
        return [(0.0, 0.0), (0.0, float(size)), (float(size), float(size)), 
                (float(size), 0.0), (0.0, 0.0)]
    
    def triangle_waypoints(self, size=5):
        h = size * np.sqrt(3) / 2
        return [(0.0, 0.0), (size/2, h), (float(size), 0.0), (0.0, 0.0)]
    
    def rhombus_waypoints(self, width=6, height=4):
        return [(0.0, 0.0), (width/2, float(height)), (float(width), 0.0), 
                (width/2, float(-height)), (0.0, 0.0)]
    
    def circle_waypoints(self, radius=5, points=20):
        return [(radius*np.cos(2*np.pi*i/points), radius*np.sin(2*np.pi*i/points)) 
                for i in range(points+1)]
    
    def figure8_waypoints(self, radius=5, points=40):
        t = np.linspace(0, 2*np.pi, points+1)
        return [(radius*np.sin(ti), radius*np.sin(ti)*np.cos(ti)) for ti in t]
    
    def s_curve_waypoints(self, width=10, height=5, points=30):
        t = np.linspace(0, np.pi, points)
        return [(width*(ti/np.pi), height*np.sin(ti)) for ti in t]
    
    def spiral_waypoints(self, turns=3, points_per_turn=20, spacing=1):
        waypoints = []
        for t in np.linspace(0, 2*np.pi*turns, points_per_turn*turns):
            r = spacing * t / (2*np.pi)
            waypoints.append((r*np.cos(t), r*np.sin(t)))
        return waypoints
    
    def zigzag_waypoints(self, width=8, height=4, num_zigs=4):
        waypoints = [(0.0, 0.0)]
        for i in range(1, num_zigs+1):
            x = i * width / num_zigs
            y = float(height) if i % 2 == 1 else 0.0
            waypoints.append((x, y))
        return waypoints
    
    def star_waypoints(self, radius=5, points=5):
        waypoints = []
        outer_radius = radius
        inner_radius = radius * 0.4
        for i in range(points * 2):
            angle = i * np.pi / points - np.pi/2
            r = outer_radius if i % 2 == 0 else inner_radius
            waypoints.append((r*np.cos(angle), r*np.sin(angle)))
        waypoints.append(waypoints[0])
        return waypoints
    
    def hexagon_waypoints(self, size=5):
        waypoints = []
        for i in range(7):
            angle = i * np.pi / 3
            waypoints.append((size*np.cos(angle), size*np.sin(angle)))
        return waypoints
    
    def U_turn_waypoints(self, length=8, width=4):
        return [(0.0, 0.0), (0.0, float(length)), (float(width), float(length)), 
                (float(width), 0.0)]
    
    def slalom_waypoints(self, length=10, width=3, gates=5):
        waypoints = [(0.0, 0.0)]
        for i in range(1, gates+1):
            x = i * length / gates
            y = float(width) if i % 2 == 1 else float(-width)
            waypoints.append((x, y))
        return waypoints
    
    def parking_maneuver(self):
        """Parallel parking path"""
        return [(0.0, 0.0), (5.0, 0.0), (5.0, 2.0), (3.0, 3.0), (0.0, 3.0)]
    
    def narrow_corridor(self):
        """Narrow corridor with 90-degree turns"""
        return [(0.0, 0.0), (3.0, 0.0), (3.0, 3.0), (6.0, 3.0), (6.0, 0.0), (9.0, 0.0)]
    
    def roundabout(self):
        """Circular roundabout entry/exit"""
        angles = np.linspace(0, 1.5*np.pi, 8)
        r = 3.0
        waypoints = [(0.0, 0.0), (2.0, 0.0)]
        waypoints += [(r*np.cos(a)+5, r*np.sin(a)+3) for a in angles]
        waypoints += [(8.0, 3.0), (10.0, 3.0)]
        return waypoints
    
    def chicane(self):
        """Racing chicane - alternating sharp turns"""
        return [(0.0, 0.0), (2.0, 2.0), (4.0, -2.0), (6.0, 2.0), (8.0, -2.0), (10.0, 0.0)]
    
    def warehouse_path(self):
        """Warehouse navigation with tight corners"""
        return [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (1.0, 3.0), 
                (1.0, 6.0), (5.0, 6.0), (5.0, 9.0), (0.0, 9.0)]
    
    def obstacle_avoidance(self):
        """S-shaped obstacle avoidance"""
        return [(0.0, 0.0), (2.0, 0.0), (3.0, 1.0), (3.0, 3.0), (2.0, 4.0), (0.0, 4.0)]
    
    # ==================== PUBLISHING FUNCTIONS ====================
    
    def mirror_waypoints_y(self, waypoints):
        """
        Mirror waypoints along Y-axis (flip left/right)
        
        Args:
            waypoints: List of (x, y) tuples
        
        Returns:
            Mirrored waypoints
        """
        return [(x, -y) for x, y in waypoints]
    
    def get_manual_waypoints(self):
        """
        Get waypoints manually from user input via terminal
        
        Returns:
            List of (x, y) tuples entered by user
        """
        print("\n" + "="*70)
        print(" "*15 + "📝 MANUAL WAYPOINT ENTRY 📝")
        print("="*70)
        print("\nEnter waypoints one by one. You can:")
        print("  • Type coordinates as: x,y  (e.g., '2.5,3.0')")
        print("  • Type 'done' when finished")
        print("  • Type 'cancel' to cancel and go back")
        print("  • Type 'show' to see entered waypoints")
        print("="*70)
        
        waypoints = []
        
        while True:
            try:
                print(f"\nWaypoint {len(waypoints)+1}:", end=" ")
                user_input = input().strip().lower()
                
                if user_input == 'done':
                    if len(waypoints) < 2:
                        print("⚠️  You need at least 2 waypoints. Please add more.")
                        continue
                    print(f"\n✓ Total waypoints entered: {len(waypoints)}")
                    break
                
                elif user_input == 'cancel':
                    print("\n❌ Cancelled manual entry.")
                    return []
                
                elif user_input == 'show':
                    if not waypoints:
                        print("  No waypoints entered yet.")
                    else:
                        print(f"\n  Entered waypoints ({len(waypoints)} total):")
                        for i, (x, y) in enumerate(waypoints):
                            print(f"    {i+1}. ({x:.2f}, {y:.2f})")
                    continue
                
                else:
                    # Parse x,y coordinates
                    parts = user_input.split(',')
                    if len(parts) != 2:
                        print("❌ Invalid format. Use: x,y (e.g., '2.5,3.0')")
                        continue
                    
                    x = float(parts[0].strip())
                    y = float(parts[1].strip())
                    
                    waypoints.append((x, y))
                    print(f"   ✓ Added: ({x:.2f}, {y:.2f})")
            
            except ValueError:
                print("❌ Invalid numbers. Please use format: x,y (e.g., '2.5,3.0')")
            except KeyboardInterrupt:
                print("\n\n⚠️  Interrupted. Cancelling manual entry.")
                return []
        
        return waypoints
    
    def publish_waypoints(self, waypoints, delay=0.5, mirror=False):
        """
        Publish waypoints one by one to /clicked_point topic
        
        Args:
            waypoints: List of (x, y) tuples
            delay: Delay between publishing each waypoint (seconds)
            mirror: Whether to mirror along Y-axis (flip left/right)
        """
        # Apply mirroring if requested
        if mirror:
            waypoints = self.mirror_waypoints_y(waypoints)
            self.get_logger().info('Applied Y-axis mirroring (left↔right)')
        
        self.get_logger().info(f'Publishing {len(waypoints)} waypoints...')
        
        for i, (x, y) in enumerate(waypoints):
            # Create PointStamped message
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.point.x = float(x)
            msg.point.y = float(y)
            msg.point.z = 0.0
            
            # Publish
            self.point_pub.publish(msg)
            
            self.get_logger().info(f'  Published waypoint {i+1}/{len(waypoints)}: ({x:.2f}, {y:.2f})')
            
            # Wait before next waypoint
            time.sleep(delay)
        
        self.get_logger().info('✓ All waypoints published!')
    
    def display_menu(self):
        """Display the test case selection menu"""
        print("\n" + "="*70)
        print(" "*20 + "TEST CASE SELECTION MENU")
        print("="*70)
        print("\nAvailable Test Cases:")
        print("-"*70)
        
        for num, (name, _) in sorted(self.test_cases.items()):
            print(f"  {num:2d}. {name}")
        
        print("-"*70)
        print("  0. Exit")
        print("="*70)
        print("\n💡 TIPS:")
        print("   • Type '20' for manual waypoint entry (type your own coordinates!)")
        print("   • Add 'm' to mirror any path (e.g., '1m' for mirrored Square)")
        print("="*70)
    
    def run_interactive(self):
        """Run interactive test case selection"""
        while True:
            self.display_menu()
            
            try:
                choice = input("\nEnter test case number (0 to exit, add 'm' for mirror, e.g., '1m'): ").strip()
                
                if not choice:
                    continue
                
                # Check for mirror flag
                mirror = False
                if choice.lower().endswith('m'):
                    mirror = True
                    choice = choice[:-1].strip()
                
                choice_num = int(choice)
                
                if choice_num == 0:
                    self.get_logger().info('Exiting...')
                    break
                
                if choice_num in self.test_cases:
                    name, waypoints = self.test_cases[choice_num]
                    
                    # Special handling for manual entry (test case 20)
                    if choice_num == 20:
                        waypoints = self.get_manual_waypoints()
                        if not waypoints:
                            print("\n  Returning to menu...")
                            continue
                        # Manual entry can still be mirrored if 'm' was added
                        name = "Custom (Manual Entry)"
                    
                    mirror_text = " (MIRRORED)" if mirror else ""
                    print(f"\n✓ Selected: {name}{mirror_text}")
                    print(f"  Number of waypoints: {len(waypoints)}")
                    
                    # Show preview with mirroring applied if requested
                    preview_waypoints = self.mirror_waypoints_y(waypoints) if mirror else waypoints
                    print(f"\n  Waypoints{' (mirrored)' if mirror else ''}:")
                    for i, (x, y) in enumerate(preview_waypoints[:5]):  # Show first 5
                        print(f"    {i+1}. ({x:.2f}, {y:.2f})")
                    if len(waypoints) > 5:
                        print(f"    ... and {len(waypoints)-5} more")
                    
                    confirm = input(f"\n  Publish these waypoints? (y/n): ").strip().lower()
                    
                    if confirm == 'y':
                        print(f"\n📤 Publishing waypoints for '{name}'{mirror_text}...")
                        self.publish_waypoints(waypoints, delay=0.3, mirror=mirror)
                        print(f"\n✅ Done! Check RViz to see the path.")
                        
                        cont = input("\n  Publish another test case? (y/n): ").strip().lower()
                        if cont != 'y':
                            break
                    else:
                        print("  Cancelled.")
                else:
                    print(f"\n❌ Invalid choice: {choice_num}")
                    print("   Please select a number from the menu.")
            
            except ValueError:
                print("\n❌ Please enter a valid number (optionally followed by 'm' for mirror).")
            except KeyboardInterrupt:
                print("\n\n⚠️  Interrupted by user. Exiting...")
                break
            except Exception as e:
                self.get_logger().error(f'Error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = TestCasePublisher()
    
    print("\n" + "="*70)
    print(" "*15 + "🎯 TEST CASE PUBLISHER 🎯")
    print("="*70)
    print("\nThis tool publishes predefined waypoints to /clicked_point topic")
    print("Make sure the waypoint_collector node is running!")
    print("="*70)
    
    try:
        node.run_interactive()
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
