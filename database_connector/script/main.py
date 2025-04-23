#!/usr/bin/env python3

from database_connector.database_connector_interface import DatabaseNodePython
import rclpy

def main(args=None):
    rclpy.init(args=args)
    node = DatabaseNodePython()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down from keyboard interrupt")
    finally:
        # Cleanup resources properly
        if "node" in locals() and rclpy.ok():
            node.destroy_node()
        # Only call shutdown if ROS is still initialized
        if rclpy.ok():
            rclpy.shutdown()
        
if __name__ == '__main__':
    main()