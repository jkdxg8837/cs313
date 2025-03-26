import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi
import threading

def main(args=None):
    rclpy.init(args=args)
    rate_node = rclpy.create_node('rate_node')
    
    t = threading.Thread(target=rclpy.spin, args=(rate_node,), daemon=True)
    t.start()
    
    frq = 10  # Frequency: Hz
    fwd_time = 4  # Time to move forward in seconds
    rot_time = 5  # Time to rotate in seconds

    rate = rate_node.create_rate(frq)  # Create a rate object with 10Hz
    pub = rate_node.create_publisher(Twist, 'cmd_vel', 10)

    try:
        msg = Twist()
        
        for i in range(4):  # Loop for four sides of the square
            # Move forward
            msg.linear.x = 0.5  # Set linear speed (adjust as needed)
            msg.angular.z = 0.0  # No rotation
            for _ in range(fwd_time * frq):
                pub.publish(msg)
                rate_node.get_logger().info('[Translation] Moving Forward')
                rate.sleep()
            
            # Stop before rotating
            msg.linear.x = 0.0
            pub.publish(msg)
            rate.sleep()
            
            # Rotate 90 degrees
            msg.angular.z = pi / 2 / rot_time  # Set angular speed (adjust as needed)
            for _ in range(rot_time * frq):
                pub.publish(msg)
                rate_node.get_logger().info('[Rotation] Rotating')
                rate.sleep()
            
            # Stop after rotation
            msg.angular.z = 0.0
            pub.publish(msg)
            rate.sleep()
            rate_node.get_logger().info('[Stop] Stopping')

    except KeyboardInterrupt:
        pass
    
    rate_node.destroy_node()
    rclpy.shutdown()
    t.join()

if __name__ == '__main__':
    main()
