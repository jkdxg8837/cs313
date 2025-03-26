import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Mover(Node):
    def __init__(self):
        super().__init__('vel_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.i = 0
        self.get_logger().info("Press CTRL + C to terminate")
    def timer_callback(self):
        msg = Twist()
        # Set linear velocity (move forward)
        msg.linear.x = 0.5  # Adjust speed as needed
        
        # Set angular velocity (turn)
        msg.angular.z = 0.3  # Adjust turning rate as needed
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
def main(args=None):
    rclpy.init(args=args)
    mover = Mover()
    rclpy.spin(mover)
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()