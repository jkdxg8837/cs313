import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi

def main(args=None):
    rclpy.init(args=args)
    rate_node = rclpy.create_node('rate_node')

    import threading
    t = threading.Thread(target=rclpy.spin, args=(rate_node,), daemon=True)
    # the rate relies on the spin function running in a separate thread
    t.start()

    frq = 10 # frequence: Hz
    fwd_time = 4
    rot_time = 5 

    rate = rate_node.create_rate(frq) # create a rate object with 10Hz
    # use rate.sleep() to keep the frequency
    pub = rate_node.create_publisher(Twist, 'cmd_vel', 10)
    try:
        msg = Twist()

        for i in range(4*fwd_time*frq):
        # TODO: do something here
            rate_node.get_logger().info('[Translation] Publishing: "%s"' % i)
            rate.sleep() # the code will sleep to keep the frequency, in this case 10Hz

            if (i+1) % (fwd_time*frq) == 0:
                rate.sleep()
            for _ in range(rot_time*frq):
            # TODO: do something here
                rate_node.get_logger().info('[Rotation] Publishing: "%s"' % i) 
                rate.sleep()

            rate_node.get_logger().info('[Stop] Publishing: "%s"' % i)

    except KeyboardInterrupt:
        pass 

    rate_node.destroy_node() 
    rclpy.shutdown()
    t.join()

if __name__ == '__main__': main()


from typing import List
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter

class Mover(Node):
    def __init__(self, node_name: str = 'mover'):
        super().__init__(node_name=node_name)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback) # 1.0s period, 1Hz 
        self.i = 0
        self.get_logger().info('Mover has been initialized.')

def timer_callback(self):
    # TODO: do something here
    # this function will be called every 1.0s 

    pass