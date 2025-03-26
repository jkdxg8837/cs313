#! /usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from move_turtlebot3.utils import degrees_to_rad
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

class Mover(Node):
    def __init__(self):
        super().__init__('vel_publisher')
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.reset_gazebo_srv = self.create_client(Empty, "/reset_world")

        while not self.reset_gazebo_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"gazebo {self.reset_gazebo_srv.srv_name} is not available. Waiting.... ")

        self.callback_period = 0.1 # s
        self.timer_ = self.create_timer(self.callback_period, self.timer_callback)
        self.i = 0
        self.get_logger().info("Press CTRL + C to terminate")
        self.checkpoint = 0
        self.counter = 0
        self.forward_speed = 0.4 # m/s
        self.angular_speed = 0.4 # w/s
        self.forward_distance = 4 # m
        turn_angle_error = 5
        self.turn_angle = degrees_to_rad(90 + turn_angle_error)
        self.forward_count = self.forward_distance / self.forward_speed / self.callback_period
        self.rotation_count = self.turn_angle / self.angular_speed / self.callback_period
        self.curr_odom_pose = None
        self.final_positions = []
        
    def odom_callback(self, msg):
        self.curr_odom_pose = msg.pose.pose
        
    def get_curr_possition(self):
        while not self.curr_odom_pose:
            continue
        curr_pose = self.curr_odom_pose
        self.get_logger().info(f"possition = [{curr_pose.position.x}, {curr_pose.position.y}]") 
        return [curr_pose.position.x, curr_pose.position.y]
    
    def timer_callback(self):
        msg = Twist()

        if self.checkpoint % 2 == 0:
            if self.counter > self.forward_count:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.checkpoint += 1
                self.counter = 0  
            else:
                msg.linear.x = self.forward_speed
                msg.angular.z = 0.0
                self.counter += 1
        else:
            if self.counter > self.rotation_count:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.checkpoint += 1
                self.counter = 0  
            else:
                msg.linear.x = 0.0
                msg.angular.z = self.angular_speed
                self.counter += 1
        
        if self.checkpoint == 7:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.checkpoint = 0
            self.vel_publisher.publish(msg)
            raise SystemExit
                            
        self.vel_publisher.publish(msg)
        self.get_logger().debug(f'Checkpoint: {self.checkpoint}, Publishing: {msg}')

    def reset_simulation(self):
        self.get_logger().info(f'Calling simulation reset')
        self.checkpoint = 0
        request = Empty.Request()
        future = self.reset_gazebo_srv.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'simulation reset complete')
        
    def add_final_possition(self, possition):
        self.final_positions.append(possition)
        
    def average(self):
        N = len(self.final_positions)
        
        if N == 0:
            self.get_logger().error("Final possitions are emtpy. Unable to calculate average ")
            return None
        
        avg_x = sum([pos[0] for pos in self.final_positions])/N
        avg_y = sum([pos[1] for pos in self.final_positions])/N
     
        return avg_x, avg_y
    
    def covariance(self): 
        N = len(self.final_positions)
        
        if N == 0:
            self.get_logger().error("Final possitions are emtpy. Unable to calculate covariance ")
            return None
        
        avg_x, avg_y = self.average()
        covariance = sum([(pos[0] - avg_x) * (pos[1] - avg_y) for pos in self.final_positions])/N
        autoCor_x = sum([pow(pos[0] - avg_x, 2) for pos in self.final_positions])/N
        autoCor_y = sum([pow(pos[1] - avg_x, 2) for pos in self.final_positions])/N
        
        self.get_logger().info("Covariance Matrix \n |%s     %s| \n |%s     %s| \n" % (autoCor_x, covariance, covariance, autoCor_y))
        return ([[autoCor_x, covariance], [covariance, autoCor_y]])
        
        
        
   
def main(args=None):
    rclpy.init(args=args)
    mover = Mover()
    num_iteration = 2
    
    for i in range(num_iteration):
        try: 
            rclpy.spin(mover)
        except SystemExit:
            rclpy.logging.get_logger("main").info(f"Completion of Simulation episode {i}")
            mover.add_final_possition(mover.get_curr_possition())
            mover.reset_simulation()
            
    print(mover.covariance())

    rclpy.logging.get_logger("main").info('Done')       
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()