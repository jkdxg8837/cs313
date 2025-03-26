#! /usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Pose2D
from move_turtlebot3.utils import degrees_to_rad
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import tf_transformations
from math import pi, sqrt, atan2, cos, sinh, copysign, fabs, degrees
import numpy as np
from move_turtlebot3.controller import Controller

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
        self.rate = self.create_rate(2)
        self.get_logger().info("Press CTRL + C to terminate")
        
        self.MAX_FORWARD_SPEED = 0.5 # m/s
        self.FORWARD_OFFSET = 0.01 # m/s
        self.MAX_ANGULAR_SPEED = 0.5 # rad/s
        self.ROTATION_THREASHOLD = 0.01 # rad
        self.DISTANCE_THREASHOLD = 0.01 # m
        self.waypoints = np.array([[4, 0], [4, 4], [0, 4], [0, 0]])
        
        self.final_positions = []
        self.waypoints_index = 0
        self.pose = Pose2D()
        self.pose_initialized = False
        self.rotation_state = True
        self.trajectory = []
        
        
        self.rotate_controller = Controller(set_point=self.find_waypoint_direction_delta(waypoint=self.waypoints[self.waypoints_index]))
        self.speed_controller = Controller(set_point=0)

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        # Transfer the theta data from euler sys.
        self.get_logger().info(f"odom_callback")
        quaternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(quaternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.pose_initialized = True
        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        # self.logging_counter += 1
        # if self.logging_counter == 100:
        #     self.logging_counter = 0
        #     self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
        #     self.get_logger().debug("odom: x=" + str(self.pose.x) +\
        #         ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))
    
    
    def timer_callback(self):
        if not self.pose_initialized:
            return
        
        if self.waypoints_index < 4:
            
            if self.rotation_state:
                self.rotation_state = not self.rotate_towards(self.find_waypoint_direction_delta(waypoint=self.waypoints[self.waypoints_index]))
            else:
                is_next_waypoint = self.move_towards(self.waypoints[self.waypoints_index])
                
                if is_next_waypoint:
                    self.waypoints_index+= 1
        
        else:
            self.waypoints_index = 0
            raise SystemExit
    
    
    def find_waypoint_direction_delta(self, waypoint):
        target = waypoint - [self.pose.x, self.pose.y]
        return atan2(target[1], target[0])


    def corrected_yaw(self, waypoint_direction_delta):
        yaw = self.pose.theta
        sign = lambda x: copysign(1, x)

        if fabs(yaw) + fabs(waypoint_direction_delta) > pi and sign(yaw) != sign(waypoint_direction_delta):
            yaw = yaw + copysign(2*pi, waypoint_direction_delta)

        return yaw
     
    def calc_distance(self, waypoint):
        dir_vect = waypoint - np.array([self.pose.x, self.pose.y])
        return sqrt(dir_vect[0]**2 + dir_vect[1]**2)
    
    
    def rotate_towards(self, waypoint_direction_delta):

        self.get_logger().info(f"waypoint_direction_change: {degrees(waypoint_direction_delta)}")

        current_yaw = self.corrected_yaw(waypoint_direction_delta)

        if fabs(current_yaw - waypoint_direction_delta) > self.ROTATION_THREASHOLD:
            vel = Twist()
            vel.angular.z = self.rotate_controller.update(current_yaw)
            self.vel_publisher.publish(vel)
            return False
        else:
            vel = Twist()
            self.vel_publisher.publish(vel)
            return True


    def move_towards(self, waypoint):

        waypoint_direction_delta = self.find_waypoint_direction_delta(waypoint)
        self.rotate_controller.setPoint(waypoint_direction_delta)
        current_distance = self.calc_distance(waypoint)

        self.get_logger().info("current distance: %f, current waypoint direction_delta: %f", current_distance, degrees(waypoint_direction_delta))

        current_yaw = self.corrected_yaw(waypoint_direction_delta)
        self.get_logger().debug("current yaw: %f", degrees(current_yaw))


        if current_distance > self.DISTANCE_THREASHOLD:
            vel = Twist()
            move = -self.speed_controller.update(current_distance)
            vel.linear.x = min(move, self.MAX_FORWARD_SPEED) + self.FORWARD_OFFSET
            vel.angular.z = self.rotate_controller.update(current_yaw)
            self.vel_pub.publish(vel)
            return False
        else:
            vel = Twist()
            self.vel_pub.publish(vel)
            return True
             
             
    def get_curr_possition(self):
        while not self.pose.x:
            continue

        self.get_logger().info(f"possition = [{self.pose.x}, {self.pose.y}]") 
        return [self.pose.x, self.pose.y]
    

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