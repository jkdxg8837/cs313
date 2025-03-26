class AddMotionNoise(Node):  
    def __init__(self):  
        super().__init__('add_motion_noise') 
        self.declare_parameter('linear_noise', value=0.001) 
        self.declare_parameter('angular_noise', value=0.001) 
        self.declare_parameter('topic_name', value='cmd_vel') 

        self.linear_noise = self.get_parameter('linear_noise').get_parameter_value().double_value 
        self.angular_noise = self.get_parameter('angular_noise').get_parameter_value().double_value 
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Twist, self.topic_name, 10) 
        self.subscription_ = self.create_subscription(Twist, self.topic_name, self.cmd_vel_callback, 10)  

        self.noise_data = []  # Store noise samples
        self.get_logger().info('AddMotionNoise has been initialized.') 

    def cmd_vel_callback(self, msg: Twist): 
        noise_x = np.random.normal(0, self.linear_noise)
        noise_z = np.random.normal(0, self.angular_noise)

        msg.linear.x += noise_x
        msg.angular.z += noise_z
        self.publisher_.publish(msg)

        # Store noise values for analysis
        self.noise_data.append([noise_x, noise_z])

        # Compute covariance matrix after collecting enough samples
        if len(self.noise_data) >= 100:  
            noise_array = np.array(self.noise_data)
            cov_matrix = np.cov(noise_array, rowvar=False)

            # Verify symmetry
            symmetric = np.allclose(cov_matrix, cov_matrix.T)

            # Compute scatter range (sqrt of diagonal)
            scatter_range = np.sqrt(np.diag(cov_matrix))

            self.get_logger().info(f"Covariance Matrix:\n{cov_matrix}")
            self.get_logger().info(f"Is Symmetric: {symmetric}")
            self.get_logger().info(f"Scatter Range: {scatter_range}")

            # Reset data collection
            self.noise_data = []
