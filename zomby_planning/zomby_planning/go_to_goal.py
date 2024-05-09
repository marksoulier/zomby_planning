import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import numpy as np
from geometry_msgs.msg import Twist

class GPSReceiver(Node):
    def __init__(self):
        super().__init__('gps_receiver')
        self.get_logger().info('Go to Goal node has started.')
        self.subscription_goal = self.create_subscription(
            NavSatFix,
            'gps_coordinates_go',
            self.listener_callback,
            10)
        self.subscription_goal  # prevent unused variable warning
        #subscriber to current position
        self.subscription_current = self.create_subscription(
            NavSatFix,
            'gps_pose',
            self.listener_callback,
            10)
        self.subscription_current
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        #set origin point
        self.origin_lat = 41.740413338287134
        self.origin_lon = -111.81146033145339

        # Current position
        self.current_lat = None
        self.current_lon = None
        #testing
        self.current_lat = 41.740413338287134
        self.current_lon = -111.81146033145339

        # List to store (latitude, longitude) tuples
        self.gps_coordinates = []

        #go to goal parameters
        self.v_d = 1.0  # Desired velocity m/s

        #create timer
        # Timer to periodically update velocity
        self.timer = self.create_timer(0.1, self.update_velocity)

    def current_callback(self, msg):
        # Update current position
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def update_velocity(self):
        if not self.gps_coordinates or self.current_lat is None:
            return  # Wait until at least one goal position and current position are received
        
        # Use the first goal position in the list
        goal_lat, goal_lon = self.gps_coordinates[0]
        
        # Convert lat, lon to x, y for both goal and current positions
        goal_x, goal_y = self.lat_lon_to_xy(goal_lat, goal_lon)
        current_x, current_y = self.lat_lon_to_xy(self.current_lat, self.current_lon)

        # Publish velocity based on current and first goal position
        self.publish_velocity(goal_x, goal_y, current_x, current_y)

        # Here, decide when to remove the first goal position from the list
        # For example, if the goal is reached or after a certain condition is met
        if (goal_x - current_x) ** 2 + (goal_y - current_y) ** 2 < 0.1 ** 2:
            self.gps_coordinates.pop(0)
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Received lat: {msg.latitude}, lon: {msg.longitude}')
        self.gps_coordinates.append((msg.latitude, msg.longitude))
        # Here you could also convert lat, lon to x, y and append to another list

    def lat_lon_to_xy(self, lat, lon):
        if not hasattr(self, 'origin_lat') or not hasattr(self, 'origin_lon'):
            self.origin_lat = lat
            self.origin_lon = lon
            return (0, 0)
        
        # Assuming a simple flat Earth projection
        earth_radius = 6371000  # meters
        delta_lat = lat - self.origin_lat
        delta_lon = lon - self.origin_lon
        x = delta_lon * (earth_radius * np.cos(np.radians(self.origin_lat)))
        y = delta_lat * earth_radius
        return (x, y)

    def publish_velocity(self, goal_x, goal_y, current_x, current_y):
        # Calculate vector g (direction to the goal) and scale it by v_d
        delta_x = goal_x - current_x
        delta_y = goal_y - current_y
        distance = (delta_x ** 2 + delta_y ** 2) ** 0.5

        # Normalize the direction
        if distance > 0:
            norm_gx = delta_x / distance
            norm_gy = delta_y / distance
        else:
            norm_gx, norm_gy = 0.0, 0.0

        # Scale by v_d
        scaled_vx = norm_gx * self.v_d
        scaled_vy = norm_gy * self.v_d

        # For simplicity, assuming direct control over linear velocity and heading towards the goal
        # Angular velocity calculation could be based on the desired heading direction
        # Here's a simple proportional controller for angular velocity as an example
        angle_to_goal = np.arctan2(delta_y, delta_x)
        current_heading = 0  # Assume you have a way to get the robot's current heading
        angular_error = angle_to_goal - current_heading
        # Simple P-controller for angular velocity
        k_theta = 1.0  # Proportional gain for angular velocity
        scaled_w = k_theta * angular_error

        # Construct the Twist message
        vel_msg = Twist()
        vel_msg.linear.x = scaled_vx
        vel_msg.angular.z = scaled_w

        # Publish the message
        self.publisher_.publish(vel_msg)
        self.get_logger().info(f'Publishing: linear={scaled_vx}, angular={scaled_w}')
    
def main(args=None):
    rclpy.init(args=args)
    gps_receiver = GPSReceiver()
    rclpy.spin(gps_receiver)
    gps_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
