import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class Robot(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        # Publish the new velocity on the /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.velocity = Twist()
        self.current_position = None

    def move(self, x, z):
        self.velocity.linear.x = x
        self.velocity.angular.z = z
        self.publisher_.publish(self.velocity)
        self.get_logger().info(f'Linear = {self.velocity.linear.x}, Angular = {self.velocity.angular.z}')
        
    def odom_callback(self, msg):
        # Extract position from the odometry message
        self.current_position = msg.pose.pose.position
        self.get_logger().info(f'Current position: x={self.current_position.x}, y={self.current_position.y}')

def main(args=None):
    rclpy.init(args=args)
    robot = Robot()

    try:
        while rclpy.ok():
            # User input
            try:
                x = float(input("Enter linear velocity x: "))
                z = float(input("Enter angular velocity z: "))
            
            except ValueError:
                print("Invalid input. Please enter numeric values.")
                continue 

            robot.move(x, z)
            
            time.sleep(1)
            # stop the robot
            robot.move(0.0, 0.0)
            
            rclpy.spin_once(robot)
            
            # Display the updated position
            #if robot.current_position:
             #   rclpy.spin_once(robot)
            #else:
             #   print("Position not yet available. Waiting for odometry updates...")
            
    except Exception as e:
        print(f"Error: {e}")
    
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Shutting down....")
    
    finally:
        robot.destroy_node()

if __name__ == '__main__':
    main()