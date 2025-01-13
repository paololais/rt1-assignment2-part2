import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool
from threading import Thread
import time

class Robot(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        # Publish the new velocity on the /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        
         # Publisher for position in feet
        self.position_publisher_ = self.create_publisher(Float32MultiArray, 'robot_position_feet', 10)
        
        # Service to stop/restart the robot
        self.service_ = self.create_service(SetBool, 'control_robot', self.control_robot_callback)
        
        self.velocity = Twist()
        self.current_position = None
        self.robot_active = True

    def move(self, x, z):
        if self.robot_active:
            self.velocity.linear.x = x
            self.velocity.angular.z = z
            self.publisher_.publish(self.velocity)
            self.get_logger().info(f'Linear = {self.velocity.linear.x}, Angular = {self.velocity.angular.z}')
        else:
            self.get_logger().warn("Robot is stopped. Cannot move.")

    def stop(self):
        self.robot_active = False
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.publisher_.publish(self.velocity)
        self.get_logger().info("Robot has been stopped.")
    
    def restart(self,x,z):
        self.robot_active = True
        self.move(x,z)
        self.get_logger().info("Robot has been restarted.")
    
    def odom_callback(self, msg):
        # Extract position from the odometry message
        self.current_position = msg.pose.pose.position
        self.get_logger().info(f'Current position: x={self.current_position.x}, y={self.current_position.y}')
        
        # Publish position in feet
        position_in_feet = Float32MultiArray()
        position_in_feet.data = [
            self.current_position.x * 3.28,  # Convert meters to feet
            self.current_position.y * 3.28
        ]
        self.position_publisher_.publish(position_in_feet)
        self.get_logger().info(f'Position in feet: x={position_in_feet.data[0]}, y={position_in_feet.data[1]}')
    
    def control_robot_callback(self, request, response):
        # Handle stop/restart requests
        if request.data:
            self.robot_active = False
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.0
            self.publisher_.publish(self.velocity)  # Stop the robot
            response.success = True
            response.message = "Robot stopped."
        else:
            self.robot_active = True
            response.success = True
            response.message = "Robot restarted."
        
        self.get_logger().info(response.message)
        return response

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
            print(" press 's' to Stop the robot")
            choice = input("Enter your choice: ")
            
            if choice == "s":
                robot.stop()
                print("press 's' to restart the robot")
                choice = input("Enter your choice: ")
                if choice == "r":
                    robot.restart(x,z)
                    print("The robot has been restarted.")
                    time.sleep(3)
                    robot.move(0.0, 0.0)
                    
            time.sleep(3)
            robot.move(0.0, 0.0)
        
            rclpy.spin_once(robot)
            
    except Exception as e:
        print(f"Error: {e}")
    
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Shutting down....")
    
    finally:
        robot.destroy_node()

if __name__ == '__main__':
    main()