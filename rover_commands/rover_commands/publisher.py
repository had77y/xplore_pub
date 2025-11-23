import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('Info_publisher')
        # TODO: Create a publisher of type Twist
        # Your code here
        self.publisher_=self.create_publisher(String, 'command',18)
        
        self.timer = self.create_timer(0.5, self.cmd_acquisition)
        self.get_logger().info('Publisher node started')

        # TODO: Create a loop here to ask users a prompt and send messages accordingly

        # Function that prompts user for a direction input, and sends the command
    def cmd_acquisition(self):
        command = input("Enter command:  ")
        msg= String()
        msg.data=command
        self.publisher_.publish(msg);
        self.get_logger().info('command: '+ command +'sended ')
        # TODO: Complete the function to transform the input into the right command.
        # Your code here
        pass
        
def keyboard_callback(keyboard_event):
    msg= String()
    msg=keyboard_event
    self.publisher_.publish(msg)
    self.get_logger().info('command: '+ command +' sended ')
     
    
    
    
def main(args=None):
    rclpy.init(args=args)   # Init ROS python
    node = TrajectoryPublisher()  # Create a Node instance
    rclpy.spin(node)  # Run the node in a Thread
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
