import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.time = 0

    def create_twist(self, linear_x, angular_z):
        '''
        Input a specified linear (x) and angular (z) velocity
        Output a Twist message with those velocities that can be published to /turtle1/cmd_vel to control the turtle's movement.
        '''
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    def get_twist_msg(self):
        '''
        The goal is to generate a rectangle, I chose side lengths of 1 and 4 units with angles of pi/2.

        Here is the breakdown of the rectangle generation:
        

        First Side: Travel a linear distance of 1 unit and then turn pi/2 radians.
        Second Side: Travel a linear distance of 4 units and then turn pi/2 radians.
        Third Side: Travel a linear distance of 1 unit and then turn pi/2 radians.
        Fourth Side: Travel a linear distance of 4 units and then turn pi/2 radians.

        Finally, stop the turtle indefinitely after completing the rectangle.
        '''
        #NEED TO TEST THIS ONE AGAIN
        if 0 <= self.time < 2:
            msg = self.create_twist(1.0, 0.0)
        elif 2 <= self.time < 4:
            msg = self.create_twist(0.0, 1.57079632679)
        elif 4 <= self.time < 6:
            msg = self.create_twist(4.0, 0.0)
        elif 6 <= self.time < 8:
            msg = self.create_twist(0.0, 1.57079632679)
        elif 8 <= self.time < 10:
            msg = self.create_twist(1.0, 0.0)
        elif 10 <= self.time < 12:
            msg = self.create_twist(0.0, 1.57079632679)
        elif 12 <= self.time < 14:
            msg = self.create_twist(4.0, 0.0)
        elif 14 <= self.time < 16:
            msg = self.create_twist(0.0, 1.57079632679)
        else:
            msg = self.create_twist(0.0, 0.0)
        return msg

    
    def timer_callback(self):
        msg = self.get_twist_msg()       
        self.publisher.publish(msg)
        self.time += 1
        print("Generating rectangle, time: {}".format(self.time))

def main(args=None):
    rclpy.init(args=args)

    turtle_controller = TurtleController()

    rclpy.spin(turtle_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()