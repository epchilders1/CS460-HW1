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
        The goal is to generate a circle, I chose a radius of 4 and angular velocity of pi/2. 
        It is important to maintain this ratio between linear and angular velocity to ensure a perfect circle.

        Here is the breakdown of the circle generation:
        
        First quadrant: Transform radians from 0 to pi/2 with a total linear distance of 4 units.
        Second quadrant: Transform radians from pi/2 to pi with a total linear distance of 4 units.
        Third quadrant: Transform radians from pi to 3pi/2 with a total linear distance of 4 units.
        Fourth quadrant: Transform radians from 3pi/2 to 2pi/0 with a total linear distance of 4 units.

        Finally, stop the turtle indefinitely after completing the circle.
        '''
        if 0 <= self.time < 2:
            msg = self.create_twist(4.0, 1.57079632679)
        elif 2 <= self.time < 4:
            msg = self.create_twist(4.0, 1.57079632679)
        elif 4 <= self.time < 6:
            msg = self.create_twist(4.0, 1.57079632679)
        elif 6 <= self.time < 8:
            msg = self.create_twist(4.0, 1.57079632679)
        else:
            msg = self.create_twist(0.0, 0.0)
        return msg

    
    def timer_callback(self):
        msg = self.get_twist_msg()       
        self.publisher.publish(msg)
        self.time += 1
        print("Generating circle, time: {}".format(self.time))

def main(args=None):
    rclpy.init(args=args)

    turtle_controller = TurtleController()

    rclpy.spin(turtle_controller)

    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()