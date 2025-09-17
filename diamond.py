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
        The goal is to generate a diamond, I chose a side lengths of 2 units and angles of pi/3 and 2pi/3. 

        Here is the breakdown of the diamond generation:
        

        First Edge: Transform radians from 0 to pi/3 and travel a linear distance of 2 units.
        Second Edge: Transform radians from pi/3 to 2pi/3 and travel a linear distance of 2 units.
        Third Edge: Transform radians from 2pi/3 to 4pi/3 and travel a linear distance of 2 units.
        Fourth Edge: Transform radians from 4pi/3 to 5pi/3 and travel a linear distance of 2 units.

        Finally, stop the turtle indefinitely after completing the diamond
        '''
        if 0 <= self.time < 2:
            msg = self.create_twist(0.0, 1.0471975512)
        elif 2 <= self.time < 4:
            msg = self.create_twist(2.0, 0.0)
        elif 4 <= self.time < 6:
            msg = self.create_twist(0.0, 1.0471975512)
        elif 6 <= self.time < 8:
            msg = self.create_twist(2.0, 0.0)
        elif 10 <= self.time < 12:
            msg = self.create_twist(0.0, 2.094395102)
        elif 12 <= self.time < 14:
            msg = self.create_twist(2.0, 0.0)
        elif 14 <= self.time < 16:
            msg = self.create_twist(0.0, 1.0471975512)
        elif 16 <= self.time < 18:
            msg = self.create_twist(2.0, 0.0)
        else:
            msg = self.create_twist(0.0, 0.0)
        return msg

    
    def timer_callback(self):
        msg = self.get_twist_msg()       
        self.publisher.publish(msg)
        self.time += 1
        print("Generating diamond, time: {}".format(self.time))

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