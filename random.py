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
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    def get_twist_msg(self):
        if 0 <= self.time < 2:
            msg = self.create_twist(0.0, -1.57079632679)
        elif 2 <= self.time < 4:
            msg = self.create_twist(1.0, 0.0)
        elif 4 <= self.time < 6:
            msg = self.create_twist(0.0, .7853981634)
        elif 6 <= self.time < 8:
            msg = self.create_twist(1.0, 0.0)
        elif 10 <= self.time < 12:
            msg = self.create_twist(0.0, 1.57079632679)
        elif 12 <= self.time < 14:
            msg = self.create_twist(1.0, 0.0)
        elif 14 <= self.time < 16:
            msg = self.create_twist(0.0, .7853981634)
        elif 16 <= self.time < 18:
            msg = self.create_twist(1.0, 0.0)
        elif 18 <= self.time < 20:
            msg = self.create_twist(0.0, .7853981634)
        elif 20 <= self.time < 22:
            msg = self.create_twist(2.0, 0.0)
        elif 22 <= self.time < 24:
            msg = self.create_twist(0.0, 1.57079632679)
        elif 24 <= self.time < 26:
            msg = self.create_twist(2.0, 0.0)
        elif 26 <= self.time < 28:
            msg = self.create_twist(0.0, .7853981634)
        elif 28 <= self.time < 30:
            msg = self.create_twist(1.0, 0.0)
        elif 30 <= self.time < 32:
            msg = self.create_twist(0.0, .7853981634)
        elif 32 <= self.time < 34:
            msg = self.create_twist(2.0, 0.0)
        elif 34 <= self.time < 36:
            msg = self.create_twist(0.0, -.7853981634)
        elif 36 <= self.time < 38:
            msg = self.create_twist(1.0, 0.0)
        elif 38 <= self.time < 40:
            msg = self.create_twist(0.0, 3.1415926535)
        elif 40 <= self.time < 42:
            msg = self.create_twist(1.0, 0.0)    
        elif 42 <= self.time < 44:
            msg = self.create_twist(0.0, .7853981634)  
        elif 44 <= self.time < 46:
            msg = self.create_twist(1.0, 0.0)   
        elif 46 <= self.time < 48:
            msg = self.create_twist(0.0, 1.57079632679)
        elif 48 <= self.time < 50:
            msg = self.create_twist(1.0, 0.0)  
        elif 50 <= self.time < 52:
            msg = self.create_twist(0.0, .7853981634)  
        elif 52 <= self.time < 54:
            msg = self.create_twist(1.0, 0.0)  
        elif 54 <= self.time < 56:
            msg = self.create_twist(0.0, .7853981634)  
        elif 56 <= self.time < 58:
            msg = self.create_twist(2.0, 0.0)  
        elif 58 <= self.time < 60:
            msg = self.create_twist(0.0, 1.57079632679)
        elif 60 <= self.time < 62:
            msg = self.create_twist(2.0, 0.0)  
        elif 62 <= self.time < 64:
            msg = self.create_twist(0.0, .7853981634)  
        elif 64 <= self.time < 66:
            msg = self.create_twist(1.0, 0.0) 
        elif 66 <= self.time < 68:
            msg = self.create_twist(0.0, .7853981634)  
        elif 68 <= self.time < 70:
            msg = self.create_twist(2.0, 0.0)  

        else:
            msg = self.create_twist(0.0, 0.0)
        return msg

    
    def timer_callback(self):
        msg = self.get_twist_msg()       
        self.publisher.publish(msg)
        self.time += 1
        print("time: {}".format(self.time))

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