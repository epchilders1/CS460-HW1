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
        The goal is to generate a cool S. This multiple lines and turns of varying lengths and angles.

        Here is the breakdown of the cool S generation:
        

        1. (0–2s): Rotate at -pi/2 radians.
        2. (2–4s): Move forward 1 unit.
        3. (4–6s): Rotate at +pi/4 radians.
        4. (6–8s): Move forward 1 unit.
        5. (10–12s): Rotate at +pi/2 radians.
        6. (12–14s): Move forward 1 unit.
        7. (14–16s): Rotate at +pi/4 radians.
        8. (16–18s): Move forward 1 unit.
        9. (18–20s): Rotate at +pi/4 radians.
        10. (20–22s): Move forward 2 units.
        11. (22–24s): Rotate at +pi/2 radians.
        12. (24–26s): Move forward 2 units.
        13. (26–28s): Rotate at +pi/4 radians.
        14. (28–30s): Move forward 1 unit.
        15. (30–32s): Rotate at +pi/4 radians.
        16. (32–34s): Move forward 2 units.
        17. (34–36s): Rotate at -pi/4 radians.
        18. (36–38s): Move forward 1 unit.
        19. (38–40s): Rotate at pi radians.
        20. (40–42s): Move forward 1 unit.
        21. (42–44s): Rotate at +pi/4 radians.
        22. (44–46s): Move forward 1 unit.
        23. (46–48s): Rotate at +pi/2 radians.
        24. (48–50s): Move forward 1 unit.
        25. (50–52s): Rotate at +pi/4 radians.
        26. (52–54s): Move forward 1 unit.
        27. (54–56s): Rotate at +pi/4 radians.
        28. (56–58s): Move forward 2 units.
        29. (58–60s): Rotate at +pi/2 radians.
        30. (60–62s): Move forward 2 units.
        31. (62–64s): Rotate at +pi/4 radians.
        32. (64–66s): Move forward 1 unit.
        33. (66–68s): Rotate at +pi/4 radians.
        34. (68–70s): Move forward 2 units.
        
        Finally, stop the turtle indefinitely after completing the cool S.
        '''
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
        print("Generating cool S, time: {}".format(self.time))

def main(args=None):
    rclpy.init(args=args)

    turtle_controller = TurtleController()

    rclpy.spin(turtle_controller)

    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()