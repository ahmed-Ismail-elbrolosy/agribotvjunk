import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

msg = """
Control Your AgriBot!
---------------------------
Moving around:
        w
   a    s    d

w : move forward
s : move backward
a : rotate left
d : rotate right

Speed Control:
t : increase linear speed (+0.5)
y : decrease linear speed (-0.5)
g : increase angular speed (+0.5)
h : decrease angular speed (-0.5)

space key, k : force stop
CTRL-C to quit
"""

# (x, y, z, th)
moveBindings = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
}

# (linear_step, angular_step)
speedBindings = {
    't': (0.5, 0.0),
    'y': (-0.5, 0.0),
    'g': (0.0, 0.5),
    'h': (0.0, -0.5),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('teleop_node')
    
    # QoS 10 matches standard defaults
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    settings = termios.tcgetattr(sys.stdin)

    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0

    try:
        print(msg)
        print(f"Current Speed: {speed} | Current Turn: {turn}")
        
        while True:
            key = getKey(settings)
            
            # MOVEMENT KEYS
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
                
            # SPEED ADJUSTMENT KEYS
            elif key in speedBindings.keys():
                speed += speedBindings[key][0]
                turn += speedBindings[key][1]

                # Prevent 0 or negative speed mess-ups (optional, keep at least 0.1)
                if speed < 0.1: speed = 0.1
                if turn < 0.1: turn = 0.1
                
                print(f"Speed updated! Linear: {speed} | Angular: {turn}")
                
                # Don't move while changing speed (safety)
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0

            # STOP/QUIT
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'): # CTRL-C
                    break

            # Publish the message
            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            publisher.publish(twist)

    except Exception as e:
        print(e)

    finally:
        # Stop robot on exit
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        publisher.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()