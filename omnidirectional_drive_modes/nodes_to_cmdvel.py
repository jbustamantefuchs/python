import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import re

# Direction mapping
direction_map = {
    1: (1.0, 1.0), 2: (1.0, -1.0), 3: (-1.0, 1.0), 4: (-1.0, -1.0),
    0: (0.0, 0.0), 8: (1.0, 0.0), 7: (-1.0, 0.0), 5: (0.0, 1.0),
    6: (0.0, -1.0), 9: (0.0, 0.0), 10: (0.0, 0.0)
}

def callback(data):
    twist = Twist()

    # We use a regular expression to extract the numbers inside the parentheses
    match = re.match(r'\((\d+),(\d+)\)', data.data)
    if match:
        direction = int(match.group(1))  # Direction
        speed = int(match.group(2))      # Speed
    else:
        rospy.logwarn(f"Incorrect message format: {data.data}")
        return

    linear_x, linear_y = direction_map.get(direction, (0.0, 0.0))
    twist.linear.x = linear_x * (speed / 10)
    twist.linear.y = linear_y * (speed / 10)
    twist.angular.z = -speed * 0.1 if direction == 9 else speed * 0.1 if direction == 10 else 0.0
    cmd_vel_pub.publish(twist)

# Initialize the node and subscribers
rospy.init_node('direction_speed_to_cmd_vel')
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.Subscriber('/direction_n_speed', String, callback)
rospy.Subscriber('/nr_controller', String, callback)
rospy.spin()
