import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Twist

# Initialize the matplotlib figure and axis with a smaller window (0 to 60)
fig, ax = plt.subplots()
ax.set_xlim(0, 60)
ax.set_ylim(0, 60)

# Define the size and initial position of the square (larger)
square_size = 30  # Larger size of the square
x, y = 30, 30  # Initial position in the center of the screen
angle = 0  # Rotation angle in degrees

# Define the direction and color of the square
linear_x = 0.0
linear_y = 0.0
angular_z = 0.0
color = (1, 0, 0)  # Red in normalized RGB

# Function to subscribe to cmd_vel
def cmd_vel_callback(msg):
    global linear_x, linear_y, angular_z, color

    linear_x = msg.linear.x
    linear_y = msg.linear.y
    angular_z = msg.angular.z

    # Calculate color based on the absolute value of the components
    max_value = 18.0
    value = max(abs(linear_x), abs(linear_y), abs(angular_z))

    if value == 0:
        color = (1, 0, 0)  # Red
    elif value <= max_value:
        # Scale between red and blue
        red = max(0, 1 - (value / max_value))
        blue = value / max_value
        green = (red + blue) / 2  # Mix red and blue for green
        color = (red, green, blue)

# Subscribe to /cmd_vel
rospy.init_node('square_movement', anonymous=True)
rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

# Create the square as a matplotlib object
square = plt.Rectangle((x - square_size / 2, y - square_size / 2), square_size, square_size, color=color)
ax.add_patch(square)

# Function to move and rotate the square
def update_square():
    global x, y, angle, linear_x, linear_y, angular_z, color

    # If linear.x, linear.y, and angular.z are 0, reset to initial position and angle
    if linear_x == 0 and linear_y == 0 and angular_z == 0:
        x, y = 30, 30  # Reset to center of the window
        angle = 0  # Reset angle

    # Move the square based on the linear.x and linear.y inputs (with a small scaling factor)
    move_scale = 1  # Scale factor for movement (smaller for smoother motion)
    if linear_x > 0:
        y += move_scale  # Move upward
    elif linear_x < 0:
        y -= move_scale  # Move downward

    if linear_y > 0:
        x += move_scale  # Move left (invert y)
    elif linear_y < 0:
        x -= move_scale  # Move right (invert y)

    # Make sure the square does not go off-screen
    x = max(square_size / 2, min(x, 60 - square_size / 2))
    y = max(square_size / 2, min(y, 60 - square_size / 2))

    # Update the color based on the current values
    square.set_facecolor(color)

    # Update the square’s position
    square.set_xy((x - square_size / 2, y - square_size / 2))

    # If angular.z is 0, reset the rotation angle to 0
    if angular_z == 0:
        angle = 0
    else:
        angle += angular_z * 2  # Control rotation speed

    # Apply rotation around the center of the square
    t = plt.gca().transData
    rotate = plt.matplotlib.transforms.Affine2D().rotate_deg_around(x, y, angle)
    square.set_transform(rotate + t)

# Update loop
while not rospy.is_shutdown():
    update_square()

    # Clear and update the figure
    plt.draw()
    plt.pause(0.05)  # Pause for update

# Close the matplotlib window when finished
plt.close()
