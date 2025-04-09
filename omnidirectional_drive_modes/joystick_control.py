#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import pygame


def check_axis(value):
    if value > 0.2:
        return 1
    elif value < -0.2:
        return -1
    else:
        return 0

def main():
    rospy.init_node('joystick_controller', anonymous=True)
    pub = rospy.Publisher('direction_n_speed', String, queue_size=10)
    pygame.init()
    pygame.joystick.init()


    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        rospy.logerr("No command has been detected.")
        return


    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    rospy.loginfo("Controller detected: %s", joystick.get_name())

    speed = 0
    previous_direction = 0
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            pygame.event.pump()
            axis_0 = joystick.get_axis(0)  # Horizontal left axis
            axis_1 = joystick.get_axis(1)  # Vertical left axis
            axis_2 = joystick.get_axis(2)  # Horizontal right axis
            button_7 = joystick.get_button(7)
            button_6 = joystick.get_button(6)

            if button_7:
                speed += 2
                rospy.loginfo("Speed increase: %d", speed)
            elif button_6:
                speed = max(0, speed - 2)
                rospy.loginfo("Speed decrease: %d", speed)

            if axis_0 > 0.5 and axis_1 > 0.5:
                direction = 1  # Diagonal Right Forward
            elif axis_0 > 0.5 and axis_1 < -0.5:
                direction = 2  # Diagonal Right Backward
            elif axis_0 < -0.5 and axis_1 > 0.5:
                direction = 3  # Diagonal Left Forward
            elif axis_0 < -0.5 and axis_1 < -0.5:
                direction = 4  # Diagonal Left Backward
            elif axis_0 > 0.5:
                direction = 5  # Right
            elif axis_0 < -0.5:
                direction = 6  # Left
            elif axis_1 > 0.5:
                direction = 7  # Backward
            elif axis_1 < -0.5:
                direction = 8  # Forward
            elif axis_2 > 0.5:
                direction = 9  # Right Turn
            elif axis_2 < -0.5:
                direction = 10  # Left Turn
            else:
                direction = 0  # STOP


            if direction != previous_direction:
                message = f"({direction},{speed})"
                pub.publish(message)
                previous_direction = direction
                rospy.loginfo("Sent: %s", message)

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Leaving the program...")

    finally:
        pygame.quit()

if __name__ == '__main__':
    main()

