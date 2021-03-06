#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


class MovePhantomX(object):

    def __init__(self):
        # Instantiate class variables
        self.cmd_vel_pub = rospy.Publisher('/phantomx/cmd_vel', Twist, queue_size=10)
        self.last_cmdvel_command = Twist()
        self._cmdvel_pub_rate = rospy.Rate(10)
        self.shutdown_detected = False

    def move_robot(self, twist_object):
        # Publish twist object
        self.cmd_vel_pub.publish(twist_object)
                                    
    def clean_class(self):
        # Stop Robot
        twist_object = Twist()
        twist_object.angular.z = 0.0
        self.move_robot(twist_object)
        self.shutdown_detected = True

def main():
    # Instantiate node and create MovePhantomX object
    rospy.init_node('move_robot_node', anonymous=True)
    move_phantom_x_object = MovePhantomX()
    twist_object = Twist()
    # Make it start turning
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        move_phantom_x_object.clean_class()
        rospy.loginfo("shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        move_phantom_x_object.move_robot(twist_object)
        rate.sleep()

if __name__ == '__main__':
    main()