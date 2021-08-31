#!/usr/bin/env python
import rospy
import cv2
import time
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from move_robot import MovePhantomX

class LineFollower(object):

    def __init__(self):

        # Instantiate variables needed for robot line following
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.objective_sub = rospy.Subscriber("/objective", String, self.listen_for_color)
        self.direction_sub = rospy.Subscriber("/green_line_direction", String, self.listen_for_direction)
        self.move_phantom_x_object = MovePhantomX()
        # Default to None
        self.dest_color = None
        self.lower_color = np.array([0, 0, 0])
        self.upper_color = np.array([0, 0, 0])
        # Default to green_right
        self.line_direction = "green_right"

    def listen_for_color(self, star_color):

        # Set upper and lower HSV limits for the specified color of the destination-star
        if star_color.data.lower() == "blue":
            self.dest_color = "Blue"
            self.lower_color = np.array([104, 200, 0])
            self.upper_color = np.array([111, 255, 255])
        elif star_color.data.lower() == "green":
            self.dest_color = "Yellow"
            self.lower_color = np.array([30, 215, 0])
            self.upper_color = np.array([53, 255, 255])
        elif star_color.data.lower() == "red":
            self.dest_color = "Red"
            self.lower_color = np.array([0, 128, 0])
            self.upper_color = np.array([5, 255, 255])
        else: # Default to None
            self.dest_color = None
            self.lower_color = np.array([0, 0, 0])
            self.upper_color = np.array([0, 0, 0])

        if self.dest_color:
            rospy.loginfo("Color destination has been set: " + self.dest_color + ". Will be stopping at this color")

    def listen_for_direction(self, line_direction):

        # Set turn direction for following green line
        if line_direction.data.lower() == "green_left":
            self.line_direction = "green_left"
        else: # Default to green_right
            self.dest_color = "green_right"
            self.lower_color = np.array([0, 0, 0])
            self.upper_color = np.array([0, 0, 0])

        rospy.loginfo("Line direction has been set: " + self.line_direction)

    def camera_callback(self, data):
        
        try:
            # Select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # Get image dimensions
        height, width, channels = cv_image.shape
        height = int(height)
        width = int(width)
        channels = int(channels)
        descentre = 160
        rows_to_watch = 20
        # Set crop values and crop image
        aux1 = int(((height)/2)+descentre)
        aux2 = int((height)/2+(descentre+rows_to_watch))
        crop_img = cv_image[aux1:aux2][1:width]
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        hsv_full_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Get HSV limits for green line
        lower_green = np.array([55, 120, 200])
        upper_green = np.array([95, 255, 255])
        # Threshold the HSV image to get only greem colors
        mask = cv2.inRange(hsv, lower_green, upper_green)
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
        # Grab centroids
        contours, _, = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
        centres = []
        for i in range(len(contours)):
            moments = cv2.moments(contours[i])
            try:
                centres.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
                cv2.circle(res, centres[-1], 10, (0, 255, 0), -1)
            except ZeroDivisionError:
                pass

        # Detect if specified color of destination star has been detected
        if self.dest_color:
            # For visualization purposes, detect star both in the distance and immediately in front of robot
            mask_dest = cv2.inRange(hsv, self.lower_color, self.upper_color)
            mask_detection = cv2.inRange(hsv_full_image, self.lower_color, self.upper_color)
            res_dest = cv2.bitwise_and(crop_img, crop_img, mask = mask_dest)
            res_detection = cv2.bitwise_and(cv_image, cv_image, mask = mask_detection)
            m_dest = cv2.moments(mask_dest, False)
            m_detection = cv2.moments(mask_detection, False)
            try:
                cx_dest, cy_dest = m_dest['m10']/m_dest['m00'], m_dest['m01']/m_dest['m00']
            except ZeroDivisionError:
                cx_dest, cy_dest = None, None

            # Draw the centroid in the resulting image
            if (cx_dest and cy_dest):
                cv2.circle(res_dest,(int(cx_dest), int(cy_dest)), 5,(0,0,255),-1)
            # Display detection images
            cv2.imshow("RES_DEST", res_dest)
            cv2.imshow("RES_DETECTION", res_detection)

        # Follow right or left most centroid depending on specified direction
        centroid_index = 0
        index = 0
        if self.line_direction == "green_right":
            max_x_value = 0
            for candidate in centres:
                # Retrieve the cx value
                cx = candidate[0]
                # Get the Cx more to the right
                if cx >= max_x_value:
                    max_x_value = cx
                    centroid_index = index
                index += 1
        else:
            min_x_value = width
            for candidate in centres:
                # Retrieve the cx value
                cx = candidate[0]
                # Get the Cx more to the left
                if cx <= min_x_value:
                    min_x_value = cx
                    centroid_index = index
                index += 1
        try:
            cx = centres[centroid_index][0]
            cy = centres[centroid_index][1]
        except:
            cy, cx = height/2, width/2

        # Draw the centroid in the resulting image
        cv2.circle(res,(int(cx), int(cy)), 5,(0,0,255),-1)
        # Display images
        cv2.imshow("Original", cv_image)
        cv2.imshow("RES", res)
        cv2.waitKey(1)

        twist_object = Twist()
        if self.dest_color and (cx_dest and cy_dest):
            # We have reached destination. Stop moving
            time.sleep(15)
            twist_object.linear.x = 0
            twist_object.angular.z = 0
            rospy.loginfo(f"Reached destination color: " + self.dest_color + ". Stopping robot")
            self.clean_up()
            rospy.signal_shutdown("Reached destination")
        else:
            # Keep moving robot
            error_x = cx - width / 2
            twist_object = Twist()
            twist_object.linear.x = 0.4
            twist_object.angular.z = -error_x / 400

        # Make it start turning
        self.move_phantom_x_object.move_robot(twist_object)

    # Close all windows and clean move robot object
    def clean_up(self):
        self.move_phantom_x_object.clean_class()
        cv2.destroyAllWindows()
        


def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        rate.sleep()

if __name__ == '__main__':
    main()
