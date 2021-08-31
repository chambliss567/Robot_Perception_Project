#!/usr/bin/env python
import sys
import os
import rospy
from sensor_msgs.msg import Image
from people_msgs.msg import PositionMeasurementArray
import rospkg

try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except Exception as e:
    print(e)
    print("No path needed to be deleted")

print("Importing cv2")
import cv2
print("Importing cv2..Done")
import face_recognition
from cv_bridge import CvBridge, CvBridgeError

class FaceRecogniser(object):

    def __init__(self):
        rospy.loginfo("Start FaceRecogniser Init process...")
        # Get an instance of RosPack with the default search paths
        self.rate = rospy.Rate(5)
        rospack = rospkg.RosPack()
        # Get the file path for my_face_recogniser
        self.path_to_package = rospack.get_path('my_hexapod_perception')
        self.bridge_object = CvBridge()
        rospy.loginfo("Start camera suscriber...")
        self.cam_topic = "/camera/rgb/image_raw"
        self._check_cam_ready()
        self.image_sub = rospy.Subscriber(self.cam_topic,Image,self.camera_callback)
        # Face detector subscriber to check if any faces have been detected
        self.face_detector_client =  rospy.Subscriber("/face_detector/people_tracker_measurements_array",
                                                    PositionMeasurementArray,
                                                    self.face_detect_subs_callback)
        self.detected_faces = None
        rospy.loginfo("Finished FaceRecogniser Init process...Ready")

    def face_detect_subs_callback(self, msg):
        # Save position measurement of detected face
        self.detected_faces = msg

    def _check_cam_ready(self):
      self.cam_image = None
      while self.cam_image is None and not rospy.is_shutdown():
         try:
               self.cam_image = rospy.wait_for_message(self.cam_topic, Image, timeout=1.0)
               rospy.logdebug("Current "+self.cam_topic+" READY=>" + str(self.cam_image))

         except:
               rospy.logerr("Current "+self.cam_topic+" not ready yet, retrying.")

    def camera_callback(self, data):
        self.cam_image = data

    def loop(self):

        while not rospy.is_shutdown():
            # Only start if face has been detected
            if self.detected_faces:
                self.recognise(self.cam_image)
                self.rate.sleep()

    def recognise(self ,data):

        # Get a reference to webcam #0 (the default one)
        try:
            # Select bgr8 because its the OpneCV encoding by default
            video_capture = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # Load the image of the girl
        image_path = os.path.join(self.path_to_package,"images/olive_hex.png")
        olive_image = face_recognition.load_image_file(image_path)
        olive_face_encoding = face_recognition.face_encodings(olive_image)[0]
        # Initialize some variables
        face_locations = []
        face_encodings = []
        face_names = []

        # Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(video_capture)
        face_encodings = face_recognition.face_encodings(video_capture, face_locations)

        if not face_encodings:
            rospy.logwarn("No Faces found, please get closer...")

        face_names = []
        for face_encoding in face_encodings:
            # See if the face is a match for the known face
            match_olive = face_recognition.compare_faces([olive_face_encoding], face_encoding)
            name = "Unknown"

            if match_olive[0]:
                rospy.loginfo("MATCH - Olive")
                name = "Olive"
            else:
                rospy.logwarn("NO Match")

            face_names.append(name)
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            # Draw a box around the face
            cv2.rectangle(video_capture, (left, top), (right, bottom), (0, 0, 255), 2)
            # Draw a label with a name below the face
            cv2.rectangle(video_capture, (left, bottom - 35), (right, bottom), (0, 0, 255))
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(video_capture, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        # Display the resulting image
        cv2.imshow("Image window", video_capture)
        cv2.waitKey(1)




def main():
    rospy.init_node('face_recognising_python_node', anonymous=True, log_level=rospy.INFO)

    face_recogniser_object = FaceRecogniser()

    face_recogniser_object.loop()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()