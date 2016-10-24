#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
from intro_to_robotics.image_converter import ToOpenCV, depthToOpenCV

def process_image(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_bound = np.array([0, 10, 10])
    upper_bound = np.array([10,255,255])
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
    M = cv2.moments(mask)
    location = None
    magnitude = 0
    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        magnitude = M['m00']
        location = (cx-320, cy-240) #scale so that 0,0 is center of screen
        #draw a circle image where we detected the midpoint of the object
        cv2.circle(image, (cx,cy), 3, (0,0,255), -1)

    cv2.imshow("processing result", image)
    cv2.imshow("image_mask", mask)

    #waitKey() is necessary for making all the cv2.imshow() commands work
    cv2.waitKey(1)
    return location, magnitude


class Node:
    def __init__(self):
        #register a subscriber callback that receives images
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=1)
        self.state = "SEARCHING"

        self.movement_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def image_callback(self, ros_image):
        # convert the ros image to a format openCV can use
        cv_image = np.asarray(ToOpenCV(ros_image))

        #run our vision processing algorithm to pick out the object
        #returns the location (x,y) of the object on the screen, and the
        #magnitude of the discovered object. Magnitude can be used to estimate
        #distance
        location, magnitude = process_image(cv_image)

        #log the processing results
        rospy.logdebug("image location: {}\tmagnitude: {}".format(location, magnitude))

        ###########
        # Insert turtlebot controlling logic here!
        ###########
        cmd = Twist()

        #here we have a little state machine for controlling the turtlebot
        #possible states:
        #TRACKING: we are currently moving towards an object
        #print the current state
        rospy.loginfo("state: {}".format(self.state))

        if self.state == "TRACKING":
            #check if we can't see an object
            if(location is None):
                #if we can't see an object, then we should search for one
                self.state = "SEARCHING"
                return
            #else...

            #go forward
            cmd.linear.x = 0.3

            #this is a basic proportional controller, where the magnitude of
            #rotation is based on how far away the object is from center
            #we apply a 10px hysteresis for smoothing
            if(location[0] > 10):
                cmd.angular.z = -0.002 * location[0]
            elif(location[0] < 10):
                cmd.angular.z = -0.002 * location[0]

            #check if we are close to the object
            if magnitude > 21000000:
                #calculate a time 3 seconds into the future
                #we will rotate for this period of time
                self.rotate_expire = rospy.Time.now() + rospy.Duration.from_sec(3)
                #set state to rotating
                self.state = "ROTATING_AWAY"

        elif self.state == "ROTATING_AWAY":
            #check if we are done rotating
            if rospy.Time.now() < self.rotate_expire:
                cmd.angular.z = -0.5
            else: #after we have rotated away, search for new target
                self.state = "SEARCHING"

        elif self.state == "SEARCHING":
            #here we just spin  until we see an object
            cmd.angular.z = -0.5
            if location is not None:
                #when we see an object, start tracking it!
                self.state = "TRACKING"

        #this state is currently unused, but we could use it for exiting
        elif self.state == "STOP":
            rospy.signal_shutdown("done tracking, time to exit!")





        #publish command to the turtlebot
        self.movement_pub.publish(cmd)


if __name__ == "__main__":
    rospy.init_node("turtlebot_vision_controller")
    node = Node()

    #this function loops and returns when the node shuts down
    #all logic should occur in the callback function
    rospy.spin()
