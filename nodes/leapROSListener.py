#! /usr/bin/env python
import Leap, sys, thread, time
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture
import PIL
import numpy as np
import rospy
import tf
from tf.transformations import quaternion_from_euler

from std_msgs.msg import String
from multimodal_writer.msg import HandInfoList, HandInfo, FingerInfo, FingersList

TOPIC_HAND_INFO = "hands_topic"

class FrameListener(Leap.Listener):


    global publish_hand
    publish_hand  = rospy.Publisher(TOPIC_HAND_INFO, HandInfoList, queue_size=10)

    def on_init(self, controller):
        publish_hand = rospy.Publisher(TOPIC_HAND_INFO, HandInfoList, queue_size=10)
        print "Initialized"

    def on_connect(self, controller):
        print "Connected"

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def on_frame(self, controller):
        frame = controller.frame()

        if (frame.hands) :
            rospy.loginfo("Frame has %d hands!",len(frame.hands))
            stamp = rospy.get_rostime()
            print type(stamp)

            #We are going to publish all the hand information available
            hand_msg_list = HandInfoList()
            for  i in  range(0,len(frame.hands)):
                hand = frame.hands[i];
                if (hand.confidence>=0.5):
                    rospy.loginfo("Hand %d with confidence %f",i, hand.confidence)
                    hand_msg = HandInfo()
                    # The Leap library reports all positions wrt to the leap motion device position
                    hand_msg.header.frame_id = "hand"+str(i)
                    hand_msg.header.stamp = stamp

                    hand_msg.id = hand.id
                    hand_msg.time_visible = hand.time_visible

                    hand_msg.pose.position.x = hand.palm_position.x
                    hand_msg.pose.position.y = hand.palm_position.y
                    hand_msg.pose.position.z = hand.palm_position.z


                    quat = quaternion_from_euler(hand.palm_normal.roll,hand.direction.pitch,hand.direction.yaw)

                    hand_msg.pose.orientation.x = quat[0]
                    hand_msg.pose.orientation.y = quat[1]
                    hand_msg.pose.orientation.z = quat[2]
                    hand_msg.pose.orientation.w = quat[3]

                    hand_msg.velocity.x = hand.palm_velocity.x
                    hand_msg.velocity.y = hand.palm_velocity.y
                    hand_msg.velocity.z = hand.palm_velocity.z

                    hand_msg.sphere_radius = hand.sphere_radius
                    hand_msg.sphere_center.x = hand.sphere_center.x
                    hand_msg.sphere_center.y = hand.sphere_center.y
                    hand_msg.sphere_center.z = hand.sphere_center.z

                    hand_msg.stabilized_pose.position.x = hand.stabilized_palm_position.x
                    hand_msg.stabilized_pose.position.y = hand.stabilized_palm_position.y
                    hand_msg.stabilized_pose.position.z = hand.stabilized_palm_position.z

                    hand_msg.stabilized_pose.orientation = hand_msg.pose.orientation

                    #finger positions!
                    finger_msg_list = FingersList()
                    #std::vector<leap_client::FingerInfo> finger_msg_list;
                    fingers = hand.fingers;
                    if (len(fingers) > 0):
                        for j in range(0, len(fingers)):
                            finger_msg = FingerInfo()
                            finger_msg.header.frame_id = "finger"+str(j)

                            finger_msg.id = fingers[j].id
                            finger_msg.header.stamp = stamp
                            finger_msg.hand_id = hand_msg.id
                            finger_msg.time_visible = fingers[j].time_visible
                            finger_msg.tip_position.x = fingers[j].tip_position.x
                            finger_msg.tip_position.y = fingers[j].tip_position.y
                            finger_msg.tip_position.z = fingers[j].tip_position.z
                            finger_msg.pointing_direction.x = fingers[j].direction.x
                            finger_msg.pointing_direction.y = fingers[j].direction.y
                            finger_msg.pointing_direction.z = fingers[j].direction.z

                            finger_msg_list.fingers.append(finger_msg)

                        hand_msg.fingers = finger_msg_list.fingers
                        #publish the hand message
                        hand_msg_list.hands.append(hand_msg)
                msg = HandInfoList()
                msg.hands = hand_msg_list.hands
            publish_hand.publish(msg)


if __name__ == '__main__':
	# init node
    rospy.init_node("leap_client")

    mylistener = FrameListener()
    controller = Leap.Controller()
    #controller.set_policy_flags(Leap.Controller.POLICY_BACKGROUND_FRAMES)
    controller.add_listener(mylistener)
    while True:
        rospy.spin()
