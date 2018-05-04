#! /usr/bin/env python
import rospy
from multimodal_writer.msg import HandInfoList, HandInfo, FingerInfo
from visualization_msgs.msg import Marker, MarkerArray
import tf
from geometry_msgs.msg import Pose, Vector3, Quaternion, Point
from tf import transformations as transf
from std_msgs.msg import Header, ColorRGBA

br = tf.TransformBroadcaster()


rospy.init_node("leap_viz")
marker_pub = rospy.Publisher("hand_markers",MarkerArray,queue_size=10)

TOPIC_HAND_INFO = "hands_topic"

def leap_callback(msg):

    marker_array = MarkerArray()
    #for each hand in the list
    rospy.loginfo("Frame has %d hands!",len(msg.hands))
    for i in range(0, len(msg.hands)):
        hand = msg.hands[i]
        # publish the transform


        hand_origin = (-hand.pose.position.z/1000,
                                              -hand.pose.position.x/1000,
                                               hand.pose.position.y/1000)
        hand_orientation = (-hand.pose.orientation.x,
                                                         -hand.pose.orientation.y,
                                                         -hand.pose.orientation.z,
                                                          hand.pose.orientation.w)

        br.sendTransform(hand_origin, hand_orientation, hand.header.stamp, "leap", hand.header.frame_id)

        # create a hand marker
        hand_marker =  Marker(type = Marker.CUBE, color=ColorRGBA(0.0,1.0,0.0,0.8))
        hand_marker.header = hand.header
        hand_marker.header.stamp = hand.header.stamp
        hand_marker.ns = "leap"
        hand_marker.id = hand.id
        #hand_marker.action = Marker.ADD
        hand_marker.scale.x = 0.1
        hand_marker.scale.y = 0.07
        hand_marker.scale.z = 0.02
        hand_marker.lifetime = rospy.Duration(0.1)

        marker_array.markers.append(hand_marker)
        # create a marker for the fingers
        lines_marker = Marker()
        lines_marker.header.frame_id = "leap"
        lines_marker.header.stamp = hand.header.stamp
        lines_marker.ns = "leap_lines"
        lines_marker.id = hand.id
        lines_marker.type= Marker.LINE_LIST
        lines_marker.action = Marker.ADD
        lines_marker.scale.x = 0.02
        lines_marker.scale.y = 0.02
        lines_marker.scale.z = 0.02
        lines_marker.color.r = 0.9
        lines_marker.color.g = 0.1
        lines_marker.color.b = 0.1
        lines_marker.color.a = 0.8
        lines_marker.lifetime = rospy.Duration.from_sec(0.1)

        for j in range(0,len(hand.fingers)):
            # publish a transform for the fingertips
            finger = hand.fingers[j]
            finger_transform = Pose()
            tip_origin = (-finger.tip_position.z/1000,
                                                 -finger.tip_position.x/1000,
                                                  finger.tip_position.y/1000)
            tip_orientation = (-hand.pose.orientation.x,
                                                        -hand.pose.orientation.y,
                                                        -hand.pose.orientation.z,
                                                        hand.pose.orientation.w)
            br.sendTransform(tip_origin, tip_orientation, hand.header.stamp, "leap", finger.header.frame_id)

            # create the fingertip marker
            finger_marker = Marker()
            finger_marker.header.frame_id = finger.header.frame_id;
            finger_marker.header.stamp = finger.header.stamp
            finger_marker.ns = hand.header.frame_id
            finger_marker.id = finger.id
            finger_marker.type= Marker.CUBE;
            finger_marker.action = Marker.ADD;
            finger_marker.scale.x = 0.02
            finger_marker.scale.y = 0.02
            finger_marker.scale.z = 0.02
            finger_marker.color.r = 0.7
            finger_marker.color.g = 0.7
            finger_marker.color.b = 0.3
            finger_marker.color.a = 0.8
            finger_marker.lifetime = rospy.Duration.from_sec(0.1)
            marker_array.markers.append(finger_marker)

            hand_point = Point()
            hand_point.x = hand_origin[0]
            hand_point.y = hand_origin[1]
            hand_point.z = hand_origin[2]
            tip_point = Point()
            tip_point.x = tip_origin[0]
            tip_point.y = tip_origin[1]
            tip_point.z = tip_origin[2]
            lines_marker.points.append(hand_point)
            lines_marker.points.append(tip_point)

        marker_array.markers.append(lines_marker)


    # publish the markers
    if(len(marker_array.markers)>0):
        marker_pub.publish(marker_array)





#marker_pub = rospy.Publisher("hand_markers",MarkerArray,queue_size=10)


br = tf.TransformBroadcaster()

leap_sub = rospy.Subscriber("hands_topic", HandInfoList,leap_callback)

rospy.spin()

#if __name__ == '__main__':#
#    main()
