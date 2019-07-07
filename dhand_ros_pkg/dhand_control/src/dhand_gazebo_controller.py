#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64, Int16

FINGER_TOP_MIDDLE_TOPIC = "/dhand/joint_finger_top_middle_position_controller/command"
FINGER_TOP_LEFT_TOPIC   = "/dhand/joint_finger_top_left_position_controller/command"
FINGER_TOP_RIGHT_TOPIC  = "/dhand/joint_finger_top_right_position_controller/command"

DHAND_CMD_TOPIC = "/cmd"

def dhand_cmd_cb(cmd_msg):
    global finger_top_middle_pub, finger_top_left_pub, finger_top_right_pub

    cmd_id = cmd_msg.data
    angle_msg = Float64()

    if cmd_id == 0:    
        angle_msg.data = 0
    else:
        angle_msg.data = 0.3

    finger_top_middle_pub.publish(angle_msg)
    finger_top_left_pub.publish(angle_msg)
    finger_top_right_pub.publish(angle_msg)

def dhand_gazebo_controller():
    global finger_top_middle_pub, finger_top_left_pub, finger_top_right_pub
    finger_top_middle_pub = rospy.Publisher(FINGER_TOP_MIDDLE_TOPIC, Float64, queue_size=10)
    finger_top_left_pub = rospy.Publisher(FINGER_TOP_LEFT_TOPIC, Float64, queue_size=10)
    finger_top_right_pub = rospy.Publisher(FINGER_TOP_RIGHT_TOPIC, Float64, queue_size=10)

    rospy.init_node('dhand_gazebo_controller', anonymous=True)
    rospy.Subscriber(DHAND_CMD_TOPIC, Int16, dhand_cmd_cb)
    rospy.spin()


if __name__ == '__main__':
    try:
        dhand_gazebo_controller()
    except rospy.ROSInterruptException:
        pass