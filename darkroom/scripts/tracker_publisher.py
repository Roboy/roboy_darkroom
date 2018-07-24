import triad_openvr
import time
import sys
import rospy
import tf
import numpy as np
from pyquaternion import Quaternion
import math
from roboy_communication_middleware.srv import FrameIK

br = tf.TransformBroadcaster()
li = tf.TransformListener()

rospy.init_node('tracker_1_tf_broadcaster')

v = triad_openvr.triad_openvr()
v.print_discovered_objects()

interval = 1/10

initial_pose = v.devices["tracker_1"].get_pose_quaternion()
q_init = Quaternion(initial_pose[6],initial_pose[3],initial_pose[4],initial_pose[5])
q_init_wrist = Quaternion()

print("waiting for roboy/middleware/FrameIK service to become available")
rospy.wait_for_service('roboy/middleware/FrameIK')
print("let's go then")

import sys, select

try:
    (trans,rot) = li.lookupTransform('/world', '/wrist_left', rospy.Time(0))
    initial_pose[0] = initial_pose[0]-trans[0]
    initial_pose[1] = initial_pose[1]-trans[2]
    initial_pose[2] = initial_pose[2]-trans[1]
    q_init_wrist = Quaternion(rot[3],rot[0],rot[1],rot[2])


except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    print("using initial_pose instead")

while not rospy.is_shutdown():
    start = time.time()
    txt = ""
    pose = v.devices["tracker_1"].get_pose_quaternion()

    q_current = Quaternion(pose[6],pose[3],pose[4],pose[5])

    q = q_current*q_init.inverse

    pos = np.array([pose[0]-initial_pose[0],pose[1]-initial_pose[1],pose[2]-initial_pose[2]])
    rotX = np.array([[1,0,0], [ 0, 0 ,1], [ 0, -1 ,0]])
    pos = rotX.dot(pos)
    # print(pos)

    br.sendTransform([pos[0],pos[1],-pos[2]],
                     (q[1],q[2],q[3],q[0]),
                     rospy.Time.now(),
                     "tracker_1",
                     "world")
    # i, o, e = select.select( [sys.stdin], [], [], 0.01 )
    # if (i):
    #     sys.stdin.readline().strip()
    try:
        key_srv = rospy.ServiceProxy('roboy/middleware/FrameIK', FrameIK)
        resp1 = key_srv(ik_type=0, frame_id='tracker_1')
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e