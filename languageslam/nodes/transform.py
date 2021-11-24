#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('languageslam')
from cartographer_msgs_ros import SubmapList, SubmapEntry

import tf2_ros
import geometry_msgs.msg
import system as sys



'''
BRIEF:  
        This node creates tf transfroms based on the first submap of a trajectory in a submaplist
        It is used to bring the individual robot/map frames into one final frame so 
        their own position allignes with the globally optimized map

        This will publish a new transform every time a new submap_list is received so they will 
        stay updated as the global optimization improves its guess
'''


header_frame,child_frame=''
trajectory_id=0


def callback(submap):
    pose=submap[trajectory_id].pose
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = header_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = pose.position.orientation.x
    t.transform.rotation.y = pose.position.orientation.y
    t.transform.rotation.z = pose.position.orientation.z
    t.transform.rotation.w = pose.position.orientation.w

    br.sendTransform(t)


if __name__=='__main__':
    rospy.init_node('tf_transformer', anonymous=True)
    args=rospy.myargv(argv=sys.argv)
    if len(args) < 3:
        print("usage: my_node.py arg1 arg2")
    else:
        header_frame=args[1]
        child_frame=args[2]
        trajectory_id=args[3]
        sub = rospy.Subscriber('/submap_list', SubmapList, callback)
    rospy.spin()