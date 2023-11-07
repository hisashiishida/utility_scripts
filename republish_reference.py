# Created on: 2023-11-02

# Copyright (c) 2015-2023 Johns Hopkins University
# Released under MIT License
# Author: Hisashi Ishida

import rospy 
import PyKDL
import click

from geometry_msgs.msg import PoseStamped
import rostopic

def FrameFromPoseMsg(p):
    """
    :param p: input pose
    :type p: :class:`geometry_msgs.msg.Pose`
    :return: New :class:`PyKDL.Frame` object

    Convert a pose represented as a ROS Pose message to a :class:`PyKDL.Frame`.
    There must be a standard package to perform this conversion, if you find it, please remove this code.
    """
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(p.orientation.x,
                                                 p.orientation.y,
                                                 p.orientation.z,
                                                 p.orientation.w),
                       PyKDL.Vector(p.position.x,
                                    p.position.y,
                                    p.position.z))

def FrameToPoseMsg(f):
    """
    :param f: input pose
    :type f: :class:`PyKDL.Frame`

    Return a ROS PoseStamped message for the Frame f.
    There must be a standard package to perform this conversion, if you find it, please remove this code.
    """
    p = PoseStamped()
    p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = f.M.GetQuaternion()
    p.pose.position.x = f.p[0]
    p.pose.position.y = f.p[1]
    p.pose.position.z = f.p[2]
    return p

class optical_tracker_crtk:
    def __init__(self, ns_old, ns_new, topic_names, ref_name):
        ## Create dictionaries for the data
        self.topic_names = topic_names
        self.ref_name = ref_name
        self.data = {}
        self.data_new = {}
        self.is_new = False
        self.subs = []
        self.pubs = []

        # Initialize the dictionaries
        for topic in topic_names:
            self.data[topic] = PyKDL.Frame(PyKDL.Rotation.Quaternion(0,0,0,1),PyKDL.Vector(0,0,0))
            self.data_new[topic] = PyKDL.Frame(PyKDL.Rotation.Quaternion(0,0,0,1),PyKDL.Vector(0,0,0))
        self.data[self.ref_name] = PyKDL.Frame(PyKDL.Rotation.Quaternion(0,0,0,1),PyKDL.Vector(0,0,0))
        self.data_new[self.ref_name] = PyKDL.Frame(PyKDL.Rotation.Quaternion(0,0,0,1),PyKDL.Vector(0,0,0))
        
        # print(self.data)

        # Create rostopic publisher and subscriber
        for topic in topic_names:
            sub = rospy.Subscriber(ns_old + "/" + topic + "/measured_cp", PoseStamped, self.measured_cp_cb, topic)
            pub = rospy.Publisher(ns_new + "/" + topic + "/measured_cp", PoseStamped, queue_size=1)
            self.subs.append(sub)
            self.pubs.append(pub)
        
        # Create rostopic publisher and subscriber for reference
        sub = rospy.Subscriber(ns_old + "/" + ref_name + "/measured_cp", PoseStamped, self.measured_cp_cb, ref_name)
        pub = rospy.Publisher(ns_new + "/" + ref_name + "/measured_cp", PoseStamped, queue_size=1)
        self.subs.append(sub)
        self.pubs.append(pub)

    def measured_cp_cb(self, data, topic):
        self.data[topic] = FrameFromPoseMsg(data.pose)
        self.data_new[topic] = PyKDL.Frame(PyKDL.Rotation.Quaternion(0,0,0,1),PyKDL.Vector(0,0,0))
        self.is_new = True
    
    def change_reference(self):
        if self.is_new:
            for idx, topic in enumerate(self.topic_names):
                self.data_new[topic] = self.data[self.ref_name].Inverse() *  self.data[topic]
                self.pubs[idx].publish(FrameToPoseMsg(self.data_new[topic]))
            
            self.data_new[self.ref_name] = self.data[self.ref_name].Inverse() *  self.data[self.ref_name]
            self.pubs[-1].publish(FrameToPoseMsg(self.data_new[self.ref_name]))
            
            self.is_new = False
                
@click.command()
@click.option('--ns1', help='old namespace')
@click.option('--ns2', help='new namespace')
@click.option('--topics', '-t', multiple=True)
@click.option('--ref', default='reference', help='name of rostopic for reference.')
@click.option('--hz', default= 200, help='refreshing rate')
def main(ns1, ns2, topics, ref, hz):

    print('----------- Input Details ------------')
    print("old namespaces:", ns1)
    print("new namespaces:", ns2)
    print('topics:', topics)
    print('reference:', ref)
    print('--------------------------------------')

    OT = optical_tracker_crtk(ns1, ns2, topics, ref)

    rospy.init_node('node_name')
    r = rospy.Rate(hz)
    while not rospy.is_shutdown():
        OT.change_reference()
        r.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 