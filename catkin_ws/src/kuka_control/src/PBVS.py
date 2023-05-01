#!/usr/bin/env python3

'''
    File name: PBVS.py
    Author: Lucca Leão
    Date created: 28/12/2022
'''

import sys
import time
import rospy
import tf
import numpy as np
import statistics
import PyKDL as kdl 
import kdl_parser_py.urdf as kdl_parser
#import kdl_parser_py.urdf as kdl_parser

from scipy.integrate import solve_ivp
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint
)

class PBVS(object):
    def __init__(self):
        rospy.Subscriber('/joint_states', JointState, self.update_joints)
        rospy.Subscriber('/aruco_single/pose', PoseStamped, self.setMarkerFound)
        self.pub = rospy.Publisher('/position_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.rate = rospy.Rate(10)
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        flag, self.tree = kdl_parser.treeFromParam('/robot_description')
        self.joint_names = rospy.get_param('/controller_joint_names')
        self.chain = self.tree.getChain('base_link','link_6')
        self.fkine = kdl.ChainFkSolverPos_recursive(self.chain)
        self.solverPos = kdl.ChainIkSolverPos_LMA(self.chain)
        self.num_joints = self.chain.getNrOfJoints()
        self.joints = kdl.JntArray(self.num_joints)
        #self.cameraTf = kdl.Frame(kdl.Rotation.EulerZYX(0,0,0), kdl.Vector(0.159378,0.00531843,-0.0202029))
        self.cameraTf = kdl.Frame(kdl.Rotation.EulerZYX(0,0,0), kdl.Vector(0.00531843,-0.0202029,0.159378))
        #print(self.joint_names)
        #print(self.joints)
        self.goals = [[0,0,0.15,np.pi/2,np.pi/2,0]
                      ,[-0.14,-0.05,0.09,np.pi/2,np.pi/2-np.pi/4,0]
                      ,[-0.13,-0.10,0.13,np.pi/2,np.pi/2+np.pi/6,-np.pi/2]
                      ,[-0.14,0.04,0.09,np.pi/2,np.pi/2-np.pi/4,0]
                      ,[0.14,-0.05,0.09,np.pi/2,np.pi/2+np.pi/4,0]]
        self.goal_joints = [[2.0352258899333355, -1.3618286461730276, 1.449506139053659, -0.031469262028441115, 1.452007532816085, 0.07538374368350508],
                        [1.6184232542183854, -0.8044772366871494, 1.4838629733683688, 0.3366696649006673, 0.9861137942257749, -0.5502274625361658],
                        [2.709014061420861, -1.0052043344979187, 1.9232914418956464, -0.5920610707757745, 0.8338449789349853, 0.6686737517287772],
                        [1.707647451945137, -1.150916512768058, 2.085850342364061, -0.03590351669163394, 0.5965455570601269, -0.22595193797917382],
                        [2.239523425739526, -0.9245352681122855, 1.5153932601399716, -0.04606895481615142, 0.9559843430066769, 0.3029214928983926]]
        self.searchRoutine = [[2.5979687730055674, -1.2153287692700065, 1.745419398250195, 0.13484869576189057, 0.9287876830063775, 0.585026516522853],
                        [1.4353307765229468, -1.3316539149725484, 1.9017140690986003, -0.18478413052365567, 0.9007381388050799, -0.3908933588186231]]
        self.goal_config = []
        self.goal_set = False
        self.markerFound = False
        self.markerEstimate = PoseStamped()
        self.searchMarker()
        #self.estimateMarker()
        #self.moveToGoal()

    def searchMarker(self):
        while not self.markerFound:
            print('Searching for marker...')
            for trajPoint in self.searchRoutine:
                self.publishTarget(trajPoint)
                time.sleep(0.200)
        if(self.markerFound):
            print('Found marker.')
            self.goToFirstPosition()

    def setMarkerFound(self, pose_msg):
        if(not self.markerFound):
            self.markerFound = True
            print('Found marker')
    
    def goToFirstPosition(self):
        print('Now going to first position')
        goal = self.goals[0]
        trans = [goal[0],goal[1],goal[2]]
        rot = tf.transformations.quaternion_from_euler(goal[3],goal[4],goal[5])
        goal_kdl = kdl.Frame(kdl.Rotation.EulerZYX(goal[3],goal[4],goal[5]), kdl.Vector(goal[0],goal[1],goal[2]))
        goal_kdl = self.cameraTf*goal_kdl
        kdl_trans = [goal_kdl.p[0],goal_kdl.p[1],goal_kdl.p[2]]
        kdl_ori = tf.transformations.quaternion_from_euler(goal_kdl.M.GetEulerZYX()[0],goal_kdl.M.GetEulerZYX()[1],goal_kdl.M.GetEulerZYX()[2])
        foundTf = False
        while not foundTf:
            try:
                self.br.sendTransform(kdl_trans, kdl_ori, rospy.Time.now(), "first_goal", "aruco_marker_frame")
                self.listener.waitForTransform('/base_link','first_goal', rospy.Time(0), rospy.Duration(4))
                (t,r) = self.listener.lookupTransform('/base_link','first_goal', rospy.Time(0))
                print(t,r)
                foundTf = True
            except:
                self.br.sendTransform(kdl_trans, kdl_ori, rospy.Time.now(), "first_goal", "aruco_marker_frame")
        goalFrame = kdl.Frame(kdl.Rotation.Quaternion(r[0],r[1],r[2],r[3]), kdl.Vector(t[0],t[1],t[2]))
        q_out = kdl.JntArray(self.num_joints)
        self.solverPos.CartToJnt(self.joints, goalFrame, q_out)
        target = [q_out[0],q_out[1],q_out[2],q_out[3],q_out[4],q_out[5]]
        end_time = time.time() + 15
        while time.time() < end_time:
            self.publishTarget(target)
        print('In first position. Now estimating marker pose.')
        self.estimateMarker()

    def estimateMarker(self):
        print('Estimating marker pose')
        samples = 0
        t_x = []
        t_y = []
        t_z = []
        quat_x = []
        quat_y = []
        quat_z = []
        quat_w = []
        while samples <= 50:
            print('Got marker sample #'+str(samples))
            pose = rospy.wait_for_message('/aruco_single/pose', PoseStamped)
            t_x.append(pose.pose.position.x)
            t_y.append(pose.pose.position.y)
            t_z.append(pose.pose.position.z)
            quat_x.append(pose.pose.orientation.x)
            quat_y.append(pose.pose.orientation.y)
            quat_z.append(pose.pose.orientation.z)
            quat_w.append(pose.pose.orientation.w)
            samples = samples+1
        self.markerEstimate.pose.position.x = statistics.mean(t_x)
        self.markerEstimate.pose.position.y = statistics.mean(t_y)
        self.markerEstimate.pose.position.z = statistics.mean(t_z)
        self.markerEstimate.pose.orientation.x = statistics.mean(quat_x)
        self.markerEstimate.pose.orientation.y = statistics.mean(quat_y)
        self.markerEstimate.pose.orientation.z = statistics.mean(quat_z)
        self.markerEstimate.pose.orientation.w = statistics.mean(quat_w)
        self.markerEstimate.header.stamp  = rospy.Time.now()
        self.markerEstimate.header.frame_id = 'camera_link'
        print('Average marker pose: ')
        print(self.markerEstimate)
        self.set_goals(self.markerEstimate)

    def update_joints(self, joint_msg):
        for i, n in enumerate(self.joint_names):
            index = joint_msg.name.index(n)
            self.joints[i] = joint_msg.position[index]

        self.T_current = kdl.Frame()
        self.fkine.JntToCart(self.joints, self.T_current)

    def set_goals(self, pose_msg):
        print('Setting goals')
        if(not self.goal_set):
            dist = np.sqrt(pose_msg.pose.position.x*pose_msg.pose.position.x + pose_msg.pose.position.y*pose_msg.pose.position.y + pose_msg.pose.position.z*pose_msg.pose.position.z)
            print(dist)
            self.goal_config = []
            for idx,goal in enumerate(self.goals):
                #print(goal)
                goal_name = "goal"+str(idx)
                print(goal)
                trans = [goal[0],goal[1],goal[2]]
                rot = tf.transformations.quaternion_from_euler(goal[3],goal[4],goal[5])
                
                goal_kdl = kdl.Frame(kdl.Rotation.EulerZYX(goal[3],goal[4],goal[5]), kdl.Vector(goal[0],goal[1],goal[2]))
                goal_kdl = self.cameraTf*goal_kdl
                kdl_trans = [goal_kdl.p[0],goal_kdl.p[1],goal_kdl.p[2]]
                kdl_ori = tf.transformations.quaternion_from_euler(goal_kdl.M.GetEulerZYX()[0],goal_kdl.M.GetEulerZYX()[1],goal_kdl.M.GetEulerZYX()[2])
                self.br.sendTransform(kdl_trans, kdl_ori, rospy.Time.now(), goal_name, "aruco_marker_frame")

            for idx,goal in enumerate(self.goals):
                print('idx '+str(idx))
                goal_name = "goal"+str(idx)
                try:
                    (t,r) = self.listener.lookupTransform('/base_link',goal_name, rospy.Time(0))
                    #print('Got tf: ')
                    #print(t,r)
                    if(len(self.goal_config) < len(self.goals)):
                        self.send_goal(t,r)
                except:
                    print('try again')
                    self.set_goals(pose_msg) #try again
            self.goal_set = True
            self.moveToGoal()
            #print(trans,rot)
        
    def send_goal(self,trans,rot):
        print(trans,rot)
        goalFrame = kdl.Frame(kdl.Rotation.Quaternion(rot[0],rot[1],rot[2],rot[3]), kdl.Vector(trans[0],trans[1],trans[2]))
        q_out = kdl.JntArray(self.num_joints)
        self.solverPos.CartToJnt(self.joints, goalFrame, q_out)
        target = [q_out[0],q_out[1],q_out[2],q_out[3],q_out[4],q_out[5]]
        self.goal_config.append(target)
        print('len '+str(len(self.goal_config)))

    def moveToGoal(self):
        while not rospy.is_shutdown():
            if(len(self.goal_config) > 0):
                print('Number of points: ' + str(len(self.goal_config)))
                end_time = time.time() + 15
                while time.time() < end_time:
                    self.publishTarget(self.goal_config[0])
                self.goal_config.pop(0)
            else:
                print('Trajectory finished')
                self.rate.sleep()

    def publishTarget(self, joint_target):
        joints_str = JointTrajectory()
        joints_str.header.stamp = rospy.Time.now()
        joints_str.header.frame_id = 'base_link'
        joints_str.joint_names = ['joint_a1','joint_a2','joint_a3','joint_a4','joint_a5','joint_a6']
        point = JointTrajectoryPoint()
        point.positions = joint_target
        point.velocities = [0,0,0,0,0,0]
        point.accelerations = [0,0,0,0,0,0]
        point.effort = [0,0,0,0,0,0]
        point.time_from_start = rospy.Duration(0.01)
        joints_str.points.append(point)
        self.pub.publish(joints_str)
        #print('Published joint command' + str(np.rad2deg(joint_target)) + '- Waiting for next point')

def main():
    print('Position Based Visual Servo')
    #time.sleep(15)
    rospy.init_node("kuka_pbvs")
    controller = PBVS()
    rospy.spin()


if __name__ == "__main__":
    main()
