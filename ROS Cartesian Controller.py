#!/usr/bin/env python

import math
import numpy
import time
from threading import Thread, Lock

import rospy
import tf
from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from urdf_parser_py.urdf import URDF

def S_matrix(w):
    S = numpy.zeros((3,3))
    S[0,1] = -w[2]
    S[0,2] =  w[1]
    S[1,0] =  w[2]
    S[1,2] = -w[0]
    S[2,0] = -w[1]
    S[2,1] =  w[0]
    return S


# This is the function that must be filled in as part of the Project.
def cartesian_control(joint_transforms, b_T_ee_current, b_T_ee_desired,
                      red_control, q_current, q0_desired):
    num_joints = len(joint_transforms)
    dq = numpy.zeros(num_joints)
    #-------------------- Fill in your code here ---------------------------
    A0=numpy.linalg.inv(b_T_ee_current)
    A1=b_T_ee_desired
    A2=numpy.dot(A0,A1)
    a=tf.transformations.translation_from_matrix(A2)
    b=[]
    b.append(a[0])
    b.append(a[1])
    b.append(a[2])
    c=rotation_from_matrix(A2)
    c1=c[0]*c[1][0]
    c2=c[0]*c[1][1]
    c3=c[0]*c[1][2]
    b.append(c1)
    b.append(c2)
    b.append(c3)  
    deltaX = b               ##deltaX
    dX=numpy.dot(1,deltaX) ##dX
    
    RVee = numpy.array(b_T_ee_current, dtype=numpy.float64, copy=False)
    RVee1= RVee[:3,:3]
    s=(6,6)
    RVee2=numpy.zeros(s)
    RVee2[:3,:3]=RVee1
    RVee2[3:,3:]=RVee1
    
    ##Vee=numpy.dot(RVee2,dX) ##Vee
    Vee=dX
    
    ## Vj Calculations
    ee_T_joint0=numpy.dot(A0,joint_transforms[0])
    J0=ee_T_joint0
    V0=numpy.zeros((6,6))
    V0[:3,:3]=J0[:3,:3]
    V0[3:,3:]=J0[:3,:3]
    P0=numpy.linalg.inv(joint_transforms[0])
    joint0_T_ee=numpy.dot(P0,b_T_ee_current)
    G0=joint0_T_ee
    E0=tf.transformations.translation_from_matrix(G0)
    S0=S_matrix(E0)
    Y0=numpy.dot(J0[:3,:3],S0)
    Z0=-Y0
    V0[:3,3:]=Z0
    V00=V0[:,5]   ##First Joint
    
    ee_T_joint1=numpy.dot(A0,joint_transforms[1])
    J1=ee_T_joint1
    V1=numpy.zeros((6,6))
    V1[:3,:3]=J1[:3,:3]
    V1[3:,3:]=J1[:3,:3]
    P1=numpy.linalg.inv(joint_transforms[1])
    joint1_T_ee=numpy.dot(P1,b_T_ee_current)
    G1=joint1_T_ee
    E1=tf.transformations.translation_from_matrix(G1)
    S1=S_matrix(E1)
    Y1=numpy.dot(J1[:3,:3],S1)
    Z1=-Y1
    V1[:3,3:]=Z1
    V11=V1[:,5]   ##Second Joint
    
    ee_T_joint2=numpy.dot(A0,joint_transforms[2])
    J2=ee_T_joint2
    V2=numpy.zeros((6,6))
    V2[:3,:3]=J2[:3,:3]
    V2[3:,3:]=J2[:3,:3]
    P2=numpy.linalg.inv(joint_transforms[2])
    joint2_T_ee=numpy.dot(P2,b_T_ee_current)
    G2=joint2_T_ee
    E2=tf.transformations.translation_from_matrix(G2)
    S2=S_matrix(E2)
    Y2=numpy.dot(J2[:3,:3],S2)
    Z2=-Y2
    V2[:3,3:]=Z2
    V22=V2[:,5]   ##Third Joint
    
    ee_T_joint3=numpy.dot(A0,joint_transforms[3])
    J3=ee_T_joint3
    V3=numpy.zeros((6,6))
    V3[:3,:3]=J3[:3,:3]
    V3[3:,3:]=J3[:3,:3]
    P3=numpy.linalg.inv(joint_transforms[3])
    joint3_T_ee=numpy.dot(P3,b_T_ee_current)
    G3=joint3_T_ee
    E3=tf.transformations.translation_from_matrix(G3)
    S3=S_matrix(E3)
    Y3=numpy.dot(J3[:3,:3],S3)
    Z3=-Y3
    V3[:3,3:]=Z3
    V33=V3[:,5]   ##Fourth Joint
    
    ee_T_joint4=numpy.dot(A0,joint_transforms[4])
    J4=ee_T_joint4
    V4=numpy.zeros((6,6))
    V4[:3,:3]=J4[:3,:3]
    V4[3:,3:]=J4[:3,:3]
    P4=numpy.linalg.inv(joint_transforms[4])
    joint4_T_ee=numpy.dot(P4,b_T_ee_current)
    G4=joint4_T_ee
    E4=tf.transformations.translation_from_matrix(G4)
    S4=S_matrix(E4)
    Y4=numpy.dot(J4[:3,:3],S4)
    Z4=-Y4
    V4[:3,3:]=Z4
    V44=V4[:,5]   ##Fifth Joint
    
    ee_T_joint5=numpy.dot(A0,joint_transforms[5])
    J5=ee_T_joint5
    V5=numpy.zeros((6,6))
    V5[:3,:3]=J5[:3,:3]
    V5[3:,3:]=J5[:3,:3]
    P5=numpy.linalg.inv(joint_transforms[5])
    joint5_T_ee=numpy.dot(P5,b_T_ee_current)
    G5=joint5_T_ee
    E5=tf.transformations.translation_from_matrix(G5)
    S5=S_matrix(E5)
    Y5=numpy.dot(J5[:3,:3],S5)
    Z5=-Y5
    V5[:3,3:]=Z5
    V55=V5[:,5]   ##Sixth Joint
    
    ee_T_joint6=numpy.dot(A0,joint_transforms[6])
    J6=ee_T_joint6
    V6=numpy.zeros((6,6))
    V6[:3,:3]=J6[:3,:3]
    V6[3:,3:]=J6[:3,:3]
    P6=numpy.linalg.inv(joint_transforms[6])
    joint6_T_ee=numpy.dot(P6,b_T_ee_current)
    G6=joint6_T_ee
    E6=tf.transformations.translation_from_matrix(G6)
    S6=S_matrix(E6)
    Y6=numpy.dot(J6[:3,:3],S6)
    Z6=-Y6
    V6[:3,3:]=Z6
    V66=V6[:,5]   ##Seventh Joint
    
    J=numpy.zeros((6,7)) ##Jacobian Computation
    J[:,0]=V00
    J[:,1]=V11
    J[:,2]=V22
    J[:,3]=V33
    J[:,4]=V44
    J[:,5]=V55
    J[:,6]=V66
    JINV=numpy.linalg.pinv(J)
    dq1=numpy.dot(JINV,Vee)
    #----------------------------------------------------------------------
    return dq1
    
def convert_from_message(t):
    trans = tf.transformations.translation_matrix((t.translation.x,
                                                  t.translation.y,
                                                  t.translation.z))
    rot = tf.transformations.quaternion_matrix((t.rotation.x,
                                                t.rotation.y,
                                                t.rotation.z,
                                                t.rotation.w))
    T = numpy.dot(trans,rot)
    return T

# Returns the angle-axis representation of the rotation contained in the input matrix
# Use like this:
# angle, axis = rotation_from_matrix(R)
def rotation_from_matrix(matrix):
    R = numpy.array(matrix, dtype=numpy.float64, copy=False)
    R33 = R[:3, :3]
    # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, W = numpy.linalg.eig(R33.T)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    axis = numpy.real(W[:, i[-1]]).squeeze()
    # point: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, Q = numpy.linalg.eig(R)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    # rotation angle depending on axis
    cosa = (numpy.trace(R33) - 1.0) / 2.0
    if abs(axis[2]) > 1e-8:
        sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
    elif abs(axis[1]) > 1e-8:
        sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
    else:
        sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
    angle = math.atan2(sina, cosa)
    return angle, axis

class CartesianControl(object):

    #Initialization
    def __init__(self):
        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)

        #Subscribes to command for end-effector pose
        rospy.Subscriber("/cartesian_command", Transform, self.command_callback)

        #Subscribes to command for redundant dof
        rospy.Subscriber("/redundancy_command", Float32, self.redundancy_callback)

        # Publishes desired joint velocities
        self.pub_vel = rospy.Publisher("/joint_velocities", JointState, queue_size=1)

        #This is where we hold the most recent joint transforms
        self.joint_transforms = []
        self.q_current = []
        self.x_current = tf.transformations.identity_matrix()
        self.R_base = tf.transformations.identity_matrix()
        self.x_target = tf.transformations.identity_matrix()
        self.q0_desired = 0
        self.last_command_time = 0
        self.last_red_command_time = 0

        # Initialize timer that will trigger callbacks
        self.mutex = Lock()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def command_callback(self, command):
        self.mutex.acquire()
        self.x_target = convert_from_message(command)
        self.last_command_time = time.time()
        self.mutex.release()

    def redundancy_callback(self, command):
        self.mutex.acquire()
        self.q0_desired = command.data
        self.last_red_command_time = time.time()
        self.mutex.release()        
        
    def timer_callback(self, event):
        msg = JointState()
        self.mutex.acquire()
        if time.time() - self.last_command_time < 0.5:
            dq = cartesian_control(self.joint_transforms, 
                                   self.x_current, self.x_target,
                                   False, self.q_current, self.q0_desired)
            msg.velocity = dq
        elif time.time() - self.last_red_command_time < 0.5:
            dq = cartesian_control(self.joint_transforms, 
                                   self.x_current, self.x_current,
                                   True, self.q_current, self.q0_desired)
            msg.velocity = dq
        else:            
            msg.velocity = numpy.zeros(7)
        self.mutex.release()
        self.pub_vel.publish(msg)
        
    def joint_callback(self, joint_values):
        root = self.robot.get_root()
        T = tf.transformations.identity_matrix()
        self.mutex.acquire()
        self.joint_transforms = []
        self.q_current = joint_values.position
        self.process_link_recursive(root, T, joint_values)
        self.mutex.release()

    def align_with_z(self, axis):
        T = tf.transformations.identity_matrix()
        z = numpy.array([0,0,1])
        x = numpy.array([1,0,0])
        dot = numpy.dot(z,axis)
        if dot == 1: return T
        if dot == -1: return tf.transformation.rotation_matrix(math.pi, x)
        rot_axis = numpy.cross(z, axis)
        angle = math.acos(dot)
        return tf.transformations.rotation_matrix(angle, rot_axis)

    def process_link_recursive(self, link, T, joint_values):
        if link not in self.robot.child_map: 
            self.x_current = T
            return
        for i in range(0,len(self.robot.child_map[link])):
            (joint_name, next_link) = self.robot.child_map[link][i]
            if joint_name not in self.robot.joint_map:
                rospy.logerror("Joint not found in map")
                continue
            current_joint = self.robot.joint_map[joint_name]        

            trans_matrix = tf.transformations.translation_matrix((current_joint.origin.xyz[0], 
                                                                  current_joint.origin.xyz[1],
                                                                  current_joint.origin.xyz[2]))
            rot_matrix = tf.transformations.euler_matrix(current_joint.origin.rpy[0], 
                                                         current_joint.origin.rpy[1],
                                                         current_joint.origin.rpy[2], 'rxyz')
            origin_T = numpy.dot(trans_matrix, rot_matrix)
            current_joint_T = numpy.dot(T, origin_T)
            if current_joint.type != 'fixed':
                if current_joint.name not in joint_values.name:
                    rospy.logerror("Joint not found in list")
                    continue
                # compute transform that aligns rotation axis with z
                aligned_joint_T = numpy.dot(current_joint_T, self.align_with_z(current_joint.axis))
                self.joint_transforms.append(aligned_joint_T)
                index = joint_values.name.index(current_joint.name)
                angle = joint_values.position[index]
                joint_rot_T = tf.transformations.rotation_matrix(angle, 
                                                                 numpy.asarray(current_joint.axis))
                next_link_T = numpy.dot(current_joint_T, joint_rot_T) 
            else:
                next_link_T = current_joint_T

            self.process_link_recursive(next_link, next_link_T, joint_values)
 

if __name__ == '__main__':
    rospy.init_node('cartesian_control', anonymous=True)
    cc = CartesianControl()
    rospy.spin()
