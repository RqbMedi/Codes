#!/usr/bin/env python  
import rospy

import numpy

import tf
import tf2_ros
import geometry_msgs.msg

def message_from_transform(T):
 msg=geometry_msgs.msg.Transform()
 q=tf.transformations.quaternion_from_matrix(T)
 translation=tf.transformations.translation_from_matrix(T)
 msg.rotation.x=q[0]
 msg.rotation.y=q[1]
 msg.rotation.z=q[2]
 msg.rotation.w=q[3]
 msg.translation.x=translation[0]
 msg.translation.y=translation[1]
 msg.translation.z=translation[2]
 return msg

def publish_transforms():
    object_transform = geometry_msgs.msg.TransformStamped()
    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = "base_frame"
    object_transform.child_frame_id = "object_frame"
    T1=tf.transformations.concatenate_matrices(
           tf.transformations.quaternion_matrix(
           tf.transformations.quaternion_from_euler(0.79,0.0,0.79)
           ),
           tf.transformations.translation_matrix((0.0,1.0,1.0))
           )
    object_transform.transform=message_from_transform(T1)
    br.sendTransform(object_transform)
    
    robot_transform = geometry_msgs.msg.TransformStamped()
    robot_transform.header.stamp = rospy.Time.now()
    robot_transform.header.frame_id = "base_frame"
    robot_transform.child_frame_id = "robot_frame"
    T2=tf.transformations.concatenate_matrices(
           tf.transformations.quaternion_matrix(
               tf.transformations.quaternion_about_axis(1.5,(0.0,0.0,1.0))                           
               ),
           tf.transformations.translation_matrix((0.0,-1.0,0.0))
           )
    robot_transform.transform=message_from_transform(T2)
    br.sendTransform(robot_transform)
      
    camera_transform = geometry_msgs.msg.TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "robot_frame"
    camera_transform.child_frame_id = "camera_frame"
    T7=tf.transformations.concatenate_matrices(
        tf.transformations.translation_matrix((0.0,0.1,0.1))
               )
    T4=numpy.dot(T2,T7)
    ##print(T2)
    ##print(T3)
    ##print(T4)
    T5=numpy.linalg.inv(T4)
    ##print(T5)
    T6=numpy.dot(T5,T1)
    ##print(T6)
    a=tf.transformations.translation_from_matrix(T6)
    b=numpy.linalg.norm(a)
    ##print(b)
    c=a/b
    ##print(c)
    d=numpy.cross((1,0,0),c)
    ##print(d)
    e=numpy.arccos(numpy.dot((1,0,0), c))
    ##print(e)
    T3=tf.transformations.concatenate_matrices(
           tf.transformations.translation_matrix((0.0,0.1,0.1)),
             tf.transformations.quaternion_matrix(
               tf.transformations.quaternion_about_axis(e,d)                           
               )
                )
    camera_transform.transform=message_from_transform(T3)
    br.sendTransform(camera_transform)
    


   

if __name__ == '__main__':
    rospy.init_node('project2_solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
