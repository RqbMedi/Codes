#!/usr/bin/env python  
import rospy

from std_msgs.msg import Int16
from project1_solution.msg import TwoInts

def callback(data):
       rospy.loginfo(rospy.get_caller_id() + "%s" + "%s",data.a,data.b)
       pub = rospy.Publisher('sum', Int16, queue_size=10)
       rate = rospy.Rate(10) # 10hz
       while not rospy.is_shutdown():
           sum = data.a + data.b
           rospy.loginfo(sum)
           pub.publish(sum)
           rate.sleep()
        
def node1():
       rospy.init_node('node1', anonymous=True)
       rospy.Subscriber('two_ints', TwoInts, callback) 
       rospy.spin()
      
if __name__ == '__main__':
      node1()
      
