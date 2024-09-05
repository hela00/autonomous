#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class MotorTestNode:
    def __init__(self):
        rospy.init_node('motor_test_node', anonymous=True)
        
        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscribers to motor feedback
        self.motor_feedback_sub = rospy.Subscriber('/motor_feedback', Float64, self.motor_feedback_callback)
        
        # Define test parameters
        self.test_duration = 10  # Duration of the test in seconds
        self.rate = rospy.Rate(1)  # 1 Hz
        
        rospy.loginfo("Motor Test Node Initialized")
    
    def motor_feedback_callback(self, msg):
        # Process motor feedback here
        rospy.loginfo("Received motor feedback: %f", msg.data)

    def run_test(self):
        # Define the test commands
        twist = Twist()
        twist.linear.x = 0.5  # Forward velocity
        twist.angular.z = 0.0  # No rotation
        
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < self.test_duration:
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()
        
        # Stop the robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

if __name__ == "__main__":
    try:
        node = MotorTestNode()
        node.run_test()
    except rospy.ROSInterruptException:
        pass
