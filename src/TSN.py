#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from assignment2_rt.srv import GetTarget, GetTargetResponse

class TargetServiceNode:
    def __init__(self):
        rospy.init_node("target_service_node", anonymous=True)

        self.current_target = None
        rospy.Subscriber("/target", Float64MultiArray, self.target_callback)

	#service definition
        self.target_service = rospy.Service("get_target", GetTarget, self.handle_get_target)

        rospy.loginfo("Target service node initialized and ready.")

    def target_callback(self, msg):
        """Callback to handle new target data from the "/target" topic."""
        
        if len(msg.data) == 2:
            self.current_target = (msg.data[0], msg.data[1])
            rospy.loginfo(f"New target received and stored: {self.current_target}")
        else:
            rospy.logwarn("Invalid target data received. Expected 2 values (x, y).")

    def handle_get_target(self, req):
        """Service handler to return the current target."""
        if self.current_target:
            rospy.loginfo(f"Returning current target: {self.current_target}")
            return GetTargetResponse(self.current_target[0], self.current_target[1])
        else:
            rospy.logwarn("No target is currently set.")
            return GetTargetResponse(0.0, 0.0)

if __name__ == "__main__":
    try:
        node = TargetServiceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Target service node shut down.")

