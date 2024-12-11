#! /usr/bin/env python

import rospy
import threading

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the reaching_goal action, including the
# goal message and the feedback message.
import assignment_2_2024.msg
from assignment2_rt.msg import Posvel
from nav_msgs.msg import Odometry

class User:

    def __init__(self):
        # Initialize the node
        rospy.init_node('User', anonymous=True)
        
        self.previous_status = None

        # Creates the SimpleActionClient
        self.action_client = actionlib.SimpleActionClient('reaching_goal', assignment_2_2024.msg.PlanningAction)
        self.odom_sub = rospy.Subscriber('/odom', Odometry , self.odom_callback)
        self.posvel_pub = rospy.Publisher('/posvel', Posvel, queue_size=10)

        # Waits until the action server has started up and started
        # listening for goals.
        rospy.loginfo("Waiting for action server to start...")
        self.action_client.wait_for_server()
        rospy.loginfo("Action server started")

        # Start listening to the terminal in a separate thread
        self.listen_to_input()

    def listen_to_input(self):
        """Listens to terminal input and calls the callback function with the input text."""
        threading.Thread(target=self.read_input, daemon=True).start()

    def read_input(self):
        """Reads input from the terminal and calls the callback."""
        while not rospy.is_shutdown():
            user_input = input("Enter control line: ")  # Wait for the user to enter text
            self.input_callback(user_input)

    def input_callback(self, text):
        """Callback function that checks user input and performs the requested action."""
        try:
            # Split the input text by spaces
            parts = text.split()

            # Check if the input is a "cancel" command
            if text.strip().lower() == "cancel":
                rospy.loginfo("Cancel command received. Attempting to cancel the goal.")
                self.cancel_goal()
                return

            # Check if the input has exactly 2 parts for x and y
            if len(parts) == 2:
                try:
                    # Parse x and y as float
                    x = float(parts[0])
                    y = float(parts[1])
                except ValueError:
                    raise ValueError("Both x and y must be valid numbers.")

                # Call the function to send the new goal
                self.send_new_goal(x, y)
                return

            # If input doesn't match either case, raise an error
            raise ValueError("Invalid input! Expected 'cancel' or 'x y' (two numbers for a new goal).")

        except ValueError as e:
            # Log an error message for invalid input
            rospy.logerr(f"Invalid input: {e}")

    def feedback_cb(self, feedback):
        """Callback function to handle feedback from the action server."""
        status = None
        if feedback.stat == "Target reached!":
        	rospy.loginfo("Target has been reached !")
        	return
        
        elif feedback.stat == "State 0: go to point" :
        	status = 0
        
        elif feedback.stat == "State 1: avoid obstacle":
        	status = 1
        
        else:
        	rospy.loginfo("Feedback status not recognized... [%s]", feedback.stat)
        	return
        
        if status != self.previous_status:
        	if status == 0:
        		rospy.loginfo("State 0: go to point")
        	else:
        		rospy.loginfo("State 1: avoid obstacle")
        	#self.read_input()
        	
        self.previous_status = status

    def send_new_goal(self, x, y):
        """Sends a new goal to the action server."""
        self.action_client.cancel_goal()
        goal = assignment_2_2024.msg.PlanningGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        self.action_client.send_goal(goal, feedback_cb = self.feedback_cb)
        rospy.loginfo(f"New goal sent: x = {x}, y = {y}")

    def cancel_goal(self):
        """Function to cancel the current goal."""
        self.action_client.cancel_goal()
        rospy.loginfo("Goal canceled successfully.")
        
    def odom_callback(self, data):
    	"""Callback publishing custom messages on another topic"""
    	msg = Posvel()
    	msg.x = data.pose.pose.position.x
    	msg.y = data.pose.pose.position.y
    	msg.vel_x = data.twist.twist.linear.x
    	msg.vel_y = data.twist.twist.linear.y
    	
    	self.posvel_pub.publish(msg)
    	
    	
        
    

if __name__ == '__main__':
    try:
        node = User()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("User node shut down.")

