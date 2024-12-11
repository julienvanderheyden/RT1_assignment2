#! /usr/bin/env python

import rospy


# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the reaching_goal action, including the
# goal message and the feedback message.
import assignment_2_2024.msg

def feedback_cb(feedback):
    """Callback function to handle feedback from the action server."""
    rospy.loginfo("Feedback received: Actual pose: (x: %f, y: %f, z: %f), Status: %s",
                  feedback.actual_pose.position.x,
                  feedback.actual_pose.position.y,
                  feedback.actual_pose.position.z,
                  feedback.stat)


def reaching_goal_client():
    # Creates the SimpleActionClient
    
    
    client = actionlib.SimpleActionClient('reaching_goal', assignment_2_2024.msg.PlanningAction)
    

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for action server to start...")
    client.wait_for_server()
    rospy.loginfo("Action server started, sending goal")

    # Creates a goal to send to the action server.
    goal = assignment_2_2024.msg.PlanningGoal()
    
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = 5.0
    goal.target_pose.pose.position.y = 5.0
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    # Sends the goal to the action server with a feedback callback.
    client.send_goal(goal, feedback_cb=feedback_cb)

    # Waits for the server to finish performing the action.
    rospy.loginfo("Waiting for result...")
    client.wait_for_result()

    # Prints out the result of executing the action
    result = client.get_result()
    rospy.loginfo("Action completed with result: %s", result) 

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('reaching_goal_client_py')
        reaching_goal_client()
        
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
        
        
"""
root@88df3586f88f:~/my_ros/src# rosmsg show assignment_2_2024/PlanningAction
assignment_2_2024/PlanningActionGoal action_goal
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  actionlib_msgs/GoalID goal_id
    time stamp
    string id
  assignment_2_2024/PlanningGoal goal
    geometry_msgs/PoseStamped target_pose
      std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
      geometry_msgs/Pose pose
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
assignment_2_2024/PlanningActionResult action_result
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  actionlib_msgs/GoalStatus status
    uint8 PENDING=0
    uint8 ACTIVE=1
    uint8 PREEMPTED=2
    uint8 SUCCEEDED=3
    uint8 ABORTED=4
    uint8 REJECTED=5
    uint8 PREEMPTING=6
    uint8 RECALLING=7
    uint8 RECALLED=8
    uint8 LOST=9
    actionlib_msgs/GoalID goal_id
      time stamp
      string id
    uint8 status
    string text
  assignment_2_2024/PlanningResult result
assignment_2_2024/PlanningActionFeedback action_feedback
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  actionlib_msgs/GoalStatus status
    uint8 PENDING=0
    uint8 ACTIVE=1
    uint8 PREEMPTED=2
    uint8 SUCCEEDED=3
    uint8 ABORTED=4
    uint8 REJECTED=5
    uint8 PREEMPTING=6
    uint8 RECALLING=7
    uint8 RECALLED=8
    uint8 LOST=9
    actionlib_msgs/GoalID goal_id
      time stamp
      string id
    uint8 status
    string text
  assignment_2_2024/PlanningFeedback feedback
    geometry_msgs/Pose actual_pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
    string stat


"""
