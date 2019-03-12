#!/usr/bin/env python
import rospy
import actionlib, smach, smach_ros
from smach import State, StateMachine
from sensor_msgs.msg import Joy
from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

ordered_waypoints = [
 ['initial', (0.887,0.5), (0,0,0,1)],
 ['zero',(1.35,-0.15), (0.0, 0.0, -.653, .756)],
 ['one', (0.83,-0.872),  (0.0, 0.0, 0, 1)],
 ['two', (0.955, -1.78),  (0.0, 0.0, -.653, .756)],
 ['three',(1.68, -1.75),  (0.0, 0.0, -.653, .756)],
 ['four', (2.47, -1.7),  (0.0, 0.0, -.653, .756)],
 ['five', (3.21, -1.63),  (0.0, 0.0, -.653, .756)],
 ['six', (3.9, -1.61),  (0.0, 0.0, -.653, .756)],
 ['seven',(2.73, -0.582),  (0.0, 0.0, 0.736, 0.676)],
 ['eight', (1.9, -0.655),  (0.0, 0.0, 0.0, 1.0)],
 ['nine', (3.35, -1.38),  (0.0, 0.0, 0.0, 1.0)],
 ['ten', (3.85, -0.09),  (0.0, 0.0, 736, 0.676)]
]


    		


class Waypoint(smach.State):
    def __init__(self, position, orientation):
        smach.State.__init__(self, outcomes=['success'])
        rospy.loginfo("Setting up client")
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    	rospy.loginfo("ready1")
    	self.client.wait_for_server()
    	rospy.loginfo("ready2")
    	
    	self.goal = MoveBaseGoal()
    	self.goal.target_pose.header.frame_id = 'map'
    	self.goal.target_pose.pose.position.x = position[0]
    	self.goal.target_pose.pose.position.y = position[1]
    	self.goal.target_pose.pose.position.z = 0
    	self.goal.target_pose.pose.orientation.x = orientation[0]
    	self.goal.target_pose.pose.orientation.y = orientation[1]
    	self.goal.target_pose.pose.orientation.z = orientation[2]
    	self.goal.target_pose.pose.orientation.w = orientation[3]
        self.first = 1


    def execute(self, userdata):
    	if self.first:
            self.client.send_goal(self.goal)
            self.client.wait_for_result()
            self.first =0
        return 'success'

    

def main():
    rospy.init_node('goal')
    rate = rospy.Rate(10)




    sm = StateMachine('success')
    
    with sm:
    	for i, w in enumerate(ordered_waypoints):
    		StateMachine.add(w[0], Waypoint(w[1], w[2]),transitions={'success':ordered_waypoints[(i + 1) %  len(ordered_waypoints)][0]})

    sm.execute() 

if __name__ == '__main__':
    main()
