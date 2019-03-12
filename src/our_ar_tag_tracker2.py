#!/usr/bin/env python
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from math import copysign
import rospy, numpy, smach, ros_numpy, smach_ros, time, math
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Joy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import tf

found_markers = []
start = None
position = None
orientation =  None

class GoToStart(smach.State):
    """
    Return to the start in order to go to next AR tag
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['FindTag','Done'])

        rospy.loginfo("In GO to start")

        rospy.loginfo("Setting up client")
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    	rospy.loginfo("ready")
    	self.client.wait_for_server()
        rospy.loginfo("here")

        


    def execute(self,userdata):
        rospy.loginfo("Executing State GoToStart")
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = '/odom'
        self.goal.target_pose.pose.position.x = start.position.x
        self.goal.target_pose.pose.position.y = start.position.y
        self.goal.target_pose.pose.position.z = start.position.z
        self.goal.target_pose.pose.orientation.x = start.orientation.x
        self.goal.target_pose.pose.orientation.y = start.orientation.y
        self.goal.target_pose.pose.orientation.z = start.orientation.z
        self.goal.target_pose.pose.orientation.w = start.orientation.w

        while not rospy.is_shutdown():
            rospy.loginfo("Executing GoToStart")
            self.client.send_goal(self.goal)
            self.client.wait_for_result()
            return 'FindTag'


        return 'Done'


class GoToWayPoint(smach.State):
    """
    Purpose: go to  waypoint once we are within threshold
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['GoToStart','Done'])

        self.alvar_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.alvarCallback)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)

        rospy.loginfo("Setting up client2")
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    	rospy.loginfo("ready2")
    	self.client.wait_for_server()
        rospy.loginfo("here2")

        self.pose = None
        self.goalPose = None


    def execute(self, userdata):
        rospy.loginfo("Executing GoToWayPoint")

        while not rospy.is_shutdown():
            rospy.wait_for_message('ar_pose_marker', AlvarMarkers)
            rospy.wait_for_message('odom', Odometry)

            goal = self.calculateGoal()

            self.client.send_goal(goal)
            self.client.wait_for_result()
            return 'GoToStart'

        return 'done'

    def calculateGoal(self):
        t = self.pose
        distToRobot = ros_numpy.numpify(self.pose) # p2
        distToTag = ros_numpy.numpify(self.goalPose) #p1
        #rospy.loginfo("p1")
        #rospy.loginfo(self.pose)
        #rospy.loginfo(self.pose.orientation)
        #rospy.loginfo("p2")
        

        distToTagGlobal = numpy.dot(distToRobot, distToTag) #gives us the pose of the tag w.r.t. global frame
        #rospy.loginfo(distToTagGlobal.shape)
        distToTagGlobal = ros_numpy.msgify(Pose, distToTagGlobal)

        


      

        '''goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = '/odom'
    	goal.target_pose.pose.position.x = self.pose.position.x + self.goalPose.position.x
    	goal.target_pose.pose.position.y = self.pose.position.y + self.goalPose.position.y
    	goal.target_pose.pose.position.z = self.pose.position.z 
    	goal.target_pose.pose.orientation.x = distToTagGlobal.orientation.x
    	goal.target_pose.pose.orientation.y = distToTagGlobal.orientation.y
    	goal.target_pose.pose.orientation.z = distToTagGlobal.orientation.z
    	goal.target_pose.pose.orientation.w = distToTagGlobal.orientation.w
        '''
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = '/odom'
        goal.target_pose.pose.position.x = distToTagGlobal.position.x 
        goal.target_pose.pose.position.y = distToTagGlobal.position.y 
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation.x = self.pose.orientation.x #distToTagGlobal.orientation.x
        goal.target_pose.pose.orientation.y = self.pose.orientation.y#distToTagGlobal.orientation.y
        goal.target_pose.pose.orientation.z = self.pose.orientation.z#distToTagGlobal.orientation.z
        goal.target_pose.pose.orientation.w = self.pose.orientation.w#distToTagGlobal.orientation.w

        quaternion = (  distToTagGlobal.orientation.x,
                        distToTagGlobal.orientation.y,
                        distToTagGlobal.orientation.z,
                        distToTagGlobal.orientation.w
                        )
        

        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        yaw -= math.pi/2

        dx = 0.1*math.cos(yaw)
        dy = 0.1*math.sin(yaw)
        
        goal.target_pose.pose.position.x += dx
        goal.target_pose.pose.position.y += dy


        return goal     


    def odomCallback(self, msg):
        self.pose = msg.pose.pose


    def alvarCallback(self, msg):
        try:
            #rospy.loginfo(msg.markers[0].id)
            marker = msg.markers[0]
            self.goalPose = marker.pose.pose
        except:
            pass

class FindTag(smach.State):
    def __init__(self):
        rospy.loginfo("ready3")
        smach.State.__init__(self, outcomes=['ApproachTag','Done'])
        self.cmd_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=5)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)

        rospy.wait_for_message('ar_pose_marker', AlvarMarkers)
        rospy.loginfo("here3")
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.set_cmd_vel)

        self.move_cmd = Twist()  
        # Set flag to indicate when the AR marker is visible
        self.current_marker = None
        self.look_for_marker = 1
        self.now = 0
        self.pose = None


    def execute(self,userdata):
        global start
        rospy.loginfo("Executing State FindTag")

        rospy.wait_for_message('odom', Odometry)
        #Save the start pose
        if start == None:
            start = self.pose

        while not rospy.is_shutdown():
            self.now = 1
            self.move_cmd = Twist()
            self.found = 0

            while self.found == 0:
                self.cmd_vel_pub.publish(self.move_cmd)
            self.now = 0

            return 'ApproachTag'
        return 'Done'


    def set_cmd_vel(self,msg):
        # if there is a marker do try
        global found_markers
        if self.now:
            try: 
                marker = msg.markers[0]
                self.current_marker = marker.id
                if (self.current_marker not in found_markers) and (self.current_marker != 0):
                        rospy.loginfo("FOLLOWER found Target!")
                        found_markers.append(self.current_marker)
                        self.found = 1
                        rospy.loginfo(found_markers)
                else:
                    rospy.loginfo("FOLLOWER is looking for Target")
                    self.move_cmd.linear.x = 0
                    self.move_cmd.angular.z = 0.3
            except:
                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = 0.3


    def odomCallback(self, msg):
        self.pose = msg.pose.pose

class ApproachTag(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['GoToWayPoint','Done'])
        self.cmd_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=5)

        self.move_cmd = Twist()
        
        # Set flag to indicate when the AR marker is visible
        self.target_visible = False
        self.current_marker = None
        self.found = 0
        self.success = 0
        self.now = 0
        self.look_for_marker = 1

        rospy.wait_for_message('ar_pose_marker',AlvarMarkers)
        rospy.loginfo("here4")
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.set_cmd_vel)
        

    def execute(self,userdata):

        rospy.loginfo("Executing State ApproachTag")
        
        while not rospy.is_shutdown():
            self.now = 1
            self.move_cmd = Twist()
            self.success = 0
            while self.success == 0:
                self.cmd_vel_pub.publish(self.move_cmd)
            self.now = 0
            return 'GoToWayPoint'
        return 'Done'


    def set_cmd_vel(self,msg):
        if self.now:
            try: 
                marker = msg.markers[0]
                self.current_marker = marker.id
                if not self.target_visible:
                        rospy.loginfo("FOLLOWER is Tracking Target!")
                self.target_visible = True

            except:
                    self.move_cmd.linear.x /= 1.5
                    self.move_cmd.angular.z /= 1.5
                        
                    if self.target_visible:
                        rospy.loginfo("FOLLOWER LOST Target!")
                    self.target_visible = False

                    return 

            # Get the displacement of the marker relative to the base
            target_offset_y = marker.pose.pose.position.y
            # Get the distance of the marker from the base
            target_offset_x = marker.pose.pose.position.x

            # keep ar tag in centre
            if target_offset_y > 0.1:
                speed = 0.2
            elif target_offset_y < -0.1:
                speed = -0.2
            else:
                speed = 0
            self.move_cmd.angular.z = speed

            if target_offset_x > 0.7:
                speed = 0.2
                if speed <0:
                    speed *= 1.5
            else:
                speed = 0
                self.success = 1

            self.move_cmd.linear.x = speed

def main():
    rospy.init_node('demo5')
    rate = rospy.Rate(10)
    sm = smach.StateMachine(outcomes = ['DoneProgram'])
    sm.set_initial_state(['FindTag'])

    with sm:
        smach.StateMachine.add('GoToStart', GoToStart(),
                                        transitions = {'FindTag': 'FindTag',
                                                        'Done' : 'DoneProgram'})

        smach.StateMachine.add('GoToWayPoint', GoToWayPoint(),
                                        transitions = {'GoToStart': 'GoToStart',
                                                        'Done' : 'DoneProgram'})

        smach.StateMachine.add('ApproachTag', ApproachTag(),
                                        transitions = {'GoToWayPoint': 'GoToWayPoint',
                                                        'Done' : 'DoneProgram'})

        smach.StateMachine.add('FindTag', FindTag(),
                                        transitions = {'ApproachTag': 'ApproachTag',
                                                        'Done' : 'DoneProgram'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute() 
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()