#!/usr/bin/python
"""
Sends GPS waypoints
When wall is found, navigate to the wall corner
when arrived align with corner
Wait for move commands to move parallel to the wall

"""
import roslib; roslib.load_manifest('challenge_coordinator')
import rospy
from move_msgs.srv import FindWall
from move_msgs.srv import AlignWall
from challenge_coordinator.srv import CommandMisc
from numpy import loadtxt
from sensor_msgs.msg import NavSatFix
import gps_goal_msgs.msg
import actionlib
import smach
import smach_ros


move_along_wall = False
command_misc_int = 1


def get_waypoints():
    lines = loadtxt("catkin_ws_smach/src/gps_goal/waypoints/points_outdoor.txt", delimiter=" ")
    return lines


#define state SendWaypoints
class Send_Waypoints(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pattern_found','outcome2'])
        self.count = 0;

    def execute(self, _):
        client = actionlib.SimpleActionClient('gps_goals', gps_goal_msgs.msg.ExecuteWaypointsGPSAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()
        rospy.loginfo('Executing state Send_Waypoints')
        waypoints = get_waypoints()
        goals = []
        for i in waypoints:
            goal= NavSatFix()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id= "/navsat/fix"
            goal.latitude = i[0]
            goal.longitude = i[1]
            goals.append(goal)
        goals_message = gps_goal_msgs.msg.ExecuteWaypointsGPSGoal(goals)
        client.send_goal(goals_message)

        # Waits for the server to finish performing the action.
        client.wait_for_result()
        if client.get_result() == 0:
            return 'outcome2'
        return 'pattern_found'


# define state Chase_wall
class Chase_Wall(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wall_corner_reached','outcome2'])

    def execute(self, _):
        rospy.loginfo('Executing state Chase_Wall')
        rospy.wait_for_service("/chase_wall")
        try:
            chase_wall = rospy.ServiceProxy('/chase_wall', FindWall)  # make service file
            resp1 = chase_wall(0)
            print('the out come of chase wall is: ',resp1.status)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return 'outcome2'
        return 'wall_corner_reached'


# define state Operate Manipulator
class Manipulator_Action(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['manipulator_action_complete'], input_keys=['manip_block_size'], output_keys=['move_block_size'])

    def execute(self, userdata):
        rospy.loginfo('Executing state manipulator action')
        userdata.move_block_size = userdata.manip_block_size
        return 'manipulator_action_complete'


# define state Align_wall
class Align_Wall(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['aligned_successfully','outcome2'])

    def execute(self, _):
        rospy.loginfo('Executing state Align_wall')
        rospy.wait_for_service("/align_wall")
        try:
            align_wall = rospy.ServiceProxy('/align_wall', AlignWall)  # make service file
            resp1 = align_wall(1)
            print('the out come of Align wall is: ',resp1.status)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return 'outcome2'
        return 'aligned_successfully'

#define state Move_along
class Move_Along(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['movement_complete'],input_keys=['move_block_size'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Move_Along')
        rospy.wait_for_service("/move_brick_length")
        try:
            align_wall = rospy.ServiceProxy('/move_brick_length', AlignWall)  # make service file
            resp1 = align_wall(userdata.move_block_size)
            print('move along wall status: ',resp1.status)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return 'movement_complete'
        return 'movement_complete'

#Define state Wait_For_Command
class Wait_For_Command(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['command_received','outcome2'],output_keys=['wait_block_size'])
        s = rospy.Service('CommandMisc', CommandMisc, self.command_received_cb)

    def execute(self, userdata):
        rospy.loginfo('Executing state Wait_For_Command')
        global command_misc_int
        global move_along_wall
        userdata.wait_block_size = 0
        while not move_along_wall:
            rospy.sleep(0.5)
            return 'outcome2'
        userdata.wait_block_size = command_misc_int
        move_along_wall = False
        return 'command_received'

    def command_received_cb(self,  data):
        print("I have recieved a misc command")
        global move_along_wall
        global command_misc_int
        if data.flag_name == "block":
            move_along_wall = True
            command_misc_int = data.flag_value_int
            print("sending block command")
            return 1
        return 0


def main():
    rospy.init_node('challenge_coordinator')

    #s = rospy.Service('CommandMisc', CommandMisc, command_received_cb)
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['challenge_terminated'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Send_Waypoints', Send_Waypoints(),
                               transitions={'pattern_found':'Chase_Wall', 'outcome2':'challenge_terminated'})
        smach.StateMachine.add('Chase_Wall', Chase_Wall(),
                               transitions={'wall_corner_reached':'Align_Wall', 'outcome2':'challenge_terminated'})
        smach.StateMachine.add('Align_Wall', Align_Wall(),
                               transitions={'aligned_successfully':'Wait_For_Command', 'outcome2':'challenge_terminated'})
        smach.StateMachine.add('Wait_For_Command', Wait_For_Command(),
                               transitions={'command_received':'Manipulator_Action', 'outcome2':'Wait_For_Command'},
                               remapping={'wait_block_size':'manip_block_size'})
        smach.StateMachine.add('Move_Along', Move_Along(),
                               transitions={'movement_complete':'Wait_For_Command'})
        smach.StateMachine.add('Manipulator_Action', Manipulator_Action(),
                               transitions={'manipulator_action_complete':'Move_Along'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('Challenge2', sm, '/ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
