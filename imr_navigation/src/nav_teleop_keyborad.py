#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
import sys, select, termios, tty

#this method will make the robot move to the goal location
def move_to_goal(xGoal,yGoal):

   #define a client for to send goal requests to the move_base server through a SimpleActionClient
   ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

   #wait for the action server to come up
   while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
           rospy.loginfo("Waiting for the move_base action server to come up")

   goal = MoveBaseGoal()
   
   
   #set up the frame parameters
   goal.target_pose.header.frame_id = "map"
   goal.target_pose.header.stamp = rospy.Time.now()

   # moving towards the goal*/

   goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
   goal.target_pose.pose.orientation.x = 0.0
   goal.target_pose.pose.orientation.y = 0.0
   goal.target_pose.pose.orientation.z = 0.0
   goal.target_pose.pose.orientation.w = 1.0

   print(goal.target_pose.pose.position.x)
   print(goal.target_pose.pose.position.y)
   xPrev = -1
   yPrev = -1
   if xGoal != xPrev or yGoal != yPrev:
      print("Inside")
      ac.send_goal(goal)
      xPrev = xGoal 
      yPrev = yGoal
      #ac.wait_for_result(rospy.Duration(10))

   rospy.loginfo("Sending goal location ...")

   print("Goal Send")

   #ac.wait_for_result(rospy.Duration(10))
   print("Waited")

   if(ac.get_state() ==  GoalStatus.SUCCEEDED):
           rospy.loginfo("You have reached the destination")
           return True

   else:
           rospy.loginfo("The robot failed to reach the destination")
           return False

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist: 
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == '__main__':
   rospy.init_node('map_navigation', anonymous=False)
   settings = termios.tcgetattr(sys.stdin)
   x_goal = 0.5
   y_goal = 0.5
   print ('start go to goal')
   key_timeout = 0.5
   cord = {
   "A": "0 2",
   "B": "0 5",
   "C": "0 8",
   "D": "3.5 8",
   "E": "9 9",
   "F": "9 6.5",
   "G": "9 1.5",
   "S": "0 0",
   }
   while(1):
        key = getKey(key_timeout)
        key = key.upper()
        if key != "":
            x, y = map(float,cord[key].split())
            print("Going to Place-", key)
            move_to_goal(x, y)
            print("Reached ", key)
            key = ""
        
   
   move_to_goal(x_goal,y_goal)
   rospy.spin()
