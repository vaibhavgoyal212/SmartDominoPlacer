import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist 
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
import time
import numpy as np

# Apply transformation matrix to app coordinates
def parseAppInput(x : float, y : float, z : float):
    transformedX = (x/20.2) - 10
    transformedY = 10 - (y/20.2)
    #return coordinateMatrix @ transformationMatrix
    print("transformedX:{}, tranformedY:{}".format(transformedX,transformedY))
    return transformedX,transformedY

# Publish goal to appropriate topic to move turtlebot, Assuming coordinates are valid
def send_goal_to_publisher(end_x : float,end_y : float,end_orientation : float):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()
   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move x meters forward along the x axis of the "map" coordinate frame and similarly for y 
    goal.target_pose.pose.position.x = end_x
    goal.target_pose.pose.position.y = end_y
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = end_orientation
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
    return client.get_result()

def publish_goal(end_x : float,end_y : float,end_orientation : float):
    rospy.init_node('move_turtlebot')
    return send_goal_to_publisher(end_x,end_y,end_orientation)

# Move motor at specified speed and angle, with optional time (still need to implement)
def move_motor(fwd,ang,period = 0):
    pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10) 
    mc = Twist()
    mc.linear.x = fwd
    mc.angular.z = ang 
    pub.publish(mc)

# Calibrate turtlebot to map given  
def spin():
    rospy.init_node('spin',anonymous=True) 
    start_time = time() 
    duration = 3 #in seconds 
    forward_speed = 0 
    turn_speed = 1 
    while time()<start_time+duration: 
        try: 
            move_motor(forward_speed,turn_speed,0) 
        except rospy.ROSInterruptException: 
            pass 
    else: 
        move_motor(0,0,0)


if __name__ == "__main__": 
    spin()
    transformedX, transformedY = parseAppInput(0.0,0.0,0.0)
    publish_goal(transformedX,transformedY,0.0)
