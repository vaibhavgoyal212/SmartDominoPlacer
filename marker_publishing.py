import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import time

class MarkerPublisher:

    """
    Set default values for position of the TurtleBot
    """
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    """
    Callback for subscriber function
    """
    def odom_callback(self,data : Odometry):
        #print("x:{}.y:{},z:{}".format(data.pose.pose.position.x, data.pose.pose.position.y,data.pose.pose.position.z))
        if self.x + self.y + self.z == 0.0: 
            self.change_values(data.pose.pose.position.x, data.pose.pose.position.y,data.pose.pose.position.z)

    def change_values(self,x : float,y : float,z : float):
        self.x = x
        self.y = y
        self.z = z
    
    """
    Subscriber to odom topic
    """
    def read_odom_data(self): 
        rospy.Subscriber('/odom',Odometry,self.odom_callback) 
    

"""
Sets and publishes coordinates for the region that the turtlebot should explore
"""
def publish_coordinates(x : float,y : float,z : float): 
        marker_pub = rospy.Publisher("/clicked_point",PointStamped, queue_size = 10000)
        while not rospy.is_shutdown():
            points = []
            point1 = PointStamped()
            point2 = PointStamped()
            point3 = PointStamped()
            point4 = PointStamped()
            point5 = PointStamped()
            point1.point.x,point1.point.y,point1.point.z = -5.0,5.0,-0.0
            point2.point.x,point2.point.y,point2.point.z = -5.0,-5.0,0.0
            point3.point.x,point3.point.y,point3.point.z = 5.0,-5.0,0.0
            point4.point.x,point4.point.y,point4.point.z = 5.0,5.0,0.0
            point5.point.x,point5.point.y,point5.point.z = x,y,z
            points = [point3,point4,point1,point2,point5]
            for point in points:
                point.header.frame_id = "map"
                marker_pub.publish(point)
                time.sleep(2)
            break
  
if __name__ == '__main__': 
    rospy.init_node('marker_publisher',anonymous=True)
    publisher = MarkerPublisher()
    while publisher.x + publisher.y + publisher.z == 0:
        publisher.read_odom_data()
    publish_coordinates(publisher.x,publisher.y,publisher.z)
    publisher.publish_coordinates()

# EXAMPLE OF MARKER MESSAGE
#     header: 
#   seq: 1500494
#   stamp: 
#     secs: 0
#     nsecs:         0
#   frame_id: "map"
# ns: "markers"
# id: 0
# type: 8
# action: 0
# pose: 
#   position: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   orientation: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
#     w: 1.0
# scale: 
#   x: 0.3
#   y: 0.3
#   z: 0.0
# color: 
#   r: 1.0
#   g: 0.0
#   b: 0.0
#   a: 1.0
# lifetime: 
#   secs: 0
#   nsecs:         0
# frame_locked: False
# points: 
#   - 
#     x: 5.015308380126953
#     y: -3.0279018878936768
#     z: 0.003101348876953125
# colors: []
# text: ''
# mesh_resource: ''
# mesh_use_embe^C
# color: 
#   r: 1.0
#   g: 0.0
#   b: 0.0
#   a: 1.0
# lifetime: 
#   secs: 0
#   nsecs:         0
# frame_locked: False
# points: 
#   - 
#     x: 5.015308380126953
#     y: -3.0279018878936768
#     z: 0.003101348876953125
# colors: []
# text: ''
# mesh_resource: ''
# mesh_use_embedded_materials: False

