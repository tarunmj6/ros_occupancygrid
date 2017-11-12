import roslib; roslib.load_manifest('no_weights')
import rospy
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

MAP_WIDTH  = 53.9
MAP_HEIGHT = 10.6
P0X = MAP_WIDTH  / 2.0
P0Y = MAP_HEIGHT / 2.0 
P0T = 1.57079637051

pose = (0.0, 0.0, 0.0)

def got_odom(msg):
    global pose

    p0 = msg.pose.pose.position
    (sy, sx) = (p0.x, -p0.y)
    q0 = msg.pose.pose.orientation
    q1 = (q0.x, q0.y, q0.z, q0.w)
    (_, _, st) = euler_from_quaternion(q1)

    gx = sx + P0X
    gy = sy + P0Y
    gt = st + P0T

    pose = (gx, gy, gt)

def get_pose():
    return pose
