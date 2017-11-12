#!/usr/bin/env python
import roslib; roslib.load_manifest('with_weights')
import rospy
rospy.init_node('with_weights')
import mcl_tools

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import time
import random

PAR_COUNT = 1500
cmd_vel = None

# State
parset   = [mcl_tools.random_particle() for ii in range(PAR_COUNT)]

def particle_weight(particle, scan):
    # FIXME: You should assign weight.
    return 1.0

def particle_filter(ps, control, scan):
    # FIXME: Should really particle filter.
    return [mcl_tools.random_particle() for ii in range(PAR_COUNT)]

def got_scan(msg):
    global parset
    global cmd_vel

    control = (0.0, 0.0)

    parset = particle_filter(parset, control, msg)
    mcl_tools.show_particles(parset)

    cmd = Twist()
    (cmd.linear.x, cmd.angular.z) = control
    cmd_vel.publish(cmd)

if __name__ == '__main__':
    # Uncomment for debugging if nesssary, recomment before turning in.
    #rospy.Subscriber('/stage/base_pose_ground_truth', Odometry, mcl_debug.got_odom)

    rospy.Subscriber('/robot/base_scan', LaserScan, got_scan)
    cmd_vel = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)

    mcl_tools.mcl_init('with_weights')
    mcl_tools.mcl_run_viz()
