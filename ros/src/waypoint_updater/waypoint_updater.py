#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.waypoints = None
        self.setspeed = (10.0)*(1.60934*1000/3600); # Mph to m/s
        self.trafficlight_status  = -1
        self.stop_distance = 3*self.setspeed*self.setspeed # roughly - TBD

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg
        #print 'pos: ' + str(self.pose.pose.position.x) + ',' + str(self.pose.pose.position.y) + ',' + str(self.pose.pose.position.z)

        if (self.waypoints is not None): # in case update comes before /base_waypoints
            num_wp = len(self.waypoints.waypoints)
            closestWPi = self.get_closest_waypoint()
            #print 'WP1: ' + str(self.waypoints.waypoints[closestWPi+1].pose.pose.position.x) + ',' + str(self.waypoints.waypoints[closestWPi+1].pose.pose.position.y) + ',' + str(self.waypoints.waypoints[closestWPi+1].pose.pose.position.z)

            wp2pub = []
            # assume track does not loop back to start
            startindex = min(closestWPi+1,num_wp)
            endindex = min(closestWPi+LOOKAHEAD_WPS,num_wp)
            #print 'distance',self.distance(self.waypoints.waypoints,startindex,self.trafficlight_status)
            for wpi in range(startindex,endindex):
                #if (((self.trafficlight_status-wpi)<STOP_NUM_WP)and((self.trafficlight_status-wpi)>=0)):
                if ((wpi<=self.trafficlight_status)and(self.distance(self.waypoints.waypoints,wpi,self.trafficlight_status)<self.stop_distance)):
                    self.waypoints.waypoints[wpi].twist.twist.linear.x = 0.0
                else:
                    self.waypoints.waypoints[wpi].twist.twist.linear.x = self.setspeed
                wp2pub.append(self.waypoints.waypoints[wpi])

            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time(0)
            lane.waypoints = wp2pub

            self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.trafficlight_status = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_closest_waypoint(self):
        """Identifies the closest path waypoint to the current pose position
        Returns: int: index of the closest waypoint in self.waypoints
        """
        best_dist = 9999.99
        best_i = -1
        if (self.pose is not None):
            dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
            for i in range(len(self.waypoints.waypoints)):
                this_dist = dl(self.pose.pose.position,self.waypoints.waypoints[i].pose.pose.position)
                if (this_dist<best_dist):
                    best_dist = this_dist
                    best_i = i
        else:
            rospy.logerr('Could not find current pose.')

        return best_i

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
