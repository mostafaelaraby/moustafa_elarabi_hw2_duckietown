#!/usr/bin/env python
import collections
import threading
import math
import time
import itertools
import numpy as np
import rospy
from duckietown_msgs.msg import (BoolStamped, FSMState, LanePose,
                                 Segment, SegmentList, StopLineReading,
                                 Twist2DStamped, WheelsCmdStamped)
from geometry_msgs.msg import Point

class PurePursuitNode(object):

    def __init__(self):
        self.node_name = rospy.get_name()

        ## Publishers
        #used to publish v and omega
        self.publishing_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        ## Subscribers
        # getting lane pose
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.handle_pose, queue_size=1)

        # getting segment list having white yellow  and red
        self.sub_seglist_filtered = rospy.Subscriber("~seglist_filtered", SegmentList, self.adding_new_segments, queue_size=1)

        # length of data buffer holding points
        buffer_segment_points_default = 30
        self.buffer_segment_points = self.setBotParam("~buffer_segment_points", buffer_segment_points_default) # keep 50 points at a time for each color.
        self.segment_points = collections.defaultdict(lambda: collections.deque(maxlen=self.buffer_segment_points))
        
        lookahead_dist_default = 0.3
        self.lookahead_dist = self.setBotParam("~lookahead_dist", lookahead_dist_default)
        
        max_speed_default = 0.32
        self.max_speed = self.setBotParam("~max_speed", max_speed_default)

        distance_offset_default = 0.1
        self.distance_offset = self.setBotParam("~distance_offset", distance_offset_default)
        
        # time to update bot motion
        update_interval_default  = 0.1
        self.update_interval = self.setBotParam('update_interval', update_interval_default)
        self.control_car_cmd_timer = rospy.Timer(rospy.Duration.from_sec(self.update_interval), self.control_car_cmd)

        self.segments_semaphore =  threading.Semaphore() # used to avoid having multiple threads extending the same list of segments at the same time
        self.header = None
        self.current_pose_message = None
        self.data_history_semaphore = threading.Semaphore()
        self.data_history = []
        rospy.on_shutdown(self.onShutdown)
        self.loginfo('Pure Pursuit Node initialized')
        
    def control_car_cmd(self,event):
        """
        contains pure pursuit algorithm 
        """
        self.logdebug('Updating car command based on segments')
        self.segments_semaphore.acquire()
        yellow_segment_points_count = len(self.segment_points[Segment.YELLOW])
        white_segment_points_count = len(self.segment_points[Segment.WHITE])
        yellow_points = self.segment_points[Segment.YELLOW]
        white_points = self.segment_points[Segment.WHITE]
        self.segments_semaphore.release()
        centroid = None
        found_lanes = True
        if yellow_segment_points_count> 0 :
            # then we have yellow points lane to follow
            centroid = self.compute_segment_centroids(yellow_points)
        elif white_segment_points_count >0:
            #then we have white solid color
            white_centroid = self.compute_segment_centroids(white_points, medoid=False)
            distance_offset = self.distance_offset
            if white_centroid.y>0:
                # we have the white line on the right
                self.logwarn('could not find any yellow line just white line to the right !')
                distance_offset *= -1
            else:
                # we have the white line to the left
                self.logwarn('could not find any yellow line just white line to the left !')
            centroid = white_centroid
            centroid.y += distance_offset
        else:
            self.logwarn('The bot cannot find white nor yellow lanes to follow')
            found_lanes = False 
        
        if found_lanes:
            centroid_np = np.array([centroid.x, centroid.y]) 
            distance_to_target =  np.sqrt(centroid_np.dot(centroid_np)) # sqrt((x - 0)^2 + (y - 0)^2) to get the distance to the target point
            omega = centroid.y / distance_to_target
            omega /=self.lookahead_dist
            self.logdebug('Current omega value {}'.format(omega))
            self.publish_car_speed_omega(self.max_speed, omega)

        else:
            pass #TODO try to think of a suitable scenario in that case


    def compute_segment_centroids(self, segment_points, medoid=True):
        # using medoid to get the centroids to avoid outliers effect on centroid calculation
        if medoid:
            self.segments_semaphore.acquire()
            # distance between ponts will be euclidean O(n^2) needs to be optimized
            dist_matrix =  np.zeros((len(segment_points),len(segment_points)))

            for current_indx,first_point in enumerate(segment_points):
                for other_indx in range(current_indx, len(segment_points)):
                    points_dist = abs(first_point.x - segment_points[other_indx].x) + abs(first_point.y - segment_points[other_indx].y) # manhattan distance as a start
                    dist_matrix[current_indx][other_indx]  = points_dist
                    dist_matrix[other_indx][current_indx] = points_dist
            self.segments_semaphore.release()
            medoid_point = segment_points[np.argmin(dist_matrix.sum(axis=0))]
            return medoid_point
        x_centroid = np.mean([p.x for p in segment_points])
        y_centroid = np.mean([p.y for p in segment_points])
        centroid = Point(x=x_centroid, y=y_centroid)
        return centroid


    def publish_car_speed_omega(self, v , w):
        self.logdebug('Sending command to bot with v={} and w={}'.format(v, w))
        car_cmd_message = Twist2DStamped()
        if self.header is not None:
            car_cmd_message.header = self.header
        car_cmd_message.v = v
        car_cmd_message.omega = w
        self.publishing_car_cmd.publish(car_cmd_message)
        # storing command
        self.store_history(v, w)
    
    def store_history(self, v, w):
        # used to store current linear and angular velocities for plotting later
        if self.current_pose_message is not None:
            self.data_history_semaphore.acquire()
            angle_error = self.current_pose_message.phi
            crosstrack_error = self.current_pose_message.d
            self.data_history.append([str(v), str(w), str(crosstrack_error), str(angle_error)])
            self.data_history_semaphore.release()

    def handle_pose(self, pose_msg):
        self.header = pose_msg.header
        self.current_pose_message = pose_msg

    def adding_new_segments(self, inlier_segments_msg):
        self.header = inlier_segments_msg.header
        segments = inlier_segments_msg.segments
        self.logdebug("Received {} new segments".format(len(segments)))
        for  segment in segments:
            color = segment.color
            assert color in [Segment.RED, Segment.YELLOW, Segment.WHITE]
            self.segments_semaphore.acquire() # trying to acquire the semaphore unless another thread is holding it
            if color in self.segment_points:
                self.segment_points[color].extend(segment.points)   
            else:
                self.segment_points[color] = segment.points
            self.segments_semaphore.release() # releasing semaphore for others to use it
       
    def onShutdown(self):        
        # saving list of data points into a file for plotting 
        self.loginfo('###############################')
        self.loginfo('\t'.join(['V', 'W', 'Cross track', 'Angle Error']))
        for p in self.data_history:
            self.loginfo('\t'.join(p))
        self.loginfo('###############################')
        self.loginfo('Pure Pusuit Lane Following shutting down')
        rospy.loginfo("[LaneFilterNode] Shutdown.")

        # Stop listening
        self.sub_lane_pose.unregister()
        self.sub_seglist_filtered.unregister()


        # Send stop command
        self.publish_car_speed_omega(0.0,0.0)
        rospy.sleep(1) # wait for everything to get closed and published
        self.loginfo("Shutdown")

    def logdebug(self, s):
        rospy.logdebug("[{}] {}".format(self.node_name, s))

    def logwarn(self, s):
        rospy.logwarn("[{}] {}".format(self.node_name, s))

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))

    def setBotParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value) 
        return value

if __name__ == '__main__':
    rospy.init_node('pure_pursuit', anonymous=False)
    lane_filter_purepursuit = PurePursuitNode()
    rospy.on_shutdown(lane_filter_purepursuit.onShutdown)
rospy.spin()
