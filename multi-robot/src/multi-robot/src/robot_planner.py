#!/usr/bin/env python

'''
Plan husky robot from one position to another position
'''

from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist
import math
import numpy as np
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,Pose


def angle_wrap(a):
    ## limit the angle between -pi and pi ##

    while (a > math.pi):
        a = a - 2 * math.pi
    while (a <= -math.pi):
        a = a + 2 * math.pi

    return a


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


class movement:
    def __init__(self, robot_N):
        self.position = np.array([0, 0, 0, 0])
        self.rate = rospy.Rate(10)
        self.velocity_publisher2 = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        self.goals=[]

        self.vel_msg = Twist()
        self.robot_N = robot_N
        self.track_flag = False
        self.default_pose_flag = True
        self.wait = True
        self.isnode = False

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/depth/image_raw', Image, self.image_callback)
        self.pose_sub=rospy.Subscriber('robot/position', Pose,self.pose_callback)
        self.target_sub=rospy.Subscriber("robot/goal",Point,self.target_callback)

    def target_callback(self, point):
        self.goals.append([point.x,point.y,point.z])
        self.isnode=True
        print("husky{} get person information".format(self.robot_N))

    def pose_callback(self, pose_msg):
        x=pose_msg.position.x
        y=pose_msg.position.y
        z=pose_msg.position.z

        rotq = pose_msg.orientation
        angle = euler_from_quaternion(rotq.x, rotq.y, rotq.z, rotq.w)
        self.position = np.array([x, y, z, angle[2]])

    def image_callback(self,msg):
        # BEGIN BRIDGE
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
        self.flag = 0
        
        self.close = 1
        width = self.image.shape[1]
        detection_width = 160

        start_row=200
        end_row=300

        #middle obstacle detection
        if np.isnan(self.image[255][width/2-detection_width/2:width/2+detection_width/2]).sum()==detection_width:
            self.flag = self.flag
        else:
            self.flag = self.flag + 1
        
        left_region=np.isnan(self.image[start_row:end_row,:width//2]).sum()
        right_region=np.isnan(self.image[start_row:end_row,width//2:]).sum()
        if left_region>right_region: # black area
            self.left = 1
        else:
            self.left = -1


    def update_velocity(self, v_x, v_y, v_z, av_x, av_y, av_z):
        self.vel_msg.linear.x = v_x
        self.vel_msg.linear.y = v_y
        self.vel_msg.linear.z = v_z
        self.vel_msg.angular.x = av_x
        self.vel_msg.angular.y = av_y
        self.vel_msg.angular.z = av_z
        self.velocity_publisher2.publish(self.vel_msg)
        self.rate.sleep()

    def move(self, robot_N):
        # Starts a new node
        assert int(robot_N) in {1, 2, 3, 4}

        # Receiveing the user's input
        krho = 1
        kalpha = 1

        while not rospy.is_shutdown():
            '''
            Robot plan a trajectory to arrive at goal
            '''
            print("Init successful !! husky status : wait ->{}".format(self.wait))
            # verify the robot if vacant. If vacant, accept
            while True:
                if self.wait == True and self.isnode == True:
                    self.wait = False
                    # print("control husky {}".format(robot_N), "husky status : wait ->{}".format(self.wait))

                    goals=self.goals
                    for node in goals:
                        
                    # ## The robot plan a trajectory ##
                        print("husky{} current target{}".format(robot_N,node))
                    # print("move to target point")

                    # ## turn to direction of target ##
                        pose = self.position
                        error = (node - pose[0:3])[0:2]
                        AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
                        while abs(AngleToGoal) > 0.2 and not rospy.is_shutdown():
                            error = (node - pose[0:3])[0:2]
                            AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
                            av_z = kalpha * AngleToGoal
                            self.update_velocity(0, 0, 0, 0, 0, av_z)
                            pose = self.position

                    # move the husky to the target point
                        pose = self.position
                        while (np.linalg.norm((node - pose[0:3])[0:2]) > 5) and not rospy.is_shutdown():
                            error = (node - pose[0:3])[0:2]
                            AngleToGoal = 0
                            goalDist = 0
                            if self.flag ==0:
                                AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
                                goalDist = np.linalg.norm(error)
                            else:
                                AngleToGoal = 0.5*self.left
                                goalDist = 1
                                v_x = krho * goalDist
                                av_z = kalpha * AngleToGoal
                                t0 = rospy.Time.now().to_sec()
                                t1 = t0
                                while (t1 - t0) < 1:
                                    self.update_velocity(v_x, 0, 0, 0, 0, av_z)
                                    t1 = rospy.Time.now().to_sec()
                                while (t1 - t0) < 3:
                                    v_x = 1
                                    av_z = 0
                                    self.update_velocity(v_x, 0, 0, 0, 0, av_z)
                                    t1 = rospy.Time.now().to_sec()

                            if self.close ==0:
                                t0 = rospy.Time.now().to_sec()
                                t1 = t0
                                while (t1 - t0) < 1:
                                    v_x = -1
                                    av_z = 0
                                    self.update_velocity(v_x, 0, 0, 0, 0, av_z)
                                    t1 = rospy.Time.now().to_sec()

                            v_x = krho * goalDist
                            av_z = kalpha * AngleToGoal
                            self.update_velocity(v_x, 0, 0, 0, 0, av_z)
                            pose = self.position

                        print("[robot] robot {} arrives at person".format(robot_N))
                        self.goals.remove(self.goals[0])
                            
                    self.wait = True
                    self.isnode = False

                        
        # rospy.spin()


if __name__ == '__main__':

    # Testing our function
    rospy.init_node('move_husky', anonymous=True)
    robot_N = rospy.get_param('~robot_N')
    # print("Let's move your husky{}".format(robot_N))
        
    move = movement(robot_N)
    move.move(robot_N)
