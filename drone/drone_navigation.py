#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist
from hector_uav_msgs.srv import EnableMotors
from get_drone_position import Block,State
import math


nodes = [[0.0,2.0,1],
        [7.0,1.0,1],
        [6.5,8.5,1],
        [-4.0,8.5,1],
        [-4.0,12.5,1],
        [-16.0,12.5,1],
        [-16.0,3.5,1,0],
        [-4.0,3.5,1,0]]


def angle_wrap(a):

    ## limit the angle between 0 and pi ##
     
    while(a>math.pi):
        a = a - 2*math.pi
    while(a <= -math.pi):
        a = a + 2*math.pi

    return a

def take_off(drone_N):
    # Starts a new node
    assert int(drone_N) in {1,2,3,4}

    rospy.init_node('move_drone{}'.format(drone_N))
    # check if motors are on
    if motor_on():
        print("control {}".format(drone_N))
        velocity_publisher = rospy.Publisher('drone{}/cmd_vel'.format(drone_N), Twist, queue_size=1)

        vel_msg = Twist()
        show = State()
        # Receiveing the user's input
        # speed = (input("Input your vertical_velocity: "))
        krho=0.5
        kalpha=0.5
        speed=0.1
        # distance = (input("Type your distance: "))

        take_off_distance=1
        vel_msg.linear.z = 0

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        while not rospy.is_shutdown():

            # Setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            current_distance = 0

            # Loop to move the turtle in an specified distance
            while abs(current_distance - take_off_distance)>0.2 and not rospy.is_shutdown():

                # Takes actual time to velocity calculus
                t1 = rospy.Time.now().to_sec()
                # Calculates distancePoseStamped
                position=show.show_drone_position(drone_N)
                current_distance=position[2]
                print(vel_msg)
                vel_msg.linear.z = krho*abs(current_distance - take_off_distance)
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0
                velocity_publisher.publish(vel_msg)
                print(vel_msg)

                # print("{},{}".format(resp_coordinates.pose.position.x,resp_coordinates.pose.position.y))

                # print(current_distance)

            # After the loop, stops the robot
            print("take off ready")

            for node in nodes:
                print("current target{}".format(node))
                position=show.show_drone_position(drone_N)
                relative_vector=[node[0]-position[0],node[1]-position[1]]
                distance=math.sqrt(relative_vector[0]**2+relative_vector[1]**2)
                AngleToGoal = angle_wrap(math.atan2(relative_vector[1], relative_vector[0])-position[3])
                    
                turn = kalpha * AngleToGoal
                while abs(AngleToGoal)>0.1 and not rospy.is_shutdown():
                    position=show.show_drone_position(drone_N)
                    relative_vector=[node[0]-position[0],node[1]-position[1]]
                    distance=math.sqrt(relative_vector[0]**2+relative_vector[1]**2)
                    AngleToGoal = angle_wrap(math.atan2(relative_vector[1], relative_vector[0])-position[3])
                    # print("{},{}".format(relative_vector[0],relative_vector[1]))

                    turn = kalpha * AngleToGoal
                    vel_msg.linear.x=0
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    vel_msg.angular.z=turn
                    velocity_publisher.publish(vel_msg)

                position=show.show_drone_position(drone_N)
                relative_vector=[node[0]-position[0],node[1]-position[1]]
                distance=math.sqrt(relative_vector[0]**2+relative_vector[1]**2)
                # Loop to move the turtle in an specified distance
                i=0
                while (distance>0.3) and not rospy.is_shutdown():
                    i+=1
                    position=show.show_drone_position(drone_N)
                    relative_vector=[node[0]-position[0],node[1]-position[1]]
                    distance=math.sqrt(relative_vector[0]**2+relative_vector[1]**2)
                    AngleToGoal = angle_wrap(math.atan2(relative_vector[1], relative_vector[0])-position[3])
                    # if(i%20==0):
                        # print("{},{}".format(relative_vector[0],relative_vector[1]))
                    turn = kalpha * AngleToGoal
                    vel_msg.linear.x=krho*distance
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    vel_msg.angular.x=0
                    vel_msg.angular.y=0
                    vel_msg.angular.z=turn
                    # print(vel_msg)
                    velocity_publisher.publish(vel_msg)

        else:
            return 0


# Rosservice function to turn on the drone motors
def motor_on():
    rospy.wait_for_service('drone{}/enable_motors'.format(drone_N))
    try:
        motor_on = rospy.ServiceProxy('drone{}/enable_motors'.format(drone_N), EnableMotors, True)
        turn_on = motor_on(True)
        return turn_on
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


if __name__ == '__main__':
    # Receiveing the user's input
    print("Let's move your drone")
    # drone_N = input("Select a drone to move. (Options 1, 2, 3, 4): ")
    # Testing our function
    drone_N=2
    
    take_off(drone_N)
