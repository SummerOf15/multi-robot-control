#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from hector_uav_msgs.srv import EnableMotors
from get_drone_position import Block,State
import math


nodes=[[-1,2,0],[-16,2,0],[-27,2,0],[-27,11,0],[-18,11,0],[-5,11,0],[-5,6,0],[7,6,0],[7,2,0],[1,2,0]]
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
        speed=0.5
        alpha=0.1
        # distance = (input("Type your distance: "))

        distance=1.5
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
            while abs(current_distance - distance)>0.05 and not rospy.is_shutdown():

                # Takes actual time to velocity calculus
                t1 = rospy.Time.now().to_sec()
                # Calculates distancePoseStamped
                resp_coordinates=show.show_drone_position(drone_N)
                current_distance=resp_coordinates.pose.position.z

                vel_msg.linear.z = alpha*abs(current_distance - distance)
                velocity_publisher.publish(vel_msg)

                # print("{},{}".format(resp_coordinates.pose.position.x,resp_coordinates.pose.position.y))

                # print(current_distance)

            # After the loop, stops the robot
            
            vel_msg.linear.z = 0.
            # Force the robot to stop
            velocity_publisher.publish(vel_msg)
            print("ccc")

            for node in nodes:
                print("current target{}".format(node))
                resp_coordinates=show.show_drone_position(drone_N)
                relative_vector=[(node[0]-resp_coordinates.pose.position.x),node[1]-resp_coordinates.pose.position.y]
                distance=math.sqrt(relative_vector[0]**2+relative_vector[1]**2)
                
                vel_msg.linear.x=speed*relative_vector[0]
                vel_msg.linear.z = 0
                vel_msg.linear.y = speed*relative_vector[1]
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0


                # Loop to move the turtle in an specified distance
                while (distance>0.1) and not rospy.is_shutdown():

                    resp_coordinates=show.show_drone_position(drone_N)
                    relative_vector=[(node[0]-resp_coordinates.pose.position.x),node[1]-resp_coordinates.pose.position.y]
                    
                    distance=math.sqrt(relative_vector[0]**2+relative_vector[1]**2)
                    vel_msg.linear.x=speed*relative_vector[0]
                    vel_msg.linear.y = speed*relative_vector[1]

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
    drone_N=1
    
    take_off(drone_N)
    
