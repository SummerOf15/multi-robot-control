#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from hector_uav_msgs.srv import EnableMotors
import time
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


drone_N=1
rospy.init_node('move_drone{}'.format(drone_N))
rate = rospy.Rate(10)


def takeoff(drone_N):
    velocity_publisher = rospy.Publisher('drone{}/cmd_vel'.format(drone_N), Twist, queue_size=1, latch=True)
    time.sleep(1)
    # Starts a new node
    assert int(drone_N) in {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}

    # check if motors are on
    if motor_on():

        vel_msg = Twist()

        # Receiveing the user's input
        speed_z=0.5
        distance = 2

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0.5

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        try:
            # Loop to move the turtle in an specified distance
            while (current_distance < distance) and not rospy.is_shutdown():
                # Publish the velocity
                velocity_publisher.publish(vel_msg)
                # Takes actual time to velocity calculus
                t1 = rospy.Time.now().to_sec()
                # Calculates distancePoseStamped
                current_distance = speed_z * (t1 - t0)
                # 
                # print(velocity_publisher.get_num_connections())
            # After the loop, stops the robot
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.1
            # Force the robot to stop
            velocity_publisher.publish(vel_msg)
        except KeyboardInterrupt:
            print('Interrupted')
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.1
            # Force the robot to stop
            velocity_publisher.publish(vel_msg)
            sys.exit(0)



class Block:
    def __init__(self, name):
        self._name = name


class Robot:
    def __init__(self, number, x, y, z):
        self.name = number
        self.x = x
        self.y = y
        self.z = z


class Goal:
    def __init__(self, goalListX, goalListY, retry):
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)   
        self.odom_sub=rospy.Subscriber("drone1/ground_truth/state", Odometry,self.distance_callback)
        # params & variables
        self.goalListX = goalListX
        self.goalListY = goalListY
        self.retry = retry
        
        self.goalId = 0

        self.goalMsg = PoseStamped()
        self.goalMsg.header.frame_id = "world"
        self.goalMsg.pose.orientation.x=0.0
        self.goalMsg.pose.orientation.y=0.0
        self.goalMsg.pose.orientation.z=0.0
        self.goalMsg.pose.orientation.w=1.0
        self.state=State()
        
        self.nextGoal=False

        rospy.spin()


    def distance_callback(self,data):

        # Publish the first goal
        x=data.pose.pose.position.x
        y=data.pose.pose.position.y
        distance=(x-self.goalListX[self.goalId])**2+(y-self.goalListY[self.goalId])**2
        
        # rospy.loginfo("x={},y={},distance: {}".format(x,y,distance))
        if self.pub.get_num_connections() > 0:
            if distance<1.5:
                self.goalId = self.goalId + 1
                self.goalMsg.header.stamp = rospy.Time.now()
                self.goalMsg.pose.position.x = self.goalListX[self.goalId]
                self.goalMsg.pose.position.y = self.goalListY[self.goalId]
                self.goalMsg.pose.position.z = 0.0
            
                self.pub.publish(self.goalMsg) 
                rospy.loginfo("Initial goal published! Goal ID is: %d", self.goalId)
                
                rospy.loginfo("[UPDATE]update goal id ({},{})".format(self.goalListX[self.goalId],self.goalListY[self.goalId]))
                time.sleep(5)
            else:
                self.nextGoal=False
        # rate.sleep()        
            




class State():

    _blockListDict = {
        'block_a': Block('drone1')
        }

    def __init__(self):
        pass

    def get_positions(self):
        data = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        
        blockName = "drone{}".format(drone_N)
        if blockName in data.name:
            drone_index = data.name.index(blockName)
            current_drone = Robot(number=int(blockName[-1]),
                                    x=data.pose[drone_index].position.x,
                                    y=data.pose[drone_index].position.y,
                                    z=data.pose[drone_index].position.z)

            return current_drone.x, current_drone.y, current_drone.z


def motor_on():
    rospy.wait_for_service('drone{}/enable_motors'.format(drone_N))
    try:
        motor_on = rospy.ServiceProxy('drone{}/enable_motors'.format(drone_N), EnableMotors, True)
        turn_on = motor_on(True)
        return turn_on
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

class Move:
    def __init__(self):
        self.pub=rospy.Publisher("/drone1/cmd_vel", Twist, queue_size=1)
        self.sub=rospy.Subscriber("/twist_cmd", Twist, self.sub_callback)

        self.msg=Twist()

    def sub_callback(self,data):
        self.msg=data
        self.msg.linear.z=0.0
        self.pub.publish(self.msg)
        rospy.spin()    
        

if __name__=="__main__":
    goalListX=[-32,-32,-1.0,-1.0,-0.1]
    goalListY=[-32,-16,-16,-17.0,0.0]
    retry=0

    takeoff(drone_N)
    
    # Starts a new node
    g=Goal(goalListX,goalListY,retry)
    # # m=Move()
    # while(True):
    #     g.publish_goal()
            

      