#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState
import rospy
import math


class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

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
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians


class State:

    def __init__(self):
        
        self._blockListDict = {
            # 'block_a': Block('drone1', 'base_link'),
            # 'block_b': Block('drone2', 'base_link'),
            # 'block_c': Block('drone3', 'base_link'),
            # 'block_d': Block('drone4', 'base_link'),
            'block_a': Block('drone1', ''),
            'block_b': Block('drone2', ''),
            'block_c': Block('drone3', ''),
            'block_d': Block('drone4', ''),
        }

    def show_gazebo_models(self):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            for block in self._blockListDict.itervalues():
                blockName = str(block._name)
                resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
                print('\n')
                print('State success = ', resp_coordinates.success)
                print(blockName)
                print("Position x: " + str(resp_coordinates.pose.position.x))
                print("Position y: " + str(resp_coordinates.pose.position.y))
                print("Position z: " + str(resp_coordinates.pose.position.z))
                rotq = resp_coordinates.pose.orientation
                angle = euler_from_quaternion(rotq.x, rotq.y, rotq.z, rotq.w)
                print("roll: "+str(angle[0]))
                print("pitch: "+str(angle[1]))
                print("yaw: " + str(angle[-1]))

        except rospy.ServiceException as e:
            rospy.logerr("Get Model State service call failed: {0}".format(e))
            raise e
    
    def show_drone_position(self, drone_id):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            blockName = "drone"+str(drone_id)
            resp_coordinates = model_coordinates(blockName, '')
            rotq = resp_coordinates.pose.orientation
            angle = euler_from_quaternion(rotq.x, rotq.y, rotq.z, rotq.w)
            return resp_coordinates.pose.position.x,resp_coordinates.pose.position.y,resp_coordinates.pose.position.z,angle[-1]
        except rospy.ServiceException as e:
            rospy.logerr("Get Model State service call failed: {0}".format(e))
            raise e

if __name__ == '__main__':
    show = State()
    show.show_gazebo_models()
    pos=show.show_drone_position(2)
    print(pos)