#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState
import rospy

class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class State:

    _blockListDict = {
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
        except rospy.ServiceException as e:
            rospy.logerr("Get Model State service call failed: {0}".format(e))
            raise e
    
    def show_drone_position(self, drone_id):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            blockName = "drone"+str(drone_id)
            resp_coordinates = model_coordinates(blockName, '')
            return resp_coordinates
        except rospy.ServiceException as e:
            rospy.logerr("Get Model State service call failed: {0}".format(e))
            raise e

if __name__ == '__main__':
    show = State()
    show.show_gazebo_models()