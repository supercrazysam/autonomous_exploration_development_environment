#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


rospy.init_node("insert_object")

initial_pose = Pose()
initial_pose.position.x = 1
initial_pose.position.y = 1
initial_pose.position.z = 1

f = open('/home/big/catkin_ws/src/autonomous_exploration_development_environment/src/vehicle_simulator/peds/ped.sdf','r')
sdff = f.read()

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
spawn_model_prox("some_robo_name", sdff, "robotos_name_space", initial_pose, "world")
