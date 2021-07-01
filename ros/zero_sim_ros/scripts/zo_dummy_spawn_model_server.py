#!/usr/bin/env python

from __future__ import print_function

from zero_sim_ros.srv import ZOSimSpawn, ZOSimSpawnRequest, ZOSimSpawnResponse
import rospy

def handle_zosim_spawn(req):
    print("INFO: handle_zosim_spawn")
    return ZOSimSpawnResponse(success=True, status_message="all good from dummy spawn model server")

def main():
    rospy.init_node('dummy_spawn_model_server')
    s = rospy.Service('zerosim/spawn_zosim_model', ZOSimSpawn, handle_zosim_spawn)
    print("INFO: Ready to spawn")
    rospy.spin()

if __name__ == "__main__":
    main()