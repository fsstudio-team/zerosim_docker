#!/usr/bin/env python
"""[summary]
Spawns a .zosim model in ZeroSim ROS.
Spawn ZOSim file Example:

rosrun zero_sim_ros zo_spawn_model.py --spawn_zosim \
    --input_zosim_file=/home/micahpearlman/dev/work/zero-sim/zero-sim/unity/zero-sim-unity/Assets/ZOSim/Assets/ZeroSimAssets/ZOSimSimpleBot.zosim \
    --model_name="zero_dog" \
    --pose_z=1

"""
import rospy
import os
import sys
import argparse
import json

from zero_sim_ros.srv import ZOSimSpawn, ZOSimSpawnRequest, ZOSimSpawnResponse
from zero_sim_ros.srv import ZOSimPrefabSpawn, ZOSimPrefabSpawnRequest, ZOSimPrefabSpawnResponse
from zero_sim_ros.srv import ZOSimDeleteModel, ZOSimDeleteModelRequest, ZOSimDeleteModelResponse
from geometry_msgs.msg import Point, Pose, Quaternion, Twist

import tf.transformations as tft

# from typing import Dict


# python3 def zo_spawn_zosim_model(model_name: str, zosim_data: Dict, initial_pose: Pose, zosim_namespace: str) -> bool:
def zo_spawn_zosim_model(model_name, zosim_data, initial_pose, zosim_namespace):
    """[summary]
    Spawns a .zosim model in ZeroSim ROS.

    Args:
        model_name (str): Unique name for the spawned object.
        zosim_data (Dict): .zosim JSON data in Dictionary format.
        initial_pose (Pose): Initial pose.
        zosim_namespace:  ZO Sim namespace the service is being advertised on
    """
    full_service_name = zosim_namespace + "/spawn_zosim_model"
    print("INFO: Waiting for service: %s" %  full_service_name)
    # rospy.wait_for_service(zosim_namespace+"/spawn_zosim_model")
    rospy.wait_for_service(full_service_name)
    try:
        spawn_zosim_model = rospy.ServiceProxy(full_service_name, ZOSimSpawn)
        print("INFO: Calling service %s" %
                      full_service_name)
        response = spawn_zosim_model(model_name=model_name,
                                     model_zosim=json.dumps(zosim_data),
                                     robot_namespace="", # TODO
                                     initial_pose=initial_pose,
                                     reference_frame="map")
        print("INFO: Spawn status: %s" % response.status_message)
        return response.success
    except rospy.ServiceException as e:
        print("ERROR: spawn_zosim_model service call failed: %s" % e)
    except rospy.ROSSerializationException as e:
        print("ERROR: spawn_zosim_model service call serialization failed: %s" % e)



def zo_spawn_prefab_model(model_name, model_prefab_name, unity_asset_bundle, initial_pose, zosim_namespace):
    """[summary]
    Spawns a Unity prefab model in ZeroSim ROS.

    Args:
        model_name (str): Unique name for the spawned object.
        model_prefab_name (str): Name of the prefab to spawn        
        initial_pose (Pose): Initial pose.
        unity_asset_bundle: Name of the asset bundle to use
        zosim_namespace:  ZO Sim namespace the service is being advertised on
    """
    full_service_name = zosim_namespace + "/spawn_prefab_model"

    print("INFO: Waiting for service: %s" %  full_service_name)
    rospy.wait_for_service(full_service_name)

    try:
        spawn_prefab_model = rospy.ServiceProxy(full_service_name, ZOSimPrefabSpawn)

        print("INFO: Calling service %s" % full_service_name)

        response = spawn_prefab_model(model_name=model_name,
                                     model_prefab_name=model_prefab_name,
                                     unity_asset_bundle=unity_asset_bundle,
                                     unity_asset_bundle_uri="", # TODO
                                     robot_namespace="", # TODO
                                     initial_pose=initial_pose,
                                     reference_frame="map")
        print("INFO: Spawn status: %s" % response.status_message)
        return response.success
    except rospy.ServiceException as e:
        print("ERROR: spawn_zosim_model service call failed: %s" % e)
    except rospy.ROSSerializationException as e:
        print("ERROR: spawn_zosim_model service call serialization failed: %s" % e)

def zo_delete_model(model_name, zosim_namespace):
    """[summary]
    Deletes a model in ZeroSim ROS.

    Args:
        model_name (str): Unique name for the spawned object.
        zosim_namespace:  ZO Sim namespace the service is being advertised on
    """
    full_service_name = zosim_namespace + "/delete_model"
    
    print("INFO: Waiting for service: %s" %  full_service_name)
    rospy.wait_for_service(full_service_name)
    
    try:
        delete_model = rospy.ServiceProxy(full_service_name, ZOSimDeleteModel)

        print("INFO: Calling service %s" % full_service_name)

        response = delete_model(model_name=model_name)
        print("INFO: Spawn status: %s" % response.status_message)
        return response.success
    except rospy.ServiceException as e:
        print("ERROR: spawn_zosim_model service call failed: %s" % e)
    except rospy.ROSSerializationException as e:
        print("ERROR: spawn_zosim_model service call serialization failed: %s" % e)



def main():
    # handle arguments
    parser = argparse.ArgumentParser(
        description="Spawns a .zosim object or Unity prefab in Zero Sim")
    parser.add_argument("--spawn_zosim", help="Do a .zosim file based spawn.", required=False, action='store_true')
    parser.add_argument("--spawn_prefab", help="Do a Unity prefab spawn.", required=False, action='store_true')
    parser.add_argument("--delete_model", help="Delete a model in Unity.", required=False, action='store_true')
    parser.add_argument("--input_zosim_file",
                        help=".zosim file used for input", required=False)
    parser.add_argument("--model_name",
                        help="unique name for the model", required=True)
    parser.add_argument("--model_prefab_name",
                        help="Name of the prefab model", required=False)
    parser.add_argument("--zosim_namespace",
                        help="Namespace of zosim.  Defaults to zerosim", default="/zerosim", required=False)
    parser.add_argument("--unity_asset_bundle",
                        help="The asset bundle to use.  If not specfied then uses the default internal asset bundle.", default="", required=False)

    parser.add_argument(
        "--pose_x", help="Initial pose X coordinate. 0 is default", default=0, required=False)
    parser.add_argument(
        "--pose_y", help="Initial pose Y coordinate. 0 is default", default=0, required=False)
    parser.add_argument(
        "--pose_z", help="Initial pose Z coordinate. 0 is default", default=0, required=False)
    parser.add_argument(
        "--pose_roll", help="Initial pose roll in degrees. 0 is default", default=0, required=False)
    parser.add_argument(
        "--pose_pitch", help="Initial pose pitch in degrees. 0 is default", default=0, required=False)
    parser.add_argument(
        "--pose_yaw", help="Initial pose yaw in degrees. 0 is default", default=0, required=False)

    args = parser.parse_args()


    # rospy.init_node('spawn_model')

    # Build initial pose
    initial_pose = Pose()
    initial_pose.position.x = float(args.pose_x)
    initial_pose.position.y = float(args.pose_y)
    initial_pose.position.z = float(args.pose_z)

    tmpq = tft.quaternion_from_euler(
        float(args.pose_roll), float(args.pose_pitch), float(args.pose_yaw))
    q = Quaternion(tmpq[0], tmpq[1], tmpq[2], tmpq[3])
    initial_pose.orientation = q

    if args.spawn_zosim == True:

        # read the zosim file
        try:
            with open(args.input_zosim_file) as zosim_file:
                zosim_data = json.load(zosim_file)
        except IOError:
            print("ERROR: opening file: " + zosim_file)

        # spawn the zosim model
        zo_spawn_zosim_model(model_name=args.model_name,
                            zosim_data=zosim_data,
                            initial_pose=initial_pose,
                            zosim_namespace=args.zosim_namespace)
    elif args.spawn_prefab == True:
        # spawn the prefab model
        zo_spawn_prefab_model(model_name=args.model_name,
                            model_prefab_name=args.model_prefab_name,
                            unity_asset_bundle=args.unity_asset_bundle,
                            initial_pose=initial_pose,
                            zosim_namespace=args.zosim_namespace)
    elif args.delete_model == True:
        pass
    


if __name__ == "__main__":
    main()
