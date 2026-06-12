#!/usr/bin/env python3

import argparse
import math
import sys

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState, SetModelState, SpawnModel
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
from tf.transformations import quaternion_from_euler


def model_exists(topic_name, model_name, timeout):
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        try:
            msg = rospy.wait_for_message(topic_name, ModelStates, timeout=0.5)
            if model_name in msg.name:
                return True
        except rospy.ROSException:
            pass
    return False


def force_model_pose(gazebo_namespace, model_name, pose, reference_frame, timeout):
    set_service = gazebo_namespace.rstrip("/") + "/set_model_state"
    get_service = gazebo_namespace.rstrip("/") + "/get_model_state"

    rospy.loginfo("Waiting for services %s and %s", set_service, get_service)
    rospy.wait_for_service(set_service, timeout=timeout)
    rospy.wait_for_service(get_service, timeout=timeout)
    set_model_state = rospy.ServiceProxy(set_service, SetModelState)
    get_model_state = rospy.ServiceProxy(get_service, GetModelState)

    state = ModelState()
    state.model_name = model_name
    state.pose = pose
    state.twist = Twist()
    state.reference_frame = reference_frame

    deadline = rospy.Time.now() + rospy.Duration(timeout)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        response = set_model_state(state)
        if response.success:
            current = get_model_state(model_name, reference_frame)
            if current.success:
                dx = abs(current.pose.position.x - pose.position.x)
                dy = abs(current.pose.position.y - pose.position.y)
                dz = abs(current.pose.position.z - pose.position.z)
                if max(dx, dy, dz) < 1e-3:
                    rospy.loginfo(
                        "Reset model %s to pose (%.4f, %.4f, %.4f)",
                        model_name,
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                    )
                    return True
        rospy.sleep(0.1)

    rospy.logwarn("Model %s was spawned but could not be reset to the requested pose.", model_name)
    return False


def main():
    parser = argparse.ArgumentParser(description="Spawn URDF and tolerate Gazebo queue timeout if model appears.")
    parser.add_argument("--model", required=True)
    parser.add_argument("--param", default="robot_description")
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--R", type=float, default=0.0)
    parser.add_argument("--P", type=float, default=0.0)
    parser.add_argument("--Y", type=float, default=0.0)
    parser.add_argument("--robot-namespace", default="/")
    parser.add_argument("--reference-frame", default="world")
    parser.add_argument("--gazebo-namespace", default="/gazebo")
    parser.add_argument("--poll-timeout", type=float, default=5.0)
    parser.add_argument("--reset-timeout", type=float, default=10.0)
    args, _ = parser.parse_known_args()

    rospy.init_node("urdf_spawner")

    model_xml = rospy.get_param(args.param)
    pose = Pose()
    pose.position.x = args.x
    pose.position.y = args.y
    pose.position.z = args.z
    qx, qy, qz, qw = quaternion_from_euler(args.R, args.P, args.Y)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw

    service_name = args.gazebo_namespace.rstrip("/") + "/spawn_urdf_model"
    rospy.loginfo("Waiting for service %s", service_name)
    rospy.wait_for_service(service_name)
    spawn_model = rospy.ServiceProxy(service_name, SpawnModel)

    rospy.loginfo("Calling service %s", service_name)
    response = spawn_model(
        model_name=args.model,
        model_xml=model_xml,
        robot_namespace=args.robot_namespace,
        initial_pose=pose,
        reference_frame=args.reference_frame,
    )

    if response.success:
        rospy.loginfo("Spawned model %s", args.model)
        force_model_pose(args.gazebo_namespace, args.model, pose, args.reference_frame, args.reset_timeout)
        return 0

    status = response.status_message or ""
    rospy.logwarn("Spawn service reported failure: %s", status)
    if "already exists" in status.lower() and model_exists("/gazebo/model_states", args.model, args.poll_timeout):
        rospy.loginfo("Model %s already exists; resetting it to the requested pose.", args.model)
        force_model_pose(args.gazebo_namespace, args.model, pose, args.reference_frame, args.reset_timeout)
        return 0

    if "spawn queue" in status.lower() and model_exists("/gazebo/model_states", args.model, args.poll_timeout):
        rospy.loginfo("Model %s appeared in /gazebo/model_states after queued spawn; treating as success.", args.model)
        force_model_pose(args.gazebo_namespace, args.model, pose, args.reference_frame, args.reset_timeout)
        return 0

    rospy.logerr("Failed to spawn model %s: %s", args.model, status)
    return 1


if __name__ == "__main__":
    sys.exit(main())
