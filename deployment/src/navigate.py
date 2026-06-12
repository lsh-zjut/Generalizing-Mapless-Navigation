import argparse
import os

import numpy as np
from PIL import Image as PILImage
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
import torch
import yaml

from topic_names import IMAGE_TOPIC, WAYPOINT_TOPIC
from utils import load_model, msg_to_pil, to_numpy, transform_images


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TOPOMAP_IMAGES_DIR = os.path.join(SCRIPT_DIR, "../topomaps/images")
ROBOT_CONFIG_PATH = os.path.join(SCRIPT_DIR, "../config/robot.yaml")
MODEL_CONFIG_PATH = os.path.join(SCRIPT_DIR, "../config/models.yaml")

with open(ROBOT_CONFIG_PATH, "r", encoding="utf-8") as f:
    robot_config = yaml.safe_load(f)
MAX_V = robot_config["max_v"]
RATE = robot_config["frame_rate"]

context_queue = []
context_size = None

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device:", device)


def callback_obs(msg):
    obs_img = msg_to_pil(msg)
    if context_size is not None:
        if len(context_queue) < context_size + 1:
            context_queue.append(obs_img)
        else:
            context_queue.pop(0)
            context_queue.append(obs_img)


def main(args: argparse.Namespace):
    global context_size

    with open(MODEL_CONFIG_PATH, "r", encoding="utf-8") as f:
        model_paths = yaml.safe_load(f)

    model_config_path = os.path.join(SCRIPT_DIR, model_paths[args.model]["config_path"])
    with open(model_config_path, "r", encoding="utf-8") as f:
        model_params = yaml.safe_load(f)

    context_size = model_params["context_size"]

    ckpt_path = os.path.join(SCRIPT_DIR, model_paths[args.model]["ckpt_path"])
    if not os.path.exists(ckpt_path):
        raise FileNotFoundError(f"Model weights not found at {ckpt_path}")
    print(f"Loading model from {ckpt_path}")

    model = load_model(ckpt_path, model_params, device).to(device)
    model.eval()

    waypoint_scale = None
    if model_params.get("normalize"):
        candidate_scale = args.waypoint_scale
        if candidate_scale is None:
            candidate_scale = model_params.get("waypoint_scale")
        if candidate_scale is None:
            candidate_scale = MAX_V / RATE
            print(
                "Warning: waypoint scale not set in config; falling back to MAX_V / RATE."
            )
        waypoint_scale = float(candidate_scale)
        print(f"Using waypoint scale factor: {waypoint_scale:.4f} meters/unit")

    topomap_dir = os.path.join(TOPOMAP_IMAGES_DIR, args.dir)
    topomap_filenames = sorted(
        os.listdir(topomap_dir), key=lambda x: int(x.split(".")[0])
    )
    topomap = [PILImage.open(os.path.join(topomap_dir, name)) for name in topomap_filenames]

    closest_node = 0
    assert -1 <= args.goal_node < len(topomap), "Invalid goal index"
    goal_node = len(topomap) - 1 if args.goal_node == -1 else args.goal_node

    rospy.init_node("EXPLORATION", anonymous=False)
    rate = rospy.Rate(RATE)
    rospy.Subscriber(IMAGE_TOPIC, Image, callback_obs, queue_size=1)
    waypoint_pub = rospy.Publisher(WAYPOINT_TOPIC, Float32MultiArray, queue_size=1)
    goal_pub = rospy.Publisher("/topoplan/reached_goal", Bool, queue_size=1)

    print("Registered with master node. Waiting for image observations...")

    while not rospy.is_shutdown():
        chosen_waypoint = np.zeros(4)
        if len(context_queue) > model_params["context_size"]:
            start = max(closest_node - args.radius, 0)
            end = min(closest_node + args.radius + 1, goal_node)
            batch_obs_imgs = []
            batch_goal_data = []
            for sg_img in topomap[start : end + 1]:
                transf_obs_img = transform_images(context_queue, model_params["image_size"])
                goal_data = transform_images(sg_img, model_params["image_size"])
                batch_obs_imgs.append(transf_obs_img)
                batch_goal_data.append(goal_data)

            batch_obs_imgs = torch.cat(batch_obs_imgs, dim=0).to(device)
            batch_goal_data = torch.cat(batch_goal_data, dim=0).to(device)

            distances, waypoints = model(batch_obs_imgs, batch_goal_data)
            distances = to_numpy(distances)
            waypoints = to_numpy(waypoints)
            min_dist_idx = np.argmin(distances)

            if distances[min_dist_idx] > args.close_threshold:
                chosen_waypoint = waypoints[min_dist_idx][args.waypoint]
                closest_node = start + min_dist_idx
            else:
                chosen_waypoint = waypoints[min(min_dist_idx + 1, len(waypoints) - 1)][
                    args.waypoint
                ]
                closest_node = min(start + min_dist_idx + 1, goal_node)

        if model_params["normalize"] and waypoint_scale is not None:
            chosen_waypoint[:2] *= waypoint_scale

        waypoint_msg = Float32MultiArray()
        waypoint_msg.data = chosen_waypoint
        waypoint_pub.publish(waypoint_msg)

        reached_goal = closest_node == goal_node
        goal_pub.publish(reached_goal)
        if reached_goal:
            print("Reached goal! Stopping...")
        rate.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Run deployment-only GNM topological navigation in sim_world."
    )
    parser.add_argument("--model", "-m", default="gnm", type=str)
    parser.add_argument("--waypoint", "-w", default=2, type=int)
    parser.add_argument("--dir", "-d", default="route0", type=str)
    parser.add_argument("--goal-node", "-g", default=-1, type=int)
    parser.add_argument("--close-threshold", "-t", default=3, type=int)
    parser.add_argument("--radius", "-r", default=4, type=int)
    parser.add_argument("--waypoint-scale", default=None, type=float)
    args = parser.parse_args()
    main(args)
