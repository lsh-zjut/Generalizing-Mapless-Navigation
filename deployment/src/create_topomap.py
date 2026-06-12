import argparse
import os
import shutil
import time

import rospy
from sensor_msgs.msg import Image, Joy

from topic_names import IMAGE_TOPIC
from utils import msg_to_pil


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TOPOMAP_IMAGES_DIR = os.path.join(SCRIPT_DIR, "../topomaps/images")
obs_img = None


def remove_files_in_dir(dir_path: str):
    for f in os.listdir(dir_path):
        file_path = os.path.join(dir_path, f)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print(f"Failed to delete {file_path}. Reason: {e}")


def callback_obs(msg: Image):
    global obs_img
    obs_img = msg_to_pil(msg)


def callback_joy(msg: Joy):
    if msg.buttons[0]:
        rospy.signal_shutdown("shutdown")


def main(args: argparse.Namespace):
    global obs_img
    rospy.init_node("CREATE_TOPOMAP", anonymous=False)
    rospy.Subscriber(IMAGE_TOPIC, Image, callback_obs, queue_size=1)
    rospy.Subscriber("joy", Joy, callback_joy)

    topomap_name_dir = os.path.join(TOPOMAP_IMAGES_DIR, args.dir)
    if not os.path.isdir(topomap_name_dir):
        os.makedirs(topomap_name_dir)
    else:
        print(f"{topomap_name_dir} already exists. Removing previous images...")
        remove_files_in_dir(topomap_name_dir)

    assert args.dt > 0, "dt must be positive"
    rate = rospy.Rate(1 / args.dt)
    print(f"Waiting for images on {IMAGE_TOPIC}...")
    i = 0
    start_time = float("inf")
    while not rospy.is_shutdown():
        if obs_img is not None:
            obs_img.save(os.path.join(topomap_name_dir, f"{i}.png"))
            print("saved image", i)
            i += 1
            rate.sleep()
            start_time = time.time()
            obs_img = None
        if time.time() - start_time > 2 * args.dt:
            print(f"Topic {IMAGE_TOPIC} stopped publishing. Shutting down...")
            rospy.signal_shutdown("shutdown")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=f"Record a new topomap from {IMAGE_TOPIC}.")
    parser.add_argument("--dir", "-d", default="topomap", type=str)
    parser.add_argument("--dt", "-t", default=1.0, type=float)
    args = parser.parse_args()
    main(args)
