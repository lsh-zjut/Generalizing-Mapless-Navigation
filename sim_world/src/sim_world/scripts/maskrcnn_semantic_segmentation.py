#!/usr/bin/env python3

import csv
import threading

import numpy as np
import rospy
import torch
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from torchvision.models.detection import (
    MaskRCNN_ResNet50_FPN_Weights,
    maskrcnn_resnet50_fpn,
)


class MaskRcnnSemanticNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.score_threshold = rospy.get_param("~score_threshold", 0.7)
        self.mask_threshold = rospy.get_param("~mask_threshold", 0.5)
        self.max_rate_hz = rospy.get_param("~max_rate_hz", 1.0)
        self.input_topic = rospy.get_param("~input_topic", "/front/rgb/image_raw")
        self.output_topic = rospy.get_param("~output_topic", "/front/semantic/image_raw")
        self.semantic_csv = rospy.get_param("~semantic_csv")
        requested_device = rospy.get_param("~device", "cuda")

        self.period = rospy.Duration(0.0 if self.max_rate_hz <= 0 else 1.0 / self.max_rate_hz)
        self.last_stamp = rospy.Time(0)
        self.last_wall_time = rospy.Time(0)

        self.device = self._resolve_device(requested_device)
        self.model = self._load_model()
        self.category_to_color = self._load_color_map(self.semantic_csv)
        self.background_color = np.array(self.category_to_color.get("__background__", (0, 0, 0)),
                                         dtype=np.uint8)

        self.pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, Image, self.callback, queue_size=1, buff_size=2**24)
        rospy.loginfo("maskrcnn semantic segmentation ready on %s: %s -> %s",
                      self.device.type, self.input_topic, self.output_topic)

    def _resolve_device(self, requested_device):
        if requested_device == "cuda":
            if torch.cuda.is_available():
                return torch.device("cuda")
            rospy.logwarn("CUDA requested for semantic segmentation, but torch.cuda.is_available() is false. "
                          "Falling back to CPU.")
            return torch.device("cpu")

        return torch.device(requested_device)

    def _load_model(self):
        weights = MaskRCNN_ResNet50_FPN_Weights.DEFAULT
        model = maskrcnn_resnet50_fpn(weights=weights)
        model.eval()
        model.to(self.device)
        self.categories = weights.meta["categories"]
        return model

    def _load_color_map(self, csv_path):
        color_map = {}
        with open(csv_path, newline="") as fin:
            reader = csv.DictReader(fin)
            for row in reader:
                name = row["name"].strip()
                if not row["red"] or not row["green"] or not row["blue"]:
                    continue
                color = (int(row["red"]), int(row["green"]), int(row["blue"]))
                if name == "BG":
                    color_map["__background__"] = color
                color_map.setdefault(name, color)
        color_map.setdefault("__background__", (0, 0, 0))
        return color_map

    def callback(self, msg):
        with self.lock:
            if msg.header.stamp == self.last_stamp:
                return
            now = rospy.Time.now()
            if self.period.to_sec() > 0 and self.last_wall_time != rospy.Time(0):
                if now - self.last_wall_time < self.period:
                    return

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            semantic = self.infer(cv_image)

            out = self.bridge.cv2_to_imgmsg(semantic, encoding="rgb8")
            out.header = msg.header
            self.pub.publish(out)

            self.last_stamp = msg.header.stamp
            self.last_wall_time = now

    @torch.inference_mode()
    def infer(self, rgb_image):
        tensor = torch.from_numpy(rgb_image).permute(2, 0, 1).float() / 255.0
        tensor = tensor.to(self.device)
        outputs = self.model([tensor])[0]

        semantic = np.broadcast_to(self.background_color, rgb_image.shape).copy()

        scores = outputs["scores"].detach().cpu().numpy()
        labels = outputs["labels"].detach().cpu().numpy()
        masks = outputs["masks"].detach().cpu().numpy()

        accepted = np.where(scores >= self.score_threshold)[0]
        if accepted.size == 0:
            return semantic

        # Paint low-confidence objects first so stronger detections can overwrite them.
        accepted = accepted[np.argsort(scores[accepted])]

        for idx in accepted:
            label_idx = int(labels[idx])
            if label_idx < 0 or label_idx >= len(self.categories):
                continue
            name = self.categories[label_idx]
            color = self.category_to_color.get(name)
            if color is None:
                continue
            mask = masks[idx, 0] >= self.mask_threshold
            semantic[mask] = color

        return semantic


def main():
    rospy.init_node("maskrcnn_semantic_segmentation")
    try:
        MaskRcnnSemanticNode()
        rospy.spin()
    except Exception as exc:
        rospy.logfatal("maskrcnn semantic segmentation failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
