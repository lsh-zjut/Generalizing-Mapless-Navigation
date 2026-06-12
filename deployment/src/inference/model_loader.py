from __future__ import annotations

import torch
import torch.nn as nn

from inference.models.gnm import GNM


def load_model(
    model_path: str,
    config: dict,
    device: torch.device = torch.device("cpu"),
) -> nn.Module:
    model_type = config["model_type"]
    if model_type != "gnm":
        raise ValueError(
            f"Deployment-only repository currently supports model_type='gnm', got '{model_type}'."
        )

    model = GNM(
        context_size=config["context_size"],
        len_traj_pred=config["len_traj_pred"],
        learn_angle=config["learn_angle"],
        obs_encoding_size=config["obs_encoding_size"],
        goal_encoding_size=config["goal_encoding_size"],
    )

    checkpoint = torch.load(model_path, map_location=device)
    loaded_model = checkpoint["model"]
    try:
        state_dict = loaded_model.module.state_dict()
    except AttributeError:
        state_dict = loaded_model.state_dict()
    model.load_state_dict(state_dict, strict=False)
    model.to(device)
    return model
