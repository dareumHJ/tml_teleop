# teleop/utils.py

from typing import Tuple
import yaml
from .mapping import JointMappingConfig, SafetyConfig


def load_mapping_config(path: str) -> Tuple[JointMappingConfig, SafetyConfig]:
    with open(path, "r") as f:
        cfg = yaml.safe_load(f)

    leader_joints = cfg["leader"]["joints"]
    follower_joints = cfg["follower"]["joints"]

    mapping = cfg["mapping"]
    safety = cfg["safety"]

    mapping_cfg = JointMappingConfig(
        leader_joints=leader_joints,
        follower_joints=follower_joints,
        sign=mapping.get("sign", {}),
        scale=mapping.get("scale", {}),
        offset=mapping.get("offset", {}),
    )

    safety_cfg = SafetyConfig(
        position_min=safety.get("position_min", {}),
        position_max=safety.get("position_max", {}),
        vel_limit=safety.get("vel_limit", {}),
        soft_margin=safety.get("soft_margin", 0.05),
    )

    return mapping_cfg, safety_cfg
