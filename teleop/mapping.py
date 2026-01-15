# teleop/mapping.py

from dataclasses import dataclass
from typing import List, Dict
import numpy as np

@dataclass
class JointMappingConfig:
    leader_joints: List[str]
    follower_joints: List[str]
    sign: Dict[str, float]  # joint_name -> +1.0 or -1.0
    scale: Dict[str, float]  # joint_name -> gain
    offset: Dict[str, float]  # joint_name -> radians offset

@dataclass
class SafetyConfig:
    position_min: Dict[str, float]  # joint_name -> min rad
    position_max: Dict[str, float]  # joint_name -> max rad
    vel_limit: Dict[str, float]     # joint_name -> rad/s
    soft_margin: float = 0.05       # near-limit margin

class SafetyLimiter:
    