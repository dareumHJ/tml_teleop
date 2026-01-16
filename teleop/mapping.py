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
    def __init__(self, cfg: SafetyConfig):
        self.cfg = cfg
        self._prev_pos: Dict[str, float] = {}
        self._prev_time: float = None

    def reset(self):
        self._prev_pos = {}
        self._prev_time = None

    def limit(self,
              positions: Dict[str, float],
              t: float
            ) -> Dict[str, float]:
        limited = {}

        for name, q in positions.items():
            q_min = self.cfg.position_min.get(name, -np.inf)
            q_max = self.cfg.position_max.get(name, np.inf)

            # position clamp
            q_clamped = np.clip(q, q_min + self.cfg.soft_margin,
                                   q_max - self.cfg.soft_margin)

            # simple velocity limiting (if previous state exists)
            if self._prev_time is not None and name in self._prev_pos:
                dt = max(t - self._prev_time, 1e-3)
                dq = q_clamped - self._prev_pos[name]
                v = dq / dt
                v_lim = self.cfg.vel_limit.get(name, np.inf)
                if abs(v) > v_lim:
                    dq = np.sign(v) * v_lim * dt
                    q_clamped = self._prev_pos[name] + dq

            limited[name] = float(q_clamped)

        self._prev_time = t
        self._prev_pos.update(limited)
        return limited

class JointMapper:

    def __init__(
        self,
        mapping_cfg: JointMappingConfig,
        safety_cfg: SafetyConfig | None = None,
    ):
        self.cfg = mapping_cfg
        self.safety = SafetyLimiter(safety_cfg) if safety_cfg else None

    def map_joints(
        self,
        leader_pos: Dict[str, float],
        t: float | None = None,
    ) -> Dict[str, float]:
        follower_cmd: Dict[str, float] = {}

        for l_name, f_name in zip(
            self.cfg.leader_joints,
            self.cfg.follower_joints,
        ):
            if l_name not in leader_pos:
                continue

            q_l = leader_pos[l_name]
            sign = self.cfg.sign.get(f_name, 1.0)
            scale = self.cfg.scale.get(f_name, 1.0)
            offset = self.cfg.offset.get(f_name, 0.0)

            q_f = sign * scale * q_l + offset
            follower_cmd[f_name] = float(q_f)

        if self.safety is not None and t is not None:
            follower_cmd = self.safety.limit(follower_cmd, t)

        return follower_cmd