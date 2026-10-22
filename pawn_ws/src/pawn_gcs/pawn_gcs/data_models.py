from dataclasses import dataclass
import numpy as np

@dataclass
class HazardEvent:
    id: int
    drone_id: int
    hazard_type: str      # "HUMAN", "FALL_RISK", or "CRACK"
    title: str
    score: float          # 0.0 to 10.0
    metrics: str
    xyz: np.ndarray       # shape (3,)