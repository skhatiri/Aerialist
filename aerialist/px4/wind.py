from typing import NamedTuple, Tuple

class Wind:
    class Params(NamedTuple):
        velocity_mean: float
        velocity_max: float
        direction: Tuple[float, float, float]
        velocity_variance: float = 0
        direction_variance: float = 0
        gust_start: float = 0
        gust_duration: float = 0
        gust_velocity_mean: float = 0
        gust_velocity_max: float = 0
        gust_velocity_variance: float = 0
        gust_direction: Tuple[float, float, float] = [0, 0, 0]
        gust_direction_variance: float = 0

    def __init__(self, params: Params) -> None:
        self.params = params

    def to_dict(self):
        p = self.params
        return {
            "velocity_mean": p.velocity_mean,
            "velocity_max": p.velocity_max,
            "velocity_variance": p.velocity_variance,
            "direction": list(p.direction),
            "direction_variance": p.direction_variance,
            "gust_start": p.gust_start,
            "gust_duration": p.gust_duration,
            "gust_velocity_mean": p.gust_velocity_mean,
            "gust_velocity_max": p.gust_velocity_max,
            "gust_velocity_variance": p.gust_velocity_variance,
            "gust_direction": list(p.gust_direction),
            "gust_direction_variance": p.gust_direction_variance,
        }
