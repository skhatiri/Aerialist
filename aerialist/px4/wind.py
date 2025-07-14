from typing import NamedTuple, Tuple

class Wind:
    class Params(NamedTuple):
        velocity_mean: float
        velocity_max: float
        velocity_variance: float
        direction: Tuple[float, float, float]
        direction_variance: float
        gust_start: float
        gust_duration: float
        gust_velocity_mean: float
        gust_velocity_max: float
        gust_velocity_variance: float
        gust_direction: Tuple[float, float, float]
        gust_direction_variance: float

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
