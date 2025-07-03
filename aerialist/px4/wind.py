from typing import NamedTuple, Tuple

class Wind:
    class Params(NamedTuple):
        mean_velocity: float
        max_velocity: float
        velocity_variance: float
        direction_mean: Tuple[float, float, float]
        direction_variance: float
        gust_start: float
        gust_duration: float
        gust_velocity_mean: float
        gust_velocity_max: float
        gust_velocity_variance: float
        gust_direction_mean: Tuple[float, float, float]
        gust_direction_variance: float

    def __init__(self, params: Params) -> None:
        self.params = params

    def to_dict(self):
        p = self.params
        return {
            "mean_velocity": p.mean_velocity,
            "max_velocity": p.max_velocity,
            "velocity_variance": p.velocity_variance,
            "direction_mean": list(p.direction_mean),
            "direction_variance": p.direction_variance,
            "gust_start": p.gust_start,
            "gust_duration": p.gust_duration,
            "gust_velocity_mean": p.gust_velocity_mean,
            "gust_velocity_max": p.gust_velocity_max,
            "gust_velocity_variance": p.gust_velocity_variance,
            "gust_direction_mean": list(p.gust_direction_mean),
            "gust_direction_variance": p.gust_direction_variance,
        }
