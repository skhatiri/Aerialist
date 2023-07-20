import math
from typing import List
import numpy as np
from scipy.spatial import distance


def find_nearest_index(serie: List[int], time: int) -> int:
    """
    find and returns the index of the nearest item to the given time in the timeserie
    expects the input serie to be sorted
    """
    idx = np.searchsorted(serie, time, side="left")
    if idx > 0 and (
        idx == len(serie)
        or math.fabs(time - serie[idx - 1]) < math.fabs(time - serie[idx])
    ):
        return idx - 1
    else:
        return idx


def moving_average(current_average, current_count, new_data):
    ave = (current_average * current_count + new_data) / (current_count + 1)
    return ave


def normalize(data: np.ndarray):
    normal = np.zeros(data.shape)
    for d in range(data.shape[1]):
        d_mean = np.mean(data[:, d])
        d_std = np.std(data[:, d])
        normal[:, d] = (data[:, d] - d_mean) / d_std
    return normal
