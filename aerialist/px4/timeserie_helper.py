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


def dtw_tweaked(exp_data, num_data, metric="euclidean", **kwargs):
    if len(exp_data) > len(num_data):  # ensure first array is shorter
        tmp = exp_data
        exp_data = num_data
        num_data = tmp
    c = distance.cdist(exp_data, num_data, metric=metric, **kwargs)
    d = np.zeros(c.shape)
    d[0, 0] = c[0, 0]
    n, m = c.shape
    for i in range(1, n):
        d[i, 0] = c.max()  # d[i - 1, 0]
    for j in range(1, m):
        d[0, j] = d[0, j - 1]
    for i in range(1, n):
        for j in range(1, m):
            # d[i, j] = c[i, j] + min((d[i - 1, j], d[i, j - 1], d[i - 1, j - 1]))
            d[i, j] = min(
                (d[i, j - 1], c[i, j] + d[i - 1, j - 1])
            )  # only matching points have a cost; insertions forbidden; deletions free
    return d[-1, -1], d
