from __future__ import annotations

from collections.abc import Sequence

import cv2
import numpy as np

from robot.profiling import profile_function


HSV_LUT_RANGE_THRESHOLD = 12
_HSV_LUT_SHAPE = (180, 256, 256)


def _clamp_channel(value: int, upper: int) -> int:
    return max(0, min(upper, value))


def _normalize_bounds(
    lower: np.ndarray | Sequence[int],
    upper: np.ndarray | Sequence[int],
) -> tuple[int, int, int, int, int, int]:
    h0 = int(lower[0])
    s0 = int(lower[1])
    v0 = int(lower[2])
    h1 = int(upper[0])
    s1 = int(upper[1])
    v1 = int(upper[2])

    h_low = _clamp_channel(min(h0, h1), 179)
    h_high = _clamp_channel(max(h0, h1), 179)
    s_low = _clamp_channel(min(s0, s1), 255)
    s_high = _clamp_channel(max(s0, s1), 255)
    v_low = _clamp_channel(min(v0, v1), 255)
    v_high = _clamp_channel(max(v0, v1), 255)
    return h_low, h_high, s_low, s_high, v_low, v_high


def _as_uint8_array(bound: np.ndarray | Sequence[int]) -> np.ndarray:
    if isinstance(bound, np.ndarray):
        if bound.dtype == np.uint8:
            return bound
        return bound.astype(np.uint8, copy=False)
    return np.array(bound, dtype=np.uint8)


@profile_function
def build_hsv_range_lut(
    ranges: list[tuple[np.ndarray, np.ndarray]] | list[tuple[Sequence[int], Sequence[int]]]
) -> np.ndarray | None:
    if not ranges:
        return None

    lut = np.zeros(_HSV_LUT_SHAPE, dtype=np.uint8)
    for lower, upper in ranges:
        h_low, h_high, s_low, s_high, v_low, v_high = _normalize_bounds(lower, upper)
        lut[h_low:h_high + 1, s_low:s_high + 1, v_low:v_high + 1] = 255
    return lut


def mask_from_ranges(
    hsv_frame: np.ndarray,
    ranges: list[tuple[np.ndarray, np.ndarray]] | list[tuple[Sequence[int], Sequence[int]]],
    lut: np.ndarray | None = None,
) -> np.ndarray | None:
    if hsv_frame is None or hsv_frame.size == 0:
        return None

    if lut is not None:
        return lut[hsv_frame[:, :, 0], hsv_frame[:, :, 1], hsv_frame[:, :, 2]]

    if not ranges:
        return None

    lower0 = _as_uint8_array(ranges[0][0])
    upper0 = _as_uint8_array(ranges[0][1])
    mask = cv2.inRange(hsv_frame, lower0, upper0)
    for lower, upper in ranges[1:]:
        cv2.bitwise_or(
            mask,
            cv2.inRange(hsv_frame, _as_uint8_array(lower), _as_uint8_array(upper)),
            dst=mask,
        )
    return mask
