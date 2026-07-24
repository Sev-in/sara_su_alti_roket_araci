"""Pure geometry helpers shared by guidance and safety layers."""

import math


def forward_distance_from_origin(
    x: float,
    y: float,
    origin_x: float,
    origin_y: float,
    reference_heading: float,
) -> float:
    """Return displacement projected onto the shore-normal mission axis."""
    dx = x - origin_x
    dy = y - origin_y
    return (
        dx * math.cos(reference_heading)
        + dy * math.sin(reference_heading)
    )


def is_at_or_beyond_distance(
    distance_m: float,
    minimum_m: float,
) -> bool:
    """Return True when the minimum shore distance has been reached."""
    return distance_m >= minimum_m
