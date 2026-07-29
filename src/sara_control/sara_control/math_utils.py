"""Ortak sayisal ve yonelim donusum yardimcilari."""

import math
from typing import Tuple


def clamp(value: float, lower: float, upper: float) -> float:
    """Degeri kapali ``[lower, upper]`` araliginda sinirlar."""
    return max(lower, min(upper, value))


def wrap_pi(angle_rad: float) -> float:
    """Radyan cinsinden aciyi ``[-pi, pi)`` araligina sarar."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def wrap_0_2pi(angle_rad: float) -> float:
    """Radyan cinsinden aciyi ``[0, 2*pi)`` araligina sarar."""
    return angle_rad % (2.0 * math.pi)


def angle_diff_deg(current_deg: float, reference_deg: float) -> float:
    """Iki derece acisi arasindaki en kisa isaretli farki verir."""
    difference = current_deg - reference_deg
    while difference > 180.0:
        difference -= 360.0
    while difference < -180.0:
        difference += 360.0
    return difference


def euler_to_quaternion(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> Tuple[float, float, float, float]:
    """Roll, pitch ve yaw acilarini ``(x, y, z, w)`` quaternionuna cevirir."""
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_to_euler(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Tuple[float, float, float]:
    """Quaternionu radyan cinsinden ``(roll, pitch, yaw)`` acilarina cevirir."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = clamp(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quaternion_to_yaw_pitch_roll(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Tuple[float, float, float]:
    """Quaternionu mevcut dugumlerin kullandigi ``yaw, pitch, roll`` sirasinda verir."""
    roll, pitch, yaw = quaternion_to_euler(x, y, z, w)
    return yaw, pitch, roll


def quaternion_to_roll_pitch(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Tuple[float, float]:
    """Quaterniondan radyan cinsinden roll ve pitch acilarini verir."""
    roll, pitch, _yaw = quaternion_to_euler(x, y, z, w)
    return roll, pitch
