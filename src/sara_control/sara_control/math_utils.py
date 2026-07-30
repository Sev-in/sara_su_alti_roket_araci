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


def quaternion_multiply(
    left: Tuple[float, float, float, float],
    right: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Iki ``(x, y, z, w)`` quaternionunun carpimini verir."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def normalize_quaternion(
    quaternion: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Quaternionu normalize eder; gecersiz sifir quaternionu reddeder."""
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm <= 1e-12:
        raise ValueError('Quaternion normu sifir olamaz')
    return tuple(component / norm for component in quaternion)


def relative_quaternion(
    reference: Tuple[float, float, float, float],
    current: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Baslangic yoneliminden guncel yonelime bagil donusu verir.

    Mutlak Euler acilarini birbirinden cikarmak yerine
    ``inverse(reference) * current`` hesaplanir. Boylece farkli sabit
    Pixhawk montaj acilarinda pitch ekseninin roll/yaw ile karismasi
    onlenir.
    """
    reference = normalize_quaternion(reference)
    current = normalize_quaternion(current)
    reference_inverse = (
        -reference[0],
        -reference[1],
        -reference[2],
        reference[3],
    )
    relative = normalize_quaternion(
        quaternion_multiply(reference_inverse, current))

    # q ve -q ayni donusu temsil eder. Pozitif w secmek, isaret
    # gecislerinde sayisal surekliligi iyilestirir.
    if relative[3] < 0.0:
        relative = tuple(-component for component in relative)
    return relative


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
