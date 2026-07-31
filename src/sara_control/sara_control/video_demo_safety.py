"""VIDEO_DEMO kritik yuzey kilidinin ROS'tan bagimsiz kurallari."""

import math


def should_latch_power_cutoff(
    *,
    video_demo_mode: bool,
    task_complete: bool = False,
    emergency_stop: bool = False,
    power_cutoff_on_emergency_stop: bool = True,
) -> bool:
    """D5 kesmenin yalniz gercek gorevlerde kilitlenmesini sagla."""

    if video_demo_mode:
        return False
    return bool(
        task_complete
        or (emergency_stop and power_cutoff_on_emergency_stop)
    )


def video_demo_surface_gate(
    *,
    demo_mode: bool,
    ignition_enabled: bool,
    depth_valid: bool,
    surface_detected: bool,
    depth_m: float,
    max_depth_m: float,
    pitch_rad: float,
    max_abs_pitch_rad: float,
    forward_speed_ms: float,
    max_speed_ms: float,
    actuator_status_fresh: bool,
    measurement_mode: bool,
) -> bool:
    """Kapak/3 V gosterimine izin veren tum bagimsiz kosullari denetle."""

    numeric_values = (
        depth_m,
        max_depth_m,
        pitch_rad,
        max_abs_pitch_rad,
        forward_speed_ms,
        max_speed_ms,
    )
    if not all(math.isfinite(float(value)) for value in numeric_values):
        return False
    return all((
        bool(demo_mode),
        bool(ignition_enabled),
        bool(depth_valid),
        bool(surface_detected),
        float(depth_m) <= float(max_depth_m),
        abs(float(pitch_rad)) <= float(max_abs_pitch_rad),
        abs(float(forward_speed_ms)) <= float(max_speed_ms),
        bool(actuator_status_fresh),
        bool(measurement_mode),
    ))
