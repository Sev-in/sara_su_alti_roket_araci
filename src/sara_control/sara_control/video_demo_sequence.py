"""VIDEO_DEMO zamanli manevra dizisinin ROS'tan bagimsiz tanimi."""

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class ManeuverStep:
    """Tek bir zamanli su alti manevra adimi."""

    phase: str
    duration_s: float
    heading_offset_deg: float
    pitch_deg: float


MANEUVER_STEPS: Tuple[ManeuverStep, ...] = (
    ManeuverStep('VIDEO_DEMO_DUZ_SEYIR_1', 12.0, 0.0, 0.0),
    ManeuverStep('VIDEO_DEMO_ROTA_SAG', 12.0, 25.0, 0.0),
    ManeuverStep('VIDEO_DEMO_ROTA_SOL', 12.0, -25.0, 0.0),
    ManeuverStep('VIDEO_DEMO_YUNUSLAMA_YUKARI', 10.0, 0.0, 8.0),
    ManeuverStep('VIDEO_DEMO_YUNUSLAMA_ASAGI', 10.0, 0.0, -8.0),
    ManeuverStep('VIDEO_DEMO_DUZ_SEYIR_2', 12.0, 0.0, 0.0),
)


def total_maneuver_duration_s(
    steps: Tuple[ManeuverStep, ...] = MANEUVER_STEPS,
) -> float:
    """Dalis ve yuzeye cikis haric zamanli manevra suresini dondur."""

    return sum(max(0.0, float(step.duration_s)) for step in steps)


def maneuver_step_for_elapsed(
    elapsed_s: float,
    steps: Tuple[ManeuverStep, ...] = MANEUVER_STEPS,
):
    """Gecen sure icin ``(index, adim)`` dondur; dizi bittiyse ``None``."""

    remaining = max(0.0, float(elapsed_s))
    for index, step in enumerate(steps):
        duration = max(0.0, float(step.duration_s))
        if remaining < duration:
            return index, step
        remaining -= duration
    return None


def validate_maneuver_duration(
    minimum_duration_s: float,
    steps: Tuple[ManeuverStep, ...] = MANEUVER_STEPS,
) -> float:
    """Dizinin istenen asgari hareket suresini sagladigini dogrula."""

    total = total_maneuver_duration_s(steps)
    minimum = max(0.0, float(minimum_duration_s))
    if total < minimum:
        raise ValueError(
            f'VIDEO_DEMO manevra dizisi {total:.1f} sn; '
            f'asgari {minimum:.1f} sn olmali.'
        )
    return total
