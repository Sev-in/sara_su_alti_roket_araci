"""ROS'tan bagimsiz klavye teleop komut esleme yardimcilari."""

from dataclasses import dataclass


@dataclass(frozen=True)
class TeleopCommand:
    """Bir klavye tusunun hareket niyetini tanimlar."""

    name: str
    forward: bool
    turn_sign: int


STOP = TeleopCommand('STOP', False, 0)
FORWARD = TeleopCommand('FORWARD', True, 0)
LEFT = TeleopCommand('LEFT', True, -1)
RIGHT = TeleopCommand('RIGHT', True, 1)


def command_for_key(key: str):
    """Desteklenen bir tusu komuta cevirir; bilinmeyen tusta None doner."""

    if not key:
        return None
    return {
        'w': FORWARD,
        'a': LEFT,
        'd': RIGHT,
        's': STOP,
        ' ': STOP,
    }.get(key.lower())


def adjusted_depth_target(
    current_target: float,
    change_m: float,
    minimum_m: float,
    maximum_m: float,
) -> float:
    """Derinlik hedefini arac sinirlari icinde gunceller."""

    if minimum_m > maximum_m:
        raise ValueError('minimum_m, maximum_m degerinden buyuk olamaz')
    return max(minimum_m, min(maximum_m, current_target + change_m))
