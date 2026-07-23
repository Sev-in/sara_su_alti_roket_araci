"""Arduino Uno seri protokolu icin saf yardimci fonksiyonlar.

Bu modul ROS veya donanim kutuphanesi gerektirmez. Boylece seri telemetri
ayristirma ve komut donusumleri masaustu testlerinde de dogrulanabilir.
"""

import math


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalized_fin_to_servo(
    command,
    center_deg=90.0,
    max_offset_deg=30.0,
    sign=1.0,
):
    """Normalize [-1, 1] kanat komutunu servo acisina cevir."""
    value = float(command)
    if not math.isfinite(value):
        raise ValueError('kanat komutu sonlu olmali')
    return clamp(
        float(center_deg)
        + float(sign) * clamp(value, -1.0, 1.0) * float(max_offset_deg),
        0.0,
        180.0,
    )


def normalized_thrust(command):
    """Itki komutunu Uno protokolunun [0, 1] araligina getir."""
    value = float(command)
    if not math.isfinite(value):
        raise ValueError('itki komutu sonlu olmali')
    return clamp(value, 0.0, 1.0)


def parse_telemetry_line(line):
    """Uno'nun ``anahtar=deger`` telemetri satirini ayristir.

    ``INFO,``, ``OK,`` ve ``ERR,`` yanitlari telemetri degildir ve bos
    sozluk dondurur. Bos veya sayisal olmayan alanlar yok sayilir; bozuk
    tek bir alan tum telemetri satirinin kaybedilmesine yol acmaz.
    """
    text = str(line).strip()
    if not text or text.startswith(('INFO,', 'OK,', 'ERR,', 'READY,')):
        return {}

    values = {}
    for item in text.split(','):
        if '=' not in item:
            continue
        key, raw_value = item.split('=', 1)
        key = key.strip()
        raw_value = raw_value.strip()
        if not key or not raw_value:
            continue
        try:
            values[key] = float(raw_value)
        except ValueError:
            continue
    return values


def gauge_kpa_to_absolute_pa(gauge_pressure_kpa, surface_reference_pa):
    """Uno'nun yuzeye gore kPa degerini ROS mutlak basincina cevir."""
    gauge = float(gauge_pressure_kpa)
    reference = float(surface_reference_pa)
    if not math.isfinite(gauge) or not math.isfinite(reference):
        raise ValueError('basinc degerleri sonlu olmali')
    return reference + max(0.0, gauge) * 1000.0
