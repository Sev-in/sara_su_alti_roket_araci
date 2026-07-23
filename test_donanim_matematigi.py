#!/usr/bin/env python3
"""
test_donanim_matematigi.py
=============================
Bu script, sensor_processing.py icindeki gercek SEN0257/SEN0368
hesaplarini ve eyleyici PWM matematigini ROS2/donanim kutuphaneleri
OLMADAN test eder. Gercek donanim gerektirmez.

Amac: "kod dogru hesap yapiyor mu" sorusunu, gercek Jetson'a erismeden
simdi cevaplamak.
"""

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent / 'src' / 'sara_control'
sys.path.insert(0, str(PACKAGE_ROOT))

from sara_control.sensor_processing import (  # noqa: E402
    sen0257_voltage_to_pressure,
    voltage_divider_reconstruct,
    water_sensor_logic,
)


def pulse_us_to_duty_cycle(pulse_us: float, freq_hz: float) -> int:
    period_us = 1_000_000.0 / freq_hz
    ratio = max(0.0, min(1.0, pulse_us / period_us))
    return int(round(ratio * 65535))


# ============================================================
print("=" * 60)
print("TEST 1: pulse_us_to_duty_cycle (PCA9685 PWM donusumu)")
print("=" * 60)
tests_1 = [
    # (pulse_us, freq_hz, beklenen_yaklasik_oran)
    (1000.0, 50.0, 0.05),   # 1ms darbe, 50Hz(20ms periyot) -> %5 duty
    (1500.0, 50.0, 0.075),  # 1.5ms -> orta konum
    (2000.0, 50.0, 0.10),   # 2ms -> tam konum
]
for pulse, freq, expected_ratio in tests_1:
    duty = pulse_us_to_duty_cycle(pulse, freq)
    actual_ratio = duty / 65535.0
    ok = abs(actual_ratio - expected_ratio) < 0.001
    status = "OK" if ok else "HATA"
    print(f"  [{status}] pulse={pulse}us freq={freq}Hz -> duty={duty} "
          f"(oran={actual_ratio:.4f}, beklenen={expected_ratio:.4f})")

print()
print("=" * 60)
print("TEST 2: voltage_divider_reconstruct (ADS1115 gerilim bolucu)")
print("=" * 60)
# Bilinen senaryo: sensor 2.5V ciktiginda, bolucu SONRASI ADS ne olcer?
# V_adc = V_sensor * R2/(R1+R2) = 2.5 * 20/30 = 1.6667V
# Geri hesap: V_sensor_reconstructed = V_adc * (R1+R2)/R2 = 1.6667 * 1.5 = 2.5V (tutarli olmali)
v_sensor_original = 2.5
r1, r2 = 10000.0, 20000.0
v_adc_measured = v_sensor_original * r2 / (r1 + r2)  # bolucunun GERCEKTE urettigi deger
v_sensor_reconstructed = voltage_divider_reconstruct(v_adc_measured, r1, r2)
ok = abs(v_sensor_reconstructed - v_sensor_original) < 0.001
print(f"  [{'OK' if ok else 'HATA'}] Orijinal sensor gerilimi={v_sensor_original}V -> "
      f"ADS'in olctugu={v_adc_measured:.4f}V -> geri hesaplanan={v_sensor_reconstructed:.4f}V")

print()
print("=" * 60)
print("TEST 3: voltage_to_pressure (SEN0257 gerilim->basinc)")
print("=" * 60)
tests_3 = [
    # Kullanilan SEN0257 varyanti: 0.5V=0Pa, 4.5V=1600000Pa dogrusal
    (0.5, 0.0),
    (4.5, 1600000.0),
    (2.5, 800000.0),  # tam ortada
]
for v, expected_p in tests_3:
    p = sen0257_voltage_to_pressure(v)
    ok = abs(p - expected_p) < 1.0
    print(f"  [{'OK' if ok else 'HATA'}] V={v}V -> P={p:.1f}Pa (beklenen={expected_p:.1f}Pa)")

print()
print("=" * 60)
print("TEST 4: water_sensor_logic (SEN0368 polarite)")
print("=" * 60)
tests_4 = [
    # (raw_gpio, active_high, beklenen_submerged)
    (True, True, True),    # HIGH=su var, HIGH okundu -> su var
    (False, True, False),  # HIGH=su var, LOW okundu -> su yok
    (True, False, False),  # LOW=su var, HIGH okundu -> su YOK (ters polarite)
    (False, False, True),  # LOW=su var, LOW okundu -> su VAR (ters polarite)
]
for raw, active_high, expected in tests_4:
    result = water_sensor_logic(raw, active_high)
    ok = result == expected
    print(f"  [{'OK' if ok else 'HATA'}] raw={raw}, active_high={active_high} -> "
          f"submerged={result} (beklenen={expected})")

print()
print("=" * 60)
print("TUM TESTLER TAMAMLANDI")
print("=" * 60)
