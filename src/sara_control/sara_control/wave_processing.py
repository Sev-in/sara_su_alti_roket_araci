#!/usr/bin/env python3
"""ROS'tan bagimsiz dalga ozelligi cikarma ve siniflandirma yardimcilari."""

import math
import statistics
from collections import deque
from dataclasses import dataclass

from sara_control.math_utils import (
    clamp,
    quaternion_to_roll_pitch as _quaternion_to_roll_pitch,
)

WAVE_CALM = 0
WAVE_LIGHT = 1
WAVE_MODERATE = 2
WAVE_SEVERE = 3

WAVE_NAMES = {
    WAVE_CALM: 'SAKIN',
    WAVE_LIGHT: 'HAFIF',
    WAVE_MODERATE: 'ORTA',
    WAVE_SEVERE: 'SIDDETLI',
}


@dataclass(frozen=True)
class WaveSample:
    """Ayni zaman tabanina getirilmis dalga kestirim girdileri."""

    time_s: float
    roll_rad: float
    pitch_rad: float
    vertical_accel_ms2: float
    depth_m: float


@dataclass(frozen=True)
class WaveEstimate:
    """Bir analiz penceresinden uretilen dalga ozeti."""

    ready: bool
    severity: int
    raw_severity: int
    confidence: float
    score: float
    roll_rms_deg: float
    pitch_rms_deg: float
    vertical_accel_rms_ms2: float
    depth_peak_to_peak_m: float
    dominant_period_s: float
    sample_count: int
    window_span_s: float
    maneuver_suppressed: bool


def quaternion_to_roll_pitch(x: float, y: float, z: float, w: float):
    """Normalize quaternion'dan roll ve pitch acilarini dondur."""
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 0.5 or norm > 1.5:
        return None
    x, y, z, w = (value / norm for value in values)

    return _quaternion_to_roll_pitch(x, y, z, w)


def world_vertical_acceleration(
    qx: float,
    qy: float,
    qz: float,
    qw: float,
    ax: float,
    ay: float,
    az: float,
    gravity_ms2: float = 9.80665,
):
    """Govde ivmesini dunya dikeyine cevirip yercekimini cikar.

    MAVROS ``sensor_msgs/Imu.linear_acceleration`` verisi durgun halde
    yaklasik +g tasir. Quaternion govdeden dunya eksenine donusum olarak
    kullanilir. Sonuc dalga kaynakli dikey ozgul ivme kestirimidir.
    """
    values = (qx, qy, qz, qw, ax, ay, az, gravity_ms2)
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 0.5 or norm > 1.5 or gravity_ms2 <= 0.0:
        return None
    qx, qy, qz, qw = (
        qx / norm,
        qy / norm,
        qz / norm,
        qw / norm,
    )
    world_z = (
        2.0 * (qx * qz - qw * qy) * ax
        + 2.0 * (qy * qz + qw * qx) * ay
        + (1.0 - 2.0 * (qx * qx + qy * qy)) * az
    )
    return world_z - gravity_ms2


def _linear_detrend(times, values):
    """Ortalama ve dogrusal egilimi kaldir."""
    if len(times) != len(values) or not values:
        return []
    mean_t = sum(times) / len(times)
    mean_v = sum(values) / len(values)
    denominator = sum((time_s - mean_t) ** 2 for time_s in times)
    slope = 0.0
    if denominator > 0.0:
        slope = sum(
            (time_s - mean_t) * (value - mean_v)
            for time_s, value in zip(times, values)
        ) / denominator
    return [
        value - (mean_v + slope * (time_s - mean_t))
        for time_s, value in zip(times, values)
    ]


def _rms(values):
    """Degerlerin kok-ortalama-karesini hesapla."""
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def _estimate_period(times, values, min_period_s, max_period_s):
    """Pozitif sifir gecislerinden baskin salinim periyodunu kestir."""
    if len(times) < 4 or len(times) != len(values):
        return math.nan
    crossings = []
    for index in range(1, len(values)):
        before = values[index - 1]
        after = values[index]
        if before <= 0.0 < after:
            delta = after - before
            fraction = 0.0 if delta == 0.0 else -before / delta
            crossings.append(
                times[index - 1]
                + fraction * (times[index] - times[index - 1])
            )
    periods = [
        crossings[index] - crossings[index - 1]
        for index in range(1, len(crossings))
    ]
    periods = [
        period for period in periods
        if min_period_s <= period <= max_period_s
    ]
    return statistics.median(periods) if periods else math.nan


class WaveEstimatorCore:
    """IMU ve basinc-derinlik salinimlarini birlestiren dalga kestirici."""

    def __init__(
        self,
        window_s=8.0,
        sample_rate_hz=10.0,
        minimum_fill_ratio=0.75,
        roll_rms_light_deg=2.0,
        pitch_rms_light_deg=2.0,
        vertical_accel_rms_light_ms2=0.20,
        depth_peak_to_peak_light_m=0.06,
        moderate_score=2.0,
        severe_score=4.0,
        enter_hold_s=2.0,
        exit_hold_s=4.0,
        min_period_s=1.0,
        max_period_s=10.0,
    ):
        if window_s <= 0.0 or sample_rate_hz <= 0.0:
            raise ValueError('Dalga penceresi ve ornekleme hizi pozitif olmali')
        thresholds = (
            roll_rms_light_deg,
            pitch_rms_light_deg,
            vertical_accel_rms_light_ms2,
            depth_peak_to_peak_light_m,
        )
        if any(value <= 0.0 for value in thresholds):
            raise ValueError('Dalga esikleri pozitif olmali')
        if not 0.0 < minimum_fill_ratio <= 1.0:
            raise ValueError('Minimum pencere doluluk orani gecersiz')
        if moderate_score <= 1.0 or severe_score <= moderate_score:
            raise ValueError('Dalga skor esikleri sirali olmali')

        self.window_s = float(window_s)
        self.sample_rate_hz = float(sample_rate_hz)
        self.minimum_fill_ratio = float(minimum_fill_ratio)
        self.roll_light = float(roll_rms_light_deg)
        self.pitch_light = float(pitch_rms_light_deg)
        self.accel_light = float(vertical_accel_rms_light_ms2)
        self.depth_light = float(depth_peak_to_peak_light_m)
        self.moderate_score = float(moderate_score)
        self.severe_score = float(severe_score)
        self.enter_hold_s = max(0.0, float(enter_hold_s))
        self.exit_hold_s = max(0.0, float(exit_hold_s))
        self.min_period_s = float(min_period_s)
        self.max_period_s = float(max_period_s)

        maximum_samples = max(
            8,
            int(math.ceil(self.window_s * self.sample_rate_hz * 1.5)),
        )
        self._samples = deque(maxlen=maximum_samples)
        self._severity = WAVE_CALM
        self._candidate = WAVE_CALM
        self._candidate_since_s = None

    def reset(self):
        """Ornek ve histerezis gecmisini temizle."""
        self._samples.clear()
        self._severity = WAVE_CALM
        self._candidate = WAVE_CALM
        self._candidate_since_s = None

    def update(self, sample: WaveSample, maneuver_suppressed=False):
        """Yeni ornegi ekleyip guncel dalga kestirimini dondur."""
        values = (
            sample.time_s,
            sample.roll_rad,
            sample.pitch_rad,
            sample.vertical_accel_ms2,
            sample.depth_m,
        )
        if not all(math.isfinite(value) for value in values):
            return self._empty_estimate(maneuver_suppressed)
        if self._samples and sample.time_s <= self._samples[-1].time_s:
            return self._empty_estimate(maneuver_suppressed)

        self._samples.append(sample)
        cutoff_s = sample.time_s - self.window_s
        while len(self._samples) > 3:
            if self._samples[0].time_s >= cutoff_s:
                break
            self._samples.popleft()

        minimum_samples = max(
            8,
            int(self.window_s * self.sample_rate_hz
                * self.minimum_fill_ratio),
        )
        span_s = (
            self._samples[-1].time_s - self._samples[0].time_s
            if len(self._samples) >= 2 else 0.0
        )
        minimum_span_s = self.window_s * self.minimum_fill_ratio
        if len(self._samples) < minimum_samples or span_s < minimum_span_s:
            return self._empty_estimate(
                maneuver_suppressed,
                sample_count=len(self._samples),
                window_span_s=span_s,
            )

        times = [item.time_s for item in self._samples]
        rolls = _linear_detrend(
            times,
            [item.roll_rad for item in self._samples],
        )
        pitches = _linear_detrend(
            times,
            [item.pitch_rad for item in self._samples],
        )
        accelerations = _linear_detrend(
            times,
            [item.vertical_accel_ms2 for item in self._samples],
        )
        depths = _linear_detrend(
            times,
            [item.depth_m for item in self._samples],
        )

        roll_rms_deg = math.degrees(_rms(rolls))
        pitch_rms_deg = math.degrees(_rms(pitches))
        acceleration_rms = _rms(accelerations)
        depth_peak_to_peak = max(depths) - min(depths)

        ratios = [
            roll_rms_deg / self.roll_light,
            pitch_rms_deg / self.pitch_light,
            acceleration_rms / self.accel_light,
            depth_peak_to_peak / self.depth_light,
        ]
        active_channels = sum(ratio >= 1.0 for ratio in ratios)
        angular_score = max(ratios[0], ratios[1])
        heave_score = max(ratios[2], ratios[3])
        score = 0.45 * angular_score + 0.55 * heave_score

        raw_severity = WAVE_CALM
        if score >= self.severe_score:
            raw_severity = WAVE_SEVERE
        elif score >= self.moderate_score:
            raw_severity = WAVE_MODERATE
        elif score >= 1.0:
            raw_severity = WAVE_LIGHT

        # Tek bir kanaldaki gurultu siddetli dalga karari veremez.
        if active_channels < 2:
            raw_severity = min(raw_severity, WAVE_LIGHT)

        dominant_signal = depths
        if depth_peak_to_peak < self.depth_light * 0.5:
            dominant_signal = (
                rolls if _rms(rolls) >= _rms(pitches) else pitches
            )
        dominant_period = _estimate_period(
            times,
            dominant_signal,
            self.min_period_s,
            self.max_period_s,
        )

        fill_confidence = clamp(span_s / self.window_s, 0.0, 1.0)
        agreement_confidence = clamp(active_channels / 3.0, 0.0, 1.0)
        periodic_confidence = 1.0 if math.isfinite(
            dominant_period) else 0.55
        confidence = (
            0.35 * fill_confidence
            + 0.40 * agreement_confidence
            + 0.25 * periodic_confidence
        )
        if maneuver_suppressed:
            confidence *= 0.5
            raw_severity = min(raw_severity, WAVE_LIGHT)
        confidence = clamp(confidence, 0.0, 1.0)

        self._update_hysteresis(raw_severity, sample.time_s)
        return WaveEstimate(
            ready=True,
            severity=self._severity,
            raw_severity=raw_severity,
            confidence=confidence,
            score=score,
            roll_rms_deg=roll_rms_deg,
            pitch_rms_deg=pitch_rms_deg,
            vertical_accel_rms_ms2=acceleration_rms,
            depth_peak_to_peak_m=depth_peak_to_peak,
            dominant_period_s=dominant_period,
            sample_count=len(self._samples),
            window_span_s=span_s,
            maneuver_suppressed=bool(maneuver_suppressed),
        )

    def _update_hysteresis(self, raw_severity, now_s):
        """Seviye titremesini giris/cikis bekleme sureleriyle engelle."""
        if raw_severity == self._severity:
            self._candidate = raw_severity
            self._candidate_since_s = None
            return
        if raw_severity != self._candidate:
            self._candidate = raw_severity
            self._candidate_since_s = now_s
            return
        if self._candidate_since_s is None:
            self._candidate_since_s = now_s
            return
        hold_s = (
            self.enter_hold_s
            if raw_severity > self._severity else self.exit_hold_s
        )
        if now_s - self._candidate_since_s >= hold_s:
            self._severity = raw_severity
            self._candidate_since_s = None

    def _empty_estimate(
        self,
        maneuver_suppressed=False,
        sample_count=None,
        window_span_s=0.0,
    ):
        """Pencere hazir degilken guvenli bos kestirim olustur."""
        count = len(self._samples) if sample_count is None else sample_count
        return WaveEstimate(
            ready=False,
            severity=self._severity,
            raw_severity=WAVE_CALM,
            confidence=0.0,
            score=0.0,
            roll_rms_deg=0.0,
            pitch_rms_deg=0.0,
            vertical_accel_rms_ms2=0.0,
            depth_peak_to_peak_m=0.0,
            dominant_period_s=math.nan,
            sample_count=count,
            window_span_s=window_span_s,
            maneuver_suppressed=bool(maneuver_suppressed),
        )
