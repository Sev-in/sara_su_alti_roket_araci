#!/usr/bin/env python3
"""ROS'tan bagimsiz sensor donusum ve filtreleme yardimcilari.

Bu modul donanim surucusu ile navigasyon katmaninin sorumluluklarini
ayirir:

* Donanim surucusu ADS1115 gerilimini SEN0257 basincina donusturebilir.
* Navigasyon, standart ``sensor_msgs/FluidPressure`` mesajindaki Pa
  degerini filtreleyip derinlige donusturur.

Modul ROS2 veya Jetson kutuphanelerini icermedigi icin masaustu ortaminda
birim testine uygundur.
"""

import math
import statistics
from collections import deque


SEN0257_FULL_SCALE_PA = 1_600_000.0
SEN0257_NOMINAL_ZERO_V = 0.5
SEN0257_OUTPUT_SPAN_V = 4.0


def clamp(value: float, lower: float, upper: float) -> float:
    """Degeri kapali ``[lower, upper]`` araligina sinirlar."""
    return max(lower, min(upper, value))


def voltage_divider_reconstruct(
    adc_voltage_v: float,
    r1_ohm: float,
    r2_ohm: float,
) -> float:
    """Gerilim bolucuden onceki sensor cikis gerilimini hesaplar.

    ``R1`` sensor cikisi ile ADC dugumu, ``R2`` ADC dugumu ile GND
    arasindadir.
    """
    if not math.isfinite(adc_voltage_v):
        raise ValueError('ADC gerilimi sonlu olmali')
    if r1_ohm < 0.0 or r2_ohm <= 0.0:
        raise ValueError('R1 >= 0 ve R2 > 0 olmali')
    return adc_voltage_v * (r1_ohm + r2_ohm) / r2_ohm


def sen0257_voltage_to_pressure(
    sensor_voltage_v: float,
    zero_offset_v: float = SEN0257_NOMINAL_ZERO_V,
    full_scale_pa: float = SEN0257_FULL_SCALE_PA,
    output_span_v: float = SEN0257_OUTPUT_SPAN_V,
    clamp_to_range: bool = True,
) -> float:
    """SEN0257 analog cikisini gauge basinca (Pa) donusturur.

    Projedeki sensor 0--1.6 MPa ve nominal 0.5--4.5 V varyantidir.
    ``zero_offset_v`` sahada sensor yuksuzken olculen gercek sifir
    gerilimiyle degistirilebilir.
    """
    values = (
        sensor_voltage_v,
        zero_offset_v,
        full_scale_pa,
        output_span_v,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('SEN0257 donusum girdileri sonlu olmali')
    if full_scale_pa <= 0.0 or output_span_v <= 0.0:
        raise ValueError('Tam olcek ve cikis araligi pozitif olmali')

    ratio = (sensor_voltage_v - zero_offset_v) / output_span_v
    if clamp_to_range:
        ratio = clamp(ratio, 0.0, 1.0)
    return ratio * full_scale_pa


def water_sensor_logic(raw_gpio_value: bool, active_high: bool) -> bool:
    """SEN0368 GPIO seviyesini ``su_var`` anlamina donusturur."""
    raw = bool(raw_gpio_value)
    return raw if active_high else not raw


def pressure_to_depth(
    pressure_pa: float,
    surface_reference_pa: float,
    rho_kg_m3: float,
    gravity_m_s2: float,
    clamp_at_surface: bool = True,
) -> float:
    """Hidrostatik basinctan derinlik hesaplar.

    Derinlik ekseni projeyle uyumlu olarak su altina dogru pozitiftir.
    """
    values = (
        pressure_pa,
        surface_reference_pa,
        rho_kg_m3,
        gravity_m_s2,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Derinlik donusum girdileri sonlu olmali')
    if rho_kg_m3 <= 0.0 or gravity_m_s2 <= 0.0:
        raise ValueError('Su yogunlugu ve yercekimi pozitif olmali')

    depth = (
        (pressure_pa - surface_reference_pa)
        / (rho_kg_m3 * gravity_m_s2)
    )
    return max(0.0, depth) if clamp_at_surface else depth


def normalize_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
    minimum_norm: float = 0.5,
    maximum_norm: float = 1.5,
):
    """Gecerli quaternionu normalize eder, gecersizse ``None`` dondurur."""
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm < minimum_norm or norm > maximum_norm:
        return None
    return tuple(value / norm for value in values)


def robust_median(
    samples,
    sigma: float = 3.0,
    minimum_deviation: float = 1.0,
):
    """MAD ile aykiri degerleri temizleyip medyan ve yayilim dondurur.

    Donus degeri ``(merkez, kabul_edilen_ornekler, span)`` seklindedir.
    """
    finite_samples = [
        float(value) for value in samples if math.isfinite(value)
    ]
    if not finite_samples:
        raise ValueError('En az bir sonlu ornek gerekli')

    center = statistics.median(finite_samples)
    deviations = [abs(value - center) for value in finite_samples]
    mad = statistics.median(deviations)
    threshold = max(
        float(minimum_deviation),
        float(sigma) * 1.4826 * mad,
    )
    accepted = [
        value for value in finite_samples
        if abs(value - center) <= threshold
    ]
    if not accepted:
        accepted = [center]
    robust_center = statistics.median(accepted)
    span = max(accepted) - min(accepted)
    return robust_center, accepted, span


class TimeAwareLowPass:
    """Degisken ``dt`` ile ayni zaman sabitini koruyan birinci derece LPF."""

    def __init__(self, nominal_alpha: float, nominal_dt_s: float):
        if not 0.0 < nominal_alpha <= 1.0:
            raise ValueError('LPF alpha (0, 1] araliginda olmali')
        if nominal_dt_s <= 0.0:
            raise ValueError('Nominal dt pozitif olmali')

        self.nominal_alpha = float(nominal_alpha)
        self.nominal_dt_s = float(nominal_dt_s)
        self._tau_s = (
            0.0 if nominal_alpha >= 1.0
            else -nominal_dt_s / math.log(1.0 - nominal_alpha)
        )
        self._value = None

    @property
    def value(self):
        return self._value

    def reset(self, value=None):
        """Filtre gecmisini temizler veya bilinen bir degerle baslatir."""
        self._value = value

    def update(self, measurement: float, dt_s=None) -> float:
        if not math.isfinite(measurement):
            raise ValueError('LPF olcumu sonlu olmali')
        if self._value is None:
            self._value = float(measurement)
            return self._value

        dt = self.nominal_dt_s
        if dt_s is not None and math.isfinite(dt_s) and dt_s > 0.0:
            dt = float(dt_s)
        alpha = (
            1.0 if self._tau_s <= 0.0
            else 1.0 - math.exp(-dt / self._tau_s)
        )
        self._value += alpha * (measurement - self._value)
        return self._value


class HampelFilter:
    """Tekil sensor sicrama ve darbelerini MAD tabanli olarak reddeder."""

    def __init__(
        self,
        window_size: int = 7,
        sigma: float = 3.0,
        minimum_deviation: float = 1.0,
    ):
        if window_size < 3:
            raise ValueError('Hampel penceresi en az 3 olmali')
        self._history = deque(maxlen=int(window_size))
        self.sigma = float(sigma)
        self.minimum_deviation = float(minimum_deviation)

    def reset(self):
        self._history.clear()

    def update(self, measurement: float):
        """``(deger, aykiri_mi)`` dondurur.

        Aykiri olcum pencereye eklenmez; cikis olarak son saglam pencerenin
        medyani verilir.
        """
        if not math.isfinite(measurement):
            raise ValueError('Hampel olcumu sonlu olmali')

        if len(self._history) >= 3:
            center = statistics.median(self._history)
            deviations = [
                abs(value - center) for value in self._history
            ]
            mad = statistics.median(deviations)
            threshold = max(
                self.minimum_deviation,
                self.sigma * 1.4826 * mad,
            )
            if abs(measurement - center) > threshold:
                return center, True

        self._history.append(float(measurement))
        return float(measurement), False


class DebouncedBoolean:
    """Dijital sensor durumunu belirli sure kararlilik sonrasi kabul eder."""

    def __init__(self, debounce_s: float, initial=None):
        self.debounce_s = max(0.0, float(debounce_s))
        self.stable = initial
        self._candidate = initial
        self._candidate_since_s = None

    def reset(self, initial=None):
        self.stable = initial
        self._candidate = initial
        self._candidate_since_s = None

    def update(self, value: bool, now_s: float):
        """Karar verilmediyse ``None``, aksi halde kararli degeri dondurur."""
        value = bool(value)
        if not math.isfinite(now_s):
            return self.stable

        if value != self._candidate:
            self._candidate = value
            self._candidate_since_s = float(now_s)
        elif self._candidate_since_s is None:
            self._candidate_since_s = float(now_s)

        if self.stable == self._candidate:
            return self.stable
        if now_s - self._candidate_since_s >= self.debounce_s:
            self.stable = self._candidate
        return self.stable


class SlidingLinearRegression:
    """Zaman damgali son orneklere dogru uydurup egimi hesaplar."""

    def __init__(self, window_s: float, max_samples: int = 50):
        if window_s <= 0.0 or max_samples < 3:
            raise ValueError('Regresyon penceresi ve ornek sayisi gecersiz')
        self.window_s = float(window_s)
        self._samples = deque(maxlen=int(max_samples))

    def reset(self):
        self._samples.clear()

    def update(self, timestamp_s: float, value: float):
        """Yeni ornegi ekler; yeterli veri varsa egimi, yoksa ``None`` verir."""
        if not math.isfinite(timestamp_s) or not math.isfinite(value):
            return None
        if self._samples and timestamp_s <= self._samples[-1][0]:
            return None

        self._samples.append((float(timestamp_s), float(value)))
        cutoff = timestamp_s - self.window_s
        while len(self._samples) > 3 and self._samples[0][0] < cutoff:
            self._samples.popleft()

        if len(self._samples) < 3:
            return None
        times = [sample[0] for sample in self._samples]
        values = [sample[1] for sample in self._samples]
        time_span = times[-1] - times[0]
        if time_span <= 0.0:
            return None

        mean_t = sum(times) / len(times)
        mean_v = sum(values) / len(values)
        denominator = sum((time - mean_t) ** 2 for time in times)
        if denominator <= 0.0:
            return None
        numerator = sum(
            (time - mean_t) * (sample - mean_v)
            for time, sample in zip(times, values)
        )
        return numerator / denominator
