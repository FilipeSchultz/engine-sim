"""Combustion analysis of one cylinder cycle, as done by an indicating system.

All angles are in degrees relative to the cylinder's firing TDC (positive = after).
The volume is computed from the geometry entered by the operator, not received
from the engine: wrong geometry gives wrong IMEP / heat release, as in a test cell.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Geometry:
    bore_mm: float = 100.0
    stroke_mm: float = 90.0
    rod_mm: float = 150.0
    compression_ratio: float = 10.0

    def volume(self, theta_deg: np.ndarray) -> np.ndarray:
        """Cylinder volume [m^3] from the slider-crank kinematics."""
        bore = self.bore_mm / 1000.0
        r = self.stroke_mm / 2000.0
        l = self.rod_mm / 1000.0
        area = np.pi * bore * bore / 4.0
        displacement = area * 2 * r
        clearance = displacement / max(self.compression_ratio - 1.0, 1e-6)

        theta = np.radians(theta_deg)
        sin = np.sin(theta)
        x = r * (1 - np.cos(theta)) + l - np.sqrt(np.maximum(l * l - (r * sin) ** 2, 0.0))
        return clearance + area * x

    @property
    def displacement(self) -> float:
        bore = self.bore_mm / 1000.0
        return np.pi * bore * bore / 4.0 * self.stroke_mm / 1000.0


@dataclass
class Settings:
    gamma: float = 1.33             # ratio of specific heats for the heat release
    pegging: str = "thermodynamic"  # "none" or "thermodynamic"
    polytropic_n: float = 1.32      # compression polytropic exponent for pegging
    peg_angles: tuple = (-100.0, -65.0)


@dataclass
class CycleResult:
    cylinder: int
    pmax_bar: float
    pmax_angle: float
    imep_net_bar: float
    imep_gross_bar: float
    ca10: float
    ca50: float
    ca90: float
    peg_offset_bar: float
    misfire: bool

    @property
    def burn_duration(self) -> float:
        return self.ca90 - self.ca10


@dataclass
class CycleCurves:
    theta: np.ndarray       # [deg]
    pressure: np.ndarray    # [bar], pegged
    volume: np.ndarray      # [m^3]
    mfb: np.ndarray         # 0..1 over the combustion window, NaN outside
    heat_rate: np.ndarray   # apparent heat release rate [J/deg]


def _interp_at(theta: np.ndarray, values: np.ndarray, at: float) -> float:
    return float(np.interp(at, theta, values))


def _crossing(theta: np.ndarray, values: np.ndarray, fraction: float) -> float:
    above = np.nonzero(values >= fraction)[0]
    if len(above) == 0:
        return float("nan")
    k = above[0]
    if k == 0:
        return float(theta[0])
    v0, v1 = values[k - 1], values[k]
    t = (fraction - v0) / (v1 - v0) if v1 > v0 else 0.0
    return float(theta[k - 1] + t * (theta[k] - theta[k - 1]))


def analyze_cycle(cylinder: int, theta: np.ndarray, pressure_bar: np.ndarray,
                  geometry: Geometry, settings: Settings):
    """theta: [-360, 360) deg around the firing TDC; pressure: measured [bar]."""
    volume = geometry.volume(theta)

    # Pegging: piezo sensors only measure pressure changes. Thermodynamic
    # pegging assumes a polytropic compression between two angles.
    offset = 0.0
    if settings.pegging == "thermodynamic":
        a1, a2 = settings.peg_angles
        p1 = _interp_at(theta, pressure_bar, a1)
        p2 = _interp_at(theta, pressure_bar, a2)
        v1 = float(geometry.volume(np.array([a1]))[0])
        v2 = float(geometry.volume(np.array([a2]))[0])
        n = settings.polytropic_n
        offset = (p2 * v2 ** n - p1 * v1 ** n) / (v1 ** n - v2 ** n)
    p_bar = pressure_bar + offset
    p = p_bar * 1e5

    # IMEP: net over the full cycle, gross over compression + expansion
    work_net = np.trapezoid(p, volume)
    window = (theta >= -180.0) & (theta <= 180.0)
    work_gross = np.trapezoid(p[window], volume[window])
    displacement = geometry.displacement

    # Peak pressure
    k_max = int(np.argmax(np.where(window, p_bar, -np.inf)))

    # Apparent heat release over the closed part of the cycle
    th = theta[window]
    pw = p[window]
    vw = volume[window]
    g = settings.gamma
    dv = np.diff(vw)
    dp = np.diff(pw)
    pm = 0.5 * (pw[1:] + pw[:-1])
    vm = 0.5 * (vw[1:] + vw[:-1])
    dq = g / (g - 1) * pm * dv + 1 / (g - 1) * vm * dp
    q = np.concatenate([[0.0], np.cumsum(dq)])

    i_end = int(np.argmax(q))
    i_start = int(np.argmin(q[: i_end + 1])) if i_end > 0 else 0
    released = q[i_end] - q[i_start]
    misfire = released <= 1.0 or i_end <= i_start

    mfb_window = np.full_like(q, np.nan)
    ca10 = ca50 = ca90 = float("nan")
    if not misfire:
        mfb = np.clip((q - q[i_start]) / released, 0.0, 1.0)
        mfb[:i_start] = 0.0
        mfb[i_end:] = 1.0
        mfb_window = mfb
        ca10 = _crossing(th, mfb, 0.1)
        ca50 = _crossing(th, mfb, 0.5)
        ca90 = _crossing(th, mfb, 0.9)

    dtheta = np.diff(th)
    heat_rate = np.concatenate([[0.0], np.divide(dq, dtheta, out=np.zeros_like(dq), where=dtheta > 0)])

    result = CycleResult(
        cylinder=cylinder,
        pmax_bar=float(p_bar[k_max]),
        pmax_angle=float(theta[k_max]),
        imep_net_bar=float(work_net / displacement / 1e5),
        imep_gross_bar=float(work_gross / displacement / 1e5),
        ca10=ca10, ca50=ca50, ca90=ca90,
        peg_offset_bar=float(offset),
        misfire=misfire,
    )

    full_mfb = np.full_like(theta, np.nan)
    full_mfb[window] = mfb_window
    full_rate = np.full_like(theta, np.nan)
    full_rate[window] = heat_rate

    return result, CycleCurves(theta, p_bar, volume, full_mfb, full_rate)
