"""Mesh-radio channel model - the same maths and numbers as the Python simulator
(simulation/communication.py + config/parameters.yaml `communication:`), so results from
the Gazebo stack and the fast simulator rest on one set of communication assumptions.

    received  Prx = Ptx + Gtx + Grx - PL(d) - extra loss + fading,  fading ~ N(0, sigma)
    path loss PL(d) = PL0 + 10 n log10(d / 1 m)
    PDR       logistic in SNR: 1 / (1 + exp(-(SNR - snr_mid) / snr_slope)), 0 beyond max_range
    latency   hop_latency + retx_latency * (1 / PDR - 1)
    link up/down with hysteresis (up >= link_up_pdr, down < link_down_pdr)

With the defaults a healthy link has 85 % PDR at 100 m and nothing beyond 100 m:
the organisers' 100 m radio range.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, fields


@dataclass
class RadioParams:
    tx_power_dbm: float = 12.6
    uav_antenna_gain_dbi: float = 0.0
    gcs_antenna_gain_dbi: float = 0.0
    max_range_m: float = 100.0
    path_loss_ref_db: float = 40.0
    path_loss_exponent: float = 2.6
    noise_floor_dbm: float = -90.0
    snr_mid_db: float = 8.0
    snr_slope_db: float = 1.5
    fading_sigma_db: float = 1.0
    pdr_smoothing: float = 0.4
    link_up_pdr: float = 0.55
    link_down_pdr: float = 0.45
    hop_latency_ms: float = 5.0
    retx_latency_ms: float = 8.0
    update_interval_s: float = 0.5

    @classmethod
    def from_dict(cls, data: dict | None) -> "RadioParams":
        names = {f.name for f in fields(cls)}
        unknown = set(data or {}) - names
        if unknown:
            raise ValueError(f"unknown radio parameters: {sorted(unknown)}")
        return cls(**(data or {}))


def mean_snr_db(p: RadioParams, a, b, gcs_link: bool, extra_loss_db: float = 0.0) -> tuple[float, float]:
    d = max(1.0, math.dist(a, b))
    path_loss = p.path_loss_ref_db + 10.0 * p.path_loss_exponent * math.log10(d)
    gain = p.uav_antenna_gain_dbi + (p.gcs_antenna_gain_dbi if gcs_link else p.uav_antenna_gain_dbi)
    rx = p.tx_power_dbm + gain - path_loss - extra_loss_db
    return rx - p.noise_floor_dbm, d


def pdr_from_snr(p: RadioParams, snr_db: float) -> float:
    z = (snr_db - p.snr_mid_db) / p.snr_slope_db
    return 1.0 / (1.0 + math.exp(-max(-50.0, min(50.0, z))))


def predict_pdr(p: RadioParams, a, b, gcs_link: bool = False, world_map=None) -> float:
    """Expected PDR of a healthy link (no fading, no impairment) - used by planners.
    With a 3-D map, buildings on the radio path add their attenuation."""
    extra = world_map.los_loss_db(a, b) if world_map is not None else 0.0
    snr, d = mean_snr_db(p, a, b, gcs_link, extra)
    return 0.0 if d > p.max_range_m else pdr_from_snr(p, snr)


def latency_ms(p: RadioParams, pdr: float) -> float:
    return p.hop_latency_ms + p.retx_latency_ms * (1.0 / max(pdr, 0.05) - 1.0)


def range_for_pdr(p: RadioParams, target_pdr: float, gcs_link: bool = False) -> float:
    """Distance at which a healthy link reaches target_pdr (capped at max_range_m)."""
    snr_needed = p.snr_mid_db + p.snr_slope_db * math.log(target_pdr / (1.0 - target_pdr))
    gain = p.uav_antenna_gain_dbi + (p.gcs_antenna_gain_dbi if gcs_link else p.uav_antenna_gain_dbi)
    max_pl = p.tx_power_dbm + gain - p.noise_floor_dbm - snr_needed
    reach = 10 ** ((max_pl - p.path_loss_ref_db) / (10.0 * p.path_loss_exponent))
    return min(reach, p.max_range_m)
