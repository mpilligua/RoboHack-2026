"""Naive RGB pixel → depth lookup (numpy only; no ROS).

Used by Lite3Robot and offline tests.
"""

from __future__ import annotations

from typing import Any, Optional

import numpy as np


def compute_depth_at_rgb_pixel_naive(
    *,
    rgb_width: int,
    rgb_height: int,
    depth_mm: np.ndarray,
    u_rgb: int,
    v_rgb: int,
    window_radius: int = 3,
    max_depth_mm: int = 10_000,
) -> dict[str, Any]:
    """Map RGB (u,v) into depth image by scaling + nearest-valid fallback."""
    if depth_mm.ndim != 2:
        return {"error": "depth_mm_must_be_2d"}

    if window_radius < 0:
        return {"error": "invalid_window_radius", "window_radius": window_radius}

    def depth_ok(mm: int) -> bool:
        return 0 < mm < max_depth_mm

    if u_rgb < 0 or u_rgb >= rgb_width or v_rgb < 0 or v_rgb >= rgb_height:
        return {
            "error": "rgb_pixel_out_of_bounds",
            "rgb_width": rgb_width,
            "rgb_height": rgb_height,
            "rgb_pixel": {"u": u_rgb, "v": v_rgb},
        }

    dh, dw = depth_mm.shape
    rw, rh = rgb_width, rgb_height
    ud = int(round(u_rgb * dw / rw))
    vd = int(round(v_rgb * dh / rh))
    ud = max(0, min(dw - 1, ud))
    vd = max(0, min(dh - 1, vd))

    mm0 = int(depth_mm[vd, ud])
    base = {
        "rgb_pixel": {"u": u_rgb, "v": v_rgb},
        "naive_mapped_depth_pixel": {"u": ud, "v": vd},
        "rgb_shape": {"width": rw, "height": rh},
        "depth_shape": {"width": dw, "height": dh},
        "window_radius": window_radius,
        "note": "naive RGB→depth scaling; approximate vs RealSense alignment",
    }

    if depth_ok(mm0):
        return {
            **base,
            "depth_pixel": {"u": ud, "v": vd},
            "depth_mm": mm0,
            "depth_m": round(mm0 / 1000.0, 6),
            "source": "direct",
        }

    best_mm: Optional[int] = None
    best_u = best_v = None
    best_d2: Optional[int] = None
    for dv in range(-window_radius, window_radius + 1):
        for du in range(-window_radius, window_radius + 1):
            nu, nv = ud + du, vd + dv
            if nu < 0 or nu >= dw or nv < 0 or nv >= dh:
                continue
            mm = int(depth_mm[nv, nu])
            if not depth_ok(mm):
                continue
            d2 = du * du + dv * dv
            if best_d2 is None or d2 < best_d2:
                best_d2 = d2
                best_mm = mm
                best_u, best_v = nu, nv

    if best_mm is None:
        return {
            **base,
            "error": "no_valid_depth_in_window",
            "depth_mm_direct_invalid": mm0,
        }

    return {
        **base,
        "depth_pixel": {"u": best_u, "v": best_v},
        "depth_mm": best_mm,
        "depth_m": round(best_mm / 1000.0, 6),
        "source": "nearest_valid",
        "nearest_pixel_distance_sq": best_d2,
    }
