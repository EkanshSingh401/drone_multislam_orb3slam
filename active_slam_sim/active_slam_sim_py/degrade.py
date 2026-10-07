"""Camera degradation model for the simulated D455 IR stream (overnight Stage 5).

The Gazebo D455-mirror rig renders noise-free, blur-free, perfectly exposed IR
images (docker/OPEN_ISSUES.md s1). This module degrades one 8-bit mono frame the
way a real D455 IR imager (OmniVision OV9782, global shutter) degrades it:

  1. exposure / low light   scene radiance scaled by `light` (1 = as rendered),
                            then digital `gain` applied (auto-exposure compensating
                            a dark scene raises gain, which amplifies noise);
  2. motion blur            linear kernel from the camera's angular velocity during
                            the exposure: image-plane flow at the principal point
                            is f*(-w_y, w_x) px/s (optical frame), length
                            |flow| * exposure_s; rotation about the optical axis and
                            translation (depth-dependent) are not modelled;
  3. sensor noise           shot noise (Poisson-like, sigma = sqrt(signal/k_e) DN
                            with k_e electrons per DN) + read noise (sigma_read DN),
                            both scaled by gain, then 8-bit quantization/clipping.

Levels are named presets; `clean` is the identity. Deterministic given `seed`.
"""
from dataclasses import dataclass

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


@dataclass(frozen=True)
class Degradation:
    name: str
    light: float = 1.0        # scene brightness factor before gain
    gain: float = 1.0         # digital gain (auto-exposure)
    exposure_s: float = 0.0   # exposure time used for motion blur
    sigma_read: float = 0.0   # read noise [DN at gain 1]
    k_e: float = 0.0          # electrons per DN for shot noise (0 = off)


# Presets roughly bracketing a real D455 IR stream (8-bit, 848x480 @ 30 Hz):
#   mild     indoor, good light: ~3 ms exposure, read noise ~1.5 DN
#   moderate dim room: half the light, gain 2, 10 ms exposure
#   severe   low light: quarter light, gain 4, 20 ms exposure (motion blur visible)
PRESETS = {
    "clean": Degradation("clean"),
    "mild": Degradation("mild", light=1.0, gain=1.0, exposure_s=0.003, sigma_read=1.5, k_e=4.0),
    "moderate": Degradation("moderate", light=0.5, gain=2.0, exposure_s=0.010, sigma_read=2.0, k_e=4.0),
    "severe": Degradation("severe", light=0.25, gain=4.0, exposure_s=0.020, sigma_read=2.5, k_e=4.0),
}


def motion_blur_kernel(flow_px: np.ndarray) -> np.ndarray:
    """Normalized line kernel for an image-plane displacement vector (pixels)."""
    L = float(np.hypot(*flow_px))
    n = max(1, int(np.ceil(L)))
    if n <= 1:
        return np.ones((1, 1), np.float32)
    size = 2 * n + 1
    k = np.zeros((size, size), np.float32)
    for s in np.linspace(-0.5, 0.5, 4 * n + 1):
        x = n + s * flow_px[0]
        y = n + s * flow_px[1]
        k[int(round(y)), int(round(x))] += 1.0
    return k / k.sum()


def degrade(img: np.ndarray, d: Degradation, omega_cam: np.ndarray, fx: float, fy: float,
            rng: np.random.Generator) -> np.ndarray:
    """Degrade one uint8 mono frame. omega_cam: camera angular velocity (rad/s) in the
    OPTICAL frame (x right, y down, z forward), averaged over the exposure."""
    if d.name == "clean":
        return img
    x = img.astype(np.float32)
    # 1. scene light
    x *= d.light
    # 2. motion blur (applied to the scene signal, before noise)
    if d.exposure_s > 0.0:
        flow = np.array([-omega_cam[1] * fx, omega_cam[0] * fy], np.float32) * d.exposure_s
        k = motion_blur_kernel(flow)
        if k.size > 1:
            x = cv2.filter2D(x, -1, k, borderType=cv2.BORDER_REFLECT)
    # 3. shot + read noise, then gain
    if d.k_e > 0.0:
        x = x + rng.normal(0.0, 1.0, x.shape).astype(np.float32) * np.sqrt(np.maximum(x, 0.0) / d.k_e)
    if d.sigma_read > 0.0:
        x = x + rng.normal(0.0, d.sigma_read, x.shape).astype(np.float32)
    x *= d.gain
    return np.clip(np.rint(x), 0, 255).astype(np.uint8)
