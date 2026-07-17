"""ctypes binding for the Rust swarm_core estimator (one instance per widget).

Loads the cdylib from Bazel runfiles; every widget in the simulation owns an
`Estimator` and feeds it exactly what the real firmware would see: raw IMU
samples, UWB range/AoA measurements tagged with the neighbor's advertised
pose + quality, and optional GPS/altimeter/magnetometer inputs.
"""

import ctypes
import os

_LIB = None


def _find_library():
    """Locate the cdylib next to this file in the Bazel runfiles tree."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, "..", "core", name)
        for name in ("libswarm_core_cdylib.so", "libswarm_core_cdylib.dylib")
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    raise OSError("swarm_core cdylib not found near %s — build via bazel" % here)


def _lib():
    global _LIB
    if _LIB is None:
        lib = ctypes.CDLL(_find_library())
        f32p = ctypes.POINTER(ctypes.c_float)
        f32 = ctypes.c_float
        sigs = {
            "swarm_estimator_new": ([f32] * 5, ctypes.c_void_p),
            "swarm_estimator_free": ([ctypes.c_void_p], None),
            "swarm_estimator_set_pose":
                ([ctypes.c_void_p, f32p, f32p, f32p, f32, f32, f32], None),
            "swarm_estimator_predict": ([ctypes.c_void_p, f32p, f32p, f32], None),
            "swarm_estimator_update_range":
                ([ctypes.c_void_p, f32p, f32, f32, f32], ctypes.c_int),
            "swarm_estimator_update_aoa":
                ([ctypes.c_void_p, f32p, f32, f32p, f32], ctypes.c_int),
            "swarm_estimator_update_gps":
                ([ctypes.c_void_p, f32p, f32, f32], ctypes.c_int),
            "swarm_estimator_update_alt": ([ctypes.c_void_p, f32, f32], ctypes.c_int),
            "swarm_estimator_update_mag":
                ([ctypes.c_void_p, f32p, f32p, f32], ctypes.c_int),
            "swarm_estimator_update_gravity":
                ([ctypes.c_void_p, f32p, f32], ctypes.c_int),
            "swarm_estimator_state": ([ctypes.c_void_p, f32p], None),
            "swarm_estimator_pos_cov": ([ctypes.c_void_p, f32p], None),
        }
        for name, (argtypes, restype) in sigs.items():
            fn = getattr(lib, name)
            fn.argtypes = argtypes
            fn.restype = restype
        _LIB = lib
    return _LIB


def _f3(v):
    return (ctypes.c_float * 3)(*[float(x) for x in v])


def _f4(v):
    return (ctypes.c_float * 4)(*[float(x) for x in v])


class Estimator:
    """Handle to one Rust ESKF instance."""

    def __init__(self, accel_noise=0.0, gyro_noise=0.0, accel_bias_rw=0.0,
                 gyro_bias_rw=0.0, gate=0.0):
        """Zeros mean "use the Rust-side default"; gate=-1 disables gating."""
        self._lib = _lib()
        self._h = self._lib.swarm_estimator_new(
            accel_noise, gyro_noise, accel_bias_rw, gyro_bias_rw, gate)

    def __del__(self):
        if getattr(self, "_h", None):
            self._lib.swarm_estimator_free(self._h)
            self._h = None

    def set_pose(self, p, v=(0, 0, 0), q=(1, 0, 0, 0),
                 pos_sigma=1.0, vel_sigma=0.1, att_sigma=0.1):
        self._lib.swarm_estimator_set_pose(
            self._h, _f3(p), _f3(v), _f4(q), pos_sigma, vel_sigma, att_sigma)

    def predict(self, accel, gyro, dt):
        self._lib.swarm_estimator_predict(self._h, _f3(accel), _f3(gyro), dt)

    def update_range(self, neighbor_p, neighbor_var, measured, sigma):
        return bool(self._lib.swarm_estimator_update_range(
            self._h, _f3(neighbor_p), neighbor_var, measured, sigma))

    def update_aoa(self, neighbor_p, neighbor_var, bearing_body, sigma):
        return self._lib.swarm_estimator_update_aoa(
            self._h, _f3(neighbor_p), neighbor_var, _f3(bearing_body), sigma)

    def update_gps(self, measured, sigma_h, sigma_v):
        return self._lib.swarm_estimator_update_gps(
            self._h, _f3(measured), sigma_h, sigma_v)

    def update_alt(self, z, sigma):
        return bool(self._lib.swarm_estimator_update_alt(self._h, z, sigma))

    def update_mag(self, mag_body, mag_world, sigma):
        return self._lib.swarm_estimator_update_mag(
            self._h, _f3(mag_body), _f3(mag_world), sigma)

    def update_gravity(self, accel, sigma):
        return self._lib.swarm_estimator_update_gravity(self._h, _f3(accel), sigma)

    def state(self):
        """dict with p, v, q (wxyz), ba, bg, pos_var."""
        out = (ctypes.c_float * 17)()
        self._lib.swarm_estimator_state(self._h, out)
        return {
            "p": list(out[0:3]),
            "v": list(out[3:6]),
            "q": list(out[6:10]),
            "ba": list(out[10:13]),
            "bg": list(out[13:16]),
            "pos_var": out[16],
        }

    def pos_cov(self):
        """3x3 position covariance, row-major nested lists."""
        out = (ctypes.c_float * 9)()
        self._lib.swarm_estimator_pos_cov(self._h, out)
        return [list(out[i * 3:(i + 1) * 3]) for i in range(3)]
