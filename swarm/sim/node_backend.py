"""ctypes binding for the swarm node runtime (swarm/node, feature "sim"):
the firmware's per-widget logic — estimator, cooperative rules, stationary
detection, auto-anchoring, planar mode, centroid bleed, settings registry —
one instance per simulated widget, fed synthetic sensors by world.py."""

import ctypes
import os
import struct

_LIB = None
STATE_LEN = 16


def _find_library():
    here = os.path.dirname(os.path.abspath(__file__))
    # rust_shared_library names the file after the crate (swarm_node).
    for name in ("libswarm_node.so", "libswarm_node.dylib"):
        path = os.path.join(here, "..", "node", name)
        if os.path.exists(path):
            return path
    raise OSError("swarm_node_sim cdylib not found near %s — build via bazel" % here)


def _lib():
    global _LIB
    if _LIB is None:
        lib = ctypes.CDLL(_find_library())
        vp, f32p, u8p = ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_uint8)
        i64, u32 = ctypes.c_int64, ctypes.c_uint32
        sigs = {
            "swarm_sim_node_new": ([u32, ctypes.c_bool], vp),
            "swarm_sim_node_free": ([vp], None),
            "swarm_sim_node_settings": ([vp, ctypes.POINTER(ctypes.c_uint16), ctypes.POINTER(u32), ctypes.c_size_t],
                                        ctypes.c_size_t),
            "swarm_sim_setting_id": ([u8p, ctypes.c_size_t, ctypes.POINTER(ctypes.c_int32)], ctypes.c_int32),
            "swarm_sim_node_imu": ([vp, i64, f32p, f32p], None),
            "swarm_sim_node_tick": ([vp, i64], None),
            "swarm_sim_node_mag": ([vp, i64, f32p], None),
            "swarm_sim_node_range": ([vp, i64, u32, ctypes.c_float], ctypes.c_int32),
            "swarm_sim_node_advert_tx": ([vp, u8p, ctypes.c_size_t], ctypes.c_size_t),
            "swarm_sim_node_advert_rx": ([vp, i64, u8p, ctypes.c_size_t], u32),
            "swarm_sim_node_state": ([vp, i64, f32p], None),
        }
        for name, (args, res) in sigs.items():
            fn = getattr(lib, name)
            fn.argtypes = args
            fn.restype = res
        _LIB = lib
    return _LIB


def _vec(v):
    return (ctypes.c_float * 3)(*[float(x) for x in v])


def setting_raw(name, value):
    """(registry id, raw u32) for a setting name and Python value."""
    lib = _lib()
    kind = ctypes.c_int32()
    b = name.encode()
    sid = lib.swarm_sim_setting_id((ctypes.c_uint8 * len(b)).from_buffer_copy(b), len(b), ctypes.byref(kind))
    if sid < 0:
        raise KeyError("unknown setting %r" % name)
    if kind.value == 0:
        raw = struct.unpack("<I", struct.pack("<f", float(value)))[0]
    elif kind.value == 1:
        raw = int(value) & 0xFFFFFFFF
    else:
        raw = 1 if (value is True or str(value).lower() in ("1", "true")) else 0
    return sid, raw


class Node:
    """One widget's firmware runtime."""

    def __init__(self, wid, has_imu=True, settings=None):
        self._lib = _lib()
        self.wid = wid
        self._h = self._lib.swarm_sim_node_new(wid, has_imu)
        if settings:
            self.apply(settings)

    def __del__(self):
        if getattr(self, "_h", None):
            self._lib.swarm_sim_node_free(self._h)
            self._h = None

    def apply(self, settings):
        """settings: {name: value}, an override layer over the defaults."""
        pairs = [setting_raw(k, v) for k, v in settings.items()]
        ids = (ctypes.c_uint16 * len(pairs))(*[p[0] for p in pairs])
        raws = (ctypes.c_uint32 * len(pairs))(*[p[1] for p in pairs])
        return self._lib.swarm_sim_node_settings(self._h, ids, raws, len(pairs))

    def imu(self, t, accel, gyro):
        self._lib.swarm_sim_node_imu(self._h, int(t * 1e6), _vec(accel), _vec(gyro))

    def tick(self, t):
        self._lib.swarm_sim_node_tick(self._h, int(t * 1e6))

    def mag(self, t, raw):
        self._lib.swarm_sim_node_mag(self._h, int(t * 1e6), _vec(raw))

    def range(self, t, peer, r):
        return self._lib.swarm_sim_node_range(self._h, int(t * 1e6), peer, float(r))

    def advert_tx(self):
        buf = (ctypes.c_uint8 * 64)()
        n = self._lib.swarm_sim_node_advert_tx(self._h, buf, 64)
        return bytes(buf[:n])

    def advert_rx(self, t, data):
        if not data:
            return 0
        b = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
        return self._lib.swarm_sim_node_advert_rx(self._h, int(t * 1e6), b, len(data))

    def state(self, t):
        o = (ctypes.c_float * STATE_LEN)()
        self._lib.swarm_sim_node_state(self._h, int(t * 1e6), o)
        return {"p": list(o[0:3]), "v": list(o[3:6]), "q": list(o[6:10]), "pos_var": o[10], "lock": o[11],
                "still": o[12] > 0.5, "anchored": o[13] > 0.5, "initialized": o[14] > 0.5,
                "regains": int(o[15])}
