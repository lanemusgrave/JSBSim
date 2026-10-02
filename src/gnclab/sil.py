"""Software-in-the-loop (SIL) harness: JSBSim plant <-> flight software over UDP.

The plant (this process) runs gnc_trainer with realistic actuators and
sensors, and the XML *lateral* loops (roll hold, yaw damper).  The longitudinal
autopilot runs in a **separate process**, the "flight computer"
(:func:`controller_main`), as :class:`gnclab.controllers.LongitudinalAP`.
The two talk over UDP on localhost, which is what a real SIL rig does with a flight
computer on the bench or a flight-software binary on a PC.

Packets (little-endian ``struct``; every field is documented in ``FMT``)::

    controller -> plant  HELO                            "I'm up, here's my address"
    plant -> controller  INIT  de_trim thr_trim theta_trim h0 vt0
    plant -> controller  SENS  k t theta q h_ft vt_fps alt_cmd_ft vt_cmd_fps
    controller -> plant  CMD_  k de_norm thr_norm
    plant -> controller  STOP

Two timing modes:

``lockstep``   The plant sends SENS for control frame k and **waits** for CMD k
               before stepping.  Deterministic and repeatable, faster than real
               time.  This is the CI/regression mode.  ``delay_frames`` adds a
               transport delay: CMD k is applied at frame k + delay_frames.
``realtime``   The plant paces itself to the wall clock and **never waits**:
               whatever command has arrived by the frame boundary is used
               (latest wins).  Network, OS scheduling and controller compute
               time now show up as real latency and jitter.  ``compute_ms``
               makes the controller artificially slow.

The plant returns a log at the control rate; the controller writes its own log
with its own clock, which is the raw material for the log-alignment lesson in Module 16.
"""

from __future__ import annotations

import math
import multiprocessing
import socket
import struct
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import numpy as np
import pandas as pd

FMT = {
    "HELO": "<4s",
    "INIT": "<4s5d",        # de_trim, thr_trim, theta_trim, h0_ft, vt0_fps
    "SENS": "<4sI7d",       # k, t, theta, q, h_ft, vt_fps, alt_cmd_ft, vt_cmd_fps
    "CMD_": "<4sI2d",       # k, de_norm, thr_norm
    "STOP": "<4s",
}


def pack(kind: str, *fields) -> bytes:
    return struct.pack(FMT[kind], kind.encode(), *fields)


def unpack(data: bytes) -> tuple[str, tuple]:
    kind = data[:4].decode()
    return kind, struct.unpack(FMT[kind], data)[1:]


# ---------------------------------------------------------------------------
# The flight computer
# ---------------------------------------------------------------------------
class _PacketView(dict):
    """The minimal 'fdm' that LongitudinalAP needs: a dict of property values."""


def controller_main(plant_addr, rate_hz: float, gains: dict | None = None, compute_ms: float = 0.0,
                    log_path: str | None = None, drain: bool = True, clock_offset_s: float = 0.0,
                    jitter_ms: float = 0.0, seed: int = 0) -> None:
    """Flight-software process: receive SENS, run the control law, send CMD_.

    ``drain``: before computing, read every queued packet and use only the
    newest ("latest wins").  Without it, a controller slower than its frame
    works through an ever-growing backlog of stale measurements.
    ``jitter_ms``: extra busy time per frame, uniform in [0, jitter_ms].
    ``clock_offset_s``: added to the log's clock, emulating a flight computer
    that was powered on that long before the test started.

    Top-level function so it can be the target of a *spawned* process.
    """
    t_boot = time.perf_counter() - clock_offset_s           # the flight computer's clock starts at power-on
    from gnclab.controllers import DEFAULT_GAINS, LongitudinalAP

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(10.0)
    sock.sendto(pack("HELO"), plant_addr)
    kind, (de0, thr0, th0, h0, v0) = unpack(sock.recv(256))
    assert kind == "INIT", kind
    view = _PacketView({"fcs/elevator-cmd-norm": de0, "fcs/throttle-cmd-norm[0]": thr0,
                        "attitude/theta-rad": th0, "position/h-sl-ft": h0, "velocities/vt-fps": v0})
    meas = {}
    ap = LongitudinalAP(view, rate_hz=rate_hz, gains=dict(gains or DEFAULT_GAINS), wing_leveler=False,
                        feedback=lambda _f: meas)
    rng = np.random.default_rng(seed)
    log = []
    while True:
        try:
            kind, f = unpack(sock.recv(256))
        except socket.timeout:
            break
        if drain:
            sock.setblocking(False)
            while kind != "STOP":
                try:
                    kind, f = unpack(sock.recv(256))
                except BlockingIOError:
                    break
            sock.settimeout(10.0)
        if kind == "STOP":
            break
        k, t, theta, q, h, vt, alt_cmd, vt_cmd = f
        t_rx = time.perf_counter() - t_boot
        meas.update(theta=theta, q=q, h_ft=h, vt_fps=vt)
        ap.alt_cmd, ap.vt_cmd = alt_cmd, vt_cmd
        de, dthr = ap.step(1.0 / rate_hz)
        busy = compute_ms + (rng.uniform(0, jitter_ms) if jitter_ms > 0 else 0.0)
        if busy > 0:                                        # a slow / busy flight computer
            end = time.perf_counter() + busy / 1000
            while time.perf_counter() < end:
                pass
        de_cmd = ap.de_trim + de
        thr_cmd = min(1.0, max(0.0, ap.dt_trim + dthr))
        sock.sendto(pack("CMD_", k, de_cmd, thr_cmd), plant_addr)
        log.append((t_rx, q, h, de_cmd, thr_cmd))           # NOTE: no frame number, own clock
    sock.close()
    if log_path:
        pd.DataFrame(log, columns=["t_fc", "q_meas", "h_meas_ft", "de_cmd", "thr_cmd"]).to_csv(log_path, index=False)


# ---------------------------------------------------------------------------
# The plant
# ---------------------------------------------------------------------------
@dataclass
class SILConfig:
    mode: str = "lockstep"            # "lockstep" | "realtime"
    rate_hz: float = 40.0             # controller frame rate; 120 Hz / rate must be an integer
    delay_frames: int = 0             # lockstep only: extra transport delay
    compute_ms: float = 0.0           # controller busy time per frame
    jitter_ms: float = 0.0            # + uniform random busy time in [0, jitter_ms]
    drain: bool = True                # controller keeps only the newest measurement
    fc_clock_offset_s: float = 0.0    # flight-computer log clock = wall time since "power-on"
    duration: float = 60.0
    turbulence_w20_fps: float = 0.0
    gains: dict | None = None
    card: Callable[[float, float, float], tuple[float, float]] | None = None  # (t, h0, v0) -> (alt_cmd, vt_cmd)
    controller_log: str | None = None
    seed: int = 1
    extra: dict = field(default_factory=dict)


def default_card(t: float, h0: float, v0: float) -> tuple[float, float]:
    """+100 ft at 2 s, +10 ft/s at 30 s, back to h0 at 50 s."""
    alt = h0 + (100.0 if 2.0 <= t < 50.0 else 0.0)
    vt = v0 + (10.0 if t >= 30.0 else 0.0)
    return alt, vt


LOG_PROPS = {"alt_ft": "position/h-sl-ft", "theta_rad": "attitude/theta-rad", "q_rad_sec": "velocities/q-rad_sec",
             "vt_fps": "velocities/vt-fps", "alpha_deg": "aero/alpha-deg", "phi_deg": "attitude/phi-deg",
             "elevator_rad": "fcs/elevator-pos-rad", "throttle": "fcs/throttle-total-norm"}


def _plant_fdm(cfg: SILConfig):
    from gnclab import initialize, make_fdm
    from gnclab.autopilot import engage_lateral, realism
    from gnclab.montecarlo import turbulence_on
    from gnclab.trim import trim

    fdm = make_fdm("gnc_trainer")
    fdm["simulation/randomseed"] = cfg.seed
    fdm["atmosphere/randomseed"] = cfg.seed + 1
    initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": 25 / 0.3048})
    trim(fdm, "full")
    realism(fdm, True)
    turbulence_on(fdm, cfg.turbulence_w20_fps)
    engage_lateral(fdm, course=False)              # roll hold + yaw damper stay in the XML
    for k, v in cfg.extra.items():
        fdm[k] = v
    return fdm


def run_sil(cfg: SILConfig) -> pd.DataFrame:
    """Run the plant here and the controller in a spawned process; return the plant log.

    Columns: truth (``LOG_PROPS``), commands sent/applied, and timing:
    ``cmd_frame`` (which controller frame produced the command in use),
    ``age_frames`` = k - cmd_frame, and in realtime mode ``overrun_ms``.
    """
    fdm = _plant_fdm(cfg)
    dt = fdm.get_delta_t()
    steps = int(round(1.0 / (cfg.rate_hz * dt)))
    if abs(steps * cfg.rate_hz * dt - 1.0) > 1e-9:
        raise ValueError("120 Hz / rate_hz must be an integer")
    card = cfg.card or default_card
    h0, v0 = fdm["position/h-sl-ft"], fdm["velocities/vt-fps"]
    de0, thr0 = fdm["fcs/elevator-cmd-norm"], fdm["fcs/throttle-cmd-norm[0]"]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(30.0)                           # spawning + importing jsbsim takes a few s
    ctx = multiprocessing.get_context("spawn")
    proc = ctx.Process(target=controller_main, args=(sock.getsockname(), cfg.rate_hz, cfg.gains,
                                                     cfg.compute_ms, cfg.controller_log, cfg.drain,
                                                     cfg.fc_clock_offset_s, cfg.jitter_ms, cfg.seed),
                       daemon=True)
    proc.start()
    try:
        kind, _ = unpack((msg := sock.recvfrom(256))[0])
        fc_addr = msg[1]
        sock.sendto(pack("INIT", de0, thr0, fdm["attitude/theta-rad"], h0, v0), fc_addr)
        sock.settimeout(5.0)
        log = _loop(fdm, cfg, sock, fc_addr, steps, card, h0, v0, de0, thr0)
    finally:
        try:
            sock.sendto(pack("STOP"), fc_addr)
        except Exception:                           # noqa: BLE001
            pass
        proc.join(timeout=10)
        sock.close()
    return log


def _loop(fdm, cfg, sock, fc_addr, steps, card, h0, v0, de0, thr0) -> pd.DataFrame:
    n_frames = int(round(cfg.duration * cfg.rate_hz))
    period = 1.0 / cfg.rate_hz
    pending = deque()                                # lockstep transport delay
    cmd, cmd_frame = (de0, thr0), -1
    rows = []
    realtime = cfg.mode == "realtime"
    if realtime:
        sock.setblocking(False)
    wall0 = time.perf_counter()
    for k in range(n_frames):
        t = fdm.get_sim_time()
        alt_cmd, vt_cmd = card(t, h0, v0)
        sens = (fdm["fb/theta-rad"], fdm["fb/q-rad_sec"], fdm["fb/h-ft"], fdm["fb/vt-fps"])
        sock.sendto(pack("SENS", k, t, *sens, alt_cmd, vt_cmd), fc_addr)
        overrun = 0.0
        if not realtime:
            kind, (kk, de, thr) = unpack(sock.recv(256))
            assert kind == "CMD_" and kk == k, (kind, kk, k)
            pending.append((k, de, thr))
            if len(pending) > cfg.delay_frames:
                kk, de, thr = pending.popleft()
                cmd, cmd_frame = (de, thr), kk
        else:
            # pace: this frame's sim steps belong to [k, k+1) * period of wall time
            deadline = wall0 + (k + 1) * period
            while True:                              # drain everything that has arrived
                try:
                    kind, (kk, de, thr) = unpack(sock.recv(256))
                    if kk > cmd_frame:
                        cmd, cmd_frame = (de, thr), kk
                except BlockingIOError:
                    break
        fdm["fcs/elevator-cmd-norm"], fdm["fcs/throttle-cmd-norm"] = cmd
        rows.append([t, k, alt_cmd, vt_cmd, *cmd, cmd_frame, k - cmd_frame, *sens]
                    + [fdm[p] for p in LOG_PROPS.values()])
        for _ in range(steps):
            fdm.run()
        if realtime:
            now = time.perf_counter()
            if now < deadline:
                time.sleep(max(0.0, deadline - now - 0.0005))
                while time.perf_counter() < deadline:  # spin the last half millisecond
                    pass
            else:
                overrun = (now - deadline) * 1000
            rows[-1].append(overrun)
    cols = (["t", "k", "alt_cmd_ft", "vt_cmd_fps", "de_cmd", "thr_cmd", "cmd_frame", "age_frames",
             "theta_meas", "q_meas", "h_meas_ft", "vt_meas_fps"] + list(LOG_PROPS)
            + (["overrun_ms"] if realtime else []))
    return pd.DataFrame(rows, columns=cols).set_index("t")


def in_process_reference(cfg: SILConfig) -> pd.DataFrame:
    """The same control law called directly (no sockets, no processes),
    logged the same way.  Lockstep SIL with delay_frames=0 must match it."""
    from gnclab.controllers import DEFAULT_GAINS, LongitudinalAP

    fdm = _plant_fdm(cfg)
    dt = fdm.get_delta_t()
    steps = int(round(1.0 / (cfg.rate_hz * dt)))
    card = cfg.card or default_card
    h0, v0 = fdm["position/h-sl-ft"], fdm["velocities/vt-fps"]
    meas = {}
    ap = LongitudinalAP(fdm, rate_hz=cfg.rate_hz, gains=dict(cfg.gains or DEFAULT_GAINS), wing_leveler=False,
                        feedback=lambda _f: meas)
    rows = []
    for k in range(int(round(cfg.duration * cfg.rate_hz))):
        t = fdm.get_sim_time()
        ap.alt_cmd, ap.vt_cmd = card(t, h0, v0)
        meas.update(theta=fdm["fb/theta-rad"], q=fdm["fb/q-rad_sec"], h_ft=fdm["fb/h-ft"], vt_fps=fdm["fb/vt-fps"])
        de, dthr = ap.step(1.0 / cfg.rate_hz)
        fdm["fcs/elevator-cmd-norm"] = ap.de_trim + de
        fdm["fcs/throttle-cmd-norm"] = min(1.0, max(0.0, ap.dt_trim + dthr))
        rows.append([t] + [fdm[p] for p in LOG_PROPS.values()])
        for _ in range(steps):
            fdm.run()
    return pd.DataFrame(rows, columns=["t"] + list(LOG_PROPS)).set_index("t")
