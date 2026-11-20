"""Device side of the wire-capture rig: DIAG.WIRECAP_* commands, configuration
snapshot and restore, health counters. Protocol: Documentation/Features/wirecap_test_rig.md.
"""

from __future__ import annotations

import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from dspi_test.device import DspiDevice, Stall  # noqa: E402
from dspi_test import telemetry  # noqa: E402

BCK_PIN = 24
LRCK_PIN = 25
RATES = (44100, 48000, 96000)

SLAVE_STATES = {0: "INACTIVE", 1: "ACQUIRING", 2: "RELOCKING", 3: "LOCKED"}


@dataclass
class Capture:
    samples: np.ndarray      # one integer per sample, bit i = GPIO (base + i) % 32
    base: int
    count: int
    clk_hz: int
    div: float               # PIO clock divider (int + frac / 256)
    stalled: bool            # FDEBUG RXSTALL: the SM waited on a full FIFO, so samples are not evenly spaced
    rx_under: bool

    @property
    def rate_hz(self) -> float:
        return self.clk_hz / self.div

    def line(self, gpio: int) -> np.ndarray:
        shift = (gpio - self.base) % 32
        if shift >= self.count:
            raise ValueError(f"GPIO {gpio} is not in this capture")
        return ((self.samples >> shift) & 1).astype(np.uint8)


class Rig:
    def __init__(self, dev: DspiDevice | None = None):
        self.d = dev or DspiDevice()

    # ---- sampler ----

    def cap_status(self) -> dict:
        b = self.d.pget("DIAG.WIRECAP_STATUS")
        (state, flags, base, count, clk, dint, dfrac, pio, nbytes, done, sm, d0, d1, _r,
         bufsz, seq) = struct.unpack("<BBBBIHBBIIBBBBII", b)
        return dict(state=state, stalled=bool(flags & 1), rx_under=bool(flags & 2), base=base,
                    count=count, clk_hz=clk, div_int=dint, div_frac=dfrac, pio=pio, bytes=nbytes,
                    done=done, sm=sm, dma=(d0, d1), buffer_bytes=bufsz, seq=seq)

    def buffer_bytes(self) -> int:
        return self.cap_status()["buffer_bytes"]

    def capture(self, base: int, count: int, div_int: int, div_frac: int = 0,
                nbytes: int | None = None, timeout_s: float = 2.0) -> Capture:
        nbytes = nbytes or self.buffer_bytes()
        self.d.pset("DIAG.WIRECAP_ARM", struct.pack("<BBHBBI", base, count, div_int, div_frac, 0, nbytes))
        t0 = time.monotonic()
        while True:
            st = self.cap_status()
            if st["state"] == 2:
                break
            if time.monotonic() - t0 > timeout_s:
                self.d.pset("DIAG.WIRECAP_ARM", struct.pack("<BBHBBI", 0, 1, 1, 0, 0, 0))
                raise TimeoutError(f"capture did not finish: {st}")
            time.sleep(0.005)
        raw = bytearray()
        for chunk in range((nbytes + 255) // 256):
            raw += self.d.pget("DIAG.WIRECAP_READ", chunk, length=256)
        words = np.frombuffer(bytes(raw[:nbytes]), dtype="<u4")
        if count == 32:
            samples = words.astype(np.uint32)
        else:
            per = 32 // count
            mask = (1 << count) - 1
            shifts = (np.arange(per, dtype=np.uint32) * count)
            samples = ((words[:, None] >> shifts[None, :]) & mask).astype(np.uint32).ravel()
        return Capture(samples=samples, base=base, count=count, clk_hz=st["clk_hz"],
                       div=st["div_int"] + st["div_frac"] / 256.0, stalled=st["stalled"],
                       rx_under=st["rx_under"])

    # ---- clock generator, rate ----

    def clkgen(self, enable: bool, rate: int = 48000):
        self.d.pset("DIAG.WIRECAP_CLKGEN", struct.pack("<BI", 1 if enable else 0, rate))

    def clkgen_status(self) -> dict:
        b = self.d.pget("DIAG.WIRECAP_CLKGEN")
        on, pio, sm, dma, rate, div, glitches, late, phase, pending, kind, deferred = \
            struct.unpack("<BBBBIIIIBBBB", b)
        return dict(on=bool(on), pio=pio, sm=sm, dma=dma, rate=rate, div=div / 256.0,
                    glitches=glitches, late=late, phase=phase, pending=bool(pending), kind=kind,
                    deferred_mux=deferred)

    def glitch(self, kind: int, cells: int = 0):
        self.d.pset("DIAG.WIRECAP_CLKGEN_GLITCH", bytes([kind, cells]))

    def request_rate(self, rate: int):
        self.d.pset("DIAG.WIRECAP_RATE", struct.pack("<I", rate))

    # ---- device state ----

    def rate(self) -> int:
        return telemetry.sample_rate(self.d)

    def slave_status(self) -> dict:
        b = self.d.pget("CLOCK.I2S_SLAVE_STATUS")
        st, mode, locks, losses, det, meas, slips = struct.unpack_from("<BBBBIIB", b)
        return dict(state=SLAVE_STATES.get(st, st), clock_mode=mode, lock_count=locks,
                    loss_count=losses, detected_rate=det, measured_hz=meas, slip_count=slips)

    def health(self) -> dict:
        h = {}
        h.update({f"pipe.{k}": v for k, v in telemetry.pipeline_stats(self.d).items()
                  if k in ("ring_underruns", "ring_drops")})
        h.update({f"tx.{k}": v for k, v in telemetry.spdif_tx_stats(self.d).items()})
        s = self.slave_status()
        h.update({f"slave.{k}": s[k] for k in ("lock_count", "loss_count", "slip_count")})
        return h

    def wait_settled(self, rate: int, slave: bool = False, timeout_s: float = 6.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            try:
                ok = self.rate() == rate
                if slave:
                    ok = ok and self.slave_status()["state"] == "LOCKED"
                if ok:
                    time.sleep(0.4)
                    return True
            except Stall:
                pass
            time.sleep(0.1)
        return False


# Settings the matrix touches, in the order a restore writes them back.
SNAPSHOT_SCALARS = ["CLOCK.I2S_CLOCK_MODE", "INPUT.SOURCE", "INPUT.I2S_RX_ENABLE",
                    "CLOCK.I2S_CLOCK_PIN_MODE"]


def snapshot(d: DspiDevice) -> dict:
    s = {n: d.pget(n).hex() for n in SNAPSHOT_SCALARS}
    s["types"] = [d.pget("OUTPUT.TYPE", i)[0] for i in range(4)]
    s["bck0"] = d.pget("CLOCK.I2S_BCK_PIN", 0)[0]
    s["out_enable"] = [d.pget("ROUTE.OUTPUT_ENABLE", o)[0] for o in range(9)]
    s["rate"] = telemetry.sample_rate(d)
    return s
