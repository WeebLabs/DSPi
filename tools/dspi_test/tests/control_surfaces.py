"""
Control Surfaces group: the revert as a background job.

Non-flash. Every test refuses to start over an unsaved preview, previews only
input bindings (a pot on a floating ADC pin, which drives no pin), and ends
with a revert, so the stored config and the live state end as they began.
Design: Documentation/current_architecture.md, Control Surfaces "Deferred SET,
Apply/Save/Revert".
"""

import struct
import time

from ..device import OP
from ..framework import Skip, test

CS_TYPE_POT = 3
CS_NOUN_OUTPUT_GAIN = 17
CS_ACT_ADJUST = 0
CS_STATUS_PENDING = 0x16
POT_PINS = (26, 27, 28)
BG_PACE_BASE = 512          # loop_prof.h LOOP_PROF_BG_PACE_BASE (DSPI_LOOP_PROF builds)
BG_PACE = 50                # one background unit per 50 passes: a revert spans tens of ms


def _binding(kind, noun, action, gpio, target=0):
    """24-byte CsBinding (firmware/DSPi/control_surfaces.h)."""
    return struct.pack("<BBBBBBBBBBhhhhHHBB", kind, noun, action, 0, gpio, 0xFF, 0, target, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, 0)


def _status(dev):
    s = dev.get(OP.GET_CS_STATUS, 41)
    return {"last_status": s[0], "last_slot": s[1], "slots": s[2], "dirty": s[3],
            "active": struct.unpack_from("<H", s, 4)[0]}


def _set_pace(dev, n):
    """True on a DSPI_LOOP_PROF build, which echoes the pacing it applied."""
    raw = dev.get(OP.GET_STATUS, 4, wvalue=BG_PACE_BASE + n)
    return len(raw) == 4 and struct.unpack("<I", raw)[0] == n


def _bindings(dev, n):
    return [bytes(dev.get(OP.GET_CS_BINDING, 24, wvalue=i)) for i in range(n)]


def _wait_settled(dev, timeout_s=2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        st = _status(dev)
        if st["last_status"] != CS_STATUS_PENDING:
            return st
        time.sleep(0.01)
    return None


def _clean_start(dev):
    st = _status(dev)
    if st["dirty"]:
        raise Skip("Control Surfaces has an unsaved preview; a revert would discard it")
    return st


def _free_slots(dev, st, count):
    stored = _bindings(dev, st["slots"])
    free = [i for i in range(st["slots"]) if stored[i][0] == 0]
    if len(free) < count:
        raise Skip(f"needs {count} free binding slots")
    return free[-count:], stored


def _preview_pots(dev, chk, slots):
    for k, slot in enumerate(slots):
        dev.set(OP.SET_CS_BINDING, _binding(CS_TYPE_POT, CS_NOUN_OUTPUT_GAIN, CS_ACT_ADJUST,
                                            POT_PINS[k % len(POT_PINS)]), wvalue=slot)
        st = _wait_settled(dev)
        chk.ok(st is not None and st["last_status"] == 0, f"pot preview on slot {slot} applied")


@test("control_surfaces", mutating=True)
def cs_revert_restores_stored(dev, profile, chk):
    """A revert discards a preview and lands exactly on the stored bindings, with dirty cleared."""
    st = _clean_start(dev)
    slots, stored = _free_slots(dev, st, 3)
    _preview_pots(dev, chk, slots)
    chk.eq(_status(dev)["dirty"], 1, "preview marks the config dirty")
    dev.get(OP.CS_REVERT, 1)
    done = _wait_settled(dev)
    chk.ok(done is not None, "revert completes")
    if done:
        chk.eq(done["last_status"], 0, "revert reports SUCCESS")
        chk.eq(done["dirty"], 0, "revert clears dirty")
    chk.ok(_bindings(dev, st["slots"]) == stored, "bindings equal the stored image after the revert")


@test("control_surfaces", mutating=True)
def cs_revert_reads_never_torn(dev, profile, chk):
    """Binding reads issued while a revert runs answer with the stored image, never a half-reloaded slot.

    A revert takes about a millisecond, shorter than one host read, so the check
    needs a DSPI_LOOP_PROF build, whose pacing stretches it over tens of ms.
    """
    st = _clean_start(dev)
    slots, stored = _free_slots(dev, st, 3)
    paced = False
    reads_pending = mismatches = 0
    try:
        for rep in range(20):
            _preview_pots(dev, chk, slots)
            paced = _set_pace(dev, BG_PACE)
            dev.get(OP.CS_REVERT, 1)
            # Read the previewed slots first: they change most across the revert.
            order = slots + [i for i in range(st["slots"]) if i not in slots]
            for i in order:
                pending = _status(dev)["last_status"] == CS_STATUS_PENDING
                b = bytes(dev.get(OP.GET_CS_BINDING, 24, wvalue=i))
                reads_pending += pending
                mismatches += b != stored[i]
            _set_pace(dev, 1)
            chk.ok(_wait_settled(dev) is not None, f"revert {rep + 1} completes")
    finally:
        _set_pace(dev, 1)
    chk.eq(mismatches, 0, "every binding read across 20 reverts equals the stored image")
    if paced:
        chk.ok(reads_pending > 0, "reads landed mid-revert (paced)")
        chk.note(f"{reads_pending} reads landed mid-revert across 20 paced reverts")
    else:
        chk.note("release build: no read can land mid-revert, so only the final state was checked")
    chk.eq(_status(dev)["dirty"], 0, "config clean at the end")


@test("control_surfaces", mutating=True)
def cs_set_during_revert_waits(dev, profile, chk):
    """A binding SET sent while a revert runs is kept and applied after it, not dropped or reverted."""
    st = _clean_start(dev)
    slots, stored = _free_slots(dev, st, 2)
    _preview_pots(dev, chk, slots[:1])
    late = _binding(CS_TYPE_POT, CS_NOUN_OUTPUT_GAIN, CS_ACT_ADJUST, POT_PINS[1])
    dev.get(OP.CS_REVERT, 1)
    dev.set(OP.SET_CS_BINDING, late, wvalue=slots[1])
    done = _wait_settled(dev)
    chk.ok(done is not None, "revert and the late SET both complete")
    now = _bindings(dev, st["slots"])
    chk.eq(now[slots[0]], stored[slots[0]], "the earlier preview is reverted")
    chk.eq(now[slots[1]], late, "the SET sent during the revert is applied after it")
    dev.get(OP.CS_REVERT, 1)
    done = _wait_settled(dev)
    chk.ok(done is not None and done["dirty"] == 0, "cleanup revert leaves the config clean")
    chk.ok(_bindings(dev, st["slots"]) == stored, "stored bindings restored")
