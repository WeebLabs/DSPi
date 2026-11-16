"""
Control Surfaces group: CS.REVERT as a background job. Every test SKIPs before any request unless
DSPI_TEST_CS_REVERT=1 is set, because CS.REVERT takes stored bindings down on current firmware.
Previews only pots on empty slots and ends on a revert; a failed revert falls back to
restore_preview (tests/_cs.py), whose CS.SAVE then stores the unchanged config (one flash write).
"""

import os
import struct

from ..framework import Skip, test
from ..helpers import (expect_result, expect_stall, has_command, require_page, set_retry_busy,
                       wait_for)
from ._cs import CS, PENDING, cs_limits, cs_snapshot, cs_status, restore_preview

GATE_ENV = "DSPI_TEST_CS_REVERT"
GATE_MSG = ("CS.REVERT takes stored bindings down on this firmware (known bug); "
            "set DSPI_TEST_CS_REVERT=1 to run")
CS_TYPE_POT = 3
CS_NOUN_OUTPUT_GAIN = 17
CS_ACT_ADJUST = 0
CS_STATUS_BUSY = 0x1B
POT_PINS = (26, 27, 28)
PACE = "DIAG.LOOP_PROF_BG_PACE"   # DSPI_LOOP_PROF builds only
BG_PACE = 50                      # one background unit per 50 passes: a revert spans tens of ms


def _binding(kind, noun, action, gpio, target=0):
    """24-byte CS.BINDING record (commands.md section 15.3)."""
    return struct.pack("<BBBBBBBBBBhhhhHHBB", kind, noun, action, 0, gpio, 0xFF, 0, target, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, 0)


def _pot(gpio):
    return _binding(CS_TYPE_POT, CS_NOUN_OUTPUT_GAIN, CS_ACT_ADJUST, gpio)


def _per_slot(st):
    """The status bytes a revert must not tear: everything past last_status/last_slot/dirty."""
    return st["raw"][2:3] + st["raw"][4:]


def _runtime(st):
    """Live binding and IR state (masks and per-slot apply status), without the IR learn state."""
    return st["raw"][4:24] + st["raw"][25:]


def _set_pace(dev, n):
    """True on a build that paces background jobs and reads back `n`."""
    if not has_command(dev, PACE, "SET"):
        return False
    dev.pset(PACE, struct.pack("<H", n))
    return dev.pget_u16(PACE) == n


def _bindings(dev, n):
    return [dev.pget("CS.BINDING", i) for i in range(n)]


def _wait_settled(dev, timeout_s=2.0):
    """CS.STATUS once the last deferred request has left PENDING, else None."""
    st = wait_for(lambda: cs_status(dev), lambda s: s["last"] != PENDING,
                  timeout_s=timeout_s, poll_s=0.01)
    return st if st is not None and st["last"] != PENDING else None


def _begin(dev):
    """The gate, then the clean start each test returns to: CS.STATUS, limits and record snapshot."""
    if os.environ.get(GATE_ENV) != "1":
        raise Skip(GATE_MSG)
    require_page(dev, CS)
    st = cs_status(dev)
    if st["dirty"]:
        raise Skip("Control Surfaces has an unsaved preview; a revert would discard it")
    if st["last"] == PENDING:
        raise Skip("a deferred CS request is still pending")
    lim = cs_limits(dev)
    return st, lim, cs_snapshot(dev, lim)


def _free_slots(snap, count):
    free = [i for i, b in enumerate(snap["binding"]) if b[0] == 0]
    if len(free) < count:
        raise Skip(f"needs {count} free binding slots")
    return free[-count:]


def _restore(dev, lim, snap):
    """Revert whatever preview a failed step left, then restore_preview for anything still off the snapshot.
    Completion is the dirty flag clearing: a request queued ahead of the revert can own the result pair."""
    _set_pace(dev, 1)
    _wait_settled(dev)
    if cs_status(dev)["dirty"] or _bindings(dev, lim["slots"]) != snap["binding"]:
        set_retry_busy(dev, "CS.REVERT")
        wait_for(lambda: cs_status(dev), lambda s: s["last"] != PENDING and not s["dirty"],
                 poll_s=0.01)
    restore_preview(dev, lim, snap)


def _revert(dev, chk, msg):
    expect_result(chk, lambda: dev.pset("CS.REVERT"), "VC_OK", msg)


def _preview_pots(dev, chk, slots):
    for k, slot in enumerate(slots):
        set_retry_busy(dev, "CS.BINDING", _pot(POT_PINS[k % len(POT_PINS)]), slot)
        st = _wait_settled(dev)
        chk.ok(st is not None and (st["last"], st["slot"]) == (0, slot),
               f"pot preview on slot {slot} applied")


@test("control_surfaces", mutating=True)
def cs_revert_restores_stored(dev, profile, chk):
    """A revert discards a preview and lands exactly on the stored bindings and their live state, dirty cleared."""
    st0, lim, snap = _begin(dev)
    slots = _free_slots(snap, 3)
    try:
        _preview_pots(dev, chk, slots)
        chk.eq(cs_status(dev)["dirty"], 1, "preview marks the config dirty")
        _revert(dev, chk, "CS.REVERT accepted")
        done = _wait_settled(dev)
        chk.ok(done is not None, "revert completes")
        if done:
            chk.eq((done["last"], done["slot"]), (0, 0xFF), "revert reports SUCCESS")
            chk.eq(done["dirty"], 0, "revert clears dirty")
            chk.eq(_runtime(done), _runtime(st0), "live binding and IR status equal the clean start")
        chk.ok(_bindings(dev, lim["slots"]) == snap["binding"], "bindings equal the stored image after the revert")
    finally:
        _restore(dev, lim, snap)


@test("control_surfaces", mutating=True)
def cs_revert_reads_never_torn(dev, profile, chk):
    """Binding and status reads issued while a revert runs answer with the stored image, never a half-reloaded slot.

    A revert takes about a millisecond, shorter than one host read, so the check
    needs a DSPI_LOOP_PROF build, whose DIAG.LOOP_PROF_BG_PACE stretches it over tens of ms.
    """
    st0, lim, snap = _begin(dev)
    slots = _free_slots(snap, 3)
    n = lim["slots"]
    paced = False
    reads_pending = mismatches = status_torn = 0
    try:
        for rep in range(20):
            _preview_pots(dev, chk, slots)
            before = _per_slot(cs_status(dev))
            paced = _set_pace(dev, BG_PACE)
            _revert(dev, chk, f"CS.REVERT {rep + 1} accepted")
            # Read the previewed slots first: they change most across the revert.
            for i in slots + [i for i in range(n) if i not in slots]:
                now = cs_status(dev)
                pending = now["last"] == PENDING
                b = dev.pget("CS.BINDING", i)
                reads_pending += pending
                mismatches += b != snap["binding"][i]
                status_torn += pending and _per_slot(now) != before
            _set_pace(dev, 1)
            chk.ok(_wait_settled(dev) is not None, f"revert {rep + 1} completes")
        chk.eq(mismatches, 0, "every binding read across 20 reverts equals the stored image")
        chk.eq(status_torn, 0, "status read mid-revert equals the snapshot taken before it")
        if paced:
            chk.ok(reads_pending > 0, "reads landed mid-revert (paced)")
            chk.note(f"{reads_pending} reads landed mid-revert across 20 paced reverts")
        else:
            chk.note("no DIAG.LOOP_PROF_BG_PACE: no read can land mid-revert, so only the final state was checked")
        end = cs_status(dev)
        chk.eq(end["dirty"], 0, "config clean at the end")
        chk.eq(_runtime(end), _runtime(st0), "live binding and IR status equal the clean start")
    finally:
        _restore(dev, lim, snap)


@test("control_surfaces", mutating=True)
def cs_set_during_revert_waits(dev, profile, chk):
    """A binding SET sent mid-revert is kept and applied after it; a second one, and an aux SET, answer BUSY."""
    st0, lim, snap = _begin(dev)
    slots = _free_slots(snap, 2)
    n = lim["slots"]
    late, other = _pot(POT_PINS[1]), _pot(POT_PINS[2])
    try:
        _preview_pots(dev, chk, slots[:1])
        paced = _set_pace(dev, BG_PACE)
        _revert(dev, chk, "CS.REVERT accepted")
        mid = cs_status(dev)
        expect_result(chk, lambda: dev.pset("CS.BINDING", late, slots[1]), "VC_OK",
                      "a binding SET mid-revert is queued")
        if paced:
            chk.ok(mid["last"] == PENDING and mid["slot"] == 0xFF,
                   "the SET was sent while the revert was still running")
            expect_stall(chk, lambda: dev.pset("CS.BINDING", other, slots[1]), "VC_ERR_BUSY",
                         "a second binding SET while the first waits")
            busy = cs_status(dev)
            chk.eq((busy["last"], busy["slot"]), (CS_STATUS_BUSY, slots[1]),
                   "the refused binding SET posts BUSY")
            expect_stall(chk, lambda: dev.pset("CS.AUX_STATE", b"\x00", slots[0]), "VC_ERR_BUSY",
                         "CS.AUX_STATE mid-revert")
            chk.eq(cs_status(dev)["raw"][:2], busy["raw"][:2], "the refused aux SET leaves the status alone")
        else:
            chk.note("no DIAG.LOOP_PROF_BG_PACE: the revert may finish before the SET arrives")
        _set_pace(dev, 1)
        done = wait_for(lambda: cs_status(dev),
                        lambda s: s["last"] not in (PENDING, CS_STATUS_BUSY), poll_s=0.01)
        chk.ok(done is not None, "revert and the late SET both complete")
        if done:
            chk.eq((done["last"], done["slot"]), (0, slots[1]),
                   "the final result is the late SET's SUCCESS, not the revert's")
            chk.eq(done["dirty"], 1, "the late SET marks the config dirty after the revert")
        now = _bindings(dev, n)
        chk.eq(now[slots[0]], snap["binding"][slots[0]], "the earlier preview is reverted")
        chk.eq(now[slots[1]], late, "the SET sent during the revert is applied after it")
        _revert(dev, chk, "cleanup CS.REVERT accepted")
        end = _wait_settled(dev)
        chk.ok(end is not None and end["dirty"] == 0, "cleanup revert leaves the config clean")
        chk.ok(_bindings(dev, n) == snap["binding"], "stored bindings restored")
        if end:
            chk.eq(_runtime(end), _runtime(st0), "live binding and IR status equal the clean start")
    finally:
        _restore(dev, lim, snap)
