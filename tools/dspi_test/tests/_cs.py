"""
Control Surfaces support for paged_cs, paged_cs_nouns and control_surfaces: limits, CS.STATUS, record
snapshots and the preview restore. Defines no tests.
"""

import time

from ..device import Stall

CS = 0x0A
PENDING = 0x16           # CS.STATUS last_status while a deferred request waits for the main loop
AUX_TYPES = (9, 10)      # aux output binding types (CS_TYPE_AUX_OUT, CS_TYPE_AUX_PWM)


def cs_limits(dev):
    caps = dev.pget("CS.CAPS")
    t = 4 + 4 * caps[2]
    return {"slots": caps[1], "nouns": caps[3], "ir": caps[t], "groups": caps[t + 1],
            "macros": caps[t + 2], "steps": caps[t + 3], "pages": dev.pget("CS.DISPLAY_CAPS")[0]}


def cs_status(dev):
    s = dev.pget("CS.STATUS")
    return {"last": s[0], "slot": s[1], "dirty": s[3], "raw": s}


def cs_settle(dev, timeout_s=3.0):
    """CS.STATUS once the last deferred request has left PENDING, else None."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            st = cs_status(dev)
        except (Stall, OSError):
            dev.wait_ready()
            continue
        if st["last"] != PENDING:
            return st
        time.sleep(0.01)
    return None


def cs_snapshot(dev, lim):
    def rng(cmd, n, addr=lambda i: i):
        return [dev.pget(cmd, addr=addr(i)) for i in range(n)]
    return {"binding": rng("CS.BINDING", lim["slots"]), "name": rng("CS.SLOT_NAME", lim["slots"]),
            "ir": rng("CS.IR_CMD", lim["ir"]), "group": rng("CS.GROUP", lim["groups"]),
            "macro": rng("CS.MACRO", lim["macros"]), "page": rng("CS.DISPLAY_PAGE", lim["pages"]),
            "display": dev.pget("CS.DISPLAY_CONFIG")}


def _name_payload(raw32):
    name = bytes(raw32).split(b"\0")[0]
    return name if name else b"\0"


def put_record(dev, name, payload, addr, tag):
    """Preview SET; None once CS.STATUS reports it applied, else what went wrong."""
    try:
        dev.pset(name, payload, addr=addr)
    except Stall as e:
        return f"{name} @0x{addr:04X} refused ({e})"
    st = cs_settle(dev)
    if st is None or (st["last"], st["slot"]) != (0, tag):
        return f"{name} @0x{addr:04X} not applied: {st and (hex(st['last']), hex(st['slot']))}"
    return None


def _snapshot_diff(snap, after):
    out = []
    for k, v in snap.items():
        if isinstance(v, list):
            out += [f"{k}[{i}]" for i, (a, b) in enumerate(zip(v, after[k])) if a != b]
        elif v != after[k]:
            out.append(k)
    return out


def restore_preview(dev, lim, snap):
    """Rewrite every CS record that differs from `snap`, then CS.SAVE so the preview is clean.
    Raises AssertionError naming what is left (SAVE is then never sent); returns the final CS.STATUS."""
    errors = []

    def fix(name, cur, want, addr, tag, payload=None):
        if cur != want:
            e = put_record(dev, name, want if payload is None else payload, addr, tag)
            if e:
                errors.append(e)
    for i in range(lim["slots"]):
        fix("CS.BINDING", dev.pget("CS.BINDING", addr=i), snap["binding"][i], i, i)
        fix("CS.SLOT_NAME", dev.pget("CS.SLOT_NAME", addr=i), snap["name"][i], i, i, _name_payload(snap["name"][i]))
    for i in range(lim["ir"]):
        fix("CS.IR_CMD", dev.pget("CS.IR_CMD", addr=i), snap["ir"][i], i, 0x80 | i)
    for g in range(lim["groups"]):
        fix("CS.GROUP", dev.pget("CS.GROUP", addr=g), snap["group"][g], g, 0x40 | g)
    for m in range(lim["macros"]):
        if dev.pget("CS.MACRO", addr=m) == snap["macro"][m]:
            continue
        fix("CS.MACRO_HEADER", dev.pget("CS.MACRO_HEADER", addr=m), snap["macro"][m][:33] + bytes(3), m, 0x60 | m)
        for k in range(lim["steps"]):
            a = (k << 8) | m
            fix("CS.MACRO_STEP", dev.pget("CS.MACRO_STEP", addr=a), snap["macro"][m][36 + 12 * k:48 + 12 * k],
                a, 0x60 | m)
    for p in range(lim["pages"]):
        fix("CS.DISPLAY_PAGE", dev.pget("CS.DISPLAY_PAGE", addr=p), snap["page"][p], p, 0x50 | p)
    fix("CS.DISPLAY_CONFIG", dev.pget("CS.DISPLAY_CONFIG"), snap["display"], 0, 0x50)
    left = _snapshot_diff(snap, cs_snapshot(dev, lim))
    if left:
        raise AssertionError(f"CS preview left dirty, CS.SAVE not sent: {left} differ from the pre-test snapshot"
                             + (f"; restore errors: {errors}" if errors else ""))
    st = cs_status(dev)
    if st["dirty"]:
        dev.pset("CS.SAVE")
        time.sleep(0.3)
        dev.wait_ready()
        st = cs_settle(dev, timeout_s=5.0)
        if st is None or st["dirty"]:
            raise AssertionError(f"CS.SAVE of the restored records left the preview dirty: {st and st['raw'].hex()}")
    return st
