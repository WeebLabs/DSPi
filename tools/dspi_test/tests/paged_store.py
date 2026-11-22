"""
Native paged tests for the STORE page (plan section 5.2): preset slots, directory, startup, IO config
persistence, factory reset guards and bulk parameters. Every test SKIPs without the paged STORE page.
Flash tests use one scratch slot (the highest empty one) and restore everything they change.
FACTORY_RESET is only ever sent in shapes the dispatcher refuses before its handler runs.
"""

import struct
import time

from ..framework import Skip, test
from ..helpers import expect_stall, reached, require_page, settle

STORE = 0x02
PRESET_SLOTS = 10
BULK_LEN = 6252
CHUNK = 2048


def _dir(dev, ready=False):
    d = (dev.pget_ready if ready else dev.pget)("STORE.DIRECTORY", length=7)
    return {"occupied": d[0] | (d[1] << 8), "startup_mode": d[2], "default_slot": d[3],
            "active": d[4], "output_config_mode": d[5], "mv_mode": d[6]}


def _scratch_slot(dev):
    occ = _dir(dev)["occupied"]
    return next((s for s in range(PRESET_SLOTS - 1, -1, -1) if not occ & (1 << s)), None)


def _bulk(dev):
    return dev.pget_ready("STORE.ALL_PARAMS", length=BULK_LEN)


def _paged_chunks(dev, size=CHUNK):
    return b"".join(dev.pget("STORE.ALL_PARAMS_CHUNK", off, min(size, BULK_LEN - off))
                    for off in range(0, BULK_LEN, size))


# --- Reads -------------------------------------------------------------------

@test("paged_store")
def reads_consistent(dev, profile, chk):
    """DIRECTORY fields are in range and agree with PRESET_ACTIVE, STARTUP, OUTPUT_CONFIG_MODE and MASTER_VOLUME_MODE."""
    require_page(dev, STORE)
    raw = dev.pget("STORE.DIRECTORY", length=7)
    chk.eq(len(raw), 7, "DIRECTORY length")
    d = _dir(dev)
    chk.eq(d["occupied"] >> PRESET_SLOTS, 0, "no occupied bit above slot 9")
    chk.member(d["startup_mode"], (0, 1), "startup mode")
    chk.in_range(d["default_slot"], 0, PRESET_SLOTS - 1, "default slot")
    chk.in_range(d["active"], 0, PRESET_SLOTS - 1, "last-active slot")
    chk.member(d["output_config_mode"], (0, 1), "output config mode")
    chk.member(d["mv_mode"], (0, 1), "master volume mode")
    chk.eq(dev.pget("STORE.PRESET_ACTIVE", length=1), bytes([d["active"]]), "PRESET_ACTIVE == DIRECTORY byte 4")
    chk.eq(dev.pget("STORE.STARTUP", length=2), bytes([d["startup_mode"], d["default_slot"]]),
           "STARTUP == DIRECTORY bytes 2-3")
    chk.eq(dev.pget("STORE.OUTPUT_CONFIG_MODE", length=1), bytes([d["output_config_mode"]]),
           "OUTPUT_CONFIG_MODE == DIRECTORY byte 5")
    if 0x05 in dev.paged_commands():
        chk.eq(dev.pget("ROUTE.MASTER_VOLUME_MODE", length=1), bytes([d["mv_mode"]]),
               "ROUTE.MASTER_VOLUME_MODE == DIRECTORY byte 6")
    for slot in range(PRESET_SLOTS):
        name = dev.pget("STORE.PRESET_NAME", slot, 32)
        chk.eq(len(name), 32, f"PRESET_NAME {slot} length")
        chk.ok(b"\0" in name, f"PRESET_NAME {slot} is NUL-terminated: {name!r}")


@test("paged_store")
def read_errors(dev, profile, chk):
    """Bad slot and non-zero wValue are VC_ERR_ADDRESS; GET of an action and SET of a read are VC_ERR_DIRECTION."""
    require_page(dev, STORE)
    for addr in (PRESET_SLOTS, 0x100):
        expect_stall(chk, lambda: dev.pget("STORE.PRESET_NAME", addr, 32), "VC_ERR_ADDRESS",
                     f"PRESET_NAME GET slot {addr:#x}")
    for name, n in (("STORE.DIRECTORY", 7), ("STORE.PRESET_ACTIVE", 1), ("STORE.STARTUP", 2),
                    ("STORE.OUTPUT_CONFIG_MODE", 1), ("STORE.ALL_PARAMS", BULK_LEN)):
        expect_stall(chk, lambda: dev.pget(name, 1, n), "VC_ERR_ADDRESS", f"{name} at wValue 1")
    for name in ("STORE.DIRECTORY", "STORE.PRESET_ACTIVE"):
        expect_stall(chk, lambda: dev.pset(name, b"\x00"), "VC_ERR_DIRECTION", f"SET {name}")
    for name in ("STORE.PRESET_SAVE", "STORE.PRESET_LOAD", "STORE.PRESET_DELETE", "STORE.OUTPUT_CONFIG_SAVE",
                 "STORE.FACTORY_RESET"):
        expect_stall(chk, lambda: dev.pget(name, 0, 1), "VC_ERR_DIRECTION", f"GET {name}")


# --- SET guards that never reach a handler or are refused by it (no flash) ---------------

@test("paged_store")
def set_guards(dev, profile, chk):
    """Action payloads, bad slots, bad sizes and bad startup values are refused before anything is queued."""
    require_page(dev, STORE)
    before = dev.pget("STORE.DIRECTORY", length=7)
    for name in ("STORE.PRESET_SAVE", "STORE.PRESET_LOAD", "STORE.PRESET_DELETE"):
        expect_stall(chk, lambda: dev.pset(name, b"", PRESET_SLOTS), "VC_ERR_ADDRESS", f"{name} slot 10")
        expect_stall(chk, lambda: dev.pset(name, b"", 0xFF), "VC_ERR_ADDRESS", f"{name} slot 255")
        expect_stall(chk, lambda: dev.pset(name, b"\x00", 0), "VC_ERR_LENGTH", f"{name} with a payload")
    expect_stall(chk, lambda: dev.pset("STORE.FACTORY_RESET", b"\x00"), "VC_ERR_LENGTH",
                 "FACTORY_RESET with a 1-byte payload")
    expect_stall(chk, lambda: dev.pset("STORE.OUTPUT_CONFIG_SAVE", b"\x00"), "VC_ERR_LENGTH",
                 "OUTPUT_CONFIG_SAVE with a payload")
    expect_stall(chk, lambda: dev.pset("STORE.PRESET_NAME", b"x", PRESET_SLOTS), "VC_ERR_ADDRESS",
                 "PRESET_NAME SET slot 10")
    expect_stall(chk, lambda: dev.pset("STORE.PRESET_NAME", b"", 0), "VC_ERR_LENGTH", "PRESET_NAME empty")
    expect_stall(chk, lambda: dev.pset("STORE.PRESET_NAME", b"N" * 33, 0), "VC_ERR_LENGTH", "PRESET_NAME 33 B")
    for n in (1, 3):
        expect_stall(chk, lambda: dev.pset("STORE.STARTUP", bytes(n)), "VC_ERR_LENGTH", f"STARTUP {n} B")
    expect_stall(chk, lambda: dev.pset("STORE.STARTUP", bytes([2, 0])), "VC_ERR_VALUE", "STARTUP mode 2")
    expect_stall(chk, lambda: dev.pset("STORE.STARTUP", bytes([0, PRESET_SLOTS])), "VC_ERR_VALUE",
                 "STARTUP slot 10")
    expect_stall(chk, lambda: dev.pset("STORE.STARTUP", bytes([0, 0]), 1), "VC_ERR_ADDRESS", "STARTUP wValue 1")
    for n in (0, 2):
        expect_stall(chk, lambda: dev.pset("STORE.OUTPUT_CONFIG_MODE", bytes(n)), "VC_ERR_LENGTH",
                     f"OUTPUT_CONFIG_MODE {n} B")
    time.sleep(0.2)
    chk.eq(dev.pget("STORE.DIRECTORY", length=7), before, "directory untouched by refused SETs")


# --- Bulk parameters -------------------------------------------------------------

@test("paged_store")
def all_params_reads_agree(dev, profile, chk):
    """STORE.ALL_PARAMS has the profile's header, and whole-struct reads assembled from paged chunks equal it."""
    require_page(dev, STORE)
    for attempt in range(3):
        whole = _bulk(dev)
        paged = _paged_chunks(dev)
        odd = _paged_chunks(dev, 1000)
        if whole == paged == odd:
            break
        chk.note(f"bulk reads disagreed on attempt {attempt} (live change or another host)")
    chk.eq(len(whole), BULK_LEN, "ALL_PARAMS length")
    chk.eq(paged, whole, "2048-byte paged chunks == ALL_PARAMS")
    chk.eq(odd, whole, "1000-byte paged chunks == ALL_PARAMS")
    fmt, plat, nch = struct.unpack_from("<BBB", whole)
    chk.eq((fmt, plat, nch), (profile.wire_format_version, profile.platform_id, profile.num_channels), "header")
    chk.eq(struct.unpack_from("<H", whole, 6)[0], BULK_LEN, "header payload length")
    head = dev.pget("STORE.ALL_PARAMS_CHUNK", 0, 16)
    chk.eq(head, whole[:16], "a short offset-0 chunk is the struct prefix")


@test("paged_store", mutating=True)
def all_params_set_roundtrip(dev, profile, chk):
    """A paged SET of a snapshot is a fixpoint, whole or chunked at 2048, 1000 and 64 bytes."""
    require_page(dev, STORE)
    a = _bulk(dev)
    dev.pset("STORE.ALL_PARAMS", a)
    settle(dev)
    b = _bulk(dev)
    if b != a:
        chk.note("first apply normalized the snapshot (clamped fields); comparing against the normalized blob")
    for label, send in (
            ("whole paged SET", lambda: dev.pset("STORE.ALL_PARAMS", b)),
            ("2048-byte paged chunks", lambda: [dev.pset("STORE.ALL_PARAMS_CHUNK", b[o:o + CHUNK], o)
                                                for o in range(0, BULK_LEN, CHUNK)]),
            ("64-byte paged chunks", lambda: [dev.pset("STORE.ALL_PARAMS_CHUNK", b[o:o + 64], o)
                                              for o in range(0, BULK_LEN, 64)]),
            ("1000-byte paged chunks", lambda: [dev.pset("STORE.ALL_PARAMS_CHUNK", b[o:o + 1000], o)
                                                for o in range(0, BULK_LEN, 1000)])):
        reenum = dev.reenumerations
        send()
        settle(dev)
        got = _bulk(dev)
        if dev.reenumerations != reenum:
            chk.note(f"{label}: re-enumeration during the apply, compare skipped")
            continue
        chk.eq(got, b, f"{label} leaves the state unchanged")


@test("paged_store", mutating=True)
def all_params_set_guards(dev, profile, chk):
    """Wrong lengths are VC_ERR_LENGTH, a header the apply would drop is VC_ERR_VALUE, and nothing changes."""
    require_page(dev, STORE)
    a = _bulk(dev)
    expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS", a[:100]), "VC_ERR_LENGTH", "100-byte SET")
    expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS", a[:64]), "VC_ERR_LENGTH", "64-byte SET")
    expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS", a + b"\x00"), "VC_ERR_LENGTH", "6137-byte SET")
    expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS", a, 1), "VC_ERR_ADDRESS", "SET at wValue 1")
    for off, val, what in ((0, 99, "format version"), (1, a[1] ^ 1, "platform"), (2, a[2] + 1, "channel count")):
        bad = bytearray(a)
        bad[off] = val & 0xFF
        expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS", bytes(bad)), "VC_ERR_VALUE", f"bad {what}")
    bad = bytearray(a)
    bad[0] = 99
    for o in range(0, BULK_LEN - CHUNK, CHUNK):
        dev.pset("STORE.ALL_PARAMS_CHUNK", bytes(bad[o:o + CHUNK]), o)
    last = (BULK_LEN // CHUNK) * CHUNK
    expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS_CHUNK", bytes(bad[last:]), last), "VC_ERR_VALUE",
                 "chunked SET with a bad header fails on its final chunk")
    settle(dev)
    chk.eq(_bulk(dev), a, "refused bulk SETs left the state alone")


@test("paged_store")
def chunk_errors(dev, profile, chk):
    """Chunk offsets past the struct, overruns, out-of-order SETs and reads without a snapshot are refused."""
    require_page(dev, STORE)
    expect_stall(chk, lambda: dev.pget("STORE.ALL_PARAMS_CHUNK", BULK_LEN, 64), "VC_ERR_ADDRESS", "GET at 6252")
    expect_stall(chk, lambda: dev.pget("STORE.ALL_PARAMS_CHUNK", 64, 64), "VC_ERR_STATE", "GET at 64, no session")
    expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS_CHUNK", b"\x00", BULK_LEN), "VC_ERR_ADDRESS",
                 "SET at 6252")
    expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS_CHUNK", bytes(64), BULK_LEN - 32), "VC_ERR_LENGTH",
                 "SET past the end")
    expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS_CHUNK", bytes(64), 64), "VC_ERR_STATE",
                 "SET at 64, no session")
    expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS_CHUNK", b""), "VC_ERR_LENGTH", "zero-length SET")
    snap = _bulk(dev)
    dev.pset("STORE.ALL_PARAMS_CHUNK", snap[:64], 0)
    expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS_CHUNK", snap[128:192], 128), "VC_ERR_STATE",
                 "SET skipping a chunk")
    after = _bulk(dev)
    chk.eq(after, snap, "an abandoned chunked SET applies nothing")


@test("paged_store")
def chunk_sessions_closed_by_other_requests(dev, profile, chk):
    """Any request between two chunks ends the session, and a GET session never continues as a SET or back."""
    require_page(dev, STORE)
    whole = _bulk(dev)
    dev.pget("STORE.ALL_PARAMS_CHUNK", 0, CHUNK)
    dev.pget("SYS.PLATFORM", length=7)
    expect_stall(chk, lambda: dev.pget("STORE.ALL_PARAMS_CHUNK", CHUNK, CHUNK), "VC_ERR_STATE",
                 "a GET session is gone after an unrelated request")

    dev.pget("STORE.ALL_PARAMS_CHUNK", 0, CHUNK)
    expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS_CHUNK", whole[CHUNK:2 * CHUNK], CHUNK),
                 "VC_ERR_STATE", "a SET does not continue a GET session")

    dev.pset("STORE.ALL_PARAMS_CHUNK", whole[:CHUNK], 0)
    expect_stall(chk, lambda: dev.pget("STORE.ALL_PARAMS_CHUNK", CHUNK, CHUNK), "VC_ERR_STATE",
                 "a GET does not continue a SET session")

    dev.pset("STORE.ALL_PARAMS_CHUNK", whole[:CHUNK], 0)
    dev.pget("SYS.PLATFORM", length=7)
    expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS_CHUNK", whole[CHUNK:2 * CHUNK], CHUNK),
                 "VC_ERR_STATE", "a SET session is gone after an unrelated request")

    chk.eq(_bulk(dev), whole, "no half-received chunked SET was applied")
    chk.eq(_paged_chunks(dev), whole, "a fresh paged session still reads the whole struct")


# --- Flash -------------------------------------------------------------------------

def _occupied(dev, slot):
    return (_dir(dev, ready=True)["occupied"] >> slot) & 1


def _drop_scratch(dev, slot):
    """Delete the scratch preset if a save reached flash; raise naming the slot if it will not go."""
    dev.wait_ready()
    reached(lambda: _occupied(dev, slot), 1, timeout_s=1.0, tolerate=())   # a queued save may still be landing
    if not _occupied(dev, slot):
        return
    dev.pset("STORE.PRESET_DELETE", b"", slot)
    if not reached(lambda: _occupied(dev, slot), 0, tolerate=()):
        raise AssertionError(f"scratch preset slot {slot} is still occupied after cleanup: delete it by hand")


@test("paged_store", mutating=True, flash=6)
def preset_lifecycle(dev, profile, chk):
    """PRESET_SAVE, PRESET_LOAD and PRESET_DELETE on a scratch slot, tracked through DIRECTORY, then restored."""
    require_page(dev, STORE)
    scratch = _scratch_slot(dev)
    if scratch is None:
        raise Skip("every preset slot is occupied; no scratch slot to use")
    orig_active = dev.pget("STORE.PRESET_ACTIVE", length=1)[0]
    saved = False
    try:
        saved = True
        dev.pset("STORE.PRESET_SAVE", b"", scratch)
        chk.ok(reached(lambda: dev.pget_ready("STORE.PRESET_ACTIVE", length=1)[0], scratch, tolerate=()),
               "active became scratch after save")
        chk.ok(reached(lambda: _occupied(dev, scratch), 1, tolerate=()), "scratch occupied")
        chk.eq(_dir(dev)["active"], scratch, "DIRECTORY last-active byte follows the save")
        dev.pset("STORE.PRESET_LOAD", b"", scratch)
        dev.wait_ready()
        chk.ok(reached(lambda: dev.pget_ready("STORE.PRESET_ACTIVE", length=1)[0], scratch, tolerate=()),
               "active is scratch after load")
        dev.pset("STORE.PRESET_DELETE", b"", scratch)
        chk.ok(reached(lambda: _occupied(dev, scratch), 0, tolerate=()), "scratch freed")
    finally:
        try:
            if saved:
                _drop_scratch(dev, scratch)
        finally:
            dev.pset("STORE.PRESET_LOAD", b"", orig_active)
            dev.wait_ready()
    chk.ok(reached(lambda: dev.pget_ready("STORE.PRESET_ACTIVE", length=1)[0], orig_active, tolerate=()),
           "original slot active again")


@test("paged_store", mutating=True, flash=4)
def preset_name_roundtrip(dev, profile, chk):
    """PRESET_NAME on a scratch slot: SET persists, a GET reply is a valid SET payload, 31-byte cap, restored."""
    require_page(dev, STORE)
    scratch = _scratch_slot(dev)
    if scratch is None:
        raise Skip("every preset slot is occupied; no scratch slot to use")
    name = lambda: dev.pget_ready("STORE.PRESET_NAME", scratch, 32)  # noqa: E731
    orig = name()
    try:
        dev.pset("STORE.PRESET_NAME", b"QA Paged", scratch)
        chk.ok(reached(lambda: name().split(b"\x00", 1)[0], b"QA Paged", tolerate=()), "paged SET persisted")
        got = name()
        dev.pset("STORE.PRESET_NAME", got, scratch)
        dev.wait_ready()
        chk.ok(reached(name, got, tolerate=()), "a GET reply is a valid SET payload")
        dev.pset("STORE.PRESET_NAME", b"P" * 32, scratch)
        chk.ok(reached(name, b"P" * 31 + b"\x00", tolerate=()), "32-byte SET keeps 31 + NUL")
    finally:
        dev.pset("STORE.PRESET_NAME", orig.split(b"\x00", 1)[0] or b"\x00", scratch)
        dev.wait_ready()
    chk.ok(reached(name, orig, tolerate=()), "name restored")


@test("paged_store", mutating=True, flash=3)
def startup_roundtrip(dev, profile, chk):
    """STARTUP {mode, slot} round-trips, shows in DIRECTORY bytes 2-3, and is restored."""
    require_page(dev, STORE)
    get = lambda: dev.pget_ready("STORE.STARTUP", length=2)  # noqa: E731
    orig = get()
    flipped = bytes([1 - orig[0], orig[1]])
    try:
        dev.pset("STORE.STARTUP", flipped)
        chk.ok(reached(get, flipped, tolerate=()), "paged SET persisted")
        chk.eq(dev.pget("STORE.DIRECTORY", length=7)[2:4], flipped, "DIRECTORY bytes 2-3 agree")
        dev.pset("STORE.STARTUP", orig)
        chk.ok(reached(get, orig, tolerate=()), "paged SET back to the original")
        dev.pset("STORE.STARTUP", get())
        chk.ok(reached(get, orig, tolerate=()), "a GET reply is a valid SET payload")
    finally:
        dev.pset("STORE.STARTUP", orig)
        dev.wait_ready()


@test("paged_store", mutating=True, flash=5)
def output_config_mode_and_save(dev, profile, chk):
    """OUTPUT_CONFIG_MODE round-trips {0,1}, stores 0 for other values, shows in DIRECTORY; SAVE leaves it alone."""
    require_page(dev, STORE)
    get = lambda: dev.pget_ready("STORE.OUTPUT_CONFIG_MODE", length=1)[0]  # noqa: E731
    orig = get()
    try:
        dev.pset("STORE.OUTPUT_CONFIG_MODE", b"\x01")
        chk.ok(reached(get, 1, tolerate=()), "mode 1")
        chk.eq(_dir(dev)["output_config_mode"], 1, "DIRECTORY byte 5 agrees")
        dev.pset("STORE.OUTPUT_CONFIG_MODE", b"\x05")
        chk.ok(reached(get, 0, tolerate=()), "mode 5 stores independent")
        dev.pset("STORE.OUTPUT_CONFIG_MODE", b"\x01")
        chk.ok(reached(get, 1, tolerate=()), "mode 1 again")
        dev.pset("STORE.OUTPUT_CONFIG_SAVE")
        dev.wait_ready()
        chk.ok(reached(get, 1, tolerate=()), "OUTPUT_CONFIG_SAVE leaves the mode alone")
    finally:
        dev.pset("STORE.OUTPUT_CONFIG_MODE", bytes([orig]))
        dev.wait_ready()
    chk.ok(reached(get, orig, tolerate=()), "mode restored")
