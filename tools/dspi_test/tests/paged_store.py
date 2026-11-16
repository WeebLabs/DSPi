"""
Native paged tests for the STORE page (plan section 5.2): preset slots, directory, startup, IO config
persistence, factory reset guards and bulk parameters. Every test SKIPs without the paged STORE page.
Flash tests use one scratch slot (the highest empty one) and restore everything they change.
FACTORY_RESET is only ever sent in shapes the dispatcher refuses before its handler runs.
"""

import struct
import time

from ..device import OP, PagedStall, ProtocolError, Stall, Timeout
from ..framework import Skip, test

STORE = 0x02
PRESET_SLOTS = 10
BULK_LEN = 6136
CHUNK = 2048


def _paged(dev):
    try:
        info = dev.protocol_info()
    except ProtocolError as e:
        raise AssertionError(str(e)) from None
    if info is None:
        raise Skip("no paged dispatcher: SYS.PROTOCOL_INFO STALLs (firmware 1.1.6 or older)")
    if STORE not in dev.paged_commands():
        raise Skip("firmware has no STORE page")


def _expect_stall(chk, fn, code, msg, retries=3):
    """fn() must STALL with VC code `code`. Retried because LAST_RESULT is shared by all USB hosts."""
    seen = None
    for _ in range(retries):
        try:
            r = fn()
        except PagedStall as s:
            if s.result is not None:
                return chk.eq(s.result_name, code, f"{msg}: LAST_RESULT")
            seen = s
            continue
        return chk.ok(False, f"{msg}: expected STALL with {code}, got {r!r}")
    return chk.ok(False, f"{msg}: STALLed but LAST_RESULT unreadable or overwritten ({seen})")


def _ready(dev, fn, retries=3):
    """A read that must succeed; retried through the busy window after a deferred apply."""
    for attempt in range(retries):
        try:
            return fn()
        except (Stall, Timeout):
            if attempt == retries - 1:
                raise
            dev.wait_ready()
            time.sleep(0.05)


def _poll(fn, want, timeout_s=2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if fn() == want:
            return True
        time.sleep(0.04)
    return False


def _dir(dev):
    d = dev.pget("STORE.DIRECTORY", length=7)
    return {"occupied": d[0] | (d[1] << 8), "startup_mode": d[2], "default_slot": d[3],
            "active": d[4], "output_config_mode": d[5], "mv_mode": d[6]}


def _scratch_slot(dev):
    occ = _dir(dev)["occupied"]
    return next((s for s in range(PRESET_SLOTS - 1, -1, -1) if not occ & (1 << s)), None)


def _bulk(dev):
    return _ready(dev, lambda: dev.pget("STORE.ALL_PARAMS", length=BULK_LEN))


def _settle(dev):
    dev.wait_ready()
    time.sleep(0.1)


def _paged_chunks(dev, size=CHUNK):
    return b"".join(dev.pget("STORE.ALL_PARAMS_CHUNK", off, min(size, BULK_LEN - off))
                    for off in range(0, BULK_LEN, size))


def _legacy_chunks(dev, size=CHUNK):
    return b"".join(dev.raw_get(OP.GET_ALL_PARAMS_CHUNK, min(size, BULK_LEN - off), off)
                    for off in range(0, BULK_LEN, size))


# --- Reads -------------------------------------------------------------------

@test("paged_store")
def reads_match_legacy(dev, profile, chk):
    """DIRECTORY, PRESET_ACTIVE, STARTUP, OUTPUT_CONFIG_MODE and every PRESET_NAME equal their 1.1.6 reads."""
    _paged(dev)
    chk.eq(dev.pget("STORE.DIRECTORY", length=7), dev.raw_get(OP.PRESET_GET_DIR, 7), "DIRECTORY == 0x95")
    chk.eq(dev.pget("STORE.PRESET_ACTIVE", length=1), dev.raw_get(OP.PRESET_GET_ACTIVE, 1), "PRESET_ACTIVE == 0x9A")
    chk.eq(dev.pget("STORE.STARTUP", length=2), dev.raw_get(OP.PRESET_GET_STARTUP, 3)[:2], "STARTUP == 0x97[:2]")
    chk.eq(dev.pget("STORE.OUTPUT_CONFIG_MODE", length=1), dev.raw_get(OP.GET_OUTPUT_CONFIG_MODE, 1),
           "OUTPUT_CONFIG_MODE == 0x99")
    for slot in range(PRESET_SLOTS):
        chk.eq(dev.pget("STORE.PRESET_NAME", slot, 32), dev.raw_get(OP.PRESET_GET_NAME, 32, slot),
               f"PRESET_NAME {slot} == 0x93")
    d = _dir(dev)
    chk.eq(dev.raw_get(OP.PRESET_GET_STARTUP, 3)[2], d["active"], "last-active byte lives in DIRECTORY")
    chk.member(d["startup_mode"], (0, 1), "startup mode")
    chk.in_range(d["default_slot"], 0, PRESET_SLOTS - 1, "default slot")


@test("paged_store")
def read_errors(dev, profile, chk):
    """Bad slot and non-zero wValue are VC_ERR_ADDRESS; GET of an action and SET of a read are VC_ERR_DIRECTION."""
    _paged(dev)
    for addr in (PRESET_SLOTS, 0x100):
        _expect_stall(chk, lambda: dev.pget("STORE.PRESET_NAME", addr, 32), "VC_ERR_ADDRESS",
                      f"PRESET_NAME GET slot {addr:#x}")
    for name, n in (("STORE.DIRECTORY", 7), ("STORE.PRESET_ACTIVE", 1), ("STORE.STARTUP", 2),
                    ("STORE.OUTPUT_CONFIG_MODE", 1), ("STORE.ALL_PARAMS", BULK_LEN)):
        _expect_stall(chk, lambda: dev.pget(name, 1, n), "VC_ERR_ADDRESS", f"{name} at wValue 1")
    for name in ("STORE.DIRECTORY", "STORE.PRESET_ACTIVE"):
        _expect_stall(chk, lambda: dev.pset(name, b"\x00"), "VC_ERR_DIRECTION", f"SET {name}")
    for name in ("STORE.PRESET_SAVE", "STORE.PRESET_LOAD", "STORE.PRESET_DELETE", "STORE.OUTPUT_CONFIG_SAVE",
                 "STORE.FACTORY_RESET"):
        _expect_stall(chk, lambda: dev.pget(name, 0, 1), "VC_ERR_DIRECTION", f"GET {name}")


# --- SET guards that never reach a handler or are refused by it (no flash) ---------------

@test("paged_store")
def set_guards(dev, profile, chk):
    """Action payloads, bad slots, bad sizes and bad startup values are refused before anything is queued."""
    _paged(dev)
    before = dev.pget("STORE.DIRECTORY", length=7)
    for name in ("STORE.PRESET_SAVE", "STORE.PRESET_LOAD", "STORE.PRESET_DELETE"):
        _expect_stall(chk, lambda: dev.pset(name, b"", PRESET_SLOTS), "VC_ERR_ADDRESS", f"{name} slot 10")
        _expect_stall(chk, lambda: dev.pset(name, b"", 0xFF), "VC_ERR_ADDRESS", f"{name} slot 255")
        _expect_stall(chk, lambda: dev.pset(name, b"\x00", 0), "VC_ERR_LENGTH", f"{name} with a payload")
    _expect_stall(chk, lambda: dev.pset("STORE.FACTORY_RESET", b"\x00"), "VC_ERR_LENGTH",
                  "FACTORY_RESET with a 1-byte payload")
    _expect_stall(chk, lambda: dev.pset("STORE.OUTPUT_CONFIG_SAVE", b"\x00"), "VC_ERR_LENGTH",
                  "OUTPUT_CONFIG_SAVE with a payload")
    _expect_stall(chk, lambda: dev.pset("STORE.PRESET_NAME", b"x", PRESET_SLOTS), "VC_ERR_ADDRESS",
                  "PRESET_NAME SET slot 10")
    _expect_stall(chk, lambda: dev.pset("STORE.PRESET_NAME", b"", 0), "VC_ERR_LENGTH", "PRESET_NAME empty")
    _expect_stall(chk, lambda: dev.pset("STORE.PRESET_NAME", b"N" * 33, 0), "VC_ERR_LENGTH", "PRESET_NAME 33 B")
    for n in (1, 3):
        _expect_stall(chk, lambda: dev.pset("STORE.STARTUP", bytes(n)), "VC_ERR_LENGTH", f"STARTUP {n} B")
    _expect_stall(chk, lambda: dev.pset("STORE.STARTUP", bytes([2, 0])), "VC_ERR_VALUE", "STARTUP mode 2")
    _expect_stall(chk, lambda: dev.pset("STORE.STARTUP", bytes([0, PRESET_SLOTS])), "VC_ERR_VALUE",
                  "STARTUP slot 10")
    _expect_stall(chk, lambda: dev.pset("STORE.STARTUP", bytes([0, 0]), 1), "VC_ERR_ADDRESS", "STARTUP wValue 1")
    for n in (0, 2):
        _expect_stall(chk, lambda: dev.pset("STORE.OUTPUT_CONFIG_MODE", bytes(n)), "VC_ERR_LENGTH",
                      f"OUTPUT_CONFIG_MODE {n} B")
    time.sleep(0.2)
    chk.eq(dev.pget("STORE.DIRECTORY", length=7), before, "directory untouched by refused SETs")


# --- Bulk parameters -------------------------------------------------------------

@test("paged_store")
def all_params_matches_legacy(dev, profile, chk):
    """STORE.ALL_PARAMS equals 0xA0, and whole-struct reads assembled from paged or 0xA2 chunks equal it."""
    _paged(dev)
    for attempt in range(3):
        whole = _bulk(dev)
        legacy = dev.raw_get(OP.GET_ALL_PARAMS, BULK_LEN)
        paged = _paged_chunks(dev)
        odd = _paged_chunks(dev, 1000)
        old_chunks = _legacy_chunks(dev)
        if whole == legacy == paged == odd == old_chunks:
            break
        chk.note(f"bulk reads disagreed on attempt {attempt} (live change or another host)")
    chk.eq(len(whole), BULK_LEN, "ALL_PARAMS length")
    chk.eq(whole, legacy, "ALL_PARAMS == 0xA0")
    chk.eq(paged, whole, "2048-byte paged chunks == ALL_PARAMS")
    chk.eq(odd, whole, "1000-byte paged chunks == ALL_PARAMS")
    chk.eq(old_chunks, whole, "0xA2 chunks == ALL_PARAMS")
    fmt, plat, nch = struct.unpack_from("<BBB", whole)
    chk.eq((fmt, plat, nch), (profile.wire_format_version, profile.platform_id, profile.num_channels), "header")
    head = dev.pget("STORE.ALL_PARAMS_CHUNK", 0, 16)
    chk.eq(head, whole[:16], "a short offset-0 chunk is the struct prefix")


@test("paged_store", mutating=True)
def all_params_set_roundtrip(dev, profile, chk):
    """A paged SET of a snapshot is a fixpoint, whole or chunked, through the new and the 0xA3 chunk paths."""
    _paged(dev)
    a = _bulk(dev)
    dev.pset("STORE.ALL_PARAMS", a)
    _settle(dev)
    b = _bulk(dev)
    if b != a:
        chk.note("first apply normalized the snapshot (clamped fields); comparing against the normalized blob")
    for label, send in (
            ("whole paged SET", lambda: dev.pset("STORE.ALL_PARAMS", b)),
            ("2048-byte paged chunks", lambda: [dev.pset("STORE.ALL_PARAMS_CHUNK", b[o:o + CHUNK], o)
                                                for o in range(0, BULK_LEN, CHUNK)]),
            ("64-byte paged chunks", lambda: [dev.pset("STORE.ALL_PARAMS_CHUNK", b[o:o + 64], o)
                                              for o in range(0, BULK_LEN, 64)]),
            ("0xA3 chunks", lambda: [dev.raw_set(OP.SET_ALL_PARAMS_CHUNK, b[o:o + CHUNK], o)
                                     for o in range(0, BULK_LEN, CHUNK)])):
        reenum = dev.reenumerations
        send()
        _settle(dev)
        got = _bulk(dev)
        if dev.reenumerations != reenum:
            chk.note(f"{label}: re-enumeration during the apply, compare skipped")
            continue
        chk.eq(got, b, f"{label} leaves the state unchanged")


@test("paged_store", mutating=True)
def all_params_set_guards(dev, profile, chk):
    """Wrong lengths are VC_ERR_LENGTH, a header the apply would drop is VC_ERR_VALUE, and nothing changes."""
    _paged(dev)
    a = _bulk(dev)
    _expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS", a[:100]), "VC_ERR_LENGTH", "100-byte SET")
    _expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS", a[:64]), "VC_ERR_LENGTH", "64-byte SET")
    _expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS", a + b"\x00"), "VC_ERR_LENGTH", "6137-byte SET")
    _expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS", a, 1), "VC_ERR_ADDRESS", "SET at wValue 1")
    for off, val, what in ((0, 99, "format version"), (1, a[1] ^ 1, "platform"), (2, a[2] + 1, "channel count")):
        bad = bytearray(a)
        bad[off] = val & 0xFF
        _expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS", bytes(bad)), "VC_ERR_VALUE", f"bad {what}")
    bad = bytearray(a)
    bad[0] = 99
    for o in range(0, BULK_LEN - CHUNK, CHUNK):
        dev.pset("STORE.ALL_PARAMS_CHUNK", bytes(bad[o:o + CHUNK]), o)
    last = (BULK_LEN // CHUNK) * CHUNK
    _expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS_CHUNK", bytes(bad[last:]), last), "VC_ERR_VALUE",
                  "chunked SET with a bad header fails on its final chunk")
    _settle(dev)
    chk.eq(_bulk(dev), a, "refused bulk SETs left the state alone")


@test("paged_store")
def chunk_errors(dev, profile, chk):
    """Chunk offsets past the struct, overruns, out-of-order SETs and reads without a snapshot are refused."""
    _paged(dev)
    _expect_stall(chk, lambda: dev.pget("STORE.ALL_PARAMS_CHUNK", BULK_LEN, 64), "VC_ERR_ADDRESS", "GET at 6136")
    _expect_stall(chk, lambda: dev.pget("STORE.ALL_PARAMS_CHUNK", 64, 64), "VC_ERR_STATE", "GET at 64, no session")
    _expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS_CHUNK", b"\x00", BULK_LEN), "VC_ERR_ADDRESS",
                  "SET at 6136")
    _expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS_CHUNK", bytes(64), BULK_LEN - 32), "VC_ERR_LENGTH",
                  "SET past the end")
    _expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS_CHUNK", bytes(64), 64), "VC_ERR_STATE",
                  "SET at 64, no session")
    _expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS_CHUNK", b""), "VC_ERR_LENGTH", "zero-length SET")
    snap = _bulk(dev)
    dev.pset("STORE.ALL_PARAMS_CHUNK", snap[:64], 0)
    _expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS_CHUNK", snap[128:192], 128), "VC_ERR_STATE",
                  "SET skipping a chunk")
    after = _bulk(dev)
    chk.eq(after, snap, "an abandoned chunked SET applies nothing")


@test("paged_store")
def chunk_sessions_do_not_cross_paths(dev, profile, chk):
    """A chunk session opened on one path (paged or 0xA2/0xA3) is torn down by a chunk request on the other."""
    _paged(dev)
    whole = _bulk(dev)
    dev.pget("STORE.ALL_PARAMS_CHUNK", 0, CHUNK)
    chk.stalls(lambda: dev.raw_get(OP.GET_ALL_PARAMS_CHUNK, CHUNK, CHUNK), "0xA2 does not continue a paged GET")
    _expect_stall(chk, lambda: dev.pget("STORE.ALL_PARAMS_CHUNK", CHUNK, CHUNK), "VC_ERR_STATE",
                  "the paged GET session is gone after the 0xA2 request")

    dev.raw_get(OP.GET_ALL_PARAMS_CHUNK, CHUNK, 0)
    _expect_stall(chk, lambda: dev.pget("STORE.ALL_PARAMS_CHUNK", CHUNK, CHUNK), "VC_ERR_STATE",
                  "a paged GET does not continue a 0xA2 session")
    chk.stalls(lambda: dev.raw_get(OP.GET_ALL_PARAMS_CHUNK, CHUNK, CHUNK),
               "the 0xA2 session is gone after the paged request")

    dev.pset("STORE.ALL_PARAMS_CHUNK", whole[:CHUNK], 0)
    chk.stalls(lambda: dev.raw_set(OP.SET_ALL_PARAMS_CHUNK, whole[CHUNK:2 * CHUNK], CHUNK),
               "0xA3 does not continue a paged SET")
    dev.raw_set(OP.SET_ALL_PARAMS_CHUNK, whole[:CHUNK], 0)
    _expect_stall(chk, lambda: dev.pset("STORE.ALL_PARAMS_CHUNK", whole[CHUNK:2 * CHUNK], CHUNK),
                  "VC_ERR_STATE", "a paged SET does not continue a 0xA3 session")

    chk.eq(_bulk(dev), whole, "no half-received chunked SET was applied")
    chk.eq(_paged_chunks(dev), whole, "a fresh paged session still reads the whole struct")


# --- Flash -------------------------------------------------------------------------

@test("paged_store", mutating=True, flash=6)
def preset_lifecycle(dev, profile, chk):
    """PRESET_SAVE, PRESET_LOAD and PRESET_DELETE on a scratch slot, seen through both protocols, then restored."""
    _paged(dev)
    scratch = _scratch_slot(dev)
    if scratch is None:
        raise Skip("every preset slot is occupied; no scratch slot to use")
    orig_active = dev.pget("STORE.PRESET_ACTIVE", length=1)[0]
    try:
        dev.pset("STORE.PRESET_SAVE", b"", scratch)
        chk.ok(_poll(lambda: _ready(dev, lambda: dev.pget("STORE.PRESET_ACTIVE", length=1))[0], scratch),
               "active became scratch after save")
        chk.ok(_poll(lambda: (_ready(dev, lambda: _dir(dev))["occupied"] >> scratch) & 1, 1), "scratch occupied")
        chk.eq(dev.raw_get(OP.PRESET_GET_DIR, 7), dev.pget("STORE.DIRECTORY", length=7), "0x95 agrees")
        dev.pset("STORE.PRESET_LOAD", b"", scratch)
        dev.wait_ready()
        chk.ok(_poll(lambda: _ready(dev, lambda: dev.pget("STORE.PRESET_ACTIVE", length=1))[0], scratch),
               "active is scratch after load")
        dev.pset("STORE.PRESET_DELETE", b"", scratch)
        chk.ok(_poll(lambda: (_ready(dev, lambda: _dir(dev))["occupied"] >> scratch) & 1, 0), "scratch freed")
    finally:
        dev.pset("STORE.PRESET_LOAD", b"", orig_active)
        dev.wait_ready()
    chk.ok(_poll(lambda: _ready(dev, lambda: dev.pget("STORE.PRESET_ACTIVE", length=1))[0], orig_active),
           "original slot active again")


@test("paged_store", mutating=True, flash=4)
def preset_name_roundtrip(dev, profile, chk):
    """PRESET_NAME on a scratch slot: paged and 0x94 writes, paged and 0x93 reads, 31-byte cap, restored."""
    _paged(dev)
    scratch = _scratch_slot(dev)
    if scratch is None:
        raise Skip("every preset slot is occupied; no scratch slot to use")
    name = lambda: _ready(dev, lambda: dev.pget("STORE.PRESET_NAME", scratch, 32))  # noqa: E731
    orig = name()
    try:
        dev.pset("STORE.PRESET_NAME", b"QA Paged", scratch)
        chk.ok(_poll(lambda: name().split(b"\x00", 1)[0], b"QA Paged"), "paged SET persisted")
        chk.eq(dev.raw_get(OP.PRESET_GET_NAME, 32, scratch), name(), "0x93 agrees")
        dev.pset("STORE.PRESET_NAME", name(), scratch)
        dev.raw_set(OP.PRESET_SET_NAME, b"QA Legacy", scratch)
        chk.ok(_poll(lambda: name().split(b"\x00", 1)[0], b"QA Legacy"), "0x94 SET, paged GET")
        dev.pset("STORE.PRESET_NAME", b"P" * 32, scratch)
        chk.ok(_poll(name, b"P" * 31 + b"\x00"), "32-byte SET keeps 31 + NUL")
    finally:
        dev.pset("STORE.PRESET_NAME", orig.split(b"\x00", 1)[0] or b"\x00", scratch)
        dev.wait_ready()
    chk.ok(_poll(name, orig), "name restored")


@test("paged_store", mutating=True, flash=3)
def startup_roundtrip(dev, profile, chk):
    """STARTUP {mode, slot} round-trips, agrees with 0x96/0x97 both ways, and is restored."""
    _paged(dev)
    get = lambda: _ready(dev, lambda: dev.pget("STORE.STARTUP", length=2))  # noqa: E731
    orig = get()
    flipped = bytes([1 - orig[0], orig[1]])
    try:
        dev.pset("STORE.STARTUP", flipped)
        chk.ok(_poll(get, flipped), "paged SET persisted")
        chk.eq(dev.raw_get(OP.PRESET_GET_STARTUP, 3)[:2], flipped, "0x97 agrees")
        dev.raw_set(OP.PRESET_SET_STARTUP, orig)
        chk.ok(_poll(get, orig), "0x96 SET, paged GET")
        dev.pset("STORE.STARTUP", get())
        chk.ok(_poll(get, orig), "a GET reply is a valid SET payload")
    finally:
        dev.pset("STORE.STARTUP", orig)
        dev.wait_ready()


@test("paged_store", mutating=True, flash=5)
def output_config_mode_and_save(dev, profile, chk):
    """OUTPUT_CONFIG_MODE round-trips {0,1}, stores 0 for other values, agrees with 0x98/0x99; SAVE leaves it alone."""
    _paged(dev)
    get = lambda: _ready(dev, lambda: dev.pget("STORE.OUTPUT_CONFIG_MODE", length=1))[0]  # noqa: E731
    orig = get()
    try:
        dev.pset("STORE.OUTPUT_CONFIG_MODE", b"\x01")
        chk.ok(_poll(get, 1), "mode 1")
        chk.eq(dev.raw_get(OP.GET_OUTPUT_CONFIG_MODE, 1), b"\x01", "0x99 agrees")
        dev.pset("STORE.OUTPUT_CONFIG_MODE", b"\x05")
        chk.ok(_poll(get, 0), "mode 5 stores independent, as 0x98 did")
        dev.raw_set(OP.SET_OUTPUT_CONFIG_MODE, b"\x01")
        chk.ok(_poll(get, 1), "0x98 SET, paged GET")
        dev.pset("STORE.OUTPUT_CONFIG_SAVE")
        dev.wait_ready()
        chk.ok(_poll(get, 1), "OUTPUT_CONFIG_SAVE leaves the mode alone")
    finally:
        dev.pset("STORE.OUTPUT_CONFIG_MODE", bytes([orig]))
        dev.wait_ready()
    chk.ok(_poll(get, orig), "mode restored")
