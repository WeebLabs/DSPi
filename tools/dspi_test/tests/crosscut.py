"""
Cross-cutting contract & coverage suites that span the whole command surface.

  * contract_oversized_set_stalls — every data-stage SET STALLs on >64-byte payload.
  * unknown_opcode_stalls         — unused opcodes hit the default case => STALL.
  * index_ceiling_stalls          — the (differing) GET index ceilings all STALL.
  * all_gets_respond              — every pure-read GET opcode returns data, no crash.
  * quarantine_enter_bootloader   — 0xF0 documented as excluded (never sent).
"""

from ..device import OP, OPCODE_NAME, Stall
from ..framework import test, Skip


# Pure data-stage SETs (host->device payload); oversized must STALL.
_OVERSIZE_SETS = ["SET_MASTER_VOLUME", "SET_USER_VOLUME", "SET_PREAMP", "SET_EQ_PARAM",
                  "SET_LEVELLER_AMOUNT", "SET_MATRIX_ROUTE", "SET_DELAY", "SET_LOUDNESS_REF"]


@test("crosscut")
def contract_oversized_set_stalls(dev, profile, chk):
    """Any OUT SET with wLength > 64 STALLs (transport oversize guard)."""
    for name in _OVERSIZE_SETS:
        op = getattr(OP, name, None)
        if op is None:
            continue
        chk.stalls(lambda op=op: dev.set(op, b"\x00" * 65), f"{name} 65B -> STALL")


@test("crosscut")
def unknown_opcode_stalls(dev, profile, chk):
    """Opcodes with no handler hit the switch default and STALL (GET) — never silently 'work'."""
    used = set(OPCODE_NAME.keys())
    candidates = [0x40, 0x41, 0x68, 0x6F, 0x84, 0x8F, 0xA5, 0xCB, 0xCF, 0xDF, 0xE9, 0xEE]
    tested = 0
    for op in candidates:
        if op in used:
            continue
        tested += 1
        chk.stalls(lambda op=op: dev.get(op, 4), f"unknown opcode 0x{op:02X} GET -> STALL")
    chk.ok(tested >= 5, f"tested {tested} unknown opcodes")


@test("crosscut")
def index_ceiling_stalls(dev, profile, chk):
    """The various GET index ceilings all STALL one past the limit (and pass at the limit-1)."""
    nc, nb, no = profile.num_channels, profile.band_ceiling, profile.num_output_channels
    ni = profile.num_input_channels
    cases = [
        # (label, fn-just-past-ceiling, fn-last-valid)
        ("EQ channel", lambda: dev.get(OP.GET_EQ_PARAM, 4, wvalue=(nc << 8)),
                       lambda: dev.get(OP.GET_EQ_PARAM, 4, wvalue=((nc - 1) << 8))),
        ("EQ band", lambda: dev.get(OP.GET_EQ_PARAM, 4, wvalue=(nb << 4)),
                    lambda: dev.get(OP.GET_EQ_PARAM, 4, wvalue=((nb - 1) << 4))),
        ("delay channel", lambda: dev.get(OP.GET_DELAY, 4, wvalue=nc),
                          lambda: dev.get(OP.GET_DELAY, 4, wvalue=nc - 1)),
        ("legacy gain", lambda: dev.get(OP.GET_CHANNEL_GAIN, 4, wvalue=3),
                        lambda: dev.get(OP.GET_CHANNEL_GAIN, 4, wvalue=2)),
        ("legacy mute", lambda: dev.get(OP.GET_CHANNEL_MUTE, 1, wvalue=3),
                        lambda: dev.get(OP.GET_CHANNEL_MUTE, 1, wvalue=2)),
        ("preamp ch", lambda: dev.get(OP.GET_PREAMP_CH, 4, wvalue=ni),
                      lambda: dev.get(OP.GET_PREAMP_CH, 4, wvalue=ni - 1)),
        ("output enable", lambda: dev.get(OP.GET_OUTPUT_ENABLE, 1, wvalue=no),
                          lambda: dev.get(OP.GET_OUTPUT_ENABLE, 1, wvalue=no - 1)),
        ("output pin", lambda: dev.get(OP.GET_OUTPUT_PIN, 1, wvalue=profile.num_pin_outputs),
                       lambda: dev.get(OP.GET_OUTPUT_PIN, 1, wvalue=profile.num_pin_outputs - 1)),
        ("output type", lambda: dev.get(OP.GET_OUTPUT_TYPE, 1, wvalue=profile.num_spdif),
                        lambda: dev.get(OP.GET_OUTPUT_TYPE, 1, wvalue=profile.num_spdif - 1)),
        ("channel name", lambda: dev.get(OP.GET_CHANNEL_NAME, 32, wvalue=nc),
                         lambda: dev.get(OP.GET_CHANNEL_NAME, 32, wvalue=nc - 1)),
        ("preset name", lambda: dev.get(OP.PRESET_GET_NAME, 32, wvalue=10),
                        lambda: dev.get(OP.PRESET_GET_NAME, 32, wvalue=9)),
        ("matrix output", lambda: dev.get(OP.GET_MATRIX_ROUTE, 8, wvalue=no),
                          lambda: dev.get(OP.GET_MATRIX_ROUTE, 8, wvalue=no - 1)),
    ]
    for label, past, last in cases:
        chk.stalls(past, f"{label} past ceiling STALL")
        chk.no_stall(last, f"{label} last-valid OK")


@test("crosscut")
def all_gets_respond(dev, profile, chk):
    """Every pure-read GET opcode returns data at wValue=0 — broad no-crash liveness sweep."""
    # Only names that are pure reads (GET_* / PRESET_GET_*); never the SET-via-wValue
    # or action opcodes (those would mutate state / trigger flash / reset).
    read_ops = sorted(
        (op, name) for op, name in OPCODE_NAME.items()
        if (name.startswith("REQ_GET_") or name.startswith("REQ_PRESET_GET_"))
    )
    responded = 0
    for op, name in read_ops:
        try:
            data = dev.get(op, 64, wvalue=0)
            chk.ok(len(data) >= 1, f"{name} returned no bytes")
            responded += 1
        except Stall as e:
            chk.ok(False, f"{name} unexpectedly STALLed at wValue=0: {e}")
    chk.note(f"{responded}/{len(read_ops)} read opcodes responded")
    chk.ok(responded == len(read_ops), "all read opcodes responded")


@test("crosscut")
def quarantine_enter_bootloader(dev, profile, chk):
    """0xF0 ENTER_BOOTLOADER is intentionally NEVER transmitted (drops device to BOOTSEL)."""
    raise Skip("excluded by policy: would re-enumerate device into BOOTSEL and end the session")

# (Former quarantine_load_params removed: 0x52 was the deprecated LOAD_PARAMS and
#  is now REQ_SAVE_OUTPUT_CONFIG — a safe action command covered by
#  presets.output_config_save.)
