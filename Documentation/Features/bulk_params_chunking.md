# Chunked Bulk Params: Host-Side Migration Guide

*Last updated: 2026-11-22 (commands on the paged protocol of firmware 1.1.7, size at wire V35, result codes)*

## Why this exists

At wire format V16 the bulk-params blob (`WireBulkParams`) grew to **5864
bytes**, exact-size-only. Windows cannot move that in one vendor control
transfer: WinUSB enforces a documented **4096-byte maximum** on the data
stage, and libusb's Windows backend rejects larger requests with
`LIBUSB_ERROR_INVALID_PARAM` before anything reaches the wire (GitHub issue
#62). All three Zadig driver options route through the same backend, so no
driver swap avoids it. Chrome's WebUSB on Windows binds through WinUSB and
inherits the same cap.

The firmware therefore provides chunked access to the same buffer. Both
commands are on the `STORE` page of the paged protocol (see `../commands.md`):

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Notes |
|---|---|---|---|---|---|---|
| `STORE.ALL_PARAMS` | 0x02 | 0x40 | GET, SET | none (0) | GET 6284, SET 6284 | Single shot. A SET must be the whole struct |
| `STORE.ALL_PARAMS_CHUNK` | 0x02 | 0x41 | GET, SET | byte offset | GET var, SET var | `wLength` is the chunk size. USB only |

At wire V36 the struct is 6284 bytes. The single-shot `STORE.ALL_PARAMS`
remains the right choice on macOS, Linux, and on the UART/I2C control
transports (no size cap there). `STORE.ALL_PARAMS_CHUNK` is refused on UART
and I2C with `VC_ERR_BLOCKED`. The wire payload is byte-identical to the
single-shot command; only the transfer is split. No wire format version
change. Firmware 1.1.6 and older used opcodes 0xA0/0xA1 (single shot) and
0xA2/0xA3 (chunked) for the same transfers.

## What a host must change

Only the transport layer of the bulk read/write helpers. Everything that
parses or builds `WireBulkParams` is untouched.

Pick a chunk size of at most 4096; **2048 is recommended** (comfortable
margin, four transfers per direction at wire V35).

Every request below uses `wIndex = 0x0202` (page 0x02 in the high byte,
vendor interface 2 in the low byte).

### Read (replaces one `STORE.ALL_PARAMS` GET)

1. Issue control-IN `bRequest=0x41, wValue=0, wLength=2048`. Receiving
   offset 0 makes the device snapshot the complete struct into an internal
   buffer under its bulk lock, so all chunks come from one coherent image.
2. Repeat with `wValue = 2048`, then `4096`, then `6144`, until you have
   accumulated the full struct. The final chunk returns short (6284 - 6144
   = 108 bytes at wire V35); the device clamps `wLength` past the end for you.
   An offset at or past the end STALLs with `VC_ERR_ADDRESS`.
3. Concatenate in offset order. Sanity-check `header.payload_length`
   against what you received, as with `STORE.ALL_PARAMS`.

You may read `header` fields from the first chunk to learn the exact total
instead of hardcoding 6284.

### Write (replaces one `STORE.ALL_PARAMS` SET)

1. Build the full payload exactly as for `STORE.ALL_PARAMS`.
2. Issue control-OUT `bRequest=0x41, wValue=0` with the first 2048 bytes,
   then `wValue=2048`, then `wValue=4096`, then `wValue=6144` with the
   remainder.
3. Chunks **must be sequential and non-overlapping starting at offset 0**.
   An out-of-order offset STALLs at SETUP with `VC_ERR_STATE` and leaves the
   session open, so the host may resend the expected offset or restart
   from 0. A chunk that runs past the end STALLs with `VC_ERR_LENGTH`.
4. When the final byte lands, the device checks the header and applies the
   blob through the same deferred main-loop path as `STORE.ALL_PARAMS`
   (same validation, same notify rebaseline). A header the apply would
   refuse STALLs that final chunk with `VC_ERR_VALUE` and nothing is
   applied. There is no separate commit command.

### Error handling and session rules

- A STALL on a chunk means the session was rejected or lost, except for the
  out-of-order SET case above. Restart the whole transfer from offset 0.
  Offset 0 always starts a fresh session. `SYS.LAST_RESULT` gives the
  reason: `VC_ERR_STATE` for no session or out of order, `VC_ERR_BUSY` for
  a lock held elsewhere.
- Do not interleave other vendor requests between chunks; any non-chunk
  vendor request tears the open session down (by design, so an abandoned
  session cannot wedge the shared buffer).
- Do not pause longer than about 3 seconds between chunks; an idle session
  is reaped and later offsets will STALL.
- A STALL at offset 0 (`VC_ERR_BUSY`) usually means the bulk buffer is
  momentarily owned by another transport (UART/I2C bulk in flight, or a
  pending apply); retry after a short delay.
- One session at a time; starting a GET session cancels an unfinished SET
  session and vice versa.

### pyusb sketch (tools/dspi_test)

```python
TOTAL = 6284          # wire V36, or read header.payload_length from the first chunk
CHUNK = 2048
STORE_WINDEX = (0x02 << 8) | 2    # page STORE, vendor interface 2
ALL_PARAMS_CHUNK = 0x41

def get_all_params(dev):
    data = b""
    off = 0
    while off < TOTAL:
        n = min(CHUNK, TOTAL - off)
        data += dev.ctrl_transfer(0xC1, ALL_PARAMS_CHUNK, wValue=off, wIndex=STORE_WINDEX,
                                  data_or_wLength=n, timeout=1000).tobytes()
        off += n
    return data

def set_all_params(dev, blob):
    assert len(blob) == TOTAL
    for off in range(0, TOTAL, CHUNK):
        chunk = blob[off:off + CHUNK]
        dev.ctrl_transfer(0x41, ALL_PARAMS_CHUNK, wValue=off, wIndex=STORE_WINDEX,
                          data_or_wLength=chunk, timeout=1000)
```

### Choosing single-shot vs chunked at runtime

Simplest portable policy: try `STORE.ALL_PARAMS` once; on
`LIBUSB_ERROR_INVALID_PARAM` (or any pre-wire size rejection), fall back to
chunked and remember the choice. Alternatively, always use chunked on
`sys.platform == "win32"` / when running under WebUSB; the chunked path
works identically on every OS, so a host may also just use it everywhere.

## Firmware reference

Implementation: `firmware/DSPi/vendor/vc_bulk.c` (chunk sessions, bulk lock
and header check) and `firmware/DSPi/vendor/vc_page_store.c`. Sessions ride the shared bulk owner lock
introduced with the external control interfaces, so USB chunking, UART/I2C
bulk streaming, and single-shot USB transfers all serialize against the one
`bulk_param_buf`.
