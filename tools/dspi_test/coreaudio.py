"""coreaudio.py: macOS HAL access for what PortAudio does not expose.

Opening a stream at a rate does NOT move the device: CoreAudio accepts the
request and resamples in software, leaving the hardware at its old rate. DSPi
follows the USB rate, so the device only changes when the HAL's
kAudioDevicePropertyNominalSampleRate is set. Every call degrades to a no-op
off macOS or if the frameworks will not load.
"""

from __future__ import annotations

import ctypes
import ctypes.util
import sys
import time

_CA = _CF = None
if sys.platform == "darwin":
    try:
        _CA = ctypes.CDLL("/System/Library/Frameworks/CoreAudio.framework/CoreAudio")
        _CF = ctypes.CDLL("/System/Library/Frameworks/CoreFoundation.framework/CoreFoundation")
    except OSError:                                        # pragma: no cover
        _CA = _CF = None


def _fourcc(s: str) -> int:
    return int.from_bytes(s.encode("ascii"), "big")


kAudioObjectSystemObject = 1
kAudioHardwarePropertyDevices = _fourcc("dev#")
kAudioObjectPropertyName = _fourcc("lnam")
kAudioDevicePropertyNominalSampleRate = _fourcc("nsrt")
kAudioDevicePropertyAvailableNominalSampleRates = _fourcc("nsr#")
kAudioDevicePropertyStreams = _fourcc("stm#")
kAudioObjectPropertyScopeGlobal = _fourcc("glob")
kAudioObjectPropertyScopeInput = _fourcc("inpt")
kAudioObjectPropertyScopeOutput = _fourcc("outp")
kCFStringEncodingUTF8 = 0x08000100


class _Addr(ctypes.Structure):
    _fields_ = [("mSelector", ctypes.c_uint32),
                ("mScope", ctypes.c_uint32),
                ("mElement", ctypes.c_uint32)]


class _ValueRange(ctypes.Structure):
    _fields_ = [("mMinimum", ctypes.c_double), ("mMaximum", ctypes.c_double)]


def available() -> bool:
    return _CA is not None and _CF is not None


def _prop_size(obj, addr):
    size = ctypes.c_uint32(0)
    st = _CA.AudioObjectGetPropertyDataSize(
        ctypes.c_uint32(obj), ctypes.byref(addr), 0, None, ctypes.byref(size))
    return None if st != 0 else size.value


def _get(obj, addr, buf):
    size = ctypes.c_uint32(ctypes.sizeof(buf))
    st = _CA.AudioObjectGetPropertyData(
        ctypes.c_uint32(obj), ctypes.byref(addr), 0, None,
        ctypes.byref(size), ctypes.byref(buf))
    return st == 0


def _device_ids():
    addr = _Addr(kAudioHardwarePropertyDevices, kAudioObjectPropertyScopeGlobal, 0)
    n = _prop_size(kAudioObjectSystemObject, addr)
    if not n:
        return []
    buf = (ctypes.c_uint32 * (n // 4))()
    return list(buf) if _get(kAudioObjectSystemObject, addr, buf) else []


def _device_name(dev_id) -> str:
    addr = _Addr(kAudioObjectPropertyName, kAudioObjectPropertyScopeGlobal, 0)
    ref = ctypes.c_void_p()
    if not _get(dev_id, addr, ref) or not ref:
        return ""
    out = ctypes.create_string_buffer(256)
    ok = _CF.CFStringGetCString(ref, out, 256, kCFStringEncodingUTF8)
    _CF.CFRelease(ref)
    return out.value.decode("utf-8", "replace") if ok else ""


def _has_streams(dev_id, scope) -> bool:
    addr = _Addr(kAudioDevicePropertyStreams, scope, 0)
    return bool(_prop_size(dev_id, addr))


def find_devices(name_substr: str):
    """[(dev_id, name, has_input, has_output)] for HAL devices matching a name."""
    if not available():
        return []
    hits = []
    for d in _device_ids():
        n = _device_name(d)
        if name_substr.lower() in n.lower():
            hits.append((d, n, _has_streams(d, kAudioObjectPropertyScopeInput),
                         _has_streams(d, kAudioObjectPropertyScopeOutput)))
    return hits


def get_rate(dev_id):
    addr = _Addr(kAudioDevicePropertyNominalSampleRate, kAudioObjectPropertyScopeGlobal, 0)
    val = ctypes.c_double(0)
    return val.value if _get(dev_id, addr, val) else None


def supported_rates(dev_id):
    """Discrete rates the device advertises (ranges collapse to their bounds)."""
    addr = _Addr(kAudioDevicePropertyAvailableNominalSampleRates,
                 kAudioObjectPropertyScopeGlobal, 0)
    n = _prop_size(dev_id, addr)
    if not n:
        return []
    buf = (_ValueRange * (n // ctypes.sizeof(_ValueRange)))()
    if not _get(dev_id, addr, buf):
        return []
    out = []
    for r in buf:
        out.append(r.mMinimum)
        if r.mMaximum != r.mMinimum:
            out.append(r.mMaximum)
    return sorted(set(out))


def set_rate(dev_id, rate: float, timeout_s: float = 3.0) -> bool:
    """Set the device's nominal rate and wait for the HAL to report it applied.

    The set is asynchronous; the HAL reconfigures the device and only then
    reports the new value, so poll rather than trusting the return status.
    """
    addr = _Addr(kAudioDevicePropertyNominalSampleRate, kAudioObjectPropertyScopeGlobal, 0)
    val = ctypes.c_double(float(rate))
    st = _CA.AudioObjectSetPropertyData(
        ctypes.c_uint32(dev_id), ctypes.byref(addr), 0, None,
        ctypes.c_uint32(ctypes.sizeof(val)), ctypes.byref(val))
    if st != 0:
        return False
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        cur = get_rate(dev_id)
        if cur is not None and abs(cur - rate) < 1.0:
            return True
        time.sleep(0.05)
    return False


def set_rate_by_name(name_substr: str, rate: float) -> bool:
    """Set every HAL device matching `name_substr` to `rate`.

    The DSPI_LOOPBACK build exposes two UAC functions, so macOS creates two
    devices (playback and capture); both must move or play_record straddles a
    rate boundary and CoreAudio resamples one side.
    """
    devs = find_devices(name_substr)
    if not devs:
        return False
    return all(set_rate(d, rate) for d, _n, _i, _o in devs)


if __name__ == "__main__":
    name = sys.argv[1] if len(sys.argv) > 1 else "DSPi"
    if not available():
        print("CoreAudio HAL unavailable on this platform")
        raise SystemExit(1)
    for d, n, has_in, has_out in find_devices(name):
        io = ("in" if has_in else "") + ("/out" if has_out else "")
        print(f"[{d}] {n}  ({io})  rate={get_rate(d)}  supports={supported_rates(d)}")


# --- Stream physical format (channel count) ---------------------------------
# DSPi's USB alts differ in channel count; the HAL picks one through the output
# stream's physical format, which PortAudio cannot set.

kAudioStreamPropertyPhysicalFormat = _fourcc("pft ")
kAudioStreamPropertyAvailablePhysicalFormats = _fourcc("pft#")   # an array of plain ASBDs, one per rate


class _ASBD(ctypes.Structure):
    _fields_ = [("mSampleRate", ctypes.c_double), ("mFormatID", ctypes.c_uint32),
                ("mFormatFlags", ctypes.c_uint32), ("mBytesPerPacket", ctypes.c_uint32),
                ("mFramesPerPacket", ctypes.c_uint32), ("mBytesPerFrame", ctypes.c_uint32),
                ("mChannelsPerFrame", ctypes.c_uint32), ("mBitsPerChannel", ctypes.c_uint32),
                ("mReserved", ctypes.c_uint32)]


def _streams(dev_id, scope):
    addr = _Addr(kAudioDevicePropertyStreams, scope, 0)
    n = _prop_size(dev_id, addr)
    if not n:
        return []
    buf = (ctypes.c_uint32 * (n // 4))()
    return list(buf) if _get(dev_id, addr, buf) else []


def _formats(stream_id):
    addr = _Addr(kAudioStreamPropertyAvailablePhysicalFormats, kAudioObjectPropertyScopeGlobal, 0)
    n = _prop_size(stream_id, addr)
    if not n:
        return []
    buf = (_ASBD * (n // ctypes.sizeof(_ASBD)))()
    return list(buf) if _get(stream_id, addr, buf) else []


def output_formats(dev_id):
    """[(channels, bits, rate)] the first output stream offers."""
    sts = _streams(dev_id, kAudioObjectPropertyScopeOutput)
    return [(f.mChannelsPerFrame, f.mBitsPerChannel, f.mSampleRate) for f in _formats(sts[0])] if sts else []


def output_channels(dev_id):
    sts = _streams(dev_id, kAudioObjectPropertyScopeOutput)
    if not sts:
        return None
    fmt = _ASBD()
    addr = _Addr(kAudioStreamPropertyPhysicalFormat, kAudioObjectPropertyScopeGlobal, 0)
    return fmt.mChannelsPerFrame if _get(sts[0], addr, fmt) else None


def set_output_channels(dev_id, channels: int, rate: float, timeout_s: float = 3.0) -> bool:
    """Select the output physical format with `channels` at `rate` (the widest
    bit depth on offer) and wait for the HAL to report it applied."""
    sts = _streams(dev_id, kAudioObjectPropertyScopeOutput)
    if not sts:
        return False
    pick = None
    for f in _formats(sts[0]):
        if f.mChannelsPerFrame == channels and abs(f.mSampleRate - rate) < 1.0:
            if pick is None or f.mBitsPerChannel > pick.mBitsPerChannel:
                pick = _ASBD.from_buffer_copy(f)
    if pick is None:
        return False
    paddr = _Addr(kAudioStreamPropertyPhysicalFormat, kAudioObjectPropertyScopeGlobal, 0)
    st = _CA.AudioObjectSetPropertyData(ctypes.c_uint32(sts[0]), ctypes.byref(paddr), 0, None,
                                        ctypes.c_uint32(ctypes.sizeof(pick)), ctypes.byref(pick))
    if st != 0:
        return False
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if output_channels(dev_id) == channels:
            return True
        time.sleep(0.05)
    return False
