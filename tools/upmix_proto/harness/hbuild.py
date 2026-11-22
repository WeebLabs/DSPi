"""Host build of upmix.c for the harnesses. Needs only numpy, unlike harness.py."""
import ctypes
import os
import shutil
import subprocess
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
FW = os.path.join(REPO, 'firmware', 'DSPi')
DEFP = [100.0, 25.0, 30.0, 10.0, 100.0, 200.0, 12.0, 300.0, 7000.0, 90.0, 0.0]
FP = ctypes.POINTER(ctypes.c_float)
_I = ctypes.c_int


def ptr(a):
    return a.ctypes.data_as(FP)


def head_sources(rev='HEAD'):
    """upmix.c and upmix.h as committed at rev."""
    show = lambda f: subprocess.check_output(['git', '-C', REPO, 'show', '%s:firmware/DSPi/%s' % (rev, f)]).decode()
    return show('upmix.c'), show('upmix.h')


def tree_sources():
    return (open(os.path.join(FW, 'upmix.c')).read(), open(os.path.join(FW, 'upmix.h')).read())


def build(src_c, src_h, new):
    """Copy upmix.c/h beside the stubs so their "config.h" resolves to the stub."""
    d = tempfile.mkdtemp(prefix='upmix_h_')
    for f in os.listdir(os.path.join(HERE, 'stubs')):
        shutil.copy(os.path.join(HERE, 'stubs', f), d)
    open(os.path.join(d, 'upmix.c'), 'w').write(src_c)
    open(os.path.join(d, 'upmix.h'), 'w').write(src_h)
    shutil.copy(os.path.join(HERE, 'harness.c'), d)
    so = os.path.join(d, 'libupmix.so')
    # No FMA contraction, so the host result is comparable operation for operation.
    cmd = ['cc', '-O2', '-ffp-contract=off', '-fno-fast-math', '-Wno-unknown-attributes', '-shared', '-fPIC', '-I', d,
           os.path.join(d, 'harness.c'), '-o', so, '-lm'] + (['-DUPMIX_NEW'] if new else [])
    subprocess.check_call(cmd)
    lib = ctypes.CDLL(so)
    lib.h_run.argtypes = [FP, _I, _I, _I, FP, FP, _I] + [FP] * 5
    lib.h_coeffs.argtypes = [FP, _I, _I, _I, FP]
    lib.h_run_switch.argtypes = [FP] + [_I] * 5 + [FP, FP, _I] + [FP] * 5
    lib.h_run_ref.argtypes = [FP] + [_I] * 6 + [FP, FP, _I] + [FP] * 5
    if hasattr(lib, 'h_run_split'):
        lib.h_run_split.argtypes = [FP] + [_I] * 7 + [FP, FP, _I] + [FP] * 5
    if hasattr(lib, 'h_select_check'):
        lib.h_select_check.argtypes = []
        lib.h_profile_switch_check.argtypes = [FP, FP, _I, FP, FP, _I, _I]
    if hasattr(lib, 'h_route_check'):
        lib.h_route_check.argtypes = [_I, _I, FP, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(_I), ctypes.POINTER(_I)]
        lib.h_route_scenes.argtypes = []
    if hasattr(lib, 'h_run_back'):
        lib.h_run_back.argtypes = [FP, _I, _I, ctypes.c_float, _I, _I, FP, FP, _I, FP]
    if new:
        lib.h_run_stems.argtypes = [FP, _I, _I, _I, FP, FP, _I, _I, FP, FP, FP, FP, FP, FP, FP]
    return lib
