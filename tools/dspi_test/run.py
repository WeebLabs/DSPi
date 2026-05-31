#!/usr/bin/env python3
"""
run.py — entry point for the DSPi control-plane QA suite.

    python3 -m tools.dspi_test.run [options]
    ./tools/dspi_test/run.py        [options]

Options:
    --group G[,G...]        Run only these test groups (default: all).
    --list                  List registered tests and exit (no device needed if
                            modules import; still imports the package).
    --allow-flash           Enable preset/bulk/master-volume tests that erase
                            flash (bracketed + budget-capped).
    --allow-factory-reset   Additionally allow the one-shot factory-reset test.
    --flash-cap N           Hard cap on flash erase cycles (default 20).
    --stress N              Iterations for stress/fuzz tests (default 0 = light).
    --report PATH.md        Write a Markdown report.
    --json PATH.json        Write a JSON report.
    --no-restore            Skip the final snapshot restore (debugging only).
    --quiet                 Suppress per-test console lines.

Exit code: 0 if no FAIL/ERROR, 1 otherwise.
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime, timezone


def _import_tests():
    # Importing the package registers every @test via decorator side effects.
    from . import tests  # noqa: F401


def main(argv=None):
    ap = argparse.ArgumentParser(prog="dspi_test", description="DSPi control-plane QA suite")
    ap.add_argument("--group", default=None)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--catalog", default=None, metavar="PATH.md",
                    help="write the test catalog (from metadata) and exit")
    ap.add_argument("--allow-flash", action="store_true")
    ap.add_argument("--allow-factory-reset", action="store_true")
    ap.add_argument("--flash-cap", type=int, default=None)
    ap.add_argument("--stress", type=int, default=0)
    ap.add_argument("--report", default=None)
    ap.add_argument("--json", default=None)
    ap.add_argument("--no-restore", action="store_true")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    _import_tests()
    from .framework import REGISTRY, Runner, catalog_markdown
    from .device import DspiDevice, Disconnected

    groups = set(g.strip() for g in args.group.split(",")) if args.group else None
    cases = [tc for tc in REGISTRY if (groups is None or tc.group in groups)]

    if args.catalog:
        with open(args.catalog, "w") as f:
            f.write(catalog_markdown(cases))
        print(f"Catalog written: {args.catalog} ({len(cases)} tests)")
        return 0

    if args.list:
        cur = None
        for tc in cases:
            if tc.group != cur:
                cur = tc.group
                print(f"\n[{cur}]")
            tag = []
            if tc.flash:
                tag.append(f"flash={tc.flash}")
            if tc.needs == "factory_reset":
                tag.append("factory-reset")
            if tc.mutating:
                tag.append("mutating")
            tags = f"  ({', '.join(tag)})" if tag else ""
            print(f"  {tc.name}{tags} — {tc.doc}")
        print(f"\n{len(cases)} tests in {len(set(t.group for t in cases))} groups.")
        return 0

    try:
        dev = DspiDevice()
    except Disconnected as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 2

    from .profile import build_profile
    from . import lifecycle

    profile = build_profile(dev)
    profile.stress_iters = args.stress
    print(f"Device: {profile.summary()}")
    print(f"Running {len(cases)} tests"
          + (f" in groups {sorted(groups)}" if groups else "")
          + (" | flash ENABLED" if args.allow_flash else " | flash disabled")
          + (" | factory-reset ENABLED" if args.allow_factory_reset else "") + "\n")

    print("Capturing pre-suite snapshot...")
    snap = lifecycle.capture(dev, profile)
    print(f"  bulk={len(snap.bulk)}B  active_slot={snap.active_slot}  "
          f"occupied=0x{snap.occupied_mask:04X}  input_src={snap.input_source}\n")

    runner = Runner(dev, profile, allow_flash=args.allow_flash,
                    allow_factory_reset=args.allow_factory_reset,
                    flash_cap=args.flash_cap, verbose=not args.quiet)
    try:
        runner.run(cases)
    finally:
        if not args.no_restore:
            print("\nRestoring pre-suite state...")
            try:
                if dev.wait_ready():
                    lifecycle.restore_live(dev, snap)
                    try:
                        diffs = lifecycle.diff_bulk(dev, snap, profile)
                    except Exception:  # noqa: BLE001 — transient busy window; settle and retry once
                        dev.wait_ready()
                        diffs = lifecycle.diff_bulk(dev, snap, profile)
                    if diffs:
                        offs = ", ".join(f"@{o}:{a:02X}->{b:02X}" for o, a, b in diffs[:8])
                        print(f"  ⚠️ {len(diffs)} bulk byte(s) differ after restore: {offs}")
                    else:
                        print("  ✓ live state restored byte-for-byte")
                    dir_after = dev.get(lifecycle.OP.PRESET_GET_DIR, 7)
                    if bytes(dir_after) != bytes(snap.directory):
                        print(f"  ⚠️ directory changed: {snap.directory.hex()} -> {bytes(dir_after).hex()}")
                else:
                    print("  ✗ device unresponsive — cannot restore")
            except Exception as e:  # noqa: BLE001
                print(f"  ✗ restore failed: {e}")

    # Reports.
    ts = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
    c = runner.counts()
    total = sum(c.values())
    print(f"\n{'='*60}")
    print(f"RESULT: {c['PASS']}/{total} PASS · {c['FAIL']} FAIL · {c['ERROR']} ERROR · {c['SKIP']} SKIP")
    print(f"Transfers: {dev.gets} GET / {dev.sets} SET · flash erases: {runner.flash_used}"
          + (f" · USB re-enumerations: {dev.reenumerations}" if dev.reenumerations else ""))
    if runner.aborted:
        print(f"⚠️ ABORTED: {runner.abort_reason}")
    print("=" * 60)

    if args.report:
        with open(args.report, "w") as f:
            f.write(runner.to_markdown(timestamp=ts))
        print(f"Markdown report: {args.report}")
    if args.json:
        import json
        with open(args.json, "w") as f:
            json.dump(runner.to_json(), f, indent=2)
        print(f"JSON report: {args.json}")

    return 0 if (c["FAIL"] == 0 and c["ERROR"] == 0) else 1


if __name__ == "__main__":
    sys.exit(main())
