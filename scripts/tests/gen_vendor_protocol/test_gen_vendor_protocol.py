#!/usr/bin/env python3
"""Self-test for scripts/gen_vendor_protocol.py. Run: python3 scripts/tests/gen_vendor_protocol/test_gen_vendor_protocol.py
expected.json is the golden output for fixtures/; regenerate it only after checking the diff by eye.
"""

import contextlib
import io
import json
import os
import sys
import tempfile
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
import gen_vendor_protocol as g  # noqa: E402

FIX = os.path.join(HERE, "fixtures")
FIX_PAGES = os.path.join(FIX, "pages")
FIX_CORE = os.path.join(FIX, "vc_core.h")
GOLDEN = os.path.join(FIX, "expected.json")

HDR = 'VC_PAGE(TEST, 0x0A, 1, "Test page")\n'


def cmd(num="0x10", name="FOO", get="foo_get", set_="foo_set", addr="NONE", glen="1",
        smin="1", smax="1", flags="0", arg="0", doc='"A doc"'):
    return "VC_CMD(%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)\n" % (
        num, name, get, set_, addr, glen, smin, smax, flags, arg, doc)


def run_cli(args):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        rc = g.main(args)
    return rc, out.getvalue(), err.getvalue()


class GoodInput(unittest.TestCase):
    def test_fixtures_parse_and_match_golden(self):
        reg = g.build_registry(FIX_PAGES, FIX_CORE)
        text = g.render_json(reg)
        self.assertEqual(text, g.render_json(g.build_registry(FIX_PAGES, FIX_CORE)))
        self.assertEqual(json.loads(text), reg)
        with open(GOLDEN, encoding="utf-8") as fh:
            self.assertEqual(text, fh.read())

    def test_fixture_features(self):
        pages = {p["mnemonic"]: p for p in g.parse_pages(FIX_PAGES)}
        self.assertEqual([p["page"] for p in g.parse_pages(FIX_PAGES)], [1, 2, 5])
        sysc = {c["name"]: c for c in pages["SYS"]["commands"]}
        self.assertEqual(sysc["CLOCK_DIAG"]["build"], "DSPI_CLOCK_DIAG")
        self.assertEqual(sysc["CLOCK_DIAG"]["doc"], "Clock diagnostics, see http://example.invalid/a,b")
        self.assertEqual(sysc["DEVICE_NAME"]["flags"], ["DEFERRED", "FLASH"])
        self.assertEqual(sysc["PAGE_LIST"]["get_len"], "var")
        self.assertIsNone(sysc["ENTER_BOOTLOADER"]["get_len"])
        self.assertEqual(sysc["COMMAND_INFO"]["id"], 0x0113)
        route = {c["name"]: c for c in pages["ROUTE"]["commands"]}
        self.assertEqual(route["OUTPUT_GAIN"]["arg"], "ROUTE_FIELD_GAIN")
        self.assertEqual(route["MASTER_VOLUME"]["arg"], "0")
        self.assertEqual(route["UPMIX_ENABLE"]["build"], "PICO_RP2350")
        self.assertIsNone(route["MASTER_VOLUME"]["build"])
        self.assertEqual(pages["ROUTE"]["reserved"], [{"cmd": 0x31, "note": "upmixer tuning"}])
        store = {c["name"]: c for c in pages["STORE"]["commands"]}
        self.assertEqual(store["ALL_PARAMS"]["flags"], ["USB_ONLY", "BULK"])

    def test_check_mode_round_trip(self):
        with tempfile.TemporaryDirectory() as td:
            js = os.path.join(td, "out.json")
            base = ["--pages", FIX_PAGES, "--core-h", FIX_CORE, "--json", js, "--docs",
                    os.path.join(td, "doc.md")]
            with open(base[-1], "w") as fh:
                fh.write("# Title\n\n<!-- VC:BEGIN PAGES -->\nold\n<!-- VC:END PAGES -->\n\nprose\n")
            self.assertEqual(run_cli(base + ["--check"])[0], 1)   # JSON missing, doc stale
            self.assertEqual(run_cli(base)[0], 0)
            rc, out, _ = run_cli(base + ["--check"])
            self.assertEqual(rc, 0, out)
            with open(js, "a") as fh:
                fh.write(" ")
            rc, out, _ = run_cli(base + ["--check"])
            self.assertEqual(rc, 1)
            self.assertIn("STALE", out)

    def test_list(self):
        rc, out, _ = run_cli(["--pages", FIX_PAGES, "--core-h", FIX_CORE, "--list"])
        self.assertEqual(rc, 0)
        self.assertIn("0x1F (reserved)", out)
        self.assertLess(out.index("0x1F (reserved)"), out.index("0x20 PLATFORM"))


class Docs(unittest.TestCase):
    def setUp(self):
        self.pages = g.parse_pages(FIX_PAGES)

    def test_rewrite_and_idempotent(self):
        src = ("intro\n<!-- VC:BEGIN ROUTE -->\nstale\n<!-- VC:END ROUTE -->\nmid\n"
               "<!-- VC:BEGIN PAGES -->\n<!-- VC:END PAGES -->\ntail\n")
        new, n = g.rewrite_doc_blocks(src, self.pages)
        self.assertEqual(n, 2)
        self.assertNotIn("stale", new)
        self.assertTrue(new.startswith("intro\n") and new.endswith("tail\n"))
        self.assertIn("| 0x20 | `OUTPUT_ENABLE` | G, S | output, 0xFF = all (SET) | 1 | 1 |  |", new)
        self.assertIn("Output mute \\| 0 or 1", new)
        self.assertIn("| 0x31 | _reserved_ |", new)
        self.assertIn("(build: `PICO_RP2350` only)", new)
        self.assertIn("| 0x05 | `ROUTE` | 2 | 6 | Matrix, outputs, volume |", new)
        self.assertEqual(g.rewrite_doc_blocks(new, self.pages), (new, 2))

    def test_no_markers_is_noop(self):
        self.assertEqual(g.rewrite_doc_blocks("plain\n", self.pages), ("plain\n", 0))

    def test_index_block(self):
        src = ("## 3. ROUTE (0x05)\n\n## 4. Not a page\n\n"
               "<!-- VC:BEGIN INDEX -->\nstale\n<!-- VC:END INDEX -->\n")
        new, n = g.rewrite_doc_blocks(src, self.pages)
        self.assertEqual(n, 1)
        self.assertNotIn("stale", new)
        rows = [r for r in new.splitlines() if r.startswith("| ") and not r.startswith("| Name")]
        names = [r.split(" | ")[0][2:] for r in rows]
        self.assertEqual(names, sorted(names))
        self.assertEqual(len(rows), sum(len(p["commands"]) for p in self.pages))
        self.assertIn("| UPMIX_ENABLE | [`ROUTE.UPMIX_ENABLE`](#3-route-0x05) | `0x0530` | RP2350 only |", rows)
        self.assertIn("| CLOCK_DIAG | `SYS.CLOCK_DIAG` | `0x0160` | `DSPI_CLOCK_DIAG` |", rows)
        self.assertIn("| PRESET_SAVE | `STORE.PRESET_SAVE` | `0x0210` |  |", rows)
        self.assertEqual(g.rewrite_doc_blocks(new, self.pages), (new, 1))

    def test_index_same_name_sorts_by_page(self):
        pages = [g.parse_def_text('VC_PAGE(BBB, 0x0B, 1, "b")\n' + cmd(name="STATUS"), "b.def"),
                 g.parse_def_text('VC_PAGE(AAA, 0x0C, 1, "a")\n' + cmd(name="STATUS"), "a.def"),
                 g.parse_def_text('VC_PAGE(CCC, 0x03, 1, "c")\n' + cmd(name="STATUS")
                                  + cmd(num="0x11", name="ALPHA"), "c.def")]
        rows = g.render_index_table(pages, {"AAA": "x"})[2:]
        self.assertEqual([r.split(" | ")[1] for r in rows],
                         ["`CCC.ALPHA`", "`CCC.STATUS`", "`BBB.STATUS`", "[`AAA.STATUS`](#x)"])

    def test_page_anchors(self):
        text = "## 6. SYS (0x01)\n### 6.1 SYS (0x02)\n## 18. DIAG (0xF0) \n## Notes\n"
        self.assertEqual(g.page_anchors(text), {"SYS": "6-sys-0x01", "DIAG": "18-diag-0xf0"})

    def test_marker_errors(self):
        bad = {
            "<!-- VC:BEGIN NOPE -->\n<!-- VC:END NOPE -->\n": "unknown page NOPE",
            "<!-- VC:BEGIN SYS -->\n<!-- VC:END ROUTE -->\n": "does not match",
            "<!-- VC:BEGIN SYS -->\nx\n": "never closed",
            "<!-- VC:END SYS -->\n": "without VC:BEGIN",
            "<!-- VC:BEGIN SYS -->\n<!-- VC:BEGIN ROUTE -->\n": "inside open block",
            ("<!-- VC:BEGIN SYS -->\n<!-- VC:END SYS -->\n"
             "<!-- VC:BEGIN SYS -->\n<!-- VC:END SYS -->\n"): "second block",
        }
        for text, want in bad.items():
            with self.subTest(want=want):
                with self.assertRaises(g.DefError) as cm:
                    g.rewrite_doc_blocks(text, self.pages, "doc.md")
                self.assertIn(want, str(cm.exception))


class BadInput(unittest.TestCase):
    def assertRejects(self, text, want, line=None):
        with self.assertRaises(g.DefError) as cm:
            g.parse_def_text(text, "x.def")
        msg = str(cm.exception)
        self.assertIn(want, msg)
        if line is not None:
            self.assertIn("x.def:%d:" % line, msg)

    def test_single_file_rules(self):
        cases = [
            ("// nothing\n", "no VC_PAGE", 1),
            (HDR + HDR, "second VC_PAGE", 2),
            (cmd() + HDR, "before VC_PAGE", 1),
            (HDR + cmd(num="0x0F"), "outside 16..255", 2),
            (HDR + cmd(num="0x100"), "outside 16..255", 2),
            (HDR + cmd() + cmd(name="BAR"), "duplicate command number", 3),
            (HDR + cmd() + cmd(num="0x11"), "duplicate command name FOO", 3),
            (HDR + cmd(num="0x12") + cmd(num="0x11", name="BAR"), "must ascend", 3),
            (HDR + cmd() + 'VC_RESERVED(0x10, "x")\n', "reserved number collides", 3),
            (HDR + cmd(addr="WIDGET"), "unknown ADDR_KIND 'WIDGET'", 2),
            (HDR + cmd(addr="ADDR(NONE)"), "nested '('", 2),
            (HDR + cmd(flags="VC_F_RP2350"), "unknown flag 'VC_F_RP2350'", 2),
            (HDR + cmd(flags="VC_F_FLASH | VC_F_FLASH"), "given twice", 2),
            (HDR + cmd(flags="VC_F_FLASH |"), "joined by |", 2),
            (HDR + cmd(get="VC_NONE", glen="4"), "GET_LEN must be 0", 2),
            (HDR + cmd(get="VC_NONE", glen="VC_LEN_VAR"), "GET_LEN must be 0", 2),
            (HDR + cmd(glen="0"), "GET_LEN is 0", 2),
            (HDR + cmd(glen="300"), "reply buffer", 2),
            (HDR + cmd(smax="65"), "SET_MAX 65 (0x41) outside 0..64", 2),
            (HDR + cmd(smin="4", smax="2"), "SET_MIN 4 > SET_MAX 2", 2),
            (HDR + cmd(set_="VC_NONE"), "SET bounds must be 0, 0", 2),
            (HDR + cmd(get="VC_NONE", set_="VC_NONE", glen="0", smin="0", smax="0"),
             "neither a GET nor a SET", 2),
            (HDR + cmd(name="foo"), "uppercase identifier", 2),
            (HDR + cmd(get="0x12"), "C identifier", 2),
            ('VC_PAGE(test, 0x0A, 1, "x")\n', "uppercase identifier", 1),
            ('VC_PAGE(PAGES, 0x0A, 1, "x")\n', "reserved for a docs block", 1),
            ('VC_PAGE(INDEX, 0x0A, 1, "x")\n', "reserved for a docs block", 1),
            ('VC_PAGE(TEST, 0, 1, "x")\n', "page number 0", 1),
            (HDR + cmd(doc='"say \\"hi\\""'), "no double quotes", 2),
            (HDR + cmd(doc='"say "hi" now"'), "no double quotes", 2),
            (HDR + cmd(doc='""'), "is empty", 2),
            (HDR + cmd(arg="256"), "ARG 256", 2),
            (HDR + cmd(num="010"), "invalid number", 2),
            (HDR + 'VC_CMD(0x10, FOO, a, b, NONE, 1, 1, 1, 0, "doc")\n', "takes 11", 2),
            (HDR + 'VC_RESERVED(0x10)\n', "takes 2", 2),
            (HDR + 'VC_THING(1)\n', "unknown macro VC_THING", 2),
            (HDR + cmd().rstrip("\n") + ";\n", "unexpected character ';'", 2),
            (HDR + "#if PICO_RP2350\n" + cmd() + "#else\n#endif\n", "#else is not allowed", 4),
            (HDR + "#if A\n#elif B\n#endif\n", "#elif is not allowed", 3),
            (HDR + "#ifndef A\n#endif\n", "#ifndef is not allowed", 2),
            (HDR + "#if A\n#if B\n#endif\n#endif\n", "nested conditional", 3),
            (HDR + "#if A\n" + cmd(), "never closed with #endif", 2),
            (HDR + "#endif\n", "#endif without #if", 2),
            (HDR + "#define X 1\n", "unsupported preprocessor", 2),
            ("#if A\n" + HDR + "#endif\n", "VC_PAGE inside a conditional", 2),
            (HDR + "VC_CMD(0x10, FOO,\n#if A\n", "preprocessor line inside a macro call", 3),
            (HDR + "VC_CMD(0x10, FOO\n", "never closed", 2),
        ]
        for text, want, line in cases:
            with self.subTest(want=want, text=text):
                self.assertRejects(text, want, line)

    def test_multiple_errors_reported(self):
        text = HDR + cmd(num="0x05") + cmd(num="0x20", addr="WIDGET")
        with self.assertRaises(g.DefError) as cm:
            g.parse_def_text(text, "x.def")
        self.assertEqual(len(cm.exception.messages), 2)

    def test_accepted_forms(self):
        text = (HDR + "/* block\ncomment */\n#if PICO_RP2350 // rp2350 only\n"
                + cmd(num="16", flags="VC_F_KEEPALIVE|VC_F_USB_ONLY", glen="VC_LEN_VAR", arg="0xFF")
                + "#endif\n" + cmd(num="0x11", name="BAR", get="VC_NONE", glen="0"))
        p = g.parse_def_text(text, "x.def")
        c0, c1 = p["commands"]
        self.assertEqual((c0["cmd"], c0["build"], c0["arg"]), (16, "PICO_RP2350", "255"))
        self.assertEqual(c0["flags"], ["USB_ONLY", "KEEPALIVE"])
        self.assertIsNone(c1["build"])
        self.assertIsNone(c1["get_len"])

    def _tree(self, td, files, core="#define VC_PROTOCOL_MAJOR 1\n#define VC_PROTOCOL_MINOR 0\n"):
        pages = os.path.join(td, "pages")
        os.mkdir(pages)
        for name, text in files.items():
            with open(os.path.join(pages, name), "w") as fh:
                fh.write(text)
        core_h = os.path.join(td, "vc_core.h")
        with open(core_h, "w") as fh:
            fh.write(core)
        return pages, core_h

    def test_cross_file_rules(self):
        cases = [
            ({"a.def": 'VC_PAGE(A, 0x0A, 1, "x")\n', "b.def": 'VC_PAGE(B, 0x0A, 1, "y")\n'},
             "duplicate page number 0x0A"),
            ({"a.def": 'VC_PAGE(A, 0x0A, 1, "x")\n', "a2.def": 'VC_PAGE(A, 0x0B, 1, "y")\n'},
             "duplicate mnemonic A"),
            ({"wrong.def": 'VC_PAGE(A, 0x0A, 1, "x")\n'}, "file name must be a.def"),
            ({}, "no .def files"),
        ]
        for files, want in cases:
            with self.subTest(want=want), tempfile.TemporaryDirectory() as td:
                pages, core_h = self._tree(td, files)
                with self.assertRaises(g.DefError) as cm:
                    g.build_registry(pages, core_h)
                self.assertIn(want, str(cm.exception))

    def test_protocol_header(self):
        for core, want in (("#define VC_PROTOCOL_MAJOR 1\n", "VC_PROTOCOL_MINOR"),
                           ("#define VC_PROTOCOL_MAJOR 1\n#define VC_PROTOCOL_MAJOR 2\n"
                            "#define VC_PROTOCOL_MINOR 0\n", "defined twice")):
            with self.subTest(want=want), tempfile.TemporaryDirectory() as td:
                pages, core_h = self._tree(td, {"a.def": 'VC_PAGE(A, 0x0A, 1, "x")\n'}, core)
                with self.assertRaises(g.DefError) as cm:
                    g.build_registry(pages, core_h)
                self.assertIn(want, str(cm.exception))

    def test_cli_failure_writes_nothing(self):
        with tempfile.TemporaryDirectory() as td:
            pages, core_h = self._tree(td, {"a.def": HDR})
            js = os.path.join(td, "out.json")
            rc, _, err = run_cli(["--pages", pages, "--core-h", core_h, "--json", js])
            self.assertEqual(rc, 1)
            self.assertIn("file name must be test.def", err)
            self.assertFalse(os.path.exists(js))


class LegacyMap(unittest.TestCase):
    """check_legacy_map against the fixture registry: a consistent map passes, each drift is named by key."""

    def setUp(self):
        self.reg = g.build_registry(FIX_PAGES, FIX_CORE)
        self.doc = {
            "pages": {"SYS": 1, "STORE": 2, "ROUTE": 5},
            "commands": {
                "SYS.PLATFORM": {"page": "SYS", "cmd": 0x20, "dir": "G", "addr": "NONE", "get_len": 7},
                "SYS.DEVICE_NAME": {"page": "SYS", "cmd": 0x24, "dir": "GS", "addr": "NONE", "get_len": 32,
                                    "set": [1, 32]},
                "SYS.ENTER_BOOTLOADER": {"page": "SYS", "cmd": 0x30, "dir": "S0", "addr": "NONE"},
                "SYS.CLOCK_DIAG": {"page": "SYS", "cmd": 0x60, "dir": "G", "addr": "NONE", "get_len": 48,
                                   "build": "DSPI_CLOCK_DIAG"},
                "STORE.ALL_PARAMS": {"page": "STORE", "cmd": 0x40, "dir": "GS", "addr": "NONE",
                                     "get_len": 6136, "set": [6136, 6136]},
                "ROUTE.OUTPUT_GAIN": {"page": "ROUTE", "cmd": 0x21, "dir": "GS", "addr": "OUTPUT_ALL",
                                      "get_len": 4, "set": [4, 4]},
            },
            "opcodes": {
                "GET_PLATFORM": {"code": 0x7F, "in": {"kind": "direct", "target": "SYS.PLATFORM"}},
                "SET_OUTPUT_GAIN": {"code": 0x74, "out": {"kind": "direct", "target": "ROUTE.OUTPUT_GAIN"}},
                "SPLIT": {"code": 0x01, "in": {"kind": "split", "cases": {
                    "0": {"kind": "direct", "target": "SYS.PLATFORM"},
                    "1": {"kind": "act_addr", "target": "SYS.ENTER_BOOTLOADER"}},
                    "default": {"kind": "custom", "fn": "x", "uses": ["ROUTE.OUTPUT_GAIN:S"]}}},
            },
        }

    def errs(self):
        return g.check_legacy_map(self.reg, self.doc)

    def test_consistent_map_passes(self):
        self.assertEqual(self.errs(), [])

    def test_catalog_drift_named_by_key(self):
        c = self.doc["commands"]
        c["SYS.PLATFORM"]["cmd"] = 0x21
        c["SYS.DEVICE_NAME"]["set"] = [1, 31]
        c["ROUTE.OUTPUT_GAIN"]["get_len"] = 8
        c["ROUTE.OUTPUT_GAIN"]["addr"] = "OUTPUT"
        c["SYS.CLOCK_DIAG"]["dir"] = "GS"
        c["SYS.CLOCK_DIAG"].pop("build")
        c["SYS.GONE"] = {"page": "SYS", "cmd": 0x70, "dir": "G", "addr": "NONE", "get_len": 1}
        self.doc["pages"]["ROUTE"] = 6
        errs = "\n".join(self.errs())
        for want in ("commands.SYS.PLATFORM: cmd 33", "commands.SYS.DEVICE_NAME: set [1, 31]",
                     "commands.ROUTE.OUTPUT_GAIN: get_len 8", "commands.ROUTE.OUTPUT_GAIN: addr OUTPUT",
                     "commands.SYS.CLOCK_DIAG: dir GS", "commands.SYS.CLOCK_DIAG: build None",
                     "commands.SYS.GONE: not in the registry", "pages.ROUTE: 0x06"):
            self.assertIn(want, errs)

    def test_bulk_set_bounds_not_compared(self):
        self.doc["commands"]["STORE.ALL_PARAMS"]["set"] = [1, 2]
        self.assertEqual(self.errs(), [])

    def test_adapter_targets_and_uses(self):
        ops = self.doc["opcodes"]
        ops["GET_PLATFORM"]["in"]["target"] = "SYS.NOPE"
        ops["SET_OUTPUT_GAIN"]["out"]["target"] = "SYS.PLATFORM"
        ops["SPLIT"]["in"]["default"]["uses"] = ["ROUTE.OUTPUT_GAIN", "ROUTE.MASTER_VOLUME:G"]
        errs = "\n".join(self.errs())
        for want in ("opcodes.GET_PLATFORM.in: SYS.NOPE not in the registry",
                     "opcodes.SET_OUTPUT_GAIN.out: SYS.PLATFORM has no SET",
                     "opcodes.SPLIT.in: malformed use 'ROUTE.OUTPUT_GAIN'",
                     "opcodes.SPLIT.in: ROUTE.MASTER_VOLUME not in the commands catalog"):
            self.assertIn(want, errs)

    def test_cli_check_reports_mismatch(self):
        with tempfile.TemporaryDirectory() as td:
            js, doc, lm = (os.path.join(td, n) for n in ("r.json", "d.md", "lm.json"))
            base = ["--pages", FIX_PAGES, "--core-h", FIX_CORE, "--json", js, "--docs", doc,
                    "--legacy-map", lm]
            with open(doc, "w", encoding="utf-8") as fh:
                fh.write("x\n")
            with open(lm, "w", encoding="utf-8") as fh:
                json.dump(self.doc, fh)
            self.assertEqual(run_cli(base)[0], 0)
            rc, out, _ = run_cli(base + ["--check"])
            self.assertEqual(rc, 0, out)
            self.doc["commands"]["SYS.PLATFORM"]["get_len"] = 9
            with open(lm, "w", encoding="utf-8") as fh:
                json.dump(self.doc, fh)
            rc, out, _ = run_cli(base + ["--check"])
            self.assertEqual(rc, 1)
            self.assertIn("commands.SYS.PLATFORM: get_len 9", out)


GUIDE_DOC = """# Guide

### `GET_MATRIX_ROUTE` recipe
### `GET_STATUS` recipe
### Retired opcodes

<!-- VC:BEGIN LEGACY_MAP -->
<!-- VC:END LEGACY_MAP -->
<!-- VC:BEGIN LEGACY_WASR -->
<!-- VC:END LEGACY_WASR -->
<!-- VC:BEGIN LEGACY_STATUS -->
<!-- VC:END LEGACY_STATUS -->
<!-- VC:BEGIN GET_LENGTHS -->
<!-- VC:END GET_LENGTHS -->
<!-- VC:BEGIN RETIRED -->
<!-- VC:END RETIRED -->
"""


def guide_legacy():
    """A legacy map over the fixture registry with one adapter of every kind."""
    return {
        "vc_results": {"VC_OK": 0, "VC_ERR_UNKNOWN": 16, "VC_ERR_ADDRESS": 19, "VC_ERR_VALUE": 20,
                       "VC_ERR_STATE": 24, "VC_ERR_PIN_INVALID": 25, "VC_ERR_PIN_IN_USE": 26},
        "status_families": {
            "pin": {"success": 0, "from_vc": {"VC_ERR_PIN_INVALID": 1, "VC_ERR_PIN_IN_USE": 2,
                                              "VC_ERR_ADDRESS": 3, "VC_ERR_STATE": 4, "VC_ERR_VALUE": 5}},
            "preset": {"success": 0, "from_vc": {"VC_ERR_ADDRESS": 1, "VC_ERR_STATE": 2}},
            "ack1": {"success": 1, "from_vc": {}},
        },
        "opcodes": {
            "GET_PLATFORM": {"code": 0x7F, "old_dir": "R", "in": {"kind": "direct", "target": "SYS.PLATFORM"}},
            "SET_OUTPUT_GAIN": {"code": 0x74, "old_dir": "W",
                                "out": {"kind": "direct", "target": "ROUTE.OUTPUT_GAIN"}},
            "GET_OUTPUT_GAIN": {"code": 0x75, "old_dir": "R",
                                "in": {"kind": "direct", "target": "ROUTE.OUTPUT_GAIN"}},
            "MASTER": {"code": 0x03, "old_dir": "W+R",
                       "out": {"kind": "direct", "target": "ROUTE.MASTER_VOLUME"},
                       "in": {"kind": "split", "selector": {"shift": 0, "mask": 255}, "cases": {
                           "1": {"kind": "direct", "target": "SYS.PLATFORM", "addr": {"const": 0}},
                           "0": {"kind": "direct", "target": "ROUTE.MASTER_VOLUME"}},
                           "default": None}},
            "SET_OUTPUT_TYPE": {"code": 0xC0, "old_dir": "WR", "in": {
                "kind": "act_packed", "target": "ROUTE.OUTPUT_ENABLE", "addr": {"shift": 0, "mask": 255},
                "value": {"shift": 8, "mask": 255}, "encode": "u8", "status": "pin",
                "status_overrides": {"VC_ERR_VALUE": 1}}},
            "PRESET_SAVE": {"code": 0x90, "old_dir": "WR", "in": {
                "kind": "act_addr", "target": "STORE.PRESET_SAVE", "status": "preset",
                "addr": {"shift": 0, "mask": 255}}},
            "BOOT": {"code": 0xF0, "old_dir": "WR", "in": {"kind": "split", "selector": {"shift": 0, "mask": 65535},
                     "cases": {"65535": {"kind": "act_addr", "target": "SYS.ENTER_BOOTLOADER", "status": "ack1",
                                         "addr": {"const": 0}}},
                     "default": {"kind": "act_value", "target": "ROUTE.OUTPUT_MUTE", "status": "pin",
                                 "value": {"shift": 0, "mask": 65535}, "encode": "bool"}}},
            "GET_MATRIX_ROUTE": {"code": 0x71, "old_dir": "R", "in": {
                "kind": "custom", "fn": "get_matrix_route", "uses": ["ROUTE.MATRIX_ROUTE:G"]}},
            "SET_PREAMP": {"code": 0x44, "old_dir": "W", "out": {
                "kind": "custom", "fn": "x", "uses": ["ROUTE.MASTER_VOLUME:S"], "retired": True}},
            "GET_STATUS": {"code": 0x50, "old_dir": "R", "in": {
                "kind": "custom", "fn": "get_status", "uses": ["SYS.PLATFORM:G"],
                "fields": {"3": ["SYS.PLATFORM", "u8", 3], "16": ["SYS.CLOCK_DIAG", "i16", 10]},
                "peaks": {"0": [0, 1]}, "peak_cpu": {"2": [4, "SYS.PLATFORM", 5, 6]},
                "levels_raw": [9], "zero": [7], "zero_if_absent": ["SYS.CLOCK_DIAG"]}},
        },
    }


class GuideBlocks(unittest.TestCase):
    """The migration guide blocks rendered from a legacy map over the fixture registry."""

    def setUp(self):
        self.pages = g.parse_pages(FIX_PAGES)
        self.legacy = guide_legacy()
        patches = [mock.patch.object(g, "LEVELS_CMD", "SYS.PAGE_LIST"),
                   mock.patch.object(g, "RETIRED_TEXT", {"SET_PREAMP": "Use `ROUTE.MASTER_VOLUME`"}),
                   mock.patch.object(g, "STATUS_SELECTORS", {0: "p01", 2: "p4", 3: "out", 7: "zero",
                                                             9: "levels", 16: "temp"})]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)

    def render(self, text=GUIDE_DOC):
        new, n = g.rewrite_doc_blocks(text, self.pages, "guide.md", self.legacy)
        self.assertEqual(n, 5)
        return new

    def block(self, new, name):
        return new.split("<!-- VC:BEGIN %s -->" % name)[1].split("<!-- VC:END %s -->" % name)[0]

    def rows(self, new, name):
        return [r for r in self.block(new, name).splitlines() if r.startswith("| 0x") or r.startswith("| `")
                or r.startswith("| any") or r[2:3].isdigit()]

    def test_legacy_map_order_kinds_and_idempotent(self):
        new = self.render()
        rows = self.rows(new, "LEGACY_MAP")
        codes = [int(r.split(" | ")[0][2:], 16) for r in rows]
        self.assertEqual(codes, sorted(codes))
        # W+R: the OUT row comes first, then one IN row per selector and the refused remainder.
        master = [r for r in rows if "`MASTER`" in r]
        self.assertEqual(len(master), 4)
        self.assertIn("| W | any | same | `ROUTE.MASTER_VOLUME` | `0x0550` | S | `0` | unchanged | 4 |", master[0])
        self.assertIn("| R | old & 0xFF = 0 | split | `ROUTE.MASTER_VOLUME` | `0x0550` | G | `0` | - | 4 |",
                      master[1])
        self.assertIn("| R | old & 0xFF = 1 | split | `SYS.PLATFORM` | `0x0120` | G | `0` | - | 7 |", master[2])
        self.assertIn("| R | any other | split | none | - | - | - | 1.1.6 STALLed it | - |", master[3])
        gain = [r for r in rows if "OUTPUT_GAIN`" in r]
        self.assertTrue(all("| folded |" in r and "`old` |" in r for r in gain), gain)
        self.assertIn("| W-as-R | any | action | `ROUTE.OUTPUT_ENABLE` | `0x0520` | S | `old & 0xFF` "
                      "| u8 `old >> 8` | 1 |", new)
        self.assertEqual(g.rewrite_doc_blocks(new, self.pages, "guide.md", self.legacy), (new, 5))

    def test_custom_and_retired_rows_link_their_sections(self):
        new = self.render()
        self.assertIn("| 0x71 | [`GET_MATRIX_ROUTE`](#get_matrix_route-recipe) | R | any | custom "
                      "| `ROUTE.MATRIX_ROUTE` G | `0x0510` | - | see recipe | [see recipe](#get_matrix_route-recipe) |",
                      new)
        self.assertIn("| 0x44 | `SET_PREAMP` | W | any | retired | none | - | - | - "
                      "| [replacement](#retired-opcodes) | - |", new)
        for heading, want in (("### `GET_MATRIX_ROUTE` recipe\n", "`GET_MATRIX_ROUTE`"),
                              ("### Retired opcodes\n", "Retired opcodes")):
            with self.subTest(want=want), self.assertRaises(g.DefError) as cm:
                g.rewrite_doc_blocks(GUIDE_DOC.replace(heading, ""), self.pages, "guide.md", self.legacy)
            self.assertIn("no heading contains %s" % want, str(cm.exception))

    def test_links_fall_back_to_the_full_guide(self):
        short = GUIDE_DOC.replace("### `GET_MATRIX_ROUTE` recipe\n", "").replace("### Retired opcodes\n", "")
        new, _n = g.rewrite_doc_blocks(short, self.pages, "short.md", self.legacy, ("full.md", GUIDE_DOC))
        self.assertIn("[see recipe](full.md#get_matrix_route-recipe)", new)
        self.assertIn("[replacement](full.md#retired-opcodes)", new)
        # A heading in the doc itself wins over the full guide.
        self.assertIn("[`GET_STATUS`](#get_status-recipe)", new)

    def test_write_as_read_rows_and_status_override(self):
        rows = self.rows(self.render(), "LEGACY_WASR")
        self.assertEqual([r.split(" | ")[1] for r in rows],
                         ["`PRESET_SAVE`", "`SET_OUTPUT_TYPE`", "`BOOT`", "`BOOT`"])
        # The per-entry override moves VC_ERR_VALUE from the family's 0x05 to 0x01.
        self.assertIn("0x00 for OK. 0x01 for `VALUE` or `PIN_INVALID`. 0x02 for `PIN_IN_USE`. 0x03 for `ADDRESS`. "
                      "0x04 for `STATE` |", rows[1])
        self.assertIn("| output, 0xFF = all (SET) in `old & 0xFF`, value in `old >> 8` | 1 byte `PIN_CONFIG_*` |", rows[1])
        self.assertIn("| preset slot in `old & 0xFF` | 1 byte `PRESET_*` | `STORE.PRESET_SAVE` `0x0210` "
                      "| `old & 0xFF` | none (zero-length) | 0x00 for OK. 0x01 for `ADDRESS`. 0x02 for `STATE` |", rows[0])
        self.assertIn("| 0xFFFF | 1 byte `0x01` | `SYS.ENTER_BOOTLOADER` `0x0130` | `0` |", rows[2])
        self.assertIn("| any other | 1 byte `PIN_CONFIG_*` | `ROUTE.OUTPUT_MUTE` `0x0522` | `0` "
                      "| 1 byte: u8 `old != 0` (1 or 0) |", rows[3])

    def test_status_block(self):
        rows = self.rows(self.render(), "LEGACY_STATUS")
        self.assertEqual([r.split(" | ")[0][2:] for r in rows], ["0", "2", "3", "7", "9", "16", "any other"])
        self.assertIn("| 0 | p01 | `SYS.PAGE_LIST` | 0, 2 | u16, u16 | bits 0-15 = u16 at 0, bits 16-31 = u16 at 2 |",
                      rows[0])
        self.assertIn("| 16 | temp | `SYS.CLOCK_DIAG` (DSPI_CLOCK_DIAG), read 0 if `VC_ERR_UNKNOWN` | 10 | i16 "
                      "| sign-extend |", rows[5])
        with mock.patch.object(g, "STATUS_SELECTORS", {0: "p01"}), self.assertRaises(g.DefError) as cm:
            self.render()
        self.assertIn("differ from STATUS_SELECTORS", str(cm.exception))

    def test_get_lengths_block(self):
        rows = self.rows(self.render(), "GET_LENGTHS")
        names = [r.split(" | ")[0][2:] for r in rows]
        want = ["`%s.%s`" % (p["mnemonic"], c["name"]) for p in self.pages for c in p["commands"] if c["get"]]
        self.assertEqual(names, want)
        self.assertIn("| `SYS.PAGE_LIST` | 0x01 | 0x11 | none | var |  | 2 x pages", rows[1])
        self.assertIn("| `SYS.CLOCK_DIAG` | 0x01 | 0x60 | none | 48 | `DSPI_CLOCK_DIAG` |  |", rows)
        self.assertIn("| `STORE.ALL_PARAMS` | 0x02 | 0x40 | none | 6136 | USB only | Bulk buffer", "\n".join(rows))
        rules = dict(g.VAR_RULES)
        del rules["SYS.PAGE_LIST"]
        with mock.patch.object(g, "VAR_RULES", rules), self.assertRaises(g.DefError) as cm:
            self.render()
        self.assertIn("SYS.PAGE_LIST has a variable reply and no rule", str(cm.exception))

    def test_retired_block(self):
        rows = self.rows(self.render(), "RETIRED")
        self.assertEqual(rows, [
            "| 0x44 | `SET_PREAMP` | W | Use `ROUTE.MASTER_VOLUME` |",
            "| 0x50 | `GET_STATUS` `wValue` 0 | R | `SYS.PAGE_LIST`, u16 at offsets 0 and 2 |",
            "| 0x50 | `GET_STATUS` `wValue` 2 | R | `SYS.PAGE_LIST`, u16 at offset 8, then `SYS.PLATFORM` bytes 5 and 6 |",
            "| 0x50 | `GET_STATUS` `wValue` 7 | R | None. It always read 0 |"])
        for text, want in (({}, "retired opcode SET_PREAMP has no entry"),
                           ({"SET_PREAMP": "x", "GONE": "y"}, "RETIRED_TEXT names GONE")):
            with self.subTest(want=want), mock.patch.object(g, "RETIRED_TEXT", text), \
                    self.assertRaises(g.DefError) as cm:
                self.render()
            self.assertIn(want, str(cm.exception))

    def test_guide_block_needs_legacy_map(self):
        with self.assertRaises(g.DefError) as cm:
            g.rewrite_doc_blocks(GUIDE_DOC, self.pages, "guide.md")
        self.assertIn("block LEGACY_MAP needs the legacy map", str(cm.exception))

    def test_unknown_target_reported(self):
        self.legacy["opcodes"]["GET_PLATFORM"]["in"]["target"] = "SYS.NOPE"
        with self.assertRaises(g.DefError) as cm:
            self.render()
        self.assertIn("names SYS.NOPE, which is not in the registry", str(cm.exception))

    def test_block_names_reserved_from_page_mnemonics(self):
        for name in g.GUIDE_MARKERS:
            with self.subTest(name=name), self.assertRaises(g.DefError) as cm:
                g.parse_def_text('VC_PAGE(%s, 0x0A, 1, "x")\n' % name, "x.def")
            self.assertIn("reserved for a docs block", str(cm.exception))

    def test_cli_check_flags_stale_guide_block(self):
        with tempfile.TemporaryDirectory() as td:
            js, doc, lm = (os.path.join(td, n) for n in ("r.json", "guide.md", "lm.json"))
            base = ["--pages", FIX_PAGES, "--core-h", FIX_CORE, "--json", js, "--docs", doc,
                    "--legacy-map", lm]
            with open(doc, "w", encoding="utf-8") as fh:
                fh.write(GUIDE_DOC)
            # --check also validates the map's commands catalog, so give it one from the fixture registry.
            self.legacy["pages"] = {p["mnemonic"]: p["page"] for p in self.pages}
            self.legacy["commands"] = {
                "%s.%s" % (p["mnemonic"], c["name"]): dict(
                    {"page": p["mnemonic"], "cmd": c["cmd"], "addr": c["addr"],
                     "dir": ("G" if c["get"] else "") + ("S" if c["set"] else ""), "get_len": c["get_len"],
                     "set": [c["set_min"], c["set_max"]], "build": c["build"]})
                for p in self.pages for c in p["commands"]}
            with open(lm, "w", encoding="utf-8") as fh:
                json.dump(self.legacy, fh)
            self.assertEqual(run_cli(base)[0], 0)
            rc, out, _ = run_cli(base + ["--check"])
            self.assertEqual(rc, 0, out)
            with open(doc, encoding="utf-8") as fh:
                text = fh.read()
            with open(doc, "w", encoding="utf-8") as fh:
                fh.write(text.replace("| 0x7F |", "| 0x7E |", 1))
            rc, out, _ = run_cli(base + ["--check"])
            self.assertEqual(rc, 1)
            self.assertIn("STALE", out)
            self.assertEqual(run_cli(base)[0], 0)
            self.legacy["opcodes"]["GET_PLATFORM"]["code"] = 0x7E   # map edit, doc untouched
            with open(lm, "w", encoding="utf-8") as fh:
                json.dump(self.legacy, fh)
            rc, out, _ = run_cli(base + ["--check"])
            self.assertEqual(rc, 1)
            self.assertIn("guide.md", out)


if __name__ == "__main__":
    unittest.main()
