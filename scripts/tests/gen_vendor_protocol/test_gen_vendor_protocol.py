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
            ('VC_PAGE(PAGES, 0x0A, 1, "x")\n', "reserved for the docs", 1),
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


if __name__ == "__main__":
    unittest.main()
