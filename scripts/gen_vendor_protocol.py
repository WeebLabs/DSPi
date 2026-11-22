#!/usr/bin/env python3
"""Generate the vendor command registry JSON and doc tables from vendor/pages/*.def.

The .def grammar and the JSON schema are fixed by the vendor command overhaul plan
(Documentation/Features/vendor_command_overhaul_plan.md, sections 8.3 and 9).
Standard library only. Run with --help for usage.
"""

import argparse
import difflib
import json
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PAGES = os.path.join(REPO, "firmware", "DSPi", "vendor", "pages")
DEFAULT_CORE_H = os.path.join(REPO, "firmware", "DSPi", "vendor", "vc_core.h")
DEFAULT_JSON = os.path.join(REPO, "Documentation", "protocol", "dspi_commands.json")
DEFAULT_GUIDE = os.path.join(REPO, "Documentation", "Features", "vendor_protocol_migration_guide.md")
DEFAULT_CLIENT_UPDATE = os.path.join(REPO, "Documentation", "Features", "vendor_protocol_client_update.md")
DEFAULT_DOCS = [os.path.join(REPO, "Documentation", "commands.md"), DEFAULT_GUIDE, DEFAULT_CLIENT_UPDATE]
DEFAULT_LEGACY_MAP = os.path.join(REPO, "Documentation", "protocol", "legacy_map.json")

ADDR_KINDS = (
    "NONE", "CHANNEL", "INPUT", "OUTPUT", "OUTPUT_ALL", "SLOT", "PIN_OUTPUT",
    "PRESET", "CH_BAND", "IN_OUT", "SOURCE", "SPDIF_RX", "I2S_PAIR", "ROLE",
    "CS_SLOT", "CS_SUB", "CS_GROUP", "CS_MACRO", "CS_MACRO_STEP", "CS_PAGE",
    "CS_NOUN", "PAGE", "PAGE_CMD", "OFFSET", "INDEX", "CS_SLOT_TYPE",
)
# Canonical order; JSON lists flags in this order whatever the source order was.
FLAGS = ("USB_ONLY", "SET_USB_ONLY", "BULK", "DEFERRED", "FLASH", "RESTART", "KEEPALIVE")

ADDR_DOC = {
    "NONE": "none",
    "CHANNEL": "channel",
    "INPUT": "input channel",
    "OUTPUT": "output",
    "OUTPUT_ALL": "output, 0xFF = all (SET)",
    "SLOT": "output slot",
    "PIN_OUTPUT": "pin output",
    "PRESET": "preset slot",
    "CH_BAND": "`(channel << 8) \\| band`",
    "IN_OUT": "`(input << 8) \\| output`",
    "SOURCE": "source ID",
    "SPDIF_RX": "S/PDIF input",
    "I2S_PAIR": "I2S pair",
    "ROLE": "role 0-1",
    "CS_SLOT": "CS slot",
    "CS_SUB": "CS sub-slot",
    "CS_GROUP": "CS group",
    "CS_MACRO": "CS macro",
    "CS_MACRO_STEP": "CS macro step",
    "CS_PAGE": "CS page",
    "CS_NOUN": "CS noun",
    "PAGE": "page",
    "PAGE_CMD": "`(page << 8) \\| cmd`",
    "OFFSET": "byte offset",
    "INDEX": "index",
    "CS_SLOT_TYPE": "`(CS type << 8) \\| slot`",
}

MAX_SET_PAYLOAD = 64
MAX_GET_REPLY = 256   # vc_core.c reply buffer; larger replies must be VC_F_BULK
DOCS_PAGES_MARKER = "PAGES"
DOCS_INDEX_MARKER = "INDEX"
# Migration guide blocks, rendered from legacy_map.json plus the registry.
GUIDE_MARKERS = ("LEGACY_MAP", "LEGACY_WASR", "LEGACY_STATUS", "GET_LENGTHS", "RETIRED")
DOCS_MARKERS = (DOCS_PAGES_MARKER, DOCS_INDEX_MARKER) + GUIDE_MARKERS

UPPER_IDENT = re.compile(r"[A-Z][A-Z0-9_]*\Z")
C_IDENT = re.compile(r"[A-Za-z_][A-Za-z0-9_]*\Z")
IDENT_TOK = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
NUM_TOK = re.compile(r"[0-9][A-Za-z0-9_]*")
NUMBER = re.compile(r"(0[xX][0-9A-Fa-f]+|0|[1-9][0-9]*)\Z")
PROTO_LINE = re.compile(r"\s*#\s*define\s+(VC_PROTOCOL_MAJOR|VC_PROTOCOL_MINOR)\s+(\d+)\s*(//.*)?\Z")
BEGIN_RE = re.compile(r"\s*<!--\s*VC:BEGIN\s+(\S+)\s*-->\s*\Z")
END_RE = re.compile(r"\s*<!--\s*VC:END(?:\s+(\S+))?\s*-->\s*\Z")


class DefError(Exception):
    """Carries every validation message found, each already prefixed with file:line."""

    def __init__(self, messages):
        Exception.__init__(self, "\n".join(messages))
        self.messages = list(messages)


# ---------------------------------------------------------------------------
# Tokenizer
# ---------------------------------------------------------------------------

def tokenize(text, path):
    """Return (kind, value, line) tuples. kind is DIR, IDENT, NUM, STR or a punctuator."""
    toks = []
    i, line, n = 0, 1, len(text)
    line_start = True
    while i < n:
        c = text[i]
        if c == "\n":
            line += 1
            i += 1
            line_start = True
            continue
        if c in " \t\r\f\v":
            i += 1
            continue
        if text.startswith("//", i):
            j = text.find("\n", i)
            i = n if j < 0 else j
            continue
        if text.startswith("/*", i):
            j = text.find("*/", i + 2)
            if j < 0:
                raise DefError(["%s:%d: unterminated /* comment" % (path, line)])
            line += text.count("\n", i, j)
            i = j + 2
            continue
        if c == "#" and line_start:
            j = text.find("\n", i)
            j = n if j < 0 else j
            body = text[i:j]
            k = body.find("//")
            if k >= 0:
                body = body[:k]
            toks.append(("DIR", body.strip(), line))
            i = j
            continue
        line_start = False
        if c == '"':
            j = i + 1
            while j < n and text[j] not in '"\n\\':
                j += 1
            if j >= n or text[j] == "\n":
                raise DefError(["%s:%d: unterminated string" % (path, line)])
            if text[j] == "\\":
                raise DefError(["%s:%d: backslash in string; doc strings take no escapes and no "
                                "double quotes" % (path, line)])
            toks.append(("STR", text[i + 1:j], line))
            i = j + 1
            continue
        m = IDENT_TOK.match(text, i)
        if m:
            toks.append(("IDENT", m.group(0), line))
            i = m.end()
            continue
        m = NUM_TOK.match(text, i)
        if m:
            toks.append(("NUM", m.group(0), line))
            i = m.end()
            continue
        if c in "(),|":
            toks.append((c, c, line))
            i += 1
            continue
        raise DefError(["%s:%d: unexpected character %r" % (path, line, c)])
    return toks


# ---------------------------------------------------------------------------
# Parser and per-file validation
# ---------------------------------------------------------------------------

def _int(tok, path):
    if tok[0] != "NUM" or not NUMBER.match(tok[1]):
        raise ValueError("%s:%d: invalid number %r (use decimal or 0x hex)" % (path, tok[2], tok[1]))
    return int(tok[1], 0)


def _one(arg, what, path, line):
    if len(arg) != 1:
        text = " ".join(t[1] for t in arg)
        raise ValueError("%s:%d: %s must be a single token, got %r" % (path, line, what, text))
    return arg[0]


def _upper_ident(arg, what, path, line):
    tok = _one(arg, what, path, line)
    if tok[0] != "IDENT" or not UPPER_IDENT.match(tok[1]):
        raise ValueError("%s:%d: %s %r must be an uppercase identifier" % (path, tok[2], what, tok[1]))
    return tok[1]


def _string(arg, what, path, line):
    if len(arg) != 1 or arg[0][0] != "STR":
        raise ValueError("%s:%d: %s must be one string literal with no double quotes inside"
                         % (path, line, what))
    if not arg[0][1].strip():
        raise ValueError("%s:%d: %s is empty" % (path, line, what))
    return arg[0][1]


def _ranged(arg, what, lo, hi, path, line):
    v = _int(_one(arg, what, path, line), path)
    if not lo <= v <= hi:
        raise ValueError("%s:%d: %s %d (0x%X) outside %d..%d" % (path, line, what, v, v, lo, hi))
    return v


def _handler(arg, what, path, line):
    tok = _one(arg, what, path, line)
    if tok[0] != "IDENT" or not C_IDENT.match(tok[1]):
        raise ValueError("%s:%d: %s %r must be a C identifier or VC_NONE" % (path, tok[2], what, tok[1]))
    return None if tok[1] == "VC_NONE" else tok[1]


def _flags(arg, path, line):
    if len(arg) == 1 and arg[0][0] == "NUM":
        if _int(arg[0], path) != 0:
            raise ValueError("%s:%d: FLAGS must be 0 or VC_F_* names joined by |" % (path, line))
        return []
    names = []
    for idx, tok in enumerate(arg):
        if idx % 2:
            if tok[0] != "|":
                raise ValueError("%s:%d: FLAGS must be 0 or VC_F_* names joined by |" % (path, tok[2]))
            continue
        if tok[0] != "IDENT" or not tok[1].startswith("VC_F_") or tok[1][5:] not in FLAGS:
            raise ValueError("%s:%d: unknown flag %r (known: %s)"
                             % (path, tok[2], tok[1], ", ".join("VC_F_" + f for f in FLAGS)))
        if tok[1][5:] in names:
            raise ValueError("%s:%d: flag %s given twice" % (path, tok[2], tok[1]))
        names.append(tok[1][5:])
    if not names or len(arg) % 2 == 0:
        raise ValueError("%s:%d: FLAGS must be 0 or VC_F_* names joined by |" % (path, line))
    return [f for f in FLAGS if f in names]


def _parse_cmd(args, page_no, path, line):
    cmd = _ranged(args[0], "command number", 0x10, 0xFF, path, line)
    name = _upper_ident(args[1], "command name", path, line)
    get_fn = _handler(args[2], "get_fn", path, line)
    set_fn = _handler(args[3], "set_fn", path, line)
    addr = _one(args[4], "ADDR_KIND", path, line)
    if addr[0] != "IDENT" or addr[1] not in ADDR_KINDS:
        raise ValueError("%s:%d: unknown ADDR_KIND %r" % (path, addr[2], addr[1]))
    gl = _one(args[5], "GET_LEN", path, line)
    get_len = "var" if gl[0] == "IDENT" and gl[1] == "VC_LEN_VAR" else _int(gl, path)
    set_min = _ranged(args[6], "SET_MIN", 0, MAX_SET_PAYLOAD, path, line)
    set_max = _ranged(args[7], "SET_MAX", 0, MAX_SET_PAYLOAD, path, line)
    flags = _flags(args[8], path, line)
    at = _one(args[9], "ARG", path, line)
    if at[0] == "IDENT":
        arg = at[1]
    else:
        arg = str(_ranged(args[9], "ARG", 0, 255, path, line))
    doc = _string(args[10], "doc string", path, line)

    errs = []
    where = "%s:%d: %s" % (path, line, name)
    if get_fn is None and set_fn is None:
        errs.append("%s has neither a GET nor a SET handler (retire it with VC_RESERVED)" % where)
    if get_fn is None and get_len != 0:
        errs.append("%s: GET_LEN must be 0 when get_fn is VC_NONE" % where)
    if get_fn is not None and get_len == 0:
        errs.append("%s: has a GET handler but GET_LEN is 0 (use a size or VC_LEN_VAR)" % where)
    if isinstance(get_len, int) and get_len > 0xFFFE:
        errs.append("%s: GET_LEN %d does not fit COMMAND_INFO's u16" % (where, get_len))
    elif isinstance(get_len, int) and get_len > MAX_GET_REPLY and "BULK" not in flags:
        errs.append("%s: GET_LEN %d exceeds the %d-byte reply buffer without VC_F_BULK"
                    % (where, get_len, MAX_GET_REPLY))
    if set_min > set_max:
        errs.append("%s: SET_MIN %d > SET_MAX %d" % (where, set_min, set_max))
    if set_fn is None and (set_min or set_max):
        errs.append("%s: SET bounds must be 0, 0 when set_fn is VC_NONE" % where)
    if errs:
        raise ValueError("\n".join(errs))
    return {
        "cmd": cmd, "name": name, "id": (page_no << 8) | cmd,
        "get": get_fn is not None, "set": set_fn is not None, "addr": addr[1],
        "get_len": None if get_fn is None else get_len,
        "set_min": set_min, "set_max": set_max, "flags": flags,
        "arg": arg, "build": None, "doc": doc,
    }


def _split_call(toks, k, path):
    """Return (args, next_index) for the macro call whose '(' is at toks[k]."""
    args, cur = [], []
    open_line = toks[k][2]
    k += 1
    while k < len(toks):
        t = toks[k]
        if t[0] == ")":
            args.append(cur)
            return args, k + 1
        if t[0] == ",":
            args.append(cur)
            cur = []
        elif t[0] == "(":
            raise DefError(["%s:%d: nested '(' is not part of the grammar; write bare tokens "
                            "(e.g. CHANNEL, not ADDR(CHANNEL))" % (path, t[2])])
        elif t[0] == "DIR":
            raise DefError(["%s:%d: preprocessor line inside a macro call opened at line %d"
                            % (path, t[2], open_line)])
        else:
            cur.append(t)
        k += 1
    raise DefError(["%s:%d: macro call never closed" % (path, open_line)])


ARITY = {"VC_PAGE": 4, "VC_CMD": 11, "VC_RESERVED": 2}


def parse_def_text(text, path):
    """Parse one .def file. Returns a page dict in registry form plus 'file'/'line' keys."""
    toks = tokenize(text, path)
    errs = []
    page = None
    cond = None
    entries = []      # (number, kind, line) in file order, for the ordering check
    commands, reserved = [], []
    k = 0
    while k < len(toks):
        t = toks[k]
        if t[0] == "DIR":
            k += 1
            m = re.match(r"#\s*([A-Za-z]+)\s*(.*)\Z", t[1])
            word, rest = (m.group(1), m.group(2).strip()) if m else ("", "")
            if word in ("if", "ifdef"):
                if cond:
                    errs.append("%s:%d: nested conditional (outer #if at line %d); nesting is not "
                                "allowed" % (path, t[2], cond[1]))
                    continue
                if not rest or (word == "ifdef" and not C_IDENT.match(rest)):
                    errs.append("%s:%d: #%s needs a condition (#ifdef takes one identifier)"
                                % (path, t[2], word))
                    continue
                cond = (rest, t[2])
            elif word == "endif":
                if not cond:
                    errs.append("%s:%d: #endif without #if" % (path, t[2]))
                cond = None
            elif word in ("else", "elif", "elifdef", "elifndef", "ifndef"):
                errs.append("%s:%d: #%s is not allowed; use separate #if blocks"
                            % (path, t[2], word))
            else:
                errs.append("%s:%d: unsupported preprocessor line %r" % (path, t[2], t[1]))
            continue
        if t[0] != "IDENT" or k + 1 >= len(toks) or toks[k + 1][0] != "(":
            errs.append("%s:%d: expected VC_PAGE, VC_CMD or VC_RESERVED, got %r" % (path, t[2], t[1]))
            break
        if t[1] not in ARITY:
            errs.append("%s:%d: unknown macro %s" % (path, t[2], t[1]))
        args, k = _split_call(toks, k + 1, path)
        if t[1] not in ARITY:
            continue
        line = t[2]
        if len(args) != ARITY[t[1]] or any(not a for a in args):
            errs.append("%s:%d: %s takes %d non-empty arguments, got %d"
                        % (path, line, t[1], ARITY[t[1]], len(args)))
            continue
        try:
            if t[1] == "VC_PAGE":
                if page is not None:
                    errs.append("%s:%d: second VC_PAGE (first at line %d)" % (path, line, page["line"]))
                    continue
                if cond:
                    errs.append("%s:%d: VC_PAGE inside a conditional" % (path, line))
                mnem = _upper_ident(args[0], "page mnemonic", path, line)
                if mnem in DOCS_MARKERS:
                    raise ValueError("%s:%d: mnemonic %s is reserved for a docs block"
                                     % (path, line, mnem))
                page = {
                    "page": _ranged(args[1], "page number", 0x01, 0xFF, path, line),
                    "mnemonic": mnem,
                    "revision": _ranged(args[2], "page revision", 1, 255, path, line),
                    "doc": _string(args[3], "page description", path, line),
                    "commands": commands, "reserved": reserved,
                    "file": path, "line": line,
                }
            elif page is None:
                errs.append("%s:%d: %s before VC_PAGE" % (path, line, t[1]))
            elif t[1] == "VC_CMD":
                c = _parse_cmd(args, page["page"], path, line)
                c["build"] = cond[0] if cond else None
                commands.append(c)
                entries.append((c["cmd"], c["name"], line))
            else:
                num = _ranged(args[0], "reserved number", 0x10, 0xFF, path, line)
                reserved.append({"cmd": num, "note": _string(args[1], "reserved note", path, line)})
                entries.append((num, "VC_RESERVED", line))
        except ValueError as e:
            errs.extend(str(e).split("\n"))
    if cond:
        errs.append("%s:%d: conditional never closed with #endif" % (path, cond[1]))
    if page is None and not errs:
        errs.append("%s:1: no VC_PAGE" % path)

    seen_num, seen_name = {}, {}
    prev = None
    for num, label, line in entries:
        if num in seen_num:
            what = "reserved number collides with" if "VC_RESERVED" in (label, seen_num[num][0]) \
                else "duplicate command number, also"
            errs.append("%s:%d: 0x%02X %s %s at line %d"
                        % (path, line, num, what, seen_num[num][0], seen_num[num][1]))
        else:
            seen_num[num] = (label, line)
        if label != "VC_RESERVED":
            if label in seen_name:
                errs.append("%s:%d: duplicate command name %s (line %d)"
                            % (path, line, label, seen_name[label]))
            seen_name[label] = line
        if prev is not None and num < prev[0]:
            errs.append("%s:%d: 0x%02X after 0x%02X; entries must ascend" % (path, line, num, prev[0]))
        prev = (num, line)
    if errs:
        raise DefError(errs)
    return page


def parse_pages(pages_dir):
    """Parse every *.def in pages_dir; returns page dicts in registry form, ascending by page."""
    if not os.path.isdir(pages_dir):
        raise DefError(["%s: pages directory not found" % pages_dir])
    names = sorted(f for f in os.listdir(pages_dir) if f.endswith(".def"))
    if not names:
        raise DefError(["%s: no .def files" % pages_dir])
    errs, pages = [], []
    for fname in names:
        path = os.path.join(pages_dir, fname)
        rel = os.path.relpath(path, REPO) if path.startswith(REPO + os.sep) else path
        with open(path, encoding="utf-8") as fh:
            text = fh.read()
        try:
            p = parse_def_text(text, rel)
        except DefError as e:
            errs.extend(e.messages)
            continue
        if fname != p["mnemonic"].lower() + ".def":
            errs.append("%s:%d: file name must be %s.def for mnemonic %s"
                        % (rel, p["line"], p["mnemonic"].lower(), p["mnemonic"]))
        pages.append(p)
    by_num, by_mnem = {}, {}
    for p in pages:
        for key, table, label in ((p["page"], by_num, "page number 0x%02X" % p["page"]),
                                  (p["mnemonic"], by_mnem, "mnemonic " + p["mnemonic"])):
            if key in table:
                o = table[key]
                errs.append("%s:%d: duplicate %s, also %s:%d" % (p["file"], p["line"], label,
                                                                 o["file"], o["line"]))
            else:
                table[key] = p
    if errs:
        raise DefError(errs)
    pages.sort(key=lambda p: p["page"])
    for p in pages:
        p["commands"].sort(key=lambda c: c["cmd"])
        p["reserved"].sort(key=lambda r: r["cmd"])
        del p["file"], p["line"]
    return pages


def read_protocol_version(core_h):
    try:
        with open(core_h, encoding="utf-8") as fh:
            lines = fh.read().split("\n")
    except OSError as e:
        raise DefError(["%s: cannot read (%s)" % (core_h, e.strerror)])
    found, errs = {}, []
    for n, text in enumerate(lines, 1):
        m = PROTO_LINE.match(text)
        if m:
            if m.group(1) in found:
                errs.append("%s:%d: %s defined twice" % (core_h, n, m.group(1)))
            found[m.group(1)] = int(m.group(2))
    for key in ("VC_PROTOCOL_MAJOR", "VC_PROTOCOL_MINOR"):
        if key not in found:
            errs.append("%s: no '#define %s <decimal>' line" % (core_h, key))
    if errs:
        raise DefError(errs)
    return {"major": found["VC_PROTOCOL_MAJOR"], "minor": found["VC_PROTOCOL_MINOR"]}


def build_registry(pages_dir, core_h):
    errs, proto, pages = [], None, None
    for fn in (lambda: read_protocol_version(core_h), lambda: parse_pages(pages_dir)):
        try:
            r = fn()
        except DefError as e:
            errs.extend(e.messages)
            continue
        if isinstance(r, dict):
            proto = r
        else:
            pages = r
    if errs:
        raise DefError(errs)
    return {"protocol": proto, "pages": pages}


# ---------------------------------------------------------------------------
# Output: JSON, Markdown tables, listing
# ---------------------------------------------------------------------------

def _j(obj):
    return json.dumps(obj, separators=(", ", ": "), ensure_ascii=True)


def render_json(reg):
    out = ['{', '  "protocol": %s,' % _j(reg["protocol"]), '  "pages": [']
    for pi, p in enumerate(reg["pages"]):
        out.append('    {"page": %d, "mnemonic": %s, "revision": %d, "doc": %s,'
                   % (p["page"], _j(p["mnemonic"]), p["revision"], _j(p["doc"])))
        for key in ("commands", "reserved"):
            items = p[key]
            if not items:
                out.append('     "%s": []%s' % (key, "," if key == "commands" else "}"))
                continue
            out.append('     "%s": [' % key)
            for ii, item in enumerate(items):
                out.append("       " + _j(item) + ("," if ii + 1 < len(items) else ""))
            out.append("     ]" + ("," if key == "commands" else "}"))
        if pi + 1 < len(reg["pages"]):
            out[-1] += ","
    out += ["  ]", "}"]
    return "\n".join(out) + "\n"


def _md(text):
    return text.replace("|", "\\|")


def _dir(c):
    parts = []
    if c["get"]:
        parts.append("G")
    if c["set"]:
        parts.append("S0" if c["set_max"] == 0 and "BULK" not in c["flags"] else "S")
    return ", ".join(parts)


def _set_bytes(c):
    if not c["set"]:
        return "-"
    if "BULK" in c["flags"]:
        return "bulk"
    if c["set_min"] == c["set_max"]:
        return str(c["set_max"])
    return "%d-%d" % (c["set_min"], c["set_max"])


def render_page_table(page):
    rows = ["| Cmd | Name | Dir | wValue | GET bytes | SET bytes | Flags | Notes |",
            "|---|---|---|---|---|---|---|---|"]
    entries = [(c["cmd"], 0, c) for c in page["commands"]] + \
              [(r["cmd"], 1, r) for r in page["reserved"]]
    for num, is_res, e in sorted(entries, key=lambda x: (x[0], x[1])):
        if is_res:
            rows.append("| 0x%02X | _reserved_ | - | - | - | - | - | %s |" % (num, _md(e["note"])))
            continue
        get = "-" if e["get_len"] is None else str(e["get_len"])
        notes = _md(e["doc"])
        if e["build"]:
            notes += " (build: `%s` only)" % _md(e["build"])
        rows.append("| 0x%02X | `%s` | %s | %s | %s | %s | %s | %s |" % (
            num, e["name"], _dir(e), ADDR_DOC[e["addr"]], get, _set_bytes(e),
            ", ".join(e["flags"]), notes))
    return rows


def render_pages_table(pages):
    rows = ["| Page | Mnemonic | Revision | Commands | Description |", "|---|---|---|---|---|"]
    for p in pages:
        rows.append("| 0x%02X | `%s` | %d | %d | %s |" % (
            p["page"], p["mnemonic"], p["revision"], len(p["commands"]), _md(p["doc"])))
    return rows


def page_anchors(text):
    """Map each page mnemonic to the GitHub anchor of its '## N. MNEM (0xPP)' heading."""
    anchors = {}
    for m in re.finditer(r"^##[ \t]+(.*\b([A-Z][A-Z0-9_]*) \(0x[0-9A-Fa-f]{2}\))[ \t]*$", text, re.M):
        slug = re.sub(r"[^\w\- ]", "", m.group(1).strip().lower()).replace(" ", "-")
        anchors.setdefault(m.group(2), slug)
    return anchors


def _availability(build):
    if build is None:
        return ""
    return "RP2350 only" if build == "PICO_RP2350" else "`%s`" % _md(build)


def render_index_table(pages, anchors):
    """Every command sorted by name then page; links to the page section when its heading exists."""
    rows = ["| Name | Command | ID | Availability |", "|---|---|---|---|"]
    entries = sorted(((c["name"], p["page"], p["mnemonic"], c) for p in pages for c in p["commands"]),
                     key=lambda e: (e[0], e[1]))
    for name, _num, mnem, c in entries:
        cmd = "`%s.%s`" % (mnem, name)
        if mnem in anchors:
            cmd = "[%s](#%s)" % (cmd, anchors[mnem])
        rows.append(("| %s | %s | `0x%04X` | %s |" % (name, cmd, c["id"], _availability(c["build"]))))
    return rows


# ---------------------------------------------------------------------------
# Migration guide blocks (legacy_map.json + registry)
# ---------------------------------------------------------------------------

# Reply-length rules for VC_LEN_VAR GETs, which the registry cannot express.
VAR_RULES = {
    "SYS.PAGE_LIST": "2 x pages (`SYS.PROTOCOL_INFO` byte 6). 24 on a release build",
    "STORE.ALL_PARAMS_CHUNK": "min(wLength, 6252 - offset). wLength 0 is `VC_ERR_LENGTH`",
    "OUTPUT.LATENCY_OPTIONS": "8 x options (`OUTPUT.LATENCY_STATUS` byte 9). 56 today",
    "DYN.LIMITER_METERS": "2 x outputs. 10 on RP2040, 18 on RP2350",
    "FX.SUBHARM_METER": "2 x outputs. 10 on RP2040, 18 on RP2350",
    "MEAS.LEVELS": "2 x channels + 7. 21 on RP2040, 41 on RP2350",
    "MEAS.RTA_BAND_CENTRES": "2 x min(32, bands - 32n) for chunk n. 64, then 10 with 37 bands",
    "MEAS.RTA_BINS": "min(256, frame - offset). The frame is 145, 273 or 529 bytes for FFT order 8, 9 or 10",
    "MEAS.RTA_BANDS_ALL": "82 x live channels at the tap, 0 when none",
    "DIAG.CLOCK_DIAG": "`ClockDiagPacket`. 226 on RP2040, 248 on RP2350",
    "DIAG.WIRECAP_READ": "min(256, captured bytes - 256 x wValue)",
    "SYS.PIN_CLAIMS": "8 + 4 x claims (`SYS.PIN_STATE` byte 5). At most 208 on RP2040, 236 on RP2350",
}

# What each GET_STATUS (0x50) selector meant in 1.1.6. legacy_map.json holds where it lives now.
STATUS_SELECTORS = {
    0: "Peaks of channels 0 and 1", 1: "Peaks of channels 2 and 3",
    2: "Peak of channel 4, CPU loads", 3: "PDM ring overruns", 4: "PDM ring underruns",
    5: "PDM DMA overruns", 6: "PDM DMA underruns", 7: "Retired, always 0",
    8: "USB audio packet gaps of 2 to 50 ms (`spdif_underruns`)", 9: "Combined meter packet",
    10: "USB audio packets", 11: "USB audio alt setting", 12: "USB audio mounted",
    13: "clk_sys, Hz", 14: "Core voltage, mV", 15: "Device sample rate, Hz",
    16: "Temperature, centi-degrees C", 17: "Output DMA starvations, total",
    18: "Output DMA starvations, slot 0", 19: "Output DMA starvations, slot 1",
    20: "Output DMA starvations, slot 2", 21: "Output DMA starvations, slot 3",
    22: "USB audio ring overruns", 23: "Active input channel count",
    24: "Loopback capture overflows", 25: "Loopback capture underruns",
    26: "Output ring underrun episodes", 27: "Output ring drops",
    28: "Lowest ring fill before a write, frames", 29: "Worst input delivery gap, us",
    30: "Worst USB packet gap, us", 31: "Background steps over budget",
    32: "S/PDIF input measured rate, mHz", 33: "Clock trim, ppm x 100",
}

# The documented replacement of every retired opcode (legacy_map.json marks them "retired").
RETIRED_TEXT = {
    "SET_PREAMP": "Write `ROUTE.PREAMP` (`0x0540`) once per input channel, `wValue` = input channel "
                  "(0 to 1 on RP2040, 0 to 7 on RP2350), payload f32 dB",
    "GET_PREAMP": "Read `ROUTE.PREAMP` (`0x0540`) with `wValue` 0. 1.1.6 returned input channel 0",
    "SAVE_PARAMS": "Read `STORE.PRESET_ACTIVE` (`0x0214`), then send `STORE.PRESET_SAVE` (`0x0210`) "
                   "with `wValue` = that slot",
    "GET_USB_ERROR_STATS": "None. Its six counters always read 0. Live USB health is in "
                           "`SYS.USB_AUDIO_STATUS` and `SYS.PIPELINE_STATS`",
    "RESET_USB_ERROR_STATS": "None. It did nothing. `SYS.STATS_RESET` resets the watermarks",
    "SET_SPDIF_INPUT_ENABLE": "None per input. Set `INPUT.SPDIF_RX_COUNT` (`0x0326`) to cover the "
                              "input and `INPUT.SPDIF_RX_ENABLE` (`0x0327`) to 1",
}

LEVELS_CMD = "MEAS.LEVELS"   # what GET_STATUS "peaks", "peak_cpu" and "levels_raw" selectors read
STATUS_REPLY = {"pin": "1 byte `PIN_CONFIG_*`", "preset": "1 byte `PRESET_*`",
                "flash": "1 byte `FLASH_OK`", "ack1": "1 byte `0x01`"}
_TYPE_SIZE = {"u8": 1, "u16": 2, "i16": 2, "u32": 4, "i32": 4}


def heading_anchors(text):
    """[(heading text, GitHub anchor)] for every Markdown heading, duplicates suffixed as GitHub does."""
    out, seen = [], {}
    for m in re.finditer(r"^#{1,6}[ \t]+(.+?)[ \t]*$", text, re.M):
        slug = re.sub(r"[^\w\- ]", "", m.group(1).strip().lower()).replace(" ", "-")
        n = seen.get(slug, 0)
        seen[slug] = n + 1
        out.append((m.group(1), slug if n == 0 else "%s-%d" % (slug, n)))
    return out


def _heading_link(anchors, needle, prefix=""):
    for title, slug in anchors:
        if needle in title:
            return prefix + "#" + slug
    return None


class _Guide:
    """Shared lookups for the guide blocks. Errors collect in self.errs, prefixed with the doc path.
    xref is (relative link, text) of the full guide; links fall back to its headings."""

    def __init__(self, pages, legacy, text, path, xref=None):
        self.cmds = {"%s.%s" % (p["mnemonic"], c["name"]): (p, c) for p in pages for c in p["commands"]}
        self.doc = legacy
        self.anchors = heading_anchors(text)
        self.xref = (xref[0], heading_anchors(xref[1])) if xref else None
        self.path = path
        self.errs = []
        self.ops = sorted(legacy.get("opcodes", {}).items(), key=lambda kv: kv[1]["code"])
        self.direct_dirs = {}   # target -> {(opcode, "in"/"out")}, for the "folded" kind
        for op, rec in self.ops:
            for usb_dir in ("out", "in"):
                if usb_dir in rec:
                    for node in _adapters(rec[usb_dir]):
                        if node.get("kind") == "direct" and "target" in node:
                            self.direct_dirs.setdefault(node["target"], set()).add((op, usb_dir))

    def cmd(self, name):
        if name not in self.cmds:
            self.errs.append("%s: legacy map names %s, which is not in the registry" % (self.path, name))
            return None
        return self.cmds[name][1]

    def link(self, needle, required_for=None):
        href = _heading_link(self.anchors, needle)
        if href is None and self.xref:
            href = _heading_link(self.xref[1], needle, self.xref[0])
        if href is None and required_for:
            self.errs.append("%s: no heading contains %s, which %s needs" % (self.path, needle, required_for))
        return href

    def cmd_cell(self, name):
        c = self.cmd(name)
        if c is None:
            return "`%s`" % name
        return "`%s`%s" % (name, " (%s)" % _availability(c["build"]).replace("`", "")
                           if c["build"] else "")

    def id_cell(self, name):
        c = self.cmd(name)
        return "-" if c is None else "`0x%04X`" % c["id"]


def _field_text(spec, default):
    """An address or value rule as an expression of the old wValue ('old')."""
    if spec is None:
        return default
    if "const" in spec:
        return str(spec["const"])
    sh, m, add = spec.get("shift", 0), spec.get("mask", 0xFFFF), spec.get("add", 0)
    if (sh, m) == (0, 0xFFFF):
        base = "old"
    elif sh == 0:
        base = "old & 0x%X" % m
    elif m == 0xFFFF >> sh:
        base = "old >> %d" % sh
    else:
        base = "(old >> %d) & 0x%X" % (sh, m)
    if add:
        base += " %s %d" % ("-" if add < 0 else "+", abs(add))
    return base


def _code(text):
    return "`%s`" % text


def _selector_text(sel, key):
    k = int(key)
    sh, m = sel.get("shift", 0), sel.get("mask", 0xFFFF)
    shown = ("0x%X" % k) if k > 255 else str(k)
    if (sh, m) == (0, 0xFFFF):
        return shown
    return "%s = %s" % (_field_text(sel, "old"), shown)


def _value_text(node):
    v = _field_text(node["value"], "old")
    enc = node["encode"]
    if enc == "u8":
        return "u8 `%s`" % v
    if enc == "u8_sat":
        return "u8 `min(%s, 255)`" % v
    if enc == "bool":
        return "u8 `%s != 0` (1 or 0)" % v
    raise ValueError("unknown encode %r" % enc)


def _new_dir(c, is_get):
    if is_get:
        return "G"
    return "S0" if c["set_max"] == 0 and "BULK" not in c["flags"] else "S"


def _len_cell(c, is_get):
    if is_get:
        return str(c["get_len"])
    if "BULK" in c["flags"]:
        return str(c["get_len"]) if isinstance(c["get_len"], int) else "chunk"
    if c["set_min"] == c["set_max"]:
        return str(c["set_max"])
    return "%d-%d" % (c["set_min"], c["set_max"])


def _cases(node):
    """[(selector cell, case node or None)] of a split adapter, numeric keys ascending, default last."""
    keys = sorted(node["cases"], key=int)
    out = [(_selector_text(node["selector"], k), node["cases"][k]) for k in keys]
    out.append(("any other", node.get("default")))
    return out


def _kind(g, op, usb_dir, node):
    if node.get("retired"):
        return "retired"
    k = node["kind"]
    if k in ("act_value", "act_addr", "act_packed"):
        return "action"
    if k == "custom":
        return "custom"
    if k == "split":
        return "split"
    others = {d for o, d in g.direct_dirs.get(node["target"], ()) if o != op}
    return "folded" if others - {usb_dir} else "same"


def _map_row(g, code, op, old_dir, sel, usb_dir, node, kind):
    name_cell = "`%s`" % op
    href = g.link("`%s`" % op)
    if href:
        name_cell = "[`%s`](%s)" % (op, href)
    lead = "| 0x%02X | %s | %s | %s | %s |" % (code, name_cell, old_dir, sel, kind)
    if node is None:
        return lead + " none | - | - | - | %s | - |" % (
            "1.1.6 STALLed it" if usb_dir == "in" else "1.1.6 ignored it")
    if node.get("retired"):
        ret = g.link("Retired opcodes", "a retired row")
        return lead + " none | - | - | - | [replacement](%s) | - |" % (ret or "#")
    k = node["kind"]
    if k == "custom":
        href = g.link("`%s`" % op, "custom opcode %s" % op)
        cmds, ids = [], []
        for use in node.get("uses", []):
            tgt, _, how = use.partition(":")
            cmds.append("%s %s" % (g.cmd_cell(tgt), how))
            ids.append(g.id_cell(tgt))
        return lead + " %s | %s | - | see recipe | [see recipe](%s) | see recipe |" % (
            ", ".join(cmds) or "none", ", ".join(ids) or "-", href or "#")
    c = g.cmd(node["target"])
    if c is None:
        return lead + " ? | ? | ? | ? | ? | ? |"
    is_get = k == "direct" and usb_dir == "in"
    if k == "direct":
        addr = _field_text(node.get("addr"), "old")
        if is_get:
            payload = "-"
        elif node.get("trunc"):
            payload = "first %d bytes" % node["trunc"]
        else:
            payload = "unchanged"
    elif k == "act_addr":
        addr, payload = _field_text(node.get("addr"), "old"), "none"
    elif k == "act_value":
        addr, payload = _field_text(node.get("addr"), "0"), _value_text(node)
    elif k == "act_packed":
        addr, payload = _field_text(node["addr"], "old"), _value_text(node)
    else:
        raise ValueError("%s: unknown adapter kind %r" % (op, k))
    if c["addr"] == "NONE":
        addr = "0"   # 1.1.6 ignored wValue here, 1.1.7 requires 0
    return lead + " %s | %s | %s | %s | %s | %s |" % (
        g.cmd_cell(node["target"]), g.id_cell(node["target"]), _new_dir(c, is_get),
        _code(addr), payload, _len_cell(c, is_get))


def render_legacy_map(g):
    rows = ["| Old | Old name | Old dir | Old wValue | Change | New command | ID | New dir | New wValue "
            "| Payload | Bytes |",
            "|---|---|---|---|---|---|---|---|---|---|---|"]
    for op, rec in g.ops:
        for usb_dir in ("out", "in"):
            if usb_dir not in rec:
                continue
            old_dir = {"WR": "W-as-R"}.get(rec["old_dir"], "W" if usb_dir == "out" else "R")
            node = rec[usb_dir]
            if node.get("kind") == "split" and not node.get("retired"):
                for sel, case in _cases(node):
                    rows.append(_map_row(g, rec["code"], op, old_dir, sel, usb_dir, case, "split"))
            else:
                rows.append(_map_row(g, rec["code"], op, old_dir, "any", usb_dir, node,
                                     _kind(g, op, usb_dir, node)))
    return rows


def _status_cell(g, node):
    fam_name = node.get("status")
    fam = g.doc.get("status_families", {}).get(fam_name)
    if fam is None:
        g.errs.append("%s: status family %r is not in legacy_map.json" % (g.path, fam_name))
        return "?"
    if fam_name == "ack1" or (not fam["from_vc"] and not node.get("status_overrides")):
        if fam["success"] == 1:
            return "0x01 was success. Every refusal STALLed then and STALLs now"
        return "0x%02X was success. 1.1.6 never refused it" % fam["success"]
    by_byte = {}
    table = dict(fam["from_vc"])
    table.update(node.get("status_overrides", {}))
    for vc, byte in table.items():
        by_byte.setdefault(byte, []).append(vc)
    parts = ["0x%02X for OK" % fam["success"]]
    for byte in sorted(by_byte):
        names = ["`%s`" % n[len("VC_ERR_"):]
                 for n in sorted(by_byte[byte], key=lambda n: g.doc.get("vc_results", {}).get(n, 0))]
        names = names[0] if len(names) == 1 else "%s or %s" % (", ".join(names[:-1]), names[-1])
        parts.append("0x%02X for %s" % (byte, names))
    return ". ".join(parts)


def _wasr_row(g, rec, op, sel, node):
    name_cell = "`%s`" % op
    href = g.link("`%s`" % op)
    if href:
        name_cell = "[`%s`](%s)" % (op, href)
    lead = "| 0x%02X | %s |" % (rec["code"], name_cell)
    if node is None:
        return lead + " %s | - | none | - | - | 1.1.6 STALLed it |" % sel
    k = node["kind"]
    if k == "custom" or node.get("retired"):
        need = "Retired opcodes" if node.get("retired") else "`%s`" % op
        href = g.link(need, "write-as-read %s" % op)
        cmds = ", ".join(g.cmd_cell(u.partition(":")[0]) for u in node.get("uses", [])) or "none"
        return lead + " %s | %s | %s | see recipe | [see recipe](%s) | see recipe |" % (
            sel, STATUS_REPLY.get(node.get("status"), "see recipe"), cmds, href or "#")
    c = g.cmd(node["target"])
    if c is None:
        return lead + " %s | ? | ? | ? | ? | ? |" % sel
    cmd = "%s %s" % (g.cmd_cell(node["target"]), g.id_cell(node["target"]))
    if k == "direct":
        return lead + " %s | %s bytes | %s | `%s` | none, it is a GET | reply unchanged |" % (
            sel, c["get_len"], cmd, _field_text(node.get("addr"), "old"))
    if k == "act_addr":
        spec = node.get("addr")
        addr = _field_text(spec, "old")
        old = sel if sel != "any" else (
            "0" if spec and "const" in spec else "%s in `%s`" % (ADDR_DOC[c["addr"]], addr))
        payload = "none (zero-length)"
    elif k == "act_value":
        addr = _field_text(node.get("addr"), "0")
        old = sel if sel != "any" else "value in `%s`" % _field_text(node["value"], "old")
        payload = "1 byte: " + _value_text(node)
    else:   # act_packed
        addr = _field_text(node["addr"], "old")
        old = "%s in `%s`, value in `%s`" % (ADDR_DOC[c["addr"]], addr,
                                            _field_text(node["value"], "old"))
        payload = "1 byte: " + _value_text(node)
    return lead + " %s | %s | %s | `%s` | %s | %s |" % (
        old, STATUS_REPLY.get(node.get("status"), "?"), cmd, addr, payload, _status_cell(g, node))


def render_legacy_wasr(g):
    rows = ["| Old | Old name | Old wValue | Old reply | New command | New wValue | SET payload "
            "| Old status byte: new result |",
            "|---|---|---|---|---|---|---|---|"]
    for op, rec in g.ops:
        if rec.get("old_dir") != "WR":
            continue
        node = rec["in"]
        if node.get("kind") == "split":
            for sel, case in _cases(node):
                rows.append(_wasr_row(g, rec, op, sel, case))
        else:
            rows.append(_wasr_row(g, rec, op, "any", node))
    return rows


def _status_entry(g):
    rec = g.doc.get("opcodes", {}).get("GET_STATUS")
    if rec is None or rec.get("in", {}).get("fn") != "get_status":
        g.errs.append("%s: legacy_map.json has no GET_STATUS custom entry" % g.path)
        return None
    return rec["in"]


def _status_rows(g, e):
    """{selector: (command cell, offset cell, type cell, old-value cell)} from the GET_STATUS entry."""
    out = {}
    for sel in e.get("levels_raw", []):
        out[int(sel)] = (g.cmd_cell(LEVELS_CMD), "whole reply", "var",
                         "same bytes as the old reply")
    for sel, (a, b) in e.get("peaks", {}).items():
        out[int(sel)] = (g.cmd_cell(LEVELS_CMD), "%d, %d" % (2 * a, 2 * b), "u16, u16",
                         "bits 0-15 = u16 at %d, bits 16-31 = u16 at %d" % (2 * a, 2 * b))
    for sel, (ch, src, c0, c1) in e.get("peak_cpu", {}).items():
        out[int(sel)] = ("%s, %s" % (g.cmd_cell(LEVELS_CMD), g.cmd_cell(src)),
                         "%d, then %d and %d" % (2 * ch, c0, c1), "u16, then u8, u8",
                         "bits 0-15 = peak, 16-23 = byte %d, 24-31 = byte %d" % (c0, c1))
    for sel in e.get("zero", []):
        out[int(sel)] = ("none", "-", "-", "always 0")
    for sel, (src, typ, off) in e.get("fields", {}).items():
        ext = "sign-extend" if typ.startswith("i") and _TYPE_SIZE[typ] < 4 else \
            ("as is" if _TYPE_SIZE[typ] == 4 else "zero-extend")
        cell = g.cmd_cell(src)
        if src in e.get("zero_if_absent", []):
            cell += ", read 0 if `VC_ERR_UNKNOWN`"
        out[int(sel)] = (cell, str(off), typ, ext)
    return out


def render_legacy_status(g):
    rows = ["| Old wValue | 1.1.6 meaning | Read now | Offset | Type | Old 4-byte value |",
            "|---|---|---|---|---|---|"]
    e = _status_entry(g)
    if e is None:
        return rows
    data = _status_rows(g, e)
    if set(data) != set(STATUS_SELECTORS):
        g.errs.append("%s: GET_STATUS selectors in legacy_map.json %s differ from STATUS_SELECTORS %s"
                      % (g.path, sorted(data), sorted(STATUS_SELECTORS)))
    for sel in sorted(data):
        rows.append("| %d | %s | %s | %s | %s | %s |" % ((sel, STATUS_SELECTORS.get(sel, "?")) + data[sel]))
    rows.append("| any other | Unused (4 zero bytes) | none | - | - | no equivalent |")
    return rows


def render_get_lengths(g, pages):
    rows = ["| Command | Page | Cmd | wValue | Reply bytes | Availability | Notes |",
            "|---|---|---|---|---|---|---|"]
    for p in pages:
        for c in p["commands"]:
            if not c["get"]:
                continue
            name = "%s.%s" % (p["mnemonic"], c["name"])
            avail = [_availability(c["build"])] if c["build"] else []
            if "USB_ONLY" in c["flags"]:
                avail.append("USB only")
            note = ""
            if c["get_len"] == "var":
                if name not in VAR_RULES:
                    g.errs.append("%s: %s has a variable reply and no rule in VAR_RULES" % (g.path, name))
                note = VAR_RULES.get(name, "?")
            elif "BULK" in c["flags"]:
                note = "Bulk buffer, `min(wLength, %d)`" % c["get_len"]
            rows.append("| `%s` | 0x%02X | 0x%02X | %s | %s | %s | %s |" % (
                name, p["page"], c["cmd"], ADDR_DOC[c["addr"]],
                "var" if c["get_len"] == "var" else c["get_len"], ", ".join(avail), note))
    for name in sorted(set(VAR_RULES) - {n for n, (_p, c) in g.cmds.items() if c["get_len"] == "var"}):
        if name in g.cmds:
            g.errs.append("%s: VAR_RULES names %s, whose reply is not variable" % (g.path, name))
    return rows


def render_retired(g):
    rows = ["| Old | Old name | Old dir | Replacement |", "|---|---|---|---|"]
    retired = set()
    for op, rec in g.ops:
        for usb_dir in ("out", "in"):
            if usb_dir in rec and rec[usb_dir].get("retired"):
                retired.add(op)
                old_dir = {"WR": "W-as-R"}.get(rec["old_dir"], "W" if usb_dir == "out" else "R")
                if op not in RETIRED_TEXT:
                    g.errs.append("%s: retired opcode %s has no entry in RETIRED_TEXT" % (g.path, op))
                rows.append("| 0x%02X | `%s` | %s | %s |" % (rec["code"], op, old_dir,
                                                            RETIRED_TEXT.get(op, "?")))
        if op == "GET_STATUS":
            e, text = rec["in"], {}
            for sel, (a, b) in e.get("peaks", {}).items():
                text[int(sel)] = "%s, u16 at offsets %d and %d" % (g.cmd_cell(LEVELS_CMD), 2 * a, 2 * b)
            for sel, (ch, src, c0, c1) in e.get("peak_cpu", {}).items():
                text[int(sel)] = "%s, u16 at offset %d, then %s bytes %d and %d" % (
                    g.cmd_cell(LEVELS_CMD), 2 * ch, g.cmd_cell(src), c0, c1)
            for sel in e.get("zero", []):
                text[int(sel)] = "None. It always read 0"
            for sel in sorted(text):
                rows.append("| 0x%02X | `%s` `wValue` %d | R | %s |" % (rec["code"], op, sel, text[sel]))
    for op in sorted(set(RETIRED_TEXT) - retired):
        g.errs.append("%s: RETIRED_TEXT names %s, which legacy_map.json does not retire" % (g.path, op))
    return rows


def rewrite_doc_blocks(text, pages, path="<docs>", legacy=None, xref=None):
    """Replace every VC:BEGIN/VC:END block in text. Returns (new_text, blocks_rewritten).
    The guide blocks (GUIDE_MARKERS) need legacy, the parsed legacy_map.json. xref: see _Guide."""
    by_mnem = {p["mnemonic"]: p for p in pages}
    anchors = page_anchors(text)
    guide = None
    lines = text.splitlines(True)
    nl = "\r\n" if lines and lines[0].endswith("\r\n") else "\n"
    out, errs, seen = [], [], {}
    open_blk = None
    count = 0
    for n, ln in enumerate(lines, 1):
        b, e = BEGIN_RE.match(ln.rstrip("\r\n")), END_RE.match(ln.rstrip("\r\n"))
        if b:
            mnem = b.group(1)
            if open_blk:
                errs.append("%s:%d: VC:BEGIN %s inside open block %s (line %d)"
                            % (path, n, mnem, open_blk[0], open_blk[1]))
                continue
            if mnem in seen:
                errs.append("%s:%d: second block for %s (first at line %d)" % (path, n, mnem, seen[mnem]))
            elif mnem not in DOCS_MARKERS and mnem not in by_mnem:
                errs.append("%s:%d: VC:BEGIN names unknown page %s" % (path, n, mnem))
            seen[mnem] = n
            open_blk = (mnem, n)
            out.append(ln)
            continue
        if e:
            if not open_blk:
                errs.append("%s:%d: VC:END without VC:BEGIN" % (path, n))
            elif e.group(1) != open_blk[0]:
                errs.append("%s:%d: VC:END %s does not match VC:BEGIN %s (line %d)"
                            % (path, n, e.group(1) or "(no name)", open_blk[0], open_blk[1]))
            else:
                mnem = open_blk[0]
                if mnem in GUIDE_MARKERS and legacy is None:
                    errs.append("%s:%d: block %s needs the legacy map (--legacy-map)"
                                % (path, open_blk[1], mnem))
                    rows = None
                elif mnem in GUIDE_MARKERS:
                    if guide is None:
                        guide = _Guide(pages, legacy, text, path, xref)
                    rows = {"LEGACY_MAP": lambda: render_legacy_map(guide),
                            "LEGACY_WASR": lambda: render_legacy_wasr(guide),
                            "LEGACY_STATUS": lambda: render_legacy_status(guide),
                            "GET_LENGTHS": lambda: render_get_lengths(guide, pages),
                            "RETIRED": lambda: render_retired(guide)}[mnem]()
                elif mnem == DOCS_PAGES_MARKER:
                    rows = render_pages_table(pages)
                elif mnem == DOCS_INDEX_MARKER:
                    rows = render_index_table(pages, anchors)
                elif mnem in by_mnem:
                    rows = render_page_table(by_mnem[mnem])
                else:
                    rows = None
                if rows is not None:
                    out.extend(r + nl for r in [""] + rows + [""])
                    count += 1
            open_blk = None
            out.append(ln)
            continue
        if not open_blk:
            out.append(ln)
    if open_blk:
        errs.append("%s:%d: VC:BEGIN %s never closed" % (path, open_blk[1], open_blk[0]))
    if guide is not None:
        errs.extend(dict.fromkeys(guide.errs))   # each problem once, however many rows hit it
    if errs:
        raise DefError(errs)
    return "".join(out), count


def render_listing(reg):
    out = ["protocol %d.%d" % (reg["protocol"]["major"], reg["protocol"]["minor"])]
    for p in reg["pages"]:
        out.append("page 0x%02X %-8s rev %d  %d cmds  %s" % (
            p["page"], p["mnemonic"], p["revision"], len(p["commands"]), p["doc"]))
        entries = [(c["cmd"], 0, c) for c in p["commands"]] + [(r["cmd"], 1, r) for r in p["reserved"]]
        for num, is_res, e in sorted(entries, key=lambda x: (x[0], x[1])):
            if is_res:
                out.append("  0x%02X %-28s %s" % (num, "(reserved)", e["note"]))
                continue
            get = "-" if e["get_len"] is None else str(e["get_len"])
            extra = " ".join(e["flags"]) + ((" [#if %s]" % e["build"]) if e["build"] else "")
            out.append(("  0x%02X %-28s %-5s %-13s get=%-4s set=%-6s %s" % (
                num, e["name"], _dir(e), e["addr"], get, _set_bytes(e), extra.strip())).rstrip())
    return "\n".join(out) + "\n"


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _diff(old, new, label, limit=40):
    d = list(difflib.unified_diff(old.splitlines(), new.splitlines(),
                                  "committed/" + label, "generated/" + label, lineterm="", n=1))
    more = len(d) - limit
    return "\n".join(d[:limit]) + ("\n... %d more diff lines" % more if more > 0 else "")


def _read(path):
    try:
        with open(path, encoding="utf-8", newline="") as fh:
            return fh.read()
    except FileNotFoundError:
        return None


def _write(path, text):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as fh:
        fh.write(text)


def _adapters(node):
    """Every adapter node of one legacy_map.json opcode direction, split cases included."""
    yield node
    if node.get("kind") == "split":
        for case in node.get("cases", {}).values():
            yield from _adapters(case)
        if node.get("default"):
            yield from _adapters(node["default"])


def check_legacy_map(reg, doc):
    """Mismatches between legacy_map.json and the registry, each prefixed with its JSON key.
    Checks the commands catalog (page, number, direction, address, sizes) and every adapter target and use."""
    errs = []
    by_name = {"%s.%s" % (p["mnemonic"], c["name"]): (p, c) for p in reg["pages"] for c in p["commands"]}
    page_no = {p["mnemonic"]: p["page"] for p in reg["pages"]}
    for mnem, num in sorted(doc.get("pages", {}).items()):
        if page_no.get(mnem) != num:
            errs.append("pages.%s: 0x%02X, registry has %s" % (
                mnem, num, "0x%02X" % page_no[mnem] if mnem in page_no else "no such page"))
    for name, e in sorted(doc.get("commands", {}).items()):
        key = "commands.%s" % name
        if name not in by_name:
            errs.append("%s: not in the registry" % key)
            continue
        pg, c = by_name[name]
        if e.get("page") != pg["mnemonic"] or doc.get("pages", {}).get(e.get("page")) != pg["page"]:
            errs.append("%s: page %s, registry 0x%02X" % (key, e.get("page"), pg["page"]))
        if e.get("cmd") != c["cmd"]:
            errs.append("%s: cmd %s, registry 0x%02X" % (key, e.get("cmd"), c["cmd"]))
        d = e.get("dir", "")
        if ("G" in d) != c["get"] or ("S" in d) != c["set"]:
            errs.append("%s: dir %s, registry %s" % (key, d, _dir(c)))
        if e.get("addr") != c["addr"]:
            errs.append("%s: addr %s, registry %s" % (key, e.get("addr"), c["addr"]))
        if c["get"] and e.get("get_len") != c["get_len"]:
            errs.append("%s: get_len %s, registry %s" % (key, e.get("get_len"), c["get_len"]))
        # A BULK command checks its own length, so the registry carries no SET bounds for it.
        want_set = e.get("set", [0, 0] if d == "S0" else None)
        if c["set"] and "BULK" not in c["flags"] and want_set != [c["set_min"], c["set_max"]]:
            errs.append("%s: set %s, registry [%d, %d]" % (key, want_set, c["set_min"], c["set_max"]))
        if e.get("build") != c["build"]:
            errs.append("%s: build %s, registry %s" % (key, e.get("build"), c["build"]))
    for op, rec in sorted(doc.get("opcodes", {}).items()):
        for usb_dir in ("in", "out"):
            if usb_dir not in rec:
                continue
            key = "opcodes.%s.%s" % (op, usb_dir)
            for node in _adapters(rec[usb_dir]):
                need = []
                if "target" in node:
                    getter = node.get("kind") == "direct" and usb_dir == "in"
                    need.append((node["target"], "G" if getter else "S"))
                for use in node.get("uses", []):
                    target, _, how = use.partition(":")
                    if how not in ("G", "S"):
                        errs.append("%s: malformed use %r (want PAGE.NAME:G or :S)" % (key, use))
                        continue
                    need.append((target, how))
                for target, how in need:
                    if target not in by_name:
                        errs.append("%s: %s not in the registry" % (key, target))
                    elif target not in doc.get("commands", {}):
                        errs.append("%s: %s not in the commands catalog" % (key, target))
                    elif not by_name[target][1]["get" if how == "G" else "set"]:
                        errs.append("%s: %s has no %s" % (key, target, "GET" if how == "G" else "SET"))
    return errs


def _rel(path):
    return os.path.relpath(path, REPO) if path.startswith(REPO + os.sep) else path


def main(argv=None):
    ap = argparse.ArgumentParser(description="Generate the DSPi vendor command registry JSON "
                                 "and doc tables from firmware/DSPi/vendor/pages/*.def.")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true",
                      help="write nothing; fail if the JSON or doc blocks are stale or a .def is invalid")
    mode.add_argument("--list", action="store_true", help="print a compact listing and exit")
    ap.add_argument("--pages", default=DEFAULT_PAGES, help="directory of .def files")
    ap.add_argument("--core-h", default=DEFAULT_CORE_H, help="header holding VC_PROTOCOL_MAJOR/MINOR")
    ap.add_argument("--json", default=DEFAULT_JSON, help="registry JSON path")
    ap.add_argument("--docs", action="append", metavar="PATH",
                    help="Markdown file with VC:BEGIN/VC:END blocks (repeatable; default "
                         "Documentation/commands.md and the two Features/vendor_protocol_*.md guides)")
    ap.add_argument("--legacy-map", default=None,
                    help="old-opcode migration map that --check validates against the registry "
                         "(default Documentation/protocol/legacy_map.json when --pages is the default)")
    a = ap.parse_args(argv)
    docs = a.docs if a.docs else [p for p in DEFAULT_DOCS if os.path.exists(p)]
    if a.legacy_map is None and os.path.abspath(a.pages) == DEFAULT_PAGES:
        a.legacy_map = DEFAULT_LEGACY_MAP

    try:
        reg = build_registry(a.pages, a.core_h)
    except DefError as e:
        for m in e.messages:
            sys.stderr.write("error: %s\n" % m)
        sys.stderr.write("%d error(s); nothing written\n" % len(e.messages))
        return 1

    if a.list:
        sys.stdout.write(render_listing(reg))
        return 0

    outputs = [(a.json, render_json(reg), _read(a.json))]
    errs = []
    lm = _read(a.legacy_map) if a.legacy_map else None
    try:
        lm_doc = None if lm is None else json.loads(lm)
    except ValueError as e:
        sys.stderr.write("error: %s: %s\n" % (_rel(a.legacy_map), e))
        return 1
    guide_text = _read(DEFAULT_GUIDE)
    for path in docs:
        old = _read(path)
        if old is None:
            errs.append("%s: not found" % path)
            continue
        xref = None
        if guide_text is not None and os.path.abspath(path) != DEFAULT_GUIDE:
            rel = os.path.relpath(DEFAULT_GUIDE, os.path.dirname(os.path.abspath(path)))
            xref = (rel.replace(os.sep, "/"), guide_text)
        try:
            new, count = rewrite_doc_blocks(old, reg["pages"], _rel(path), lm_doc, xref)
        except DefError as e:
            errs.extend(e.messages)
            continue
        if count:
            outputs.append((path, new, old))
    if errs:
        for m in errs:
            sys.stderr.write("error: %s\n" % m)
        return 1

    stale = [(p, new, old) for p, new, old in outputs if new != old]
    if a.check:
        lm_errs = [] if lm_doc is None else check_legacy_map(reg, lm_doc)
        for m in lm_errs:
            print("MISMATCH %s: %s" % (_rel(a.legacy_map), m))
        for p, new, old in stale:
            if old is None:
                print("STALE %s: missing" % _rel(p))
            else:
                print("STALE %s:\n%s" % (_rel(p), _diff(old, new, os.path.basename(p))))
        if stale:
            print("%d file(s) out of date; run scripts/gen_vendor_protocol.py" % len(stale))
        if lm_errs:
            print("%d legacy map mismatch(es)" % len(lm_errs))
        if stale or lm_errs:
            return 1
        print("OK: %d page(s), %d command(s); %d file(s) up to date%s" % (
            len(reg["pages"]), sum(len(p["commands"]) for p in reg["pages"]), len(outputs),
            "" if lm is None else "; %s matches the registry" % _rel(a.legacy_map)))
        return 0

    for p, new, _old in stale:
        _write(p, new)
        print("wrote %s" % _rel(p))
    if not stale:
        print("up to date")
    return 0


if __name__ == "__main__":
    sys.exit(main())
