#!/usr/bin/env python3
"""Check that the firmware and the PC console still agree on the wire format.

The Core2 emits telemetry and CSV rows by hand-written snprintf; the console
parses them positionally and drops any line whose field count does not match.
Nothing links the two, so adding a column on one side and forgetting the other
produces a console that silently plots nothing - which is the single easiest
way to break this pair.

Reads both sources as text (no numpy, matplotlib or serial needed) and
compares the shapes.
"""

import ast
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FAIL = []


def check(cond, msg):
    if not cond:
        FAIL.append(msg)


def count_conversions(fmt):
    """Number of values a printf format string consumes."""
    return len(re.findall(r"%(?!%)[-+ #0]*[\d.*]*(?:hh|h|ll|l|j|z|t|L)?[diouxXeEfgGaAcspn]", fmt))


def c_string_at(text, start):
    """Concatenate the adjacent C string literals starting at `start`."""
    out, i, n = [], start, len(text)
    while i < n:
        if text[i] == '"':
            j = i + 1
            buf = []
            while j < n and text[j] != '"':
                if text[j] == "\\":
                    buf.append(text[j:j + 2])
                    j += 2
                    continue
                buf.append(text[j])
                j += 1
            out.append("".join(buf))
            i = j + 1
            continue
        if text[i] in " \t\r\n":
            i += 1
            continue
        break
    return "".join(out)


def py_list(name, src):
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == name:
                    return ast.literal_eval(node.value)
    return None


def main():
    serial_src = (ROOT / "src" / "serial_link.cpp").read_text()
    logger_src = (ROOT / "src" / "logger.cpp").read_text()
    console_src = (ROOT / "tools" / "rs06_console.py").read_text()

    t_fields = py_list("T_FIELDS", console_src)
    d_fields = py_list("D_FIELDS", console_src)
    alias = py_list("COL_ALIAS", console_src)
    check(t_fields is not None, "T_FIELDS not found in rs06_console.py")
    check(d_fields is not None, "D_FIELDS not found in rs06_console.py")
    if t_fields is None or d_fields is None:
        return report()

    # --- streamed telemetry -------------------------------------------------
    # serial_link.cpp has several snprintf(line, ...) calls; the telemetry one
    # is the only format that starts with the "T," row tag.
    fmt = None
    for m in re.finditer(r'snprintf\(line,\s*sizeof\(line\),\s*', serial_src):
        cand = c_string_at(serial_src, m.end())
        if cand.startswith("T,"):
            fmt = cand
            break
    check(fmt is not None, "could not find the 'T,' telemetry snprintf in serial_link.cpp")
    if fmt:
        n_fmt = count_conversions(fmt)
        check(n_fmt == len(t_fields),
              f"telemetry line emits {n_fmt} fields, console T_FIELDS expects "
              f"{len(t_fields)} -> every row would be dropped")

    # The console also announces the header; it must list the same names.
    m = re.search(r'TELEM_HEADER\s*=\s*', serial_src)
    if m:
        hdr = c_string_at(serial_src, m.end())
        hdr = hdr[len("#H "):] if hdr.startswith("#H ") else hdr
        names = [h.strip() for h in hdr.split(",")]
        names = [alias.get(n, n) for n in names]
        check(names == list(t_fields),
              f"TELEM_HEADER names {names} != console T_FIELDS {list(t_fields)}")

    # --- 1 kHz CSV dump -----------------------------------------------------
    m = re.search(r'csvHeader\(\)\s*\{\s*return\s*', logger_src)
    check(m is not None, "could not find csvHeader() in logger.cpp")
    if m:
        hdr = c_string_at(logger_src, m.end())
        cols = [alias.get(c.strip(), c.strip()) for c in hdr.split(",")]
        check(cols == list(d_fields),
              f"csvHeader() gives {cols}, console D_FIELDS expects {list(d_fields)}")

    m = re.search(r'return snprintf\(line, n,\s*', logger_src)
    if m:
        fmt = c_string_at(logger_src, m.end())
        n_fmt = count_conversions(fmt)
        check(n_fmt == len(d_fields),
              f"CSV row emits {n_fmt} columns, header declares {len(d_fields)}")

    return report()


def report():
    if FAIL:
        print("protocol contract: FAILED")
        for f in FAIL:
            print("  -", f)
        return 1
    print("protocol contract: firmware and console agree")
    return 0


if __name__ == "__main__":
    sys.exit(main())
