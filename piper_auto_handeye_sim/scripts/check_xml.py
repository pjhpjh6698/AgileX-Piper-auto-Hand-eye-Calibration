#!/usr/bin/env python3
"""Validate the package's XML/xacro/SDF files before launching.

Exists because one specific mistake kept slipping through: a "--" inside an XML
comment. It is illegal in XML but reads perfectly naturally in English prose
("the sign matters too -- +X is ..."), so it survives review and only shows up
as an ExpatError at launch time, pointing at a line number rather than the
actual cause. Three separate launch failures came from exactly this.

Run before launching, or wire it into CI:
    python3 scripts/check_xml.py
"""

import glob
import os
import re
import sys
import xml.etree.ElementTree as ET

FILES = ["urdf/*.xacro", "worlds/*.world",
         "models/*/model.sdf", "models/*/model.config"]


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    problems = 0
    checked = 0

    for pattern in FILES:
        for path in sorted(glob.glob(os.path.join(root, pattern))):
            rel = os.path.relpath(path, root)
            checked += 1
            text = open(path).read()

            # the specific trap: "--" inside a comment body
            for m in re.finditer(r"<!--(.*?)-->", text, re.DOTALL):
                if "--" in m.group(1):
                    line = text[:m.start()].count("\n") + 1
                    snippet = m.group(1).strip().replace("\n", " ")[:60]
                    print(f"BAD  {rel}:{line}  '--' inside a comment: {snippet}...")
                    problems += 1

            try:
                ET.fromstring(text)
                print(f"OK   {rel}")
            except ET.ParseError as exc:
                print(f"FAIL {rel}: {exc}")
                problems += 1

    print(f"\n{checked} file(s) checked, {problems} problem(s)")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
