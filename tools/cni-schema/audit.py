#!/usr/bin/env python3
"""Find the pages whose layout cannot be trusted, so the rest need no inspection.

Checking a rendered page against the cockpit is slow and there are 145 of them. But the
geometry is one transform applied to 145 sets of data, not 145 layouts: validating it on a
couple of pages validates it everywhere the pages behave alike. What is worth a human's time
is the pages that do not.

This audits the full schema for the two faults that can be established without the sim:

  * text placed outside the 24-column grid, which loses characters outright;
  * two elements that are drawn at once overlapping on the same line.

Only literal text is judged. Dynamic fields have no length until the sim fills them, so
flagging them would drown the real findings in guesses. On/off variants of the same field
are expected to share a position — only one is ever drawn — so they are not overlaps.

    python audit.py c130j-cni-pages.json
"""

import json
import re
import sys
from collections import defaultdict
from pathlib import Path

COLUMNS = 25


def start_column(anchor, origin, length):
    """Mirror of CniGrid.StartColumn."""
    if anchor == "Right":
        return origin - length
    if anchor == "Center":
        return origin - length // 2
    return origin


def controller_stem(ctrl):
    """Strip the on/off tail that distinguishes the variants of one field."""
    if not ctrl:
        return None
    return re.sub(r"_(on|off)(_(on|off))?$", "", ctrl)


def exclusive(a, b):
    """True when the sim can only ever draw one of the two."""
    if a.get("value") == b.get("value") and a.get("value") is not None:
        return True

    sa, sb = controller_stem(a.get("ctrl")), controller_stem(b.get("ctrl"))
    if sa and sb and sa == sb:
        return True

    # A pair fed by the same field, one shown when it is set and one when it is not.
    ca, cb = a.get("ctrl"), b.get("ctrl")
    if ca and cb and ca != cb and (controller_stem(ca) == cb or controller_stem(cb) == ca):
        return True

    return False


def audit_page(page):
    faults = []
    placed = defaultdict(list)

    for s in page["slots"]:
        if s.get("off") or s.get("lineErr", 0) > 0.02:
            continue
        value, line, col = s.get("value"), s.get("line"), s.get("col")
        if value is None or line is None or col is None:
            continue

        start = start_column(s.get("anchor", "Left"), col, len(value))
        end = start + len(value)

        if len(value) <= COLUMNS and (start < 0 or end > COLUMNS):
            faults.append(("hors grille", f"{value!r} -> {start}..{end}"))

        placed[line].append((start, end, s))

    for line, items in placed.items():
        items.sort(key=lambda t: (t[0], t[1]))
        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                (s1, e1, a), (s2, e2, b) = items[i], items[j]
                if s2 >= e1:
                    break
                if exclusive(a, b):
                    continue
                faults.append((
                    "chevauchement",
                    f"L{line}: {a['value']!r}[{s1}..{e1}] / {b['value']!r}[{s2}..{e2}]"))

    return faults


def main():
    schema = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    pages = schema["pages"]

    flagged = []
    for p in pages:
        faults = audit_page(p)
        if faults:
            flagged.append((len(faults), p["name"], p.get("title") or "", faults))

    flagged.sort(reverse=True)

    clean = len(pages) - len(flagged)
    print(f"pages auditees : {len(pages)}")
    print(f"  sans reserve : {clean}  ({100 * clean / len(pages):.0f}%)")
    print(f"  a verifier   : {len(flagged)}\n")

    for count, name, title, faults in flagged:
        print(f"{name}  ({title!r})  {count} anomalie(s)")
        for kind, detail in faults[:4]:
            print(f"    {kind:<14} {detail}")
        if len(faults) > 4:
            print(f"    ... et {len(faults) - 4} autres")
        print()


if __name__ == "__main__":
    main()
