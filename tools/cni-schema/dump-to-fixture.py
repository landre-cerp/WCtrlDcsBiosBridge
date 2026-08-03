#!/usr/bin/env python3
"""Turn a c130-probe dump into a test fixture.

The probe records what list_indication returned; the app consumes what wctrl-export.lua makes
of it. Everything about the CNI layout is derived from files outside DCS, so a dump is the only
thing that says the derivation is right - and until it is a fixture it can only be read, not
replayed. This is the step that was missing when the July captures were lost: the twelve
fixtures survived because they had been converted, the sessions behind them had not.

The raw section is read, not the parsed one. Both carry the values in discovery order, but only
the raw carries the element names alongside them, and CniSessionMap is keyed on those. They are
GUIDs the sim regenerates every session, so they are worth nothing to anyone else - which is
exactly why they must come from the same session as the values they sit with.

    python dump-to-fixture.py "D:/Saved Games/DCS/Logs/c130-probe/175918-005.txt" \
                              ../../Tests/Fixtures/Cni/carp-init-1.json
"""

import json
import re
import sys
from pathlib import Path

SEPARATOR = "-" * 41
INDICATOR = re.compile(r"^indicator (\d+):")


def parse(text):
    """Element name/value pairs from the raw section, in discovery order."""
    lines = text.splitlines()

    try:
        start = lines.index("--- raw ---") + 1
    except ValueError:
        raise SystemExit("no raw section: not a probe dump")

    blocks, name, value = [], None, []
    for line in lines[start:]:
        if line.startswith(SEPARATOR):
            if name is not None:
                blocks.append((name, "\n".join(value).rstrip("\n")))
            name, value = None, []
            continue
        if name is None:
            name = line
        else:
            value.append(line)
    if name is not None:
        blocks.append((name, "\n".join(value).rstrip("\n")))

    return blocks


def indicator_index(text):
    for line in text.splitlines():
        m = INDICATOR.match(line)
        if m:
            return int(m.group(1))
    return None


def main(argv):
    if len(argv) != 3:
        print(__doc__.strip())
        return 2

    source, target = Path(argv[1]), Path(argv[2])
    text = source.read_text(encoding="utf-8", errors="replace")

    pairs = parse(text)
    if not pairs:
        raise SystemExit(f"{source.name}: no elements")

    title = next((v for k, v in pairs if k == "cni_title"), None)

    fixture = {
        "aircraft": "C-130J-30",
        "cni": {
            "blocks": [{"k": k, "n": i, "v": v} for i, (k, v) in enumerate(pairs, start=1)],
            "idx": indicator_index(text),
            "n": len(pairs),
            # The probe is documented as pilot-seat reconnaissance and reads only that CNI.
            "seat": "pilot",
            "title": title,
        },
        "ver": 2,
    }

    target.write_text(json.dumps(fixture, ensure_ascii=False), encoding="utf-8")
    print(f"{target.name}: {len(pairs)} blocs, titre {title!r}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
