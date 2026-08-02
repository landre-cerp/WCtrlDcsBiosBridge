#!/usr/bin/env python3
"""Reduce the extracted schema to what the app actually renders from.

extract.lua keeps everything useful for understanding a page — metre coordinates, element
kinds, controller names, format strings. None of that is read at runtime, and all of it would
ship inside the binary. This drops it, keeping only what places a block on the grid.

    python slim.py c130j-cni-pages.json ../../Resources/c130j-cni-pages.json
"""

import json
import re
import sys
from collections import defaultdict
from pathlib import Path

def controller_stem(ctrl):
    """Strip the on/off tail that distinguishes the variants of one field."""
    if not ctrl:
        return None
    return re.sub(r"_(on|off)(_(on|off))?$", "", ctrl)


# The module's states, and only those. Taking the last token of any controller as a state
# instead lumps together things that merely sit side by side: a waypoint row's columns are
# named ..._alt, ..._bearing, ..._dist, ..._name, and a page's rows ..._0 .. ..._5. Measured
# across the 145 pages, off/on accounts for 476 fields and on/out for 4; every other tail
# family is a set of neighbours rather than a set of states.
VARIANT_TAIL = re.compile(r"^(?P<head>.+?)_(?P<state>on|off|out)(?P<seat>_cp|_p)?$")


def variant_of(ctrl):
    """The field a controller drives, and whether it names the selected state of it.

    Keeping the field and the state apart is what lets the runtime work out which state the sim
    is in and then apply it to every element the field drives — the GUARD word in the IDENT
    column runs off uhf1_guard_on just as the toggle does, so seeing one settles the other.

    The state is not always last. INAV CTRL draws each solution source twice, once per seat, and
    names them nav_ctrl_1_gps1_sol_on_p against nav_ctrl_1_gps1_sol_off_cp — state in the middle,
    seat at the end. The seat stays in the field: pilot and copilot are two elements in two
    places, and folding them together pairs one seat's highlight against the other's plain form.
    """
    if not ctrl:
        return None, False
    parsed = VARIANT_TAIL.match(ctrl)
    if not parsed:
        return None, False
    return parsed.group("head") + (parsed.group("seat") or ""), parsed.group("state") == "on"


def effective_controllers(slots):
    """Each slot's controller, with a toggle's children inheriting their container's.

    A toggle draws its words inside a container and only the container names a controller;
    the words themselves carry nothing. Pushing it down means a child is as identifiable as
    anything else, which is what allows the pair on the screen to be swapped as a unit.
    """
    by_name = {s["name"]: s for s in slots}

    out = {}
    for s in slots:
        ctrl = s.get("ctrl")
        parent = s.get("parent")
        while not ctrl and parent:
            up = by_name.get(parent)
            if up is None:
                break
            ctrl = up.get("ctrl")
            parent = up.get("parent")
        out[s["n"]] = ctrl
    return out


ANCHORS = {"Left": 0, "Center": 1, "Right": 2}


def specificity(fmts):
    """How much a format constrains the text, as literal characters outside the conversions.

    "%s" accepts anything and says nothing; "%d/%s" insists on a slash and so tells a frequency
    apart from the identifier beside it. Without this weight the matcher treats both as equally
    good and puts the frequency on whichever slot comes first.
    """
    if not fmts:
        return 0
    best = 0
    for f in fmts:
        literal = re.sub(r"%[-+ #0]*\d*\.?\d*[diufFgGeExXocs]", "", f)
        best = max(best, min(3, len(literal)))
    return best


def variant_font(group):
    """Kept for reference: the font this pass used to force on both variants of a field.

    Collapsing the size here made every such field read as unselected, which was the least
    wrong answer while the app had no way to tell the variants apart. It now has one — the
    controller identity emitted below — so the size is left faithful and the decision moved to
    where the evidence is.
    """
    plain = [s for s in group if not s.get("invert")]
    source = plain[-1] if plain else group[-1]
    return 1 if source.get("small") else 0


def indication_order(slots):
    """Reorder Add() order into the order list_indication reports.

    The two differ wherever a toggle is involved. add_cni_toggle emits its two containers,
    then the slash between the words, then the four word variants — but the indication nests
    each container's visible children directly beneath it, so the slash arrives after them.
    Aligning wire order against Add() order therefore strands the slash with nowhere to go.
    Grouping children under their parent here makes the two sequences comparable again.
    """
    children = {}
    for s in slots:
        if s.get("parent"):
            children.setdefault(s["parent"], []).append(s)

    ordered, emitted = [], set()
    for s in slots:
        if s.get("parent"):
            continue
        ordered.append(s)
        emitted.add(s["n"])
        for child in children.get(s["name"], []):
            ordered.append(child)
            emitted.add(child["n"])

    # A child whose parent is not itself a top-level slot would otherwise vanish.
    ordered.extend(s for s in slots if s["n"] not in emitted)
    return ordered


def pair_variants(slots):
    """Bring the two variants of one field next to each other.

    A field that can be highlighted is built twice: once inverted, once not, at the same spot,
    and the sim draws whichever matches the state. told_landing_1.lua adds all three inverted
    speeds for a row, then all three plain ones — so the wire order for "153 135 126" is
    126 first, because it is the highlighted one and its slot comes earlier.

    Aligned against that order, the renderer puts each value in the column of whichever slot
    it happened to land on, and a row reads 126 153 135 instead of 153 135 126. Interleaving
    the variants makes one emitted value per field fall on the right field whichever variant
    the sim chose.
    """
    groups = defaultdict(list)
    for s in slots:
        stem = controller_stem(s.get("ctrl"))
        if stem:
            groups[(stem, s.get("line"), s.get("col"), s.get("anchor"))].append(s)

    followers, moved = defaultdict(list), set()
    for group in groups.values():
        if len(group) < 2:
            continue
        for f in group[1:]:
            followers[group[0]["n"]].append(f)
            moved.add(f["n"])

    ordered = []
    for s in slots:
        if s["n"] in moved:
            continue
        ordered.append(s)
        ordered.extend(followers.get(s["n"], []))
    return ordered


def slim_page(p):
    # A slot is a container when another slot names it as parent: that is the element a
    # toggle draws around its words, and at runtime it is the block that carries children.
    parents = {s["parent"] for s in p["slots"] if s.get("parent")}
    ctrl_of = effective_controllers(p["slots"])

    slots = []
    for s in indication_order(pair_variants(p["slots"])):
        out = {"n": s["n"], "a": ANCHORS.get(s.get("anchor", "Left"), 0)}

        # Two different keys on purpose. "ct" is a field the module builds in two states, and
        # only those may be read as evidence of a state. "cs" is a plain controller, kept solely
        # so a page's frequency readout can be tied to the radio whose power it is showing —
        # uhf1_chan and uhf1_power_on are the same radio, and neither end then needs a device
        # number. Putting both under one key conflates them: iff_code_ident names the IDENT
        # toggle and the code readout beside it, and the readout would settle the toggle.
        ctrl = ctrl_of.get(s["n"])
        field, selected = variant_of(ctrl)
        if field:
            out["ct"] = field
            if selected:
                out["st"] = 1
        elif ctrl:
            out["cs"] = ctrl

        # The page background carries no text but the sim still emits a block for it, so
        # dropping it here would shift every following slot by one against the wire order.
        # Keep it, marked so it is never drawn.
        if s.get("kind") == "ceTexPoly":
            out["off"] = 1
        if s.get("line") is not None:
            out["l"] = s["line"]
        if s.get("col") is not None:
            out["c"] = s["col"]
        if s.get("value") is not None:
            out["v"] = s["value"]
        if s.get("fmtRx"):
            out["f"] = s["fmtRx"]
            out["w"] = specificity(s.get("fmt"))
        if s.get("small"):
            out["s"] = 1
        if s.get("invert"):
            out["i"] = 1
        if s["name"] in parents:
            out["k"] = 1
        # The only two elements the sim names rather than assigning a fresh GUID. Recording
        # which slot they are lets the matcher pin them instead of inferring them.
        if s["name"] == "cni_title":
            out["nm"] = 1
        elif s["name"] == "cni_scratchpad":
            out["nm"] = 2
        # Off-grid elements would otherwise be placed on a line they do not belong to;
        # STARTUP positions its text in absolute coordinates far outside the page grid.
        if s.get("lineErr", 0) > 0.02:
            out["off"] = 1
        slots.append(out)

    page = {"id": p["id"], "name": p["name"], "slots": slots}
    if p.get("title"):
        page["title"] = p["title"]
    if p.get("titleRegex"):
        page["rx"] = p["titleRegex"]
    if p.get("counter"):
        page["counter"] = p["counter"]
    return page


def main():
    src, dst = Path(sys.argv[1]), Path(sys.argv[2])
    full = json.loads(src.read_text(encoding="utf-8"))

    out = {
        "columns": full["columns"],
        "lines": full["lines"],
        "pages": [slim_page(p) for p in full["pages"]],
    }

    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(json.dumps(out, separators=(",", ":"), ensure_ascii=False),
                   encoding="utf-8")

    print(f"{src.name}: {src.stat().st_size:>8,} octets, "
          f"{sum(len(p['slots']) for p in full['pages'])} slots")
    print(f"{dst.name}: {dst.stat().st_size:>8,} octets, "
          f"{sum(len(p['slots']) for p in out['pages'])} slots")


if __name__ == "__main__":
    main()
