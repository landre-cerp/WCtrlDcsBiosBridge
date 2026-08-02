#!/usr/bin/env python3
"""Check the extracted CNI schema against real list_indication captures.

The extractor replays the module's page scripts outside DCS, so its output is a hypothesis
until it is confronted with what the sim actually emits. This compares, per captured page,
the static strings the schema predicts against the ones the probe recorded.

    python validate.py c130j-cni-pages.json "D:/Saved Games/DCS/Logs/c130-probe/session-1"
"""

import json
import re
import sys
from collections import Counter
from pathlib import Path

PARSED = re.compile(r'^\[\s*\d+\] "(.*)"$')


def load_dump(path):
    """Return (title, [values]) from one probe dump."""
    values, in_parsed = [], False
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("--- parsed"):
            in_parsed = True
            continue
        if line.startswith("--- raw"):
            break
        if in_parsed:
            m = PARSED.match(line)
            if m:
                values.append(m.group(1).replace('\\"', '"').replace("\\\\", "\\"))

    title = None
    raw = path.read_text(encoding="utf-8", errors="replace")
    m = re.search(r"^cni_title\n(.*)$", raw, re.M)
    if m:
        title = m.group(1)
    return title, values


def statics(page):
    """Static (controller-free) strings the page always draws."""
    return [s["value"] for s in page["slots"]
            if s.get("value") and not s.get("ctrl")]


def main():
    schema_path, dump_dir = sys.argv[1], Path(sys.argv[2])
    schema = json.loads(Path(schema_path).read_text(encoding="utf-8"))

    by_title = {}
    dynamic = []
    for p in schema["pages"]:
        if p.get("title"):
            by_title.setdefault(p["title"], []).append(p)
        elif p.get("titleRegex"):
            for rx in p["titleRegex"]:
                dynamic.append((re.compile(rx), p))

    def candidates_for(title):
        """Literal titles first; 25 pages build theirs from a printf format instead."""
        if title in by_title:
            return by_title[title], False
        hits = [p for rx, p in dynamic if rx.match(title)]
        return hits, True

    # The one-shot sweeps every indicator and has a different layout; it is reconnaissance,
    # not a single-page capture, so it is not a schema test.
    dumps = [d for d in sorted(dump_dir.glob("*.txt")) if "000-all" not in d.name]
    matched = unmatched = 0
    verdicts = Counter()

    print(f"schema: {len(schema['pages'])} pages, {len(by_title)} titres distincts")
    print(f"dumps : {len(dumps)}\n")
    print(f"{'dump':<22} {'titre':<20} {'page schema':<22} statiques trouves")
    print("-" * 92)

    for d in dumps:
        title, values = load_dump(d)
        if not title:
            continue
        candidates, via_regex = candidates_for(title)
        if not candidates:
            print(f"{d.name:<22} {title:<20} {'-- AUCUNE --':<22}")
            unmatched += 1
            verdicts["page absente du schema"] += 1
            continue

        matched += 1
        best, best_hits, best_total = None, -1, 0
        for c in candidates:
            st = statics(c)
            hits = sum(1 for s in st if s in values)
            if hits > best_hits:
                best, best_hits, best_total = c, hits, len(st)

        pct = (100 * best_hits / best_total) if best_total else 0
        flag = "" if pct >= 90 else ("  <-- ECART" if pct >= 50 else "  <-- GRAVE")
        amb = f" [{len(candidates)} cand]" if len(candidates) > 1 else ""
        amb += " [regex]" if via_regex else ""
        # Slot count vs runtime blocks: a schema that silently stopped short still scores
        # 100% on the statics it did capture, so the ratio is what exposes truncation.
        print(f"{d.name:<22} {title:<20} {best['name']:<22} "
              f"{best_hits}/{best_total} ({pct:.0f}%)  "
              f"slots={len(best['slots'])} blocs={len(values)}{amb}{flag}")

        verdicts["ok" if pct >= 90 else ("partiel" if pct >= 50 else "grave")] += 1

        if pct < 90:
            missing = [s for s in statics(best) if s not in values]
            if missing:
                print(f"{'':<22} manquants: {missing[:6]}")

    print("\n--- bilan ---")
    print(f"dumps apparies au schema : {matched}, sans page : {unmatched}")
    for k, v in verdicts.most_common():
        print(f"  {k:<24} {v}")

    dupes = {t: v for t, v in by_title.items() if len(v) > 1}
    if not dupes:
        return

    # Whether title+counter+slotcount is enough to name a page. Where it is not, the resolver
    # needs a static string unique to one member, so find one now rather than discovering the
    # gap as a page rendering as its neighbour.
    print(f"\n--- desambiguisation des {len(dupes)} titres ambigus ---")
    hard = 0
    for t, group in sorted(dupes.items(), key=lambda kv: -len(kv[1])):
        keys = [(p.get("counter"), len(p["slots"])) for p in group]
        if len(set(keys)) == len(group):
            continue

        hard += 1
        print(f"  {t!r} x{len(group)} : compteur+taille NE SUFFIT PAS")
        for p in group:
            others = set()
            for q in group:
                if q is not p:
                    others |= {s["value"] for s in q["slots"] if s.get("value")}
            uniq = sorted({s["value"] for s in p["slots"]
                           if s.get("value") and not s.get("ctrl")} - others)
            marker = uniq[0] if uniq else None
            print(f"      {p['name']:<26} compteur={p.get('counter')!r:8} "
                  f"slots={len(p['slots']):<4} discriminant={marker!r}")

    if hard == 0:
        print("  aucune : titre + compteur + nombre de slots suffit partout")


if __name__ == "__main__":
    main()
