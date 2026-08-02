# C-130J CNI-MU page schema

`list_indication` gives element names and values, nothing else. No position, no font size, no
inversion — and the names are random GUIDs regenerated every session. Everything needed to
lay a page out therefore has to come from the module's own page scripts, read offline.

`extract.lua` replays each page script under stubbed cockpit globals and records what the
sim would build: `Add()` order, geometry, static values, controllers, font size and inverted
material. `validate.py` then checks that output against real captures from
`Scripts/c130-probe`, because a layout derived outside DCS is a hypothesis until the sim
agrees with it.

## Running

```bash
"D:/Program Files/Eagle Dynamics/DCS World/bin/luae.exe" extract.lua > c130j-cni-pages.json
```

DCS's own interpreter, not a system Lua: the module targets 5.1 and this keeps the dialect
identical. Override the install path with `DCS_PATH` if it differs.

```bash
python validate.py c130j-cni-pages.json "D:/Saved Games/DCS/Logs/c130-probe/session-1"
```

Nothing in the DCS installation is written. `SHOULD_MAKE_LAYOUTS` in the module's
`definitions.lua` stays `false` — turning it on would make DCS dump layout files into
Program Files and dirty the module for integrity checks.

## Output

One entry per page, `slots` in `Add()` order:

| field | meaning |
|---|---|
| `line` / `col` / `anchor` | grid position, derived from `init_pos` against `cni_lines_` and the font advance. Columns are the module's own, 0 to 25 — the panel is told that grid rather than squeezed into the CDU's usual 24 |
| `value` | static text, when the element has one — these are the matcher's anchors |
| `fmt` | printf formats, a list when the page offers alternatives |
| `ctrl` | controller name, i.e. which sim value feeds the slot |
| `small` / `invert` | font size and inverted material, neither observable at runtime |
| `parent` | set on toggle children; the visible parent is what encodes on/off |

`slim.py` turns `ctrl` into the two keys the app reasons from: `ct`, the field with its state
tail removed, and `st` on the form the module draws when that field is selected.
`uhf1_power_on_on` and `uhf1_power_on_off` become one `ct` of `uhf1_power_on` with and without
`st`; a toggle's words inherit the `ct` of the container above them.

That is what lets the app recover a state the indication never carries. Elements sharing a `ct`
are drawn from one decision, so any of them that gives itself away settles the rest — the word
GUARD in COMM TUNE U1's IDENT column exists only while `uhf1_guard` is on, and its arrival puts
the toggle five lines below it on ON. Fields whose two forms read alike, `OFF/ON` chief among
them, give nothing away and are drawn plain.

Pages whose title is built from a format rather than a literal carry `titleRegex` instead of
`title`. 25 of them do this, `"INAV%d CTRL SOLN"` and `"%sLZ %2d INIT"` among them.

`slots` is always larger than the block count the sim emits: the schema holds every variant
of every field, the sim sends only the visible ones. Reconciling the two is the matcher's job.

## Known wrinkles

`comm_tune_1.lua` iterates `els[0..5]` and asks for `cni_lines_[(5*2)+3]` = 13, one past the
end of a table that stops at 12. The extractor extends the table by one line using the
module's own 11→12 spacing, otherwise the page truncates and COMM TUNE INDEX is lost. The sim
renders the page in full, so it tolerates this somehow; the mechanism is not understood.

Three title families cannot be told apart by title, counter or slot count: `ARRIVAL RWYS`,
`ARRIVAL STARS` and `STAR REVIEW`. This is harmless — their layouts are identical down to the
column, differing only in controller names (`rte_dest` vs `rte_origin`), and rendering never
consults those. Picking any member draws the same screen.

## Re-run when the module updates

The schema is derived from files that ship with the aircraft, so a module patch can
invalidate it silently. Re-run both steps and check that `validate.py` still reports every
captured page at 100%.
