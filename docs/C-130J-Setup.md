# C-130J CNI-MU — in a hurry

The C-130J is not carried by DCS-BIOS, so its CNI-MU pages come from the same export script
the F-14B(U) uses, reading the display straight from the cockpit and sending it over UDP.
Three steps, and if you already set up the F-14B(U) the first two are done.

> The script and the app talk a versioned protocol. Take both from the **same release** —
> a mismatched script is ignored and the page stays empty.

## 1. Install the export script

Extract the `wctrl-export-scripts-<version>.zip` asset of the release (the folder also ships
inside the application zip) into your DCS saved games, so you end up with:

```
%USERPROFILE%\Saved Games\DCS\Scripts\wctrl-export\
```

Full details, including a DCS running on another PC and where to look when nothing shows up:
[Lua export script setup](Export-Script-Setup.md).

## 2. Chain it from Export.lua

Open (or create) `%USERPROFILE%\Saved Games\DCS\Scripts\Export.lua` and **add** this line:

```lua
dofile(lfs.writedir() .. [[Scripts\wctrl-export\wctrl-export.lua]])
```

Add it, do not replace what is already there — that file is usually shared with DCS-BIOS
and SRS, and overwriting it breaks them.

## 3. Turn it on

In the app, under **GENERAL**, tick **Use DCS live data export**, then restart the bridge.
Load the C-130J in DCS and the CNI-MU appears on the CDU.

## What the C-130J shows

**The CNI-MU, and nothing else.** DCS-BIOS has no module for this aircraft, so while it is
loaded no C-130J data reaches the bridge: no gear, no clock, no switches. The augmented crew's
display is not carried either.

Pages are recognised by the title they draw, so the CDU follows whatever the CNI is showing. A
page the app does not recognise leaves the screen as it was rather than drawing it against the
wrong layout.

## Pilot and copilot

Both CNIs are read and sent. Which one reaches a CDU depends on how many are plugged in:

- **One CDU** — the pilot's CNI.
- **Two or more CDUs** — each one asks which seat it is, on the same screen the CH-47F uses.
  Pick **PILOT** or **COPILOT** on the first, and the other takes the seat you did not pick.

A CDU keeps the seat it was given for as long as the aircraft is loaded.

**It does not follow you when you change seat**, and cannot: pressing 1 or 2 in this aircraft
does not move you. Pilot and copilot share one camera point in the module's `Views-30.lua` —
the two view modes differ only in shoulder size and whether the view can turn a full circle —
and you reach the other station by leaning across, within a 6DOF box either seat already
allows. Nothing in the cockpit changes, so nothing can be read: no cockpit parameter moves,
and there is no camera position to compare. Each CNI is its own device with its own buttons,
which is why the module never needs to know where you are sitting either.

## What the highlight can and cannot follow

In the cockpit the selected item of a group is drawn inverted and large — the radio that is
powered, the transponder mode in use, the alignment source on POWER UP. The CDU shows that as
black on green, and the size follows it, because on the module they are one decision.

It follows the cockpit on:

- the message on the bottom line, which has no plain form to be confused with
- the word GUARD in COMM TUNE's IDENT column, drawn only while the guard is on — and so, by
  its absence, off
- fields whose two states spell themselves differently: ADF against `ADF/ `, ON against OUT
- **radio power**, read from the radio device rather than from the display
- the boxed INAV digit in the top-left corner, read from the PFD's active solution
- **the selected column of a table**, read off the order the sim sends it in. A row is built as
  every highlighted cell followed by every plain one, so LANDING DATA sends the flap headers as
  `100 0 50` when 100 is selected — and no reading that takes all three as plain can put `100`
  ahead of the two behind it. One flap setting is selected for the page, so the header row
  answers for the three rows of speeds below it, which have no literals of their own

Each of those answers is then kept against the element that gave it, for as long as the sim
keeps issuing that element — the identifiers are random, but they hold steady within a session.
That is what makes the answers travel:

- **past the frame.** The radio can only be paired with a page while that page prints the
  frequency it is tuned to. Once PWR's two elements are told apart, the toggle keeps following
  the radio on the pages that print nothing to pair on.
- **to the fields beside them.** Exactly one position of a rotary is lit, so settling ADF is
  settling that the MN and BTH elements next to it are the plain ones — and the first turn away
  from ADF then says which of the two it went to.
- **from the crew's own switching.** Turn a rotary and two positions swap element while the
  rest hold; a position that held was not the lit one either side of the turn. That alone reads
  POWER UP's GPS/LAST/REF, where nothing outside the CNI moves at all.

Everything learned is dropped when the page is rebuilt, which the page's own fixed text gives
away: a literal cannot move with any state, so an element behind one that changed means fresh
identifiers for the whole page.

A pair the module marks by size instead of by highlight is drawn in the form the page shows
before anything has happened — full size for the soft-key labels that go small once used
(`MSTR AV ON`, `AUTONAV`, `START`, `STOP`), small for the two NAV DB lines, whose loaded
database is the large one. Twenty-two pairs across the pages work that way.

What is still out of reach is a field whose two forms are identical and which nothing else on
the aircraft moves with. `OFF/ON` reads the same either way, and the sim sends the words without
saying which it drew. SQL, TONE and **the whole of IFF 2/3** are in that group: the module's
`add_cni_toggle` builds one container per position holding the same two words, at the same
places, in the same order, differing only in `el.material` — and material is not in the
indication. Nothing distinguishes the two but the identifier, which is random. Of the 460
highlightable fields across the 145 pages, the great majority have nothing of their own to give
away; how many end up settled depends on what the flight has shown so far.

Nothing is guessed. A highlight on the wrong half of a rotary is worse than none, because the
highlight is the whole of what a rotary says. The text itself is always correct.

The aircraft's own devices were the last place it could have come from, and they have now been
swept. Fourteen of the ninety-seven answer a reader, and every one of them is a stock ED
interface: `get_frequency`/`get_modulation`/`is_on` on the radios, `get_altitude`/`get_mode` on
the radar altimeters, `get_bank`/`get_pitch`/`get_sideslip` on the gyro, the intercom's signal
levels. Not one of the module's own `herc::` classes exposes anything — `herc::Iff` (device 51)
and `MAIN_COMPUTER` (device 1) answer nothing at all, and the CNIs themselves (25, 26, 27) are
write-only down to their opaque `link`. So IFF 2/3 and POWER UP's alignment source cannot be
read, by us or by anyone else scraping this aircraft, and the request to ASC is the whole of
what is left.

Fixed element identifiers from the module's authors, or a page indicator, would settle all of
it at once, and none of the above would be needed. The DCS-BIOS project has asked for them
([issue 1469](https://github.com/DCS-Skunkworks/dcs-bios/issues/1469)).

## If the page stays empty

It reads `WAITING FOR CNI DATA` when the socket is up but nothing is arriving. Check that
step 2 was applied to the `Export.lua` of the DCS install you actually fly, and that DCS has
been restarted since.

`CNI SCHEMA MISSING` means `Resources\c130j-cni-pages.json` did not ship alongside the exe —
reinstall rather than copying files around.

`Scripts\wctrl-export\test_client.py` prints the raw feed on UDP 31090, which tells you
whether DCS is sending before you start suspecting the bridge. The app's log reports a
protocol version mismatch if the script is older than the app, and names each page as it is
recognised.

## When the module updates

The page layouts are derived from files that ship with the aircraft, so a module patch can
invalidate them. `tools/cni-schema` regenerates and re-checks them against captured traffic;
its README has the two commands.
