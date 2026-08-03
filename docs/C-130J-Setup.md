# C-130J CNI-MU — in a hurry

The C-130J is not carried by DCS-BIOS, so its CNI-MU pages come from the same export script
the F-14B(U) uses, reading the display straight from the cockpit and sending it over UDP.
Three steps, and if you already set up the F-14B(U) the first two are done.

> The script and the app talk a versioned protocol. Take both from the **same release** —
> a mismatched script is ignored and the page stays empty.

## 1. Install the export script

Copy the `Scripts\wctrl-export` folder from the release zip into your DCS saved games:

```
%USERPROFILE%\Saved Games\DCS\Scripts\wctrl-export\
```

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

Everywhere else it shows nothing. Of the 460 highlightable fields across the 145 pages, 455
cannot be resolved: `OFF/ON` reads the same either way, and the sim sends the words without
saying which it drew. SQL, TONE, ADF's MN and BTH positions, and every field on the IFF page
are in that group.

Nothing is guessed. A highlight on the wrong half of a rotary is worse than none, because the
highlight is the whole of what a rotary says. The text itself is always correct.

Fixed element identifiers from the module's authors, or a page indicator, would settle all of
it at once. The DCS-BIOS project has asked for them.

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
