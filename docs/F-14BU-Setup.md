# F-14B(U) CDNU — in a hurry

The CDNU is not carried by DCS-BIOS, so this release adds a small export script that reads
it straight from the cockpit and sends it to the bridge over UDP. Three steps.

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
Load the F-14B(U) in DCS and the CDNU appears on the CDU.

## What the F-14B(U) shows

**The CDNU, and nothing else.** DCS-BIOS has no module for this variant, so while it is
loaded no F-14 data reaches the bridge at all: no gear lights, no clock, no RIO or radio
page. Those remain available on the plain F-14B, which DCS-BIOS does support.

## If the page stays empty

It reads `WAITING FOR CDNU DATA` when the socket is up but nothing is arriving. Check that
step 2 was applied to the `Export.lua` of the DCS install you actually fly, and that DCS
has been restarted since.

`Scripts\wctrl-export\test_client.py` prints the raw feed on UDP 31090, which tells you
whether DCS is sending before you start suspecting the bridge. The app's log reports a
protocol version mismatch if the script is older than the app.

## Also in this release

The same export script pre-fills live wind and field elevation on the A-10C takeoff
performance page. Values you type over are left alone. See
[Lua export script setup](Export-Script-Setup.md#what-uses-the-feed).
