## F-14B(U) CDNU — in a hurry

The CDNU is **not** carried by DCS-BIOS, so this release adds a small export script that
reads it straight from the cockpit and sends it to the bridge over UDP. Four steps.

> The script and the app talk a versioned protocol. Take both from **this same release** —
> a mismatched script is ignored and the CDNU page stays empty.

### 1. Install the export script

Copy the `Scripts\wctrl-export` folder from this zip into your DCS saved games:

```
%USERPROFILE%\Saved Games\DCS\Scripts\wctrl-export\
```

### 2. Chain it from Export.lua

Open (or create) `%USERPROFILE%\Saved Games\DCS\Scripts\Export.lua` and **add** this line:

```lua
dofile(lfs.writedir() .. [[Scripts\wctrl-export\wctrl-export.lua]])
```

Add it, do not replace what is already there — that file is usually shared with DCS-BIOS
and SRS, and overwriting it breaks them.

### 3. Teach DCS-BIOS about the F-14B(U)

DCS-BIOS does not support this variant, so **no** F-14 data reaches the bridge until you
add its name to two files under `Saved Games\DCS\Scripts\DCS-BIOS\`. Without this you get
the CDNU but no gear lights, no clock, and no RIO or radio page.

In `lib\AircraftList.lua`, next to the other F-14 entries:

```lua
add("F-14BU", true)
```

In `lib\modules\aircraft_modules\F-14.lua`, add `"F-14BU"` to the module's aircraft list:

```lua
local F_14 = Module:new("F-14", 0x1200, { "F-14B", "F-14A-135-GR", "F-14A-135-GR-Early", "F-14BU" })
```

Both edits are local. **A DCS-BIOS update will wipe them** — keep a note and redo them.

### 4. Turn it on

In the app, under **GENERAL**, tick **Use DCS live data export**. Restart the bridge.

In the cockpit, press the CDNU key on your CDU — **DATA** by default, changeable in the
F-14B section of the options. The RIO and radio pages keep their own keys.

### If the page stays empty

It reads `WAITING FOR CDNU DATA` when the socket is up but nothing is arriving: check that
step 2 was applied to the right `Export.lua`, and that DCS has been restarted since.

`Scripts\wctrl-export\test_client.py` prints the raw feed on UDP 31090 — useful to confirm
DCS is sending before blaming the bridge. The app's log reports a protocol version mismatch
if the script is older than the app.

### Also in this release

Live wind and field elevation now pre-fill the A-10C takeoff performance page when the same
export script is installed. Values you type over are left alone.
