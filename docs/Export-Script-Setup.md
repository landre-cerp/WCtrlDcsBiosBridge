# DCS export script (Lua) — installation

A small Lua script running inside DCS reads data the bridge cannot get any other way and
sends it over UDP to the app. Three aircraft use that feed — see
[What uses the feed](#what-uses-the-feed) below.

> The script and the app talk a versioned protocol. Take both from the **same release** —
> a mismatched script is ignored outright and no data reaches the app.

## What uses the feed

| Aircraft | What the script provides | Without it |
| --- | --- | --- |
| F-14B(U) | The whole CDNU — DCS-BIOS carries no module for this variant | The page stays empty |
| C-130J | The whole CNI-MU — same reason | The page stays empty |
| A-10C | Live wind and field elevation, pre-filled on the TAKEOFF performance page | The pages work; you type wind and elevation in by hand |

The A-10C only needs it as a convenience, and only if you use the performance pages
(**Enable Performance Pages** under the A-10C options). Values you type over are left alone —
the live feed never overwrites a manual entry.

## 1. Download

Grab `wctrl-export-scripts-<version>.zip` from the
[latest release](https://github.com/landre-cerp/WCtrlDcsBiosBridge/releases/latest) — the same
release you took the application from.

## 2. Extract

The zip contains a `Scripts\wctrl-export\` folder. Extract it into your DCS saved games root:

```
%USERPROFILE%\Saved Games\DCS\
```

so that you end up with:

```
%USERPROFILE%\Saved Games\DCS\Scripts\wctrl-export\wctrl-export.lua
%USERPROFILE%\Saved Games\DCS\Scripts\wctrl-export\wctrl-export-config.lua
```

> If you have relocated your Saved Games folder (right-click it > Properties > Location),
> extract there instead. Multiplayer/beta installs use their own folder (`DCS.openbeta`) —
> install into the one you actually fly.

The script must sit at exactly that path: it loads its configuration with a fixed module
name, so renaming the folder breaks it.

## 3. Chain it from Export.lua

Open (or create) `%USERPROFILE%\Saved Games\DCS\Scripts\Export.lua` and **add** this line:

```lua
dofile(lfs.writedir() .. [[Scripts\wctrl-export\wctrl-export.lua]])
```

Add it, do not replace what is already there — that file is usually shared with DCS-BIOS
and SRS, and overwriting it breaks them.

## 4. Turn it on in the app

Under **GENERAL**, tick **Use DCS live data export**, then restart the bridge.

## Running DCS on another PC (optional)

The bridge listens on **UDP 31090 on every interface**, and that port is fixed in the
application — there is no setting for it. Leave `port` alone in
`Scripts\wctrl-export\wctrl-export-config.lua`: a different value there sends the feed
somewhere nothing is listening, and the page stays empty.

The **address** is the one value worth touching. It defaults to `127.0.0.1`, which assumes
DCS and the bridge run on the same machine. If DCS runs on another PC, point it at the
machine running the bridge:

```lua
local WctrlExportConfig = {udp_config = {address = "192.168.1.42", port = 31090}}
```

Windows Firewall on the bridge machine has to let that inbound UDP through, and the app
must already be running before DCS starts sending.

## If nothing shows up

- Check `%USERPROFILE%\Saved Games\DCS\Logs\wctrl-export.log` — the script writes there on
  every start. No file at all means DCS never loaded it (step 3 was applied to the wrong
  install, or `Export.lua` has a syntax error earlier in the file).
- Check `dcs.log` for Lua errors mentioning `wctrl-export`.
- Make sure the script and the app come from the same release.

## Per-aircraft notes

- [F-14B(U) CDNU](F-14BU-Setup.md)
- [C-130J CNI-MU](C-130J-Setup.md)
