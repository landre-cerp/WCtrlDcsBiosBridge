# LED mapping

Every LED on a Winctrl front panel or CDU can follow any DCS-BIOS indicator of the aircraft
you fly.

Most aircraft already light some of them — the AGP32 gear lights, the CDU's master caution.
Those are the **built-in** defaults, and you can see them: open the LEDs tab and each row
shows the control it already follows, greyed out. Bind something of your own and it takes over
that LED; clear the row and the built-in comes back. The FCU/EFIS and PAP3 legends start empty
on every aircraft, because nothing in an A-10C means "AP1" — those light only once you say
what they should follow.

A built-in can watch more than one control: the F-14B's gear warning lights when any of three
OFF lights does, and shows as *PLT_GEAR_L_OFF_L +2*. A few are worked out in code instead —
the M-2000C reads its annunciators out of a lighting register, so there is no control to name
and the row says *built-in, from PCN light register* instead. Either way you can bind over them.

## Binding a LED

1. Open the **LEDs** tab.
2. Pick the **aircraft** and the **panel**.
3. On the LED you want, start typing in the DCS-BIOS control box — by name (`MASTER_CAUTION`)
   or by description ("master caution") — and pick from the list.
4. Restart the bridge. Bindings are read when the aircraft's listener starts, so the aircraft
   currently flying is read-only and shows an *in use* badge.

The box lists the module's **indicator lamps** — that is 95 of the A-10C's controls, 130 of the
F-16C's. Tick **Show all integer outputs** to reach everything else: switch positions, selectors,
gauges. String outputs (CDU lines, counters) are never listed: there is no on/off to read from
them.

A lamp is simply on or off. When you bind something with more than two positions, an operator
and a value appear next to it, so you can say *lit when the gear lever is at least 1*.

## What wins

A binding you make replaces the built-in behaviour of that LED, and only that LED. Bind the
AGP32's `GEAR 1 DOWN` to something else and the other gear lights keep working as before.

One built-in can drive several LEDs: a single gear-in-transit warning lights the AGP32's three
red UNLK triangles and the lever's red arrow together, which is why those four rows all name
the same control.

Turning on **Disable Lighting Management** (for SimApp Pro users) switches off your own
bindings, so nothing this app does fights the software that owns your panel lighting. The
built-in defaults are unaffected by that setting, as they always have been.

## Panels and their LEDs

| Panel | LEDs |
|---|---|
| FCU / EFIS | LOC, AP1, AP2, A/THR, EXPED, APPR, and FD / LS / CSTR / WPT / VOR.D / NDB / ARPT on each side |
| PAP-3 | N1, SPEED, VNAV, LVL CHG, HDG SEL, LNAV, VOR LOC, APP, ALT HOLD, V/S, CMD A, CWS A, CMD B, CWS B, A/T ARM, FD (L), FD (R) |
| AGP-32 | the three gear DOWN and UNLK lights, GEAR DOWN (red), BRK FAN HOT / ON, the six AUTO BRK lights, TERR ON ND |
| MCDU / PFP | FAIL, FM1, FM2, FM, IND, RDY |

The PDC-3 has no LEDs of its own and is not listed.

The C-130J and the F-14B(U) cannot be configured: DCS-BIOS carries no module for either, so
there are no controls to bind. Everything those two show comes from the live data export.

## The file

Bindings live in `ledmappings.json`, next to the executable, apart from `useroptions.json` so
that a mapping can be shared as it stands — post the file, drop it in, restart.

```json
{
  "Version": 1,
  "Bindings": [
    {
      "Aircraft": "A-10C",
      "Device": "FcuEfis",
      "Led": "Ap1",
      "Control": "MASTER_CAUTION",
      "Op": "NotZero",
      "Value": 0
    }
  ]
}
```

- `Aircraft` — the name as the app shows it (`A-10C`, `F/A-18C`, …).
- `Device` — `FcuEfis`, `Pap3`, `Agp32` or `Mcdu`.
- `Led` — the LED id, as the editor lists it.
- `Op` — `NotZero` (the default), `Equals`, `AtLeast` or `AtMost`; `Value` is ignored by
  `NotZero`.

Only your own bindings are stored. The built-in defaults ship with the app, so an aircraft
whose defaults are fixed or extended in a later release picks them up without you touching
the file.

A file written for an older release, or for a module DCS-BIOS has since changed, still loads:
entries naming an aircraft, a LED or a control that no longer exists are skipped with a line in
`log.txt`, and everything else is applied.
