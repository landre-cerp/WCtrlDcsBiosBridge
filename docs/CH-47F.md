# CH-47F Chinook

Basic support: the pilot or co-pilot CDU pages are mirrored to the device. No front panel
feed. The CH-47F is the only aircraft with a **seat choice** (see below).

## Supported devices

| Device | What it shows |
|--------|---------------|
| CDU (MCDU / PFP3N / PFP7 / PFP4) | Pilot or co-pilot CDU pages |

**Requires DCS-BIOS v0.11.0 or later** 
**(Nightly build 2026-6-29 recommended)** for the CH-47F CDU pages and
seat-position support.

## Single vs Multiple CDUs

The application automatically detects how many CDUs are connected and adjusts behavior accordingly:

### Single CDU Setup
If you only have **1 CDU connected**, the application will:
- Start automatically as soon as the CH-47F is detected (no seat prompt)
- Automatically switch the CDU display between pilot and co-pilot based on your seat position in DCS
- Monitor the DCS seat position to seamlessly update the CDU display

**YOU ABSOLUTELY NEED TO INSTALL DCS BIOS VERSION v0.11.0 or later (v0.11.4 recommended)**
as the seat position is not handled in previous versions.

### Multiple CDU Setup  
If you have **2 or more CDUs connected**, the application will:
- Show a **"Select your seat"** screen on each CDU once the CH-47F is detected — press the line-select key next to **PILOT** or **COPILOT**
- Assign the opposite seat to the remaining CDU(s) automatically as soon as one CDU picks a seat
- Keep each CDU fixed to its selected role (pilot or co-pilot) without automatic switching

No configuration is needed - the behavior is entirely automatic based on detection at startup.


## CDU Brightness 

DIM / BRT buttons are now working and they impact the brightness of the display, like in DCS.
Values are stored, so when you use 1 CDU and switch from pilot to copilot, brightness is kept.
Same for the key and LED backlight, controlled by the Knobs CDU1 and CDU2.


If you don't want light management at all, and leave it to SimAppPro for example, tick the Global option that says that 🙂

<img width="2103" height="1242" alt="image" src="https://github.com/user-attachments/assets/2ff01622-d4da-43ef-87ec-fac9aa7bdb22" />

