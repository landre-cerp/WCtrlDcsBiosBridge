# CH-47F


## Single vs Multiple CDUs

The application automatically detects how many CDUs are connected and adjusts behavior accordingly:

### Single CDU Setup
If you only have **1 CDU connected**, the application will:
- Display a single **"Ch-47F"** option in the aircraft selection menu (no PLT/CPLT choice)
- Automatically switch the CDU display between pilot and co-pilot based on your seat position in DCS
- Monitor the DCS seat position to seamlessly update the CDU display

**YOU ABSOLUTELY NEED TO INSTALL DCS BIOS VERSION DCS-BIOS Nightly 2025-09-21 and Later**
as the seat position is not handled in previous versions.

### Multiple CDU Setup  
If you have **2 or more CDUs connected**, the application will:
- Display both **"Ch-47F (PLT)"** and **"Ch-47F (CPLT)"** options in the aircraft selection menu
- Require you to manually select the role for each CDU
- Keep each CDU fixed to its selected role (pilot or co-pilot) without automatic switching

No configuration is needed - the behavior is entirely automatic based on detection at startup.


## CDU Brightness 

DIM / BRT buttons are now working and they impact the brightness of the display, like in DCS.
Values are stored, so when you use 1 CDU and switch from pilot to copilot, brightness is kept.
Same for the key and LED backlight, controlled by the Knobs CDU1 and CDU2.


If you don't want light management at all, and leave it to SimAppPro for example, tick the Global option that says that 🙂

<img width="2103" height="1242" alt="image" src="https://github.com/user-attachments/assets/2ff01622-d4da-43ef-87ec-fac9aa7bdb22" />

