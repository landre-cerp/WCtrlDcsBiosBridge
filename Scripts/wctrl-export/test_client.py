#!/usr/bin/env python3
"""UDP test client for wctrl-export.lua — listens on port 31090 and pretty-prints incoming JSON.

The CDNU section reports the codepoint of every character outside printable ASCII. The
cockpit font maps low control codes (JSON.lua escapes those as \\u00XX, so they survive
the wire intact) onto symbols like arrows and the degree sign, and those are exactly the
characters a naive dump would hide. The trailing census lists each one with its count, to
build the remap table the CDU font needs.
"""

import json
import socket
import datetime

PORT = 31090


def fmt_environment(env):
    if not env:
        return "  (none)"
    lines = []
    if "temperature_c" in env:
        lines.append(f"  Temp      {env['temperature_c']} C")
    if "pressure_hpa" in env:
        lines.append(f"  Pressure  {env['pressure_hpa']} hPa  /  {env.get('pressure_inhg', '?')} inHg")
    if "wind_direction_deg" in env:
        lines.append(f"  Wind      from {env['wind_direction_deg']} deg  at {env.get('wind_speed_kts', '?')} kts")
    return "\n".join(lines)


def fmt_position(pos):
    if not pos:
        return "  (none)"
    return (
        f"  Lat={pos.get('lat', '?')}  Lon={pos.get('lon', '?')}  "
        f"Alt={pos.get('alt_ft', '?')} ft"
    )


def row_bytes(row):
    """Recover the row's original bytes (surrogateescape survives undecodable input)."""
    return row.encode("utf-8", "surrogateescape")


def is_plain(ch):
    """Printable ASCII. Anything else is a glyph the cockpit font renders specially."""
    return 0x20 <= ord(ch) < 0x7F


def fmt_cdnu(rows):
    if not rows:
        return "  (none)"

    out = []
    census = {}

    for i, row in enumerate(rows):
        masked = "".join(c if is_plain(c) else "~" for c in row)
        out.append(f"  [{i}] |{masked}| len={len(row)}")

        special = [(j, c) for j, c in enumerate(row) if not is_plain(c)]
        if special:
            detail = "  ".join(f"col{j}=U+{ord(c):04X}" for j, c in special)
            out.append(f"       {detail}")
            for _, c in special:
                census[ord(c)] = census.get(ord(c), 0) + 1

    if census:
        summary = "  ".join(
            f"U+{cp:04X}x{n}" for cp, n in sorted(census.items())
        )
        out.append(f"  codepoints in use: {summary}")

    return "\n".join(out)


def print_packet(data):
    ts = datetime.datetime.now().strftime("%H:%M:%S")
    print(f"\n{'='*60}  {ts}")
    print(f"ver={data.get('ver', '?')}  aircraft={data.get('aircraft', '?')}")

    print("ENVIRONMENT")
    print(fmt_environment(data.get("environment")))

    print("POSITION")
    print(fmt_position(data.get("position")))

    if "cdnu" in data:
        print("CDNU")
        print(fmt_cdnu(data.get("cdnu")))


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", PORT))
    print(f"Listening on UDP port {PORT} — waiting for DCS data...")

    while True:
        raw, addr = sock.recvfrom(65535)
        try:
            # surrogateescape keeps bytes that are not valid UTF-8 instead of
            # replacing them, so row_bytes() can reconstruct the original datagram bytes.
            data = json.loads(raw.decode("utf-8", "surrogateescape"))
            print_packet(data)
        except json.JSONDecodeError as e:
            print(f"[PARSE ERROR] {e}")
            print(f"  raw: {raw[:200]}")


if __name__ == "__main__":
    main()
