-- C-130J CNI-MU probe. Read-only reconnaissance for the live export: works out whether
-- list_indication() reaches the CNI at all, which indicator holds it, and what the raw
-- indication looks like. Sends nothing, touches no other script.
--
-- Install:  add this line to D:\Saved Games\DCS\Scripts\Export.lua, alongside the ones
--           already there (wwt, DCS-BIOS, wctrl-export, SRS) — do not replace any of them:
--
--               dofile(lfs.writedir() .. [[Scripts\c130-probe\c130-probe.lua]])
--
--           Fly the C-130J-30 hot, pilot seat, and walk the CNI through COMM TUNE,
--           INDEX, LEGS, TOLD and PROGRESS. Remove the line again afterwards.
--
-- Output goes to Logs/c130-probe/ :
--     probe.log             narrative, starting with the resolved writedir
--     SSS-000-all.txt       one shot over every indicator, plus list_cockpit_params
--     SSS-NNN.txt           the watched indicator, written whenever its content changes
--     SSS-t-NNN.txt         the watched indicator on a fixed cadence
--
-- SSS is a per-DCS-process stamp. Restarting DCS builds a fresh Lua state with the counters
-- back at zero, so without it a second session silently overwrites the first one's early
-- dumps — which is exactly what cost the comparison the GUID stability question needs.
--
-- The two dump families answer different questions. Without dump-t-*, an indicator that
-- returns text once and then freezes is indistinguishable from one that works: both leave
-- a single dump-NNN behind. Comparing a frozen dump-t series against a visibly changing
-- cockpit is what exposes it.
--
-- The raw indication is kept unparsed on purpose. The parser in wctrl-export.lua collapses
-- "children are {" nesting and the ----- separators, and the CNI nests: indicator.lua sets
-- parent_element on its line elements, and so does add_cni_toggle.

local OUT_DIR       = lfs.writedir() .. "Logs/c130-probe/"
local LOG_PATH      = OUT_DIR .. "probe.log"
local SCAN_LIMIT    = 49
local CNI_ANCHOR    = "cni_title"
local CNI_FALLBACK  = 8            -- CNI_MU/init_pilot.lua per Cockpit/Scripts/device_init.lua
local POLL_INTERVAL = 0.25
local PERIODIC_EVERY = 15.0
local SETTLE        = 1.0
local MAX_DUMPS     = 100
local ALL_SNAPSHOTS = 12         -- full sweeps kept per session; ~110 kB each
local RAW_CAP       = 6000         -- per indicator, in the one-shot only; HDD pages are large

local INDICATION_SPLIT = "-----------------------------------------"
local CHILDREN_START   = "children are {"
local CHILDREN_END     = "}"

local SESSION = os.date("%H%M%S")

pcall(lfs.mkdir, OUT_DIR)

local function plog(msg)
    local f = io.open(LOG_PATH, "a")
    if f then
        f:write(os.date("[%Y-%m-%d %H:%M:%S] ") .. tostring(msg) .. "\n")
        f:close()
    end
end

local function write_file(name, text)
    name = SESSION .. "-" .. name
    local f = io.open(OUT_DIR .. name, "w")
    if not f then
        plog("ERROR: cannot write " .. name)
        return false
    end
    f:write(text)
    f:close()
    return true
end

-- Same parser as wctrl-export.lua, kept local so the probe stands alone. Used only to find
-- the anchor and to summarise; the dumps themselves stay raw.
local function parse_indication(indicator_id)
    local ret = {}
    local indication = list_indication(indicator_id)

    if not indication or indication == "" then
        ret[0] = 0
        return ret
    end

    local state, key, block_lines, total = {}, nil, {}, 0

    local function current_state()
        return #state > 0 and state[#state] or "none"
    end

    local function flush_block()
        if not key then return end
        local value = ""
        while #block_lines > 0 do
            if value ~= "" then value = value .. "\n" end
            value = value .. table.remove(block_lines, 1)
        end
        ret[key] = value
        total = total + 1
        ret[total] = value
        key = nil
    end

    if indication:sub(-1) ~= "\n" then indication = indication .. "\n" end

    for line in string.gmatch(indication, "([^\n]*)\n") do
        if line == INDICATION_SPLIT then
            if current_state() ~= "item" then
                table.insert(state, "item")
            else
                flush_block()
            end
        elseif line == CHILDREN_START then
            if current_state() == "item" then table.remove(state) end
            table.insert(state, "child")
            flush_block()
        elseif line == CHILDREN_END and #block_lines > 0 and #state > 0 then
            if current_state() == "item" then
                flush_block()
                table.remove(state)
            end
            if current_state() == "child" then table.remove(state) end
        elseif line and current_state() == "item" then
            if key then
                table.insert(block_lines, line)
            else
                key = line
            end
        end
    end

    if current_state() == "item" then
        flush_block()
        table.remove(state)
    end

    ret[0] = total
    return ret
end

local function raw_of(id)
    local ok, raw = pcall(list_indication, id)
    if ok and type(raw) == "string" then return raw end
    return ""
end

local function block_names(blocks)
    local names = {}
    for k in pairs(blocks) do
        if type(k) == "string" then names[#names + 1] = k end
    end
    table.sort(names)
    return names
end

local function has_anchor(blocks)
    return blocks[CNI_ANCHOR] ~= nil
end

-- The digit in the CNI's top-left corner is boxed on one crew station and plain on the other,
-- and the two forms are the same character in the same place — nothing in the CNI's own
-- indication tells them apart. SHIP SOLN on the NAV SELECT page is what moves the box, so the
-- question is whether anything outside the CNI reports it. The PFDs carry a value reading
-- "INAV_1", which is the only candidate seen so far.
--
-- Logged beside every dump rather than captured on its own, so one session of switching SHIP
-- SOLN back and forth answers it: if the label follows the box, it is the witness.
local PFD_IDS = { 0, 2 }

local function nav_solution()
    for _, id in ipairs(PFD_IDS) do
        local token = raw_of(id):match("INAV_%d")
        if token then return string.format("%s(ind %d)", token, id) end
    end
    return "aucun"
end

--- Every lone digit on the page, with the element that drew it.
---
--- All of them rather than the first: parse_indication keys blocks by name and by ordinal both,
--- pairs() walks that in no particular order, and more than one field can be a single digit.
--- Which one is the corner is then obvious from the element staying put across dumps.
local function lone_digits(id)
    local ok, blocks = pcall(parse_indication, id)
    if not ok or type(blocks) ~= "table" then return "illisible" end

    local found = {}
    for name, value in pairs(blocks) do
        if type(name) == "string" and type(value) == "string" and value:match("^%d$") then
            found[#found + 1] = string.format("%s=%s", name:sub(1, 10), value)
        end
    end

    table.sort(found)
    return #found > 0 and table.concat(found, " ") or "aucun"
end

-- Which indicators moved since the last look.
--
-- The general form of the question that keeps coming up: the cockpit shows a state the CNI's
-- own indication cannot express, and the only way forward is finding something else that moves
-- with it. SHIP SOLN was answered that way — the PFD names the active INAV — but only after
-- guessing which indicator to read. This guesses nothing: flip a switch, and the log names every
-- indicator whose content changed.
--
-- Full strings kept and compared rather than hashed. Lua compares length first and then memcmp
-- in C, so this is cheaper than any checksum written in Lua, and a checksum cheap enough to
-- compute here would be exactly the kind that misses a swapped GUID of the same length.
local last_raw = {}
local aircraft_name = "?"

local function moved_indicators()
    local moved = {}

    for id = 0, SCAN_LIMIT do
        local raw = raw_of(id)
        if raw ~= "" then
            if last_raw[id] and last_raw[id] ~= raw then
                moved[#moved + 1] = string.format("%d(%d->%d o)", id, #last_raw[id], #raw)
            end
            last_raw[id] = raw
        end
    end

    return #moved > 0 and table.concat(moved, " ") or "aucun"
end

-- Content signature: the whole string.
--
-- It used to be the length plus the first and last 64 characters, on the grounds that it caught
-- every change seen so far. It does not. Switching SHIP SOLN swaps the element that draws the
-- corner digit for its other form — same character, same place, same total length, and the GUID
-- that differs sits in the middle. Three of those went past unnoticed and were only caught by
-- the periodic dumps, which is precisely the change this probe exists to record. Four kilobytes
-- copied four times a second is not worth the blind spot.
local function signature(raw)
    return raw
end

local function describe(id, raw)
    local blocks = parse_indication(id)
    local count = blocks[0] or 0
    local out = {}

    out[#out + 1] = string.format("indicator %d: %d blocks, %d raw chars", id, count, #raw)
    out[#out + 1] = "names: " .. table.concat(block_names(blocks), " | ")
    out[#out + 1] = ""
    out[#out + 1] = "--- parsed, in discovery order ---"
    for i = 1, count do
        out[#out + 1] = string.format("[%2d] %q", i, tostring(blocks[i]))
    end
    out[#out + 1] = ""
    out[#out + 1] = "--- raw ---"
    out[#out + 1] = raw

    return table.concat(out, "\n")
end

-- ---------------------------------------------------------------------------

local started        = false
local watch_id       = nil
local dump_count     = 0
local periodic_count = 0
local last_sig       = nil
local pending_sig    = nil
local pending_since  = 0
local next_poll      = 0
local next_periodic  = 0

--- @param name string|nil  target file, defaulting to the once-per-session snapshot
local function one_shot(aircraft, name)
    local out = {}
    out[#out + 1] = "aircraft: " .. tostring(aircraft)
    out[#out + 1] = "writedir: " .. lfs.writedir()
    out[#out + 1] = "model time: " .. tostring(LoGetModelTime())
    out[#out + 1] = ""

    local found = {}

    for id = 0, SCAN_LIMIT do
        local raw = raw_of(id)
        if raw ~= "" and raw ~= "\n" then
            local blocks = parse_indication(id)
            local count = blocks[0] or 0
            local anchored = has_anchor(blocks)

            found[#found + 1] = string.format("%2d: %3d blocks, %6d chars%s",
                id, count, #raw, anchored and "   <-- has " .. CNI_ANCHOR or "")

            out[#out + 1] = "==========================================================="
            out[#out + 1] = string.format("indicator %d: %d blocks, %d raw chars%s",
                id, count, #raw, anchored and "   <-- has " .. CNI_ANCHOR or "")
            out[#out + 1] = "names: " .. table.concat(block_names(blocks), " | ")
            out[#out + 1] = ""
            if #raw > RAW_CAP then
                out[#out + 1] = raw:sub(1, RAW_CAP)
                out[#out + 1] = string.format("\n... [truncated, %d chars total]", #raw)
            else
                out[#out + 1] = raw
            end
            out[#out + 1] = ""
        end
    end

    out[#out + 1] = "==========================================================="
    out[#out + 1] = "--- list_cockpit_params ---"
    local ok, params = pcall(list_cockpit_params)
    out[#out + 1] = (ok and type(params) == "string") and params or "unavailable"

    write_file(name or "000-all.txt", table.concat(out, "\n"))

    plog("indicators with content:")
    for _, line in ipairs(found) do plog("   " .. line) end

    return found
end

local function resolve_watch()
    for id = 0, SCAN_LIMIT do
        local ok, blocks = pcall(parse_indication, id)
        if ok and blocks and has_anchor(blocks) then
            plog(string.format("watching indicator %d (%s found, %d blocks)",
                id, CNI_ANCHOR, blocks[0] or 0))
            return id
        end
    end

    -- No anchor anywhere. That is itself a result — either the CNI is dark, or the parser
    -- never runs for it. Watch the index device_init.lua points at so the periodic dumps
    -- can tell those two apart.
    plog(string.format("%s not found on any indicator 0..%d; falling back to %d",
        CNI_ANCHOR, SCAN_LIMIT, CNI_FALLBACK))
    return CNI_FALLBACK
end

local function start()
    local self_data = LoGetSelfData()
    if not self_data then return end

    started = true

    plog("=== c130-probe start ===")
    plog("writedir: " .. lfs.writedir())
    plog("aircraft: " .. tostring(self_data.Name))

    -- Loose match, not the exact "C-130J-30" from Entry/C_130J_30.lua: confirming that name
    -- is one of the things this probe is for. Anything else is another aircraft in the same
    -- session, so log the name and stay out of the way.
    local name = tostring(self_data.Name)
    if not name:find("130", 1, true) then
        plog("not a C-130J, probe idle for this mission")
        watch_id = nil
        return
    end

    aircraft_name = name
    one_shot(name)
    watch_id = resolve_watch()

    local raw = raw_of(watch_id)
    last_sig = signature(raw)
    dump_count = dump_count + 1
    write_file(string.format("%03d.txt", dump_count), describe(watch_id, raw))
    plog(string.format("%s-%03d.txt written (%d chars) | chiffres: %s | nav: %s",
        SESSION, dump_count, #raw, lone_digits(watch_id), nav_solution()))
end

local function poll()
    local now = LoGetModelTime() or 0
    if now < next_poll then return end
    next_poll = now + POLL_INTERVAL

    if not started then
        start()
        if started then next_periodic = now + PERIODIC_EVERY end
        return
    end

    if not watch_id then return end

    local raw = raw_of(watch_id)
    local sig = signature(raw)

    -- Settle before writing: a page in transit produces several intermediate states, and
    -- one file per keystroke would burn through MAX_DUMPS before reaching a real page.
    if sig ~= last_sig then
        if sig ~= pending_sig then
            pending_sig = sig
            pending_since = now
        elseif now - pending_since >= SETTLE and dump_count < MAX_DUMPS then
            last_sig = sig
            pending_sig = nil
            dump_count = dump_count + 1
            write_file(string.format("%03d.txt", dump_count), describe(watch_id, raw))
            plog(string.format("%s-%03d.txt written (%d chars) | chiffres: %s | nav: %s",
                SESSION, dump_count, #raw, lone_digits(watch_id), nav_solution()))
        end
    else
        pending_sig = nil
    end

    if now >= next_periodic and periodic_count < MAX_DUMPS then
        next_periodic = now + PERIODIC_EVERY
        periodic_count = periodic_count + 1
        write_file(string.format("t-%03d.txt", periodic_count), describe(watch_id, raw))
        plog(string.format("%s-t-%03d.txt written | chiffres: %s | nav: %s",
            SESSION, periodic_count, lone_digits(watch_id), nav_solution()))
        plog("   indicateurs modifies: " .. moved_indicators())

        -- A full snapshot per tick, not just a flag saying something moved. Half the panel
        -- ticks on its own — clocks, attitude, engine gauges — so "indicator 14 changed" is
        -- noise. What answers the question is the content either side of the crew flipping
        -- something: only that shows which element carries the state.
        if periodic_count <= ALL_SNAPSHOTS then
            one_shot(aircraft_name, string.format("t-%03d-all.txt", periodic_count))
        end
    end
end

local _prev_frame = LuaExportAfterNextFrame
local _prev_stop  = LuaExportStop

function LuaExportAfterNextFrame()
    pcall(poll)
    if _prev_frame then _prev_frame() end
end

function LuaExportStop()
    plog(string.format("=== c130-probe stop (%d change dumps, %d periodic) ===",
        dump_count, periodic_count))

    -- Re-resolve on the next mission. The indicator list is rebuilt per aircraft, so a
    -- watch_id carried over from the previous one would point at something else entirely.
    -- The dump counters deliberately keep running, so a second mission appends rather than
    -- overwriting the first one's files.
    started, watch_id, last_sig, pending_sig = false, nil, nil, nil
    next_poll, next_periodic = 0, 0

    if _prev_stop then _prev_stop() end
end
