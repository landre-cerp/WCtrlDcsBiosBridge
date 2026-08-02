package.path = package.path .. ";" .. lfs.currentdir() .. "/LuaSocket/?.lua"
package.cpath = package.cpath .. ";" .. lfs.currentdir() .. "/LuaSocket/?.dll"
package.path = lfs.writedir() .. "?.lua;" .. package.path

local M = {}
local WctrlExportConfig = require("Scripts.wctrl-export.wctrl-export-config")
local socket = require("socket") --[[@as Socket]]
local json = loadfile("Scripts\\JSON.lua")()
local udp = nil

local PROTOCOL_VERSION = 2
local SEND_INTERVAL = 0.1

local log_file_path = lfs.writedir() .. "Logs/wctrl-export.log"
local function log(msg, level)
    local f = io.open(log_file_path, "a")
    if f then
        f:write(os.date("[%Y-%m-%d %H:%M:%S] ") .. (level or "INFO") .. ": " .. tostring(msg) .. "\n")
        f:close()
    end
end

function M.init()
    log("Wctrl export: init")
    udp = socket.udp()
    udp:settimeout(0)
    log("Wctrl export: UDP socket ready")
end

local function round(v, decimals)
    local m = 10 ^ (decimals or 0)
    return math.floor(v * m + 0.5) / m
end

local function safe_call(fn)
    if type(fn) == "function" then
        local ok, result = pcall(fn)
        if ok then return result end
    end
end

-- ---------------------------------------------------------------------------
-- Indication parsing
--
-- Standalone port of DCS-BIOS' Module.parse_indication, so display scraping does
-- not require DCS-BIOS to have a module registered for the aircraft. Returns a
-- table keyed both by element name and by 1-based discovery order, with [0] set
-- to the number of blocks found.
-- ---------------------------------------------------------------------------

local INDICATION_SPLIT = "-----------------------------------------"
local CHILDREN_START = "children are {"
local CHILDREN_END = "}"

local function parse_indication(indicator_id)
    local ret = {}
    local indication = list_indication(indicator_id)

    if not indication or indication == "" then
        ret[0] = 0
        return ret
    end

    local state = {}
    local key = nil
    local block_lines = {}
    local total = 0

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

    if indication:sub(-1) ~= "\n" then
        indication = indication .. "\n"
    end

    for line in string.gmatch(indication, "([^\n]*)\n") do
        if line == INDICATION_SPLIT then
            if current_state() ~= "item" then
                table.insert(state, "item")
            else
                flush_block()
            end
        elseif line == CHILDREN_START then
            if current_state() == "item" then
                table.remove(state)
            end
            table.insert(state, "child")
            flush_block()
        elseif line == CHILDREN_END and #block_lines > 0 and #state > 0 then
            if current_state() == "item" then
                flush_block()
                table.remove(state)
            end
            if current_state() == "child" then
                table.remove(state)
            end
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

--- Same source text, but the nesting is kept.
---
--- parse_indication above flattens "children are {" into siblings, which is right for the
--- CDNU and wrong for the C-130J: its toggles draw a container element whose visible children
--- are the words, so collapsing them loses which container was emitted. Rather than change a
--- parser the F-14B(U) depends on, this is a second one.
---
--- Returns a list of { n, k, v, c } in document order, n counting every node including
--- children, plus the total.
local function parse_indication_tree(indicator_id)
    local indication = list_indication(indicator_id)
    if not indication or indication == "" then return nil, 0 end

    local root = {}
    local stack = { root }
    local node, value_lines, total = nil, nil, 0

    local function flush()
        if not node then return nil end

        total = total + 1
        node.n = total
        node.v = value_lines and table.concat(value_lines, "\n") or ""
        value_lines = nil

        local list = stack[#stack]
        list[#list + 1] = node

        local done = node
        node = nil
        return done
    end

    if indication:sub(-1) ~= "\n" then
        indication = indication .. "\n"
    end

    for line in string.gmatch(indication, "([^\n]*)\n") do
        if line == INDICATION_SPLIT then
            flush()
            node = {}
        elseif line == CHILDREN_START then
            -- The node just closed is the container; everything up to the matching brace
            -- belongs to it.
            local parent = flush()
            if parent then
                parent.c = {}
                stack[#stack + 1] = parent.c
            end
        elseif line == CHILDREN_END then
            flush()
            if #stack > 1 then table.remove(stack) end
        elseif node then
            if node.k == nil then
                node.k = line
            else
                value_lines = value_lines or {}
                value_lines[#value_lines + 1] = line
            end
        end
    end

    flush()
    return root, total
end

-- ---------------------------------------------------------------------------
-- F-14B(U) CDNU
--
-- The text rows live in the ccCDNUBake indicator, which the F-14B(U) appends to
-- the shared F-14 indicator list. parse_indication numbers every block it finds,
-- so the eight text rows sit behind the container and mask elements; the mask is
-- drawn conditionally, so the rows are taken from the end of the indication
-- rather than at a fixed offset.
-- ---------------------------------------------------------------------------

local CDNU_AIRCRAFT = "F-14BU"
local CDNU_LINE_COUNT = 8

-- There is no starting index. It differs between installs — 27 on one, 26 on another — and
-- a default that happens to be right on the developer's machine hides the search from the
-- only person able to test it.
--
-- The indication holding the rows is named only by GUIDs, so it cannot be recognised alone.
-- Its neighbour carries "cdnu_symbology_present", which gives an anchor that moves with it.
--
-- The anchor is not enough on its own. It exists with the CDNU dark, when the rows are gone
-- and the neighbour on the other side is the TID and its 95 blocks — picking on size would
-- lock onto that. So a candidate must also *look* like the CDNU: its last row is the
-- scratchpad, bracketed, on every page seen so far.
local CDNU_ANCHOR = "cdnu"
local CDNU_NEIGHBOURS = { -1, 1, -2, 2, 0 }
local SCAN_LIMIT = 60
local RESOLVE_EVERY = 50            -- update ticks between attempts (~5 s at 10 Hz)

local cdnu_indicator = nil
local cdnu_resolved = false
local resolve_wait = 0

local function has_anchor(blocks)
    for key in pairs(blocks) do
        if type(key) == "string" and key:lower():find(CDNU_ANCHOR, 1, true) then
            return true
        end
    end
    return false
end

--- Enough rows, and the last one is the bracketed scratchpad. Both are required: the count
--- alone matches several other displays, and the brackets alone would match nothing useful.
local function looks_like_cdnu(blocks)
    local count = blocks and blocks[0] or 0
    if count < CDNU_LINE_COUNT then return false end

    local last = blocks[count]
    return type(last) == "string" and last:find("%[") ~= nil and last:sub(-1) == "]"
end

--- Locates the rows by name, then confirms by shape. Returns nil rather than a guess: a
--- wrong indication that sticks is worse than none, because it looks like it works.
local function resolve_cdnu()
    local anchor
    for id = 0, SCAN_LIMIT do
        local ok, blocks = pcall(parse_indication, id)
        if ok and blocks and has_anchor(blocks) then
            anchor = id
            break
        end
    end

    if not anchor then return nil end

    for _, offset in ipairs(CDNU_NEIGHBOURS) do
        local id = anchor + offset
        if id >= 0 then
            local ok, blocks = pcall(parse_indication, id)
            if ok and looks_like_cdnu(blocks) then
                cdnu_indicator = id
                cdnu_resolved = true
                log(string.format("CDNU at indicator %d (%d blocks), anchored on %d",
                    id, blocks[0], anchor))
                return blocks
            end
        end
    end

    return nil
end

local function read_cdnu()
    local blocks, block_count

    if cdnu_resolved then
        blocks = parse_indication(cdnu_indicator)
        block_count = blocks[0] or 0
    else
        -- Nothing to find while the CDNU is dark, and scanning every indicator is not free,
        -- so retry on a timer rather than on every tick.
        if resolve_wait > 0 then
            resolve_wait = resolve_wait - 1
            return nil
        end
        resolve_wait = RESOLVE_EVERY

        blocks = resolve_cdnu()
        block_count = blocks and blocks[0] or 0
    end

    if block_count < CDNU_LINE_COUNT then
        return nil
    end

    local lines = {}
    for row = 1, CDNU_LINE_COUNT do
        lines[row] = blocks[block_count - CDNU_LINE_COUNT + row] or ""
    end

    return lines
end

-- ---------------------------------------------------------------------------
-- C-130J CNI-MU
--
-- Easier to find than the CDNU: every page names its title element "cni_title", so the anchor
-- is an exact key rather than a substring, and it sits on the indicator that actually holds
-- the page. Indicators 8, 9 and 10 are pilot, copilot and augmented crew; scanning upwards
-- lands on the pilot without needing to know that.
--
-- Blocks go out as found, values and structure only. Position, font size and inversion are
-- not in the indication at all — those come from the offline page schema on the app side.
-- ---------------------------------------------------------------------------

local CNI_AIRCRAFT   = "C-130J-30"
local CNI_ANCHOR     = "cni_title"
local CNI_MIN_BLOCKS = 6            -- a lit page never has fewer; a dark one is skipped
local CNI_HEARTBEAT  = 2.0          -- seconds between resends of an unchanged page
local CNI_MAX_BYTES  = 8000

local cni_indicator   = nil
local cni_resolved    = false
local cni_resolve_wait = 0
local cni_last_sig    = nil
local cni_last_sent   = 0
local cni_oversize_logged = false
local cni_last_content = nil

local function find_named(nodes, name)
    for _, node in ipairs(nodes or {}) do
        if node.k == name then return node end
    end
    return nil
end

--- Title element present and enough blocks to be a real page. add_scratch() is deliberately
--- not required: 20 of the module's pages do not have a scratchpad.
local function looks_like_cni(nodes, total)
    if not nodes or total < CNI_MIN_BLOCKS then return false end
    return find_named(nodes, CNI_ANCHOR) ~= nil
end

local function resolve_cni()
    for id = 0, SCAN_LIMIT do
        local ok, nodes, total = pcall(parse_indication_tree, id)
        if ok and looks_like_cni(nodes, total or 0) then
            cni_indicator = id
            cni_resolved = true
            log(string.format("CNI at indicator %d (%d blocks)", id, total))
            return nodes, total
        end
    end
    return nil, 0
end

--- Every value on the page, and which element carried it.
---
--- Sampling the ends instead was cheaper and wrong: a frequency edited in the middle of a
--- page changes neither the block count nor the first and last values, so the change went
--- unnoticed and the display only caught up on the next heartbeat two seconds later.
---
--- The names matter as much as the values. A field that can be highlighted is built twice and
--- the sim draws whichever matches its state, so selecting the other one swaps the element
--- while leaving every value on the page identical — "OFF/ON" reads the same either way. On
--- values alone that is no change at all and nothing was ever sent, which is why the highlight
--- sat still on the device while the crew clicked. The GUIDs are regenerated per session but
--- hold steady within one, so they cost nothing here and catch exactly this.
local function cni_signature(nodes)
    local parts = {}

    local function walk(list)
        for _, node in ipairs(list) do
            parts[#parts + 1] = node.k
            parts[#parts + 1] = node.v
            if node.c then walk(node.c) end
        end
    end

    walk(nodes or {})
    return table.concat(parts, "\1")
end

-- ---------------------------------------------------------------------------
-- Radio state
--
-- The CNI shows a radio's power as OFF/ON with one of the two words highlighted, and the
-- indication carries neither the highlight nor anything else that moves when it changes. The
-- radio devices, on the other hand, answer is_on() directly — and the OFF beside H1 and H2 on
-- COMM TUNE INDEX matches is_on() = false on devices 10 and 11, tuning and all.
--
-- The frequency goes out with it so the app can pair a device to a page without either end
-- hardcoding a device number: the page prints the frequency it is tuned to, and that is enough
-- to say which radio it is talking about. An id that is right on one install and wrong on
-- another is the failure the CDNU work already ran into.
--
-- Only get_frequency and is_on are ever called. SetCommand and performClickableAction sit on
-- the same objects and throw switches.
-- ---------------------------------------------------------------------------

-- Past SCAN_LIMIT, which is sized for the indicator list. The radios sit as high as device 92
-- on this aircraft.
local RADIO_SCAN_LIMIT = 100

local radio_ids = nil

local function resolve_radios()
    local found = {}

    for id = 0, RADIO_SCAN_LIMIT do
        local ok, dev = pcall(GetDevice, id)
        if ok and type(dev) == "table" and dev.get_frequency and dev.is_on then
            local readable = pcall(function() return dev:get_frequency() end)
            if readable then found[#found + 1] = id end
        end
    end

    log(string.format("radios: %s", table.concat(found, " ")))
    return found
end

local function read_radios()
    -- Only cached once something answered. Scanning before the avionics are up would otherwise
    -- settle on an empty list for the rest of the mission.
    if not radio_ids then
        local found = resolve_radios()
        if #found == 0 then return nil end
        radio_ids = found
    end

    local out = {}
    for _, id in ipairs(radio_ids) do
        local ok, dev = pcall(GetDevice, id)
        if ok and type(dev) == "table" then
            local got, freq = pcall(function() return dev:get_frequency() end)
            local got_on, on = pcall(function() return dev:is_on() end)
            if got and got_on and type(freq) == "number" then
                -- To the kilohertz: the raw reading drifts by a few hundred hertz between
                -- frames, which would make every packet look like a change.
                out[#out + 1] = { i = id, f = math.floor(freq / 1000 + 0.5), on = on and true or false }
            end
        end
    end

    return #out > 0 and out or nil
end

-- ---------------------------------------------------------------------------
-- Ship navigation solution
--
-- Every CNI page draws its own INAV number in the top-left corner, boxed when that INAV is the
-- one the aircraft has settled on and plain when it is not — which is why the pilot's 1 and the
-- copilot's 2 are never both boxed. The two forms are the same character in the same place, so
-- the CNI's own indication cannot tell them apart; switching SHIP SOLN on NAV SELECT swaps the
-- element and changes nothing else on the page.
--
-- The PFDs name the active solution outright, as a value reading "INAV_1". Reading it here is
-- what lets the app draw that corner correctly on all 145 pages.
--
-- Read on demand rather than on a timer. The PFD indication runs to 24 kB and the solution
-- changes about once a flight, so polling it at the send rate would be waste — but polling it
-- slowly costs worse than waste: the page itself changes the instant the crew switches, so the
-- packet goes out at once carrying a solution up to a whole interval old, and the corner draws
-- the wrong way round until the cache catches up. The caller therefore forces a read on exactly
-- the tick that matters, and the timer is left as a slow safety net.
-- ---------------------------------------------------------------------------

local PFD_IDS       = { 0, 2 }
local SOLN_INTERVAL = 5.0

local soln_value = nil
local soln_next  = 0

local function read_ship_solution(model_time, force)
    if not force and model_time < soln_next then return soln_value end
    soln_next = model_time + SOLN_INTERVAL

    for _, id in ipairs(PFD_IDS) do
        local ok, raw = pcall(list_indication, id)
        if ok and type(raw) == "string" then
            local digit = raw:match("INAV_(%d)")
            if digit then
                soln_value = tonumber(digit)
                return soln_value
            end
        end
    end

    soln_value = nil
    return nil
end

local function radio_signature(radios)
    if not radios then return "" end

    local parts = {}
    for _, r in ipairs(radios) do
        parts[#parts + 1] = string.format("%d:%d:%s", r.i, r.f, tostring(r.on))
    end
    return table.concat(parts, "\2")
end

local function read_cni(model_time, radios)
    local nodes, total

    if cni_resolved then
        nodes, total = parse_indication_tree(cni_indicator)
        total = total or 0

        -- Deliberately not re-checking the title anchor here. Finding the indicator needs it,
        -- staying on it must not: the BASIC page names its title element with a GUID like any
        -- other, so demanding cni_title every read would abandon a perfectly good indicator
        -- the moment that page came up. Emptiness is the only signal worth reacting to.
        if total < CNI_MIN_BLOCKS then
            cni_resolved = false
            cni_indicator = nil
            return nil
        end
    else
        -- Nothing to find while the CNI is unpowered, and scanning every indicator is not
        -- free, so retry on a timer rather than on every tick.
        if cni_resolve_wait > 0 then
            cni_resolve_wait = cni_resolve_wait - 1
            return nil
        end
        cni_resolve_wait = RESOLVE_EVERY

        nodes, total = resolve_cni()
        if not nodes then return nil end
    end

    -- The radios are folded in because a power switch changes what the page means without
    -- changing a single element on it: the highlight moves from ON to OFF and the indication
    -- reads the same either way.
    -- The page changing is exactly when the ship solution is worth re-reading: a SHIP SOLN
    -- switch reaches us as a swapped element on this page and nothing else. Everywhere else
    -- the cached value stands and the PFD goes untouched.
    local content = cni_signature(nodes)
    local turned  = content ~= cni_last_content
    cni_last_content = content

    local soln = read_ship_solution(model_time, turned)

    local signature = content .. "\3" .. radio_signature(radios) .. "\4" .. tostring(soln)
    if signature == cni_last_sig and (model_time - cni_last_sent) < CNI_HEARTBEAT then
        return nil
    end
    cni_last_sig = signature
    cni_last_sent = model_time

    local title = find_named(nodes, CNI_ANCHOR)

    return {
        seat   = "pilot",
        idx    = cni_indicator,
        title  = title and title.v or "",
        n      = total,
        blocks = nodes,
        radios = radios,
        soln   = soln,
    }
end

-- ---------------------------------------------------------------------------

local next_send = 0

function M.update()
    local model_time = safe_call(LoGetModelTime) or 0
    if model_time < next_send then return end
    next_send = model_time + SEND_INTERVAL

    local ok, err = pcall(function()
        local wind      = LoGetVectorWindVelocity()
        local atmo      = safe_call(LoGetAtmosphereParameters)
        local self_data = safe_call(LoGetSelfData)

        local data = { ver = PROTOCOL_VERSION }

        if self_data then
            data.aircraft = self_data.Name
        end

        -- Environment
        local env = {}

        if wind then
            local speed_kts = math.sqrt(wind.x ^ 2 + wind.z ^ 2) * 1.94384
            local dir = math.deg(math.atan2(-wind.x, -wind.z))
            if dir < 0 then dir = dir + 360 end
            env.wind_direction_deg = round(dir, 0)
            env.wind_speed_kts     = round(speed_kts, 1)
        end

        if atmo then
            env.temperature_c = round(atmo.temperature, 1)
            env.pressure_hpa  = round(atmo.pressure * 1.33322, 1)
            env.pressure_inhg = round(atmo.pressure / 25.4, 2)
        end

        data.environment = env

        -- Position
        if self_data and self_data.LatLongAlt then
            data.position = {
                lat    = round(self_data.LatLongAlt.Lat,  6),
                lon    = round(self_data.LatLongAlt.Long, 6),
                alt_ft = round(self_data.LatLongAlt.Alt * 3.28084, 0),
            }
        end

        -- Aircraft-specific: DCS-BIOS has no F-14B(U) module, so the CDNU rows
        -- are scraped here rather than read from the DCS-BIOS stream.
        if data.aircraft == CDNU_AIRCRAFT then
            data.cdnu = read_cdnu()
        elseif data.aircraft == CNI_AIRCRAFT then
            data.cni = read_cni(model_time, read_radios())
        end

        if udp then
            local payload = json:encode(data)

            -- The CDNU is eight fixed rows; a CNI page is up to sixty blocks carrying GUID
            -- keys, so this is the first payload big enough to be worth checking. Drop the
            -- page rather than the whole packet, so position and environment keep flowing.
            if #payload > CNI_MAX_BYTES and data.cni then
                if not cni_oversize_logged then
                    cni_oversize_logged = true
                    log(string.format("CNI payload %d bytes exceeds %d, dropping page",
                        #payload, CNI_MAX_BYTES), "WARN")
                end
                data.cni = nil
                payload = json:encode(data)
            end

            udp:sendto(payload,
                WctrlExportConfig.udp_config.address,
                WctrlExportConfig.udp_config.port)
        end
    end)

    if not ok then
        log("M.update error: " .. tostring(err), "ERROR")
    end
end

function M.stop()
    log("Wctrl export: stop")
    if udp then
        udp:close()
        udp = nil
    end
    next_send = 0

    -- The indicator list is rebuilt per aircraft, so a resolved index carried into the next
    -- mission would point at something else entirely.
    cni_indicator, cni_resolved, cni_resolve_wait = nil, false, 0
    cni_last_sig, cni_last_sent = nil, 0
    radio_ids = nil
    soln_value, soln_next = nil, 0
    cni_last_content = nil
end

-- Chain onto any previously installed export hooks (e.g. DCS-BIOS)
local _prev_start = LuaExportStart
local _prev_frame = LuaExportAfterNextFrame
local _prev_stop  = LuaExportStop

function LuaExportStart()
    M.init()
    if _prev_start then _prev_start() end
end

function LuaExportAfterNextFrame()
    M.update()
    if _prev_frame then _prev_frame() end
end

function LuaExportStop()
    M.stop()
    if _prev_stop then _prev_stop() end
end

return M
