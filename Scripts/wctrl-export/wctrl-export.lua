package.path = package.path .. ";" .. lfs.currentdir() .. "/LuaSocket/?.lua"
package.cpath = package.cpath .. ";" .. lfs.currentdir() .. "/LuaSocket/?.dll"
package.path = lfs.writedir() .. "?.lua;" .. package.path

local M = {}
local WctrlExportConfig = require("Scripts.wctrl-export.wctrl-export-config")
local socket = require("socket") --[[@as Socket]]
local json = loadfile("Scripts\\JSON.lua")()
local udp = nil

local PROTOCOL_VERSION = 6
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
-- Standalone parser for an indicator's indication text, so display scraping does
-- not require DCS-BIOS to have a module registered for the aircraft.
-- ---------------------------------------------------------------------------

local INDICATION_SPLIT = "-----------------------------------------"
local CHILDREN_START = "children are {"
local CHILDREN_END = "}"

--- Parses an indication, keeping the nesting.
---
--- The nesting is what the C-130J needs: its toggles draw a container element whose visible
--- children are the words, so flattening "children are {" into siblings would lose which
--- container was emitted — and that is the only thing saying which word is highlighted.
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
-- C-130J CNI-MU
--
-- Every page names its title element "cni_title", so the anchor is an exact key rather than a
-- substring, and it sits on the indicator that actually holds the page. Indicators 8, 9 and 10
-- are pilot, copilot and augmented crew — device_init.lua registers them in that order — so
-- scanning upwards lands on the pilot's, then the copilot's.
--
-- Both are read every tick and each keeps its own change detection, but only one page goes out
-- per packet: a page runs to a few kilobytes and only one crew member is usually touching
-- theirs, so alternating between the seats that have news costs nothing and keeps the packet
-- the size it always was.
--
-- Blocks go out as found, values and structure only. Position, font size and inversion are
-- not in the indication at all — those come from the offline page schema on the app side.
-- ---------------------------------------------------------------------------

-- Sized for the indicator list rather than for any one display: the scan walks indicators
-- from zero looking for the page, and stops well short of the device numbers further up.
local SCAN_LIMIT = 60

-- Nothing to find while the screen is dark, and scanning every indicator is not free, so a
-- failed resolve retries on a timer rather than on every tick.
local RESOLVE_EVERY = 50            -- update ticks between attempts (~5 s at 10 Hz)

local CNI_AIRCRAFT   = "C-130J-30"
local CNI_ANCHOR     = "cni_title"
local CNI_MIN_BLOCKS = 6            -- a lit page never has fewer; a dark one is skipped
local CNI_HEARTBEAT  = 2.0          -- seconds between resends of an unchanged page
local CNI_MAX_BYTES  = 8000

-- Index is the scan order the indicators come back in, and the value is what the app calls the
-- seat. A page carries no marking of its own, so this name is the only thing that says which
-- CDU it belongs on.
local CNI_SEATS = { "pilot", "copilot" }

local cni_indicators  = {}
local cni_resolved    = false
local cni_resolve_wait = 0
local cni_next_seat   = 1
local cni_oversize_logged = false

-- Per seat: the page signature last sent, when it went, and the page content last seen.
local cni_state = { {}, {} }

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

--- The pilot's CNI, and the copilot's when it is the very next indicator.
---
--- Adjacency is a guard rather than the lookup: the three CNIs are registered consecutively, so
--- a second match that is not the first plus one is the augmented crew's, found because the
--- copilot's screen was dark at that moment. Showing that as the copilot would be wrong, and
--- carrying on with the pilot's alone is what this did before there was a second seat.
local function resolve_cni()
    local found = {}

    for id = 0, SCAN_LIMIT do
        local ok, nodes, total = pcall(parse_indication_tree, id)
        if ok and looks_like_cni(nodes, total or 0) then
            found[#found + 1] = id
            log(string.format("CNI at indicator %d (%d blocks)", id, total))
            if #found == #CNI_SEATS then break end
        end
    end

    if #found == 0 then return false end

    if found[2] and found[2] ~= found[1] + 1 then
        log(string.format("CNI at indicator %d is not the copilot's, ignoring it", found[2]),
            "WARN")
        table.remove(found, 2)
    end

    cni_indicators = found
    cni_resolved = true
    return true
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
-- another is a failure this repo has already been bitten by once.
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

--- One seat's page, or nil when neither seat has news.
---
--- Every resolved seat is read each tick so each keeps its own change detection, and the
--- alternating start makes sure a busy seat cannot starve the other.
local function read_cni(model_time, radios)
    if not cni_resolved then
        -- Nothing to find while the CNI is unpowered, and scanning every indicator is not
        -- free, so retry on a timer rather than on every tick.
        if cni_resolve_wait > 0 then
            cni_resolve_wait = cni_resolve_wait - 1
            return nil
        end
        cni_resolve_wait = RESOLVE_EVERY

        if not resolve_cni() then return nil end
    end

    -- Deliberately not re-checking the title anchor here. Finding an indicator needs it,
    -- staying on it must not: the BASIC page names its title element with a GUID like any
    -- other, so demanding cni_title every read would abandon a perfectly good indicator
    -- the moment that page came up. Emptiness is the only signal worth reacting to.
    local lit, turned, count = {}, false, 0
    for i = 1, #cni_indicators do
        local nodes, total = parse_indication_tree(cni_indicators[i])
        total = total or 0

        if total >= CNI_MIN_BLOCKS then
            count = count + 1
            local content = cni_signature(nodes)
            lit[i] = { nodes = nodes, total = total }
            if content ~= cni_state[i].content then turned = true end
            cni_state[i].content = content
        end
    end

    -- One seat going dark is that screen switched off; all of them is the aircraft gone, and
    -- the indicator list is rebuilt per aircraft.
    if count == 0 then
        cni_resolved = false
        cni_indicators = {}
        return nil
    end

    -- The page changing is exactly when the ship solution is worth re-reading: a SHIP SOLN
    -- switch reaches us as a swapped element on that page and nothing else. Everywhere else
    -- the cached value stands and the PFD goes untouched.
    local soln = read_ship_solution(model_time, turned)

    -- The radios are folded in because a power switch changes what the page means without
    -- changing a single element on it: the highlight moves from ON to OFF and the indication
    -- reads the same either way.
    local tail = "\3" .. radio_signature(radios) .. "\4" .. tostring(soln)

    for step = 0, #cni_indicators - 1 do
        local seat  = (cni_next_seat + step - 1) % #cni_indicators + 1
        local page  = lit[seat]
        local state = cni_state[seat]

        if page then
            local signature = state.content .. tail
            local due = model_time - (state.sent or 0) >= CNI_HEARTBEAT

            if signature ~= state.sig or due then
                state.sig  = signature
                state.sent = model_time
                cni_next_seat = seat % #cni_indicators + 1

                local title = find_named(page.nodes, CNI_ANCHOR)

                return {
                    seat   = CNI_SEATS[seat],
                    idx    = cni_indicators[seat],
                    title  = title and title.v or "",
                    n      = page.total,
                    blocks = page.nodes,
                    radios = radios,
                    soln   = soln,
                }
            end
        end
    end

    return nil
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

        -- Aircraft-specific: DCS-BIOS carries the C-130J but not its CNI-MU display, so the
        -- pages are scraped here rather than read from the DCS-BIOS stream.
        if data.aircraft == CNI_AIRCRAFT then
            data.cni = read_cni(model_time, read_radios())
        end

        if udp then
            local payload = json:encode(data)

            -- A CNI page is up to sixty blocks carrying GUID keys, far and away the largest
            -- thing in the packet. Drop the page rather than the whole packet, so position
            -- and environment keep flowing.
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
    cni_indicators, cni_resolved, cni_resolve_wait = {}, false, 0
    cni_next_seat = 1
    cni_state = { {}, {} }
    radio_ids = nil
    soln_value, soln_next = nil, 0
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
