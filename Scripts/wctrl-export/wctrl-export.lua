package.path = package.path .. ";" .. lfs.currentdir() .. "/LuaSocket/?.lua"
package.cpath = package.cpath .. ";" .. lfs.currentdir() .. "/LuaSocket/?.dll"
package.path = lfs.writedir() .. "?.lua;" .. package.path

local M = {}
local WctrlExportConfig = require("Scripts.wctrl-export.wctrl-export-config")
local socket = require("socket") --[[@as Socket]]
local json = loadfile("Scripts\\JSON.lua")()
local udp = nil

local PROTOCOL_VERSION = 1
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
        end

        if udp then
            udp:sendto(json:encode(data),
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
