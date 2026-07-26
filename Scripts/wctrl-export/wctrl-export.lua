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
local CDNU_INDICATOR = 27
local CDNU_LINE_COUNT = 8

local function read_cdnu()
    local blocks = parse_indication(CDNU_INDICATOR)
    local block_count = blocks[0] or 0

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
