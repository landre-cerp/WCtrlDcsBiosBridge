-- Asks the C-130J's CNI device what it can do.
--
-- The indication says which text is on screen but not which field is highlighted: a field is
-- built twice, inverted and plain, and only one is ever drawn — same value, same position,
-- nothing to tell them apart. The aircraft obviously knows which, so this checks the last
-- cheap place that knowledge could surface: the device object itself.
--
-- list_cockpit_params() has already been ruled out — 160 values, none of them CNI state.
--
-- Install:  add this line to D:\Saved Games\DCS\Scripts\Export.lua, fly the C-130J once, then
--           remove it again:
--
--               dofile(lfs.writedir() .. [[Scripts\c130-probe\device-probe.lua]])
--
-- Writes Logs/c130-probe/devices.log and stops. It calls nothing on the device beyond reading
-- its method table: probing a cockpit device by invoking things at random can throw switches.

local LOG = lfs.writedir() .. "Logs/c130-probe/devices.log"

-- Cockpit/Scripts/devices.lua assigns these in file order; the CNIs are the 25th, 26th and
-- 27th. Scanned rather than assumed, because an id that is right on one install and wrong on
-- another is the failure the CDNU work already ran into.
local CNI_IDS = { 25, 26, 27 }
local SCAN_LIMIT = 100

local function log(msg)
    local f = io.open(LOG, "a")
    if f then
        f:write(os.date("[%Y-%m-%d %H:%M:%S] ") .. tostring(msg) .. "\n")
        f:close()
    end
end

local function describe(value)
    local t = type(value)
    if t == "function" then return "function" end
    if t == "table" or t == "userdata" then return t end
    return t .. " = " .. tostring(value)
end

--- Every name reachable on the object, following the metatable chain.
local function members(obj)
    local seen, names = {}, {}

    local function collect(tbl, depth)
        if type(tbl) ~= "table" or depth > 4 then return end
        for k, v in pairs(tbl) do
            if type(k) == "string" and not seen[k] then
                seen[k] = true
                names[#names + 1] = string.format("%-32s %s", k, describe(v))
            end
        end
        local mt = getmetatable(tbl)
        if type(mt) == "table" then
            collect(rawget(mt, "__index"), depth + 1)
            collect(mt, depth + 1)
        end
    end

    if type(obj) == "table" then
        collect(obj, 0)
    end

    local mt = getmetatable(obj)
    if type(mt) == "table" then
        collect(rawget(mt, "__index"), 0)
        collect(mt, 0)
    end

    table.sort(names)
    return names
end

local done = false

local function probe()
    if done then return end
    local self_data = LoGetSelfData()
    if not self_data then return end

    done = true

    log("=== device probe: " .. tostring(self_data.Name) .. " ===")

    -- Which ids answer at all, so a shifted device table shows up rather than reading as
    -- "the CNI has nothing".
    local live = {}
    for id = 0, SCAN_LIMIT do
        local ok, dev = pcall(GetDevice, id)
        if ok and dev ~= nil and type(dev) ~= "boolean" then
            live[#live + 1] = id
        end
    end
    log("devices repondant: " .. table.concat(live, " "))

    for _, id in ipairs(CNI_IDS) do
        local ok, dev = pcall(GetDevice, id)
        log("")
        log(string.format("--- device %d: %s ---", id, ok and type(dev) or "erreur"))

        if ok and dev ~= nil then
            local names = members(dev)
            if #names == 0 then
                log("   aucun membre lisible (userdata opaque)")
            else
                for _, n in ipairs(names) do log("   " .. n) end
            end

            -- get_argument_value is the one read that is always safe and tells whether the
            -- device exposes any state at all through the standard interface.
            local ok2, value = pcall(function() return dev:get_argument_value(0) end)
            log("   get_argument_value(0): " .. (ok2 and tostring(value) or "indisponible"))
        end
    end

    log("=== fin ===")
end

local _prev_frame = LuaExportAfterNextFrame

function LuaExportAfterNextFrame()
    pcall(probe)
    if _prev_frame then _prev_frame() end
end
