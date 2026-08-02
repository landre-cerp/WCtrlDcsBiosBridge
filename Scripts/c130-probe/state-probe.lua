-- Looks for the CNI's selection state anywhere the export environment can reach.
--
-- The indication reports only the elements the sim draws, and a field's two branches are built
-- identically — same words, same places — so which one is on screen shows up as nothing but a
-- fresh GUID. Measured across the module's 145 pages, 455 of the 460 highlightable fields are
-- undecidable from that alone. This checks the places the state could still be hiding.
--
-- Install: add this line to D:\Saved Games\DCS\Scripts\Export.lua, fly the C-130J, then take it
-- out again. It writes Logs/c130-probe/state.log once and stops.
--
--     dofile(lfs.writedir() .. [[Scripts\c130-probe\state-probe.lua]])
--
-- Nothing here actuates anything. Reads are restricted to an allowlist of names that can only
-- be getters; SetCommand and performClickableAction are never called, on any device, because on
-- a live cockpit device those throw switches.

local LOG        = lfs.writedir() .. "Logs/c130-probe/state.log"
local SCAN_LIMIT = 100
local CNI_IDS    = { 25, 26, 27 }

-- The subscription test mutates: registering a listener asks the engine to route commands to a
-- device that did not ask for them, and that routing outlives the probe. Off by default, and
-- worth a restart of the mission afterwards. See the note at the bottom for what it can and
-- cannot establish.
local TRY_SUBSCRIPTIONS = false

local function log(msg)
    local f = io.open(LOG, "a")
    if f then
        f:write(os.date("[%Y-%m-%d %H:%M:%S] ") .. tostring(msg) .. "\n")
        f:close()
    end
end

--- Names that cannot do anything but report. Anything outside this shape is listed but never
--- called: the device interface puts readers and actuators side by side on the same object.
local function is_reader(name)
    return name:match("^get_") ~= nil
        or name:match("^is_") ~= nil
        or name:match("^has_") ~= nil
        or name == "value"
        or name == "state"
end

local function describe(value)
    local t = type(value)
    if t == "function" or t == "table" or t == "userdata" then return t end
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
                names[#names + 1] = k
            end
        end
        local mt = getmetatable(tbl)
        if type(mt) == "table" then
            collect(rawget(mt, "__index"), depth + 1)
            collect(mt, depth + 1)
        end
    end

    if type(obj) == "table" then collect(obj, 0) end

    local mt = getmetatable(obj)
    if type(mt) == "table" then
        collect(rawget(mt, "__index"), 0)
        collect(mt, 0)
    end

    table.sort(names)
    return names
end

--- Calls one reader a few plausible ways and reports whatever comes back.
---
--- Argument shapes are guessed because the interface is undocumented: no argument for a plain
--- accessor, then a couple of small integers for the indexed ones, which is how
--- get_argument_value is shaped on the devices that have it.
local function try_reader(dev, name)
    local results = {}

    local attempts = {
        { label = "()",   call = function() return dev[name](dev) end },
        { label = "(0)",  call = function() return dev[name](dev, 0) end },
        { label = "(1)",  call = function() return dev[name](dev, 1) end },
    }

    for _, attempt in ipairs(attempts) do
        local ok, value = pcall(attempt.call)
        if ok and value ~= nil then
            results[#results + 1] = attempt.label .. " -> " .. describe(value)
        end
    end

    return results
end

--- Sweeps every device, not just the CNIs.
---
--- The earlier probe only opened 25, 26 and 27 and found them write-only. But the state the
--- display is showing belongs to the radios and the IFF as much as to the CNI — the CNI draws
--- what they tell it — so a getter on one of those would answer the question just as well, and
--- nobody has looked.
local function sweep()
    local found = 0

    for id = 0, SCAN_LIMIT do
        local ok, dev = pcall(GetDevice, id)
        if ok and dev ~= nil and type(dev) ~= "boolean" then
            local readers = {}
            for _, name in ipairs(members(dev)) do
                if is_reader(name) then readers[#readers + 1] = name end
            end

            if #readers > 0 then
                local lines = {}
                for _, name in ipairs(readers) do
                    for _, result in ipairs(try_reader(dev, name)) do
                        lines[#lines + 1] = "      " .. name .. result
                    end
                end

                if #lines > 0 then
                    found = found + 1
                    log(string.format("   device %d: %d lecteur(s)", id, #readers))
                    for _, l in ipairs(lines) do log(l) end
                end
            end
        end
    end

    if found == 0 then
        log("   aucun device n'expose de lecteur qui reponde")
    end
end

--- The `link` field on the CNI devices is userdata rather than a table, so the member walk
--- above passes straight over it. If it carries anything readable it is the last thing on these
--- objects that could.
local function inspect_links()
    for _, id in ipairs(CNI_IDS) do
        local ok, dev = pcall(GetDevice, id)
        if ok and type(dev) == "table" then
            local link = rawget(dev, "link")
            log(string.format("   device %d link: %s", id, type(link)))

            local names = members(link)
            if #names == 0 then
                log("      opaque, rien a lire")
            else
                for _, n in ipairs(names) do log("      " .. n) end
            end
        end
    end
end

--- Whether the device will even accept a subscription from here.
---
--- Worth stating what a success would and would not mean. Export.lua runs in its own Lua state;
--- GetDevice hands back a proxy that marshals calls into the cockpit state. A subscription needs
--- the traffic to come back the other way, and the device interface has no callback argument to
--- carry it — in the module's own code, listen_command only registers interest and the engine
--- then calls the device's SetCommand, which is a method of the real device and not of this
--- proxy. So the call being accepted would prove only that; the crew's clicks would still have
--- to find a way across. That is why this is off by default and last.
local function try_subscriptions()
    for _, id in ipairs(CNI_IDS) do
        local ok, dev = pcall(GetDevice, id)
        if ok and type(dev) == "table" then
            for _, name in ipairs({ "listen_command", "listen_event" }) do
                if dev[name] then
                    local accepted, err = pcall(function() return dev[name](dev, 3001) end)
                    log(string.format("   device %d %s(3001): %s", id, name,
                                      accepted and "accepte" or ("refuse - " .. tostring(err))))
                end
            end
        end
    end
end

local done = false

local function probe()
    if done then return end
    local self_data = LoGetSelfData()
    if not self_data then return end

    done = true

    log("=== state probe: " .. tostring(self_data.Name) .. " ===")

    log("")
    log("--- lecteurs sur tous les devices ---")
    sweep()

    log("")
    log("--- userdata link des CNI ---")
    inspect_links()

    log("")
    log("--- abonnements ---")
    if TRY_SUBSCRIPTIONS then
        try_subscriptions()
    else
        log("   non teste (TRY_SUBSCRIPTIONS = false)")
    end

    log("=== fin ===")
end

local _prev_frame = LuaExportAfterNextFrame

function LuaExportAfterNextFrame()
    pcall(probe)
    if _prev_frame then _prev_frame() end
end
