-- Finds the cockpit animation argument behind a lamp, by watching which ones move.
--
-- The EXEC annunciator is the case this was written for. It is not in list_cockpit_params, it
-- has no entry in clickabledata beyond the key's own press animation, and mainpanel_init.lua
-- maps nothing. What is left is that any argument can be read whether or not anything declares
-- it, so rather than look the number up, this watches every argument and reports the ones that
-- change.
--
-- Install:  add this line to D:\Saved Games\DCS\Scripts\Export.lua, alongside the ones already
--           there — do not replace any of them:
--
--               dofile(lfs.writedir() .. [[Scripts\c130-probe\arg-probe.lua]])
--
--           Fly the C-130J-30 hot, pilot seat. Let the cockpit settle, then make the lamp you
--           are after change: for EXEC, type something into the CNI scratchpad that needs
--           executing, and press EXEC. Remove the line again afterwards.
--
-- Writes Logs/c130-probe/args.log: one baseline taken after the settling delay, then a line per
-- argument that moves, with its old and new value. Nothing is written to the sim - the only
-- call made is get_argument_value, which reads.
--
-- Expect noise. Gauges, needles and switch animations all live in the same numbering, so the
-- log will carry a stream of continuously varying arguments alongside the discrete one you are
-- looking for. The one that matters steps between two values at the moment you press the key,
-- which is why the log records the time and why it is worth noting the clock when you do it.

local LOG        = lfs.writedir() .. "Logs/c130-probe/args.log"
local MAX_ARG    = 3000
-- Short on purpose. A longer wait keeps the start-up animation out of the log, but it also
-- throws away any session shorter than itself - and the test this exists for is to press one
-- key and leave. The animation is analogue anyway, and the thing being looked for is discrete,
-- so it costs nothing to record from the start and ignore the needles afterwards.
local SETTLE     = 3.0
local INTERVAL   = 0.25
local EPSILON    = 0.001    -- below this an argument is treated as unchanged: needles jitter

local baseline   = nil
local last       = {}
local started    = nil
local next_scan  = 0

local function log(msg)
    local f = io.open(LOG, "a")
    if f then
        f:write(os.date("[%H:%M:%S] ") .. tostring(msg) .. "\n")
        f:close()
    end
end

local function read_all()
    local device = GetDevice(0)
    if not device or not device.get_argument_value then return nil end

    local values = {}
    for arg = 0, MAX_ARG do
        local ok, value = pcall(device.get_argument_value, device, arg)
        if ok and type(value) == "number" and value ~= 0 then
            values[arg] = value
        end
    end
    return values
end

local previous_export = LuaExportActivityNextEvent

LuaExportActivityNextEvent = function(t)
    local next_event = t + 0.1

    if not started then
        started = t
        log("=== balayage des arguments : " .. tostring(LoGetSelfData() and LoGetSelfData().Name or "?") .. " ===")
        log(string.format("attente de %.0fs avant la reference, puis un scan toutes les %.2fs", SETTLE, INTERVAL))
    end

    if t - started >= SETTLE and t >= next_scan then
        next_scan = t + INTERVAL
        local values = read_all()

        if values == nil then
            log("get_argument_value indisponible - rien a faire")
        elseif baseline == nil then
            baseline = values
            last = values

            -- The whole baseline, not just how big it is. A lamp already lit when the probe
            -- starts never transitions, so a run that only reports changes has nothing to
            -- show for it - which is exactly what the first two runs did. Written out, the
            -- lamp is simply one of the arguments sitting at 1 here.
            local args = {}
            for arg in pairs(values) do args[#args + 1] = arg end
            table.sort(args)
            log(string.format("reference prise : %d arguments non nuls sur %d", #args, MAX_ARG + 1))
            for _, arg in ipairs(args) do
                log(string.format("  ref  arg %4d = %.3f", arg, values[arg]))
            end
            log("actionnez maintenant ce que vous cherchez")
        else
            for arg = 0, MAX_ARG do
                local now = values[arg] or 0
                local before = last[arg] or 0
                if math.abs(now - before) > EPSILON then
                    log(string.format("arg %4d : %.3f -> %.3f", arg, before, now))
                end
            end
            last = values
        end
    end

    if previous_export then
        local ok, other = pcall(previous_export, t)
        if ok and type(other) == "number" then
            next_event = math.min(next_event, other)
        end
    end

    return next_event
end
