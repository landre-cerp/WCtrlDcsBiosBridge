-- Offline extractor for the C-130J CNI-MU page layouts.
--
-- Replays each of the module's page scripts under stubbed cockpit globals and records the
-- Add() order, geometry and static values of every element. The result is the schema the
-- runtime matcher aligns incoming list_indication blocks against — which is the only way to
-- recover what the indication does not carry: screen position, font size, and inversion.
--
-- Run with DCS's own Lua so the dialect matches the module's:
--     "D:\Program Files\Eagle Dynamics\DCS World\bin\luae.exe" extract.lua > c130j-cni-pages.json
--
-- Nothing in the DCS installation is written or modified; every file is opened read-only.
-- In particular SHOULD_MAKE_LAYOUTS in the module's definitions.lua stays false — enabling it
-- would make DCS write layout dumps into Program Files and dirty the module.

local DCS  = os.getenv("DCS_PATH")  or [[D:\Program Files\Eagle Dynamics\DCS World]]
local MOD  = DCS .. [[\Mods\aircraft\C130J\Cockpit\Scripts\]]
local COMMON = DCS .. [[\Scripts\Aircrafts\_Common\Cockpit\]]

local function norm(p) return (p:gsub("\\", "/")) end

local SCRIPT_PATH = norm(MOD)
local COMMON_PATH = norm(COMMON)

-- Files whose real contents matter (they define the geometry) versus files that only exist
-- to pull in C++-backed element machinery, which is stubbed instead.
local SKIP = {
    ["elements_defs.lua"] = true,
    ["devices_defs.lua"]  = true,
    ["materials.lua"]     = true,
    ["i_18n"]             = true,
}

local function basename(p) return (norm(p):match("([^/]+)$")) or p end

-- ---------------------------------------------------------------------------
-- Minimal JSON encoder. Writing one avoids depending on the module's own JSON.lua, whose
-- load path assumes DCS's working directory.
-- ---------------------------------------------------------------------------

local function esc(s)
    s = s:gsub('[\\"]', '\\%0')
         :gsub('\n', '\\n'):gsub('\r', '\\r'):gsub('\t', '\\t')
    return (s:gsub('[\1-\31]', function(c)
        return string.format('\\u%04x', string.byte(c))
    end))
end

local encode
local function encode_array(t, out)
    out[#out + 1] = "["
    for i = 1, #t do
        if i > 1 then out[#out + 1] = "," end
        encode(t[i], out)
    end
    out[#out + 1] = "]"
end

local function encode_object(t, out)
    local keys = {}
    for k in pairs(t) do if type(k) == "string" then keys[#keys + 1] = k end end
    table.sort(keys)
    out[#out + 1] = "{"
    for i, k in ipairs(keys) do
        if i > 1 then out[#out + 1] = "," end
        out[#out + 1] = '"' .. esc(k) .. '":'
        encode(t[k], out)
    end
    out[#out + 1] = "}"
end

encode = function(v, out)
    local tv = type(v)
    if v == nil then
        out[#out + 1] = "null"
    elseif tv == "boolean" then
        out[#out + 1] = tostring(v)
    elseif tv == "number" then
        -- %.6g keeps metre offsets readable without dragging float noise into the diff.
        out[#out + 1] = (v ~= v or v == math.huge or v == -math.huge)
            and "null" or string.format("%.6g", v)
    elseif tv == "string" then
        out[#out + 1] = '"' .. esc(v) .. '"'
    elseif tv == "table" then
        if #v > 0 then encode_array(v, out) else encode_object(v, out) end
    else
        out[#out + 1] = "null"
    end
end

local function to_json(v)
    local out = {}
    encode(v, out)
    return table.concat(out)
end

-- ---------------------------------------------------------------------------
-- Sandbox
-- ---------------------------------------------------------------------------

local function new_env(recorder)
    local env = {}
    local guid_seq = 0

    -- Deterministic stand-ins for the C++ create_guid_string(). The real one returns random
    -- v4 GUIDs that change every session, so they can never be a join key; the schema keys on
    -- Add() order instead and these are only here to keep element identity unique per page.
    local function create_guid_string()
        guid_seq = guid_seq + 1
        return string.format("stub-%04d", guid_seq)
    end

    local function CreateElement(kind)
        return { __kind = kind }
    end

    -- Only elements built by add_cni_el get the vertical nudge, and it has to be undone to
    -- recover their line. Toggles bypass that path entirely (make_base_toggle_el sets
    -- init_pos itself), so undoing it for them would move their words off the line they share
    -- with the rest of the toggle. Marking them as they are added is the only reliable tell.
    local in_add_cni_el = false

    local function Add(object)
        if in_add_cni_el then object.__nudged = true end
        recorder[#recorder + 1] = object
        return object
    end

    local materials = setmetatable({}, { __index = function(_, k) return k end })

    local base = {
        -- passthrough from the host interpreter
        pairs = pairs, ipairs = ipairs, type = type, tostring = tostring, tonumber = tonumber,
        string = string, table = table, math = math, os = os, io = io, select = select,
        rawget = rawget, rawset = rawset, setmetatable = setmetatable,
        getmetatable = getmetatable, unpack = unpack, error = error, assert = assert,
        pcall = pcall, xpcall = xpcall, next = next, print = print, require = function(name)
            if name == "lfs" then
                return { attributes = function() return nil end, mkdir = function() return true end }
            end
            return setmetatable({}, { __index = function() return function() end end })
        end,

        -- cockpit globals normally provided by the engine
        CreateElement = CreateElement,
        Add = Add,
        create_guid_string = create_guid_string,
        SetScale = function() end,
        GetScale = function() return 1.0 end,
        GetAspect = function() return 0.75 end,
        MakeMaterial = function(a, b) return { a, b } end,
        METERS = "METERS",
        LIGHT_SOURCES = {},
        blend_mode = setmetatable({}, { __index = function(_, k) return k end }),
        indicator_types = setmetatable({}, { __index = function(_, k) return k end }),
        render_purpose = setmetatable({}, { __index = function(_, k) return k end }),
        materials = materials,
        fonts = setmetatable({}, { __index = function(_, k) return k end }),
        gettext = setmetatable({}, { __index = function() return function(s) return s end end }),
        _ = function(s) return s end,

        -- from elements_defs.lua, which is skipped
        h_clip_relations = { REWRITE_LEVEL = 0, COMPARE = 1, NULL = 2 },
        default_box_indices = { 0, 1, 2, 0, 2, 3 },

        LockOn_Options = {
            script_path = SCRIPT_PATH,
            common_script_path = COMMON_PATH,
            init_conditions = { Aircraft = "C-130J-30" },
        },
    }

    base._G = base

    base.dofile = function(path)
        if SKIP[basename(path)] then return end
        local chunk, err = loadfile(norm(path))
        if not chunk then error("dofile " .. tostring(path) .. ": " .. tostring(err), 0) end
        setfenv(chunk, base)
        local result = chunk()

        -- definitions.lua fills cni_lines_ up to index 12, but comm_tune_1.lua walks els[0..5]
        -- and asks for cni_lines_[(5*2)+3] = 13. In here that is a nil arithmetic fault that
        -- truncates the page; in the sim the whole page demonstrably renders, key blocks and
        -- all. Rather than lose the layout of a page that does exist, extend the table by one
        -- using the module's own 11->12 spacing and carry on. The extra line is flagged
        -- `synthetic` in the output so nothing downstream mistakes it for measured geometry.
        if basename(path) == "definitions.lua" then
            local inner = rawget(base, "add_cni_el")
            if type(inner) == "function" then
                base.add_cni_el = function(...)
                    in_add_cni_el = true
                    local ok, err = pcall(inner, ...)
                    in_add_cni_el = false
                    if not ok then error(err, 0) end
                end
            end

            local L = rawget(base, "cni_lines_")
            if type(L) == "table" and L[13] == nil and L[12] and L[11] then
                L[13] = L[12] - (L[11] - L[12])
                base.__synthetic_lines = { [13] = true }
            end
        end

        return result
    end

    base.loadfile = function(path)
        local chunk = loadfile(norm(path))
        if chunk then setfenv(chunk, base) end
        return chunk
    end

    return base
end

local function run_file(path, env)
    local chunk, err = loadfile(norm(path))
    if not chunk then return nil, "load: " .. tostring(err) end
    setfenv(chunk, env)
    local ok, rerr = pcall(chunk)
    if not ok then return nil, "run: " .. tostring(rerr) end
    return true
end

-- ---------------------------------------------------------------------------
-- Geometry -> grid
-- ---------------------------------------------------------------------------

-- The CNI's own width, measured off the module: (rpos - lpos) / advance is 25.05, and the
-- pages place nothing past 25. The panel is told this grid rather than the CDU's usual 24.
local COLUMNS = 25

local function nearest_line(env, y)
    local lines = env.cni_lines_
    if not lines or type(y) ~= "number" then return nil, nil end
    local best, best_d
    for i = 0, 13 do
        local ly = lines[i]
        if ly then
            local d = math.abs(ly - y)
            if not best_d or d < best_d then best, best_d = i, d end
        end
    end
    return best, best_d
end

-- Character advance, taken from the module's own font tables rather than hardcoded, so a
-- module update that retunes the fonts shows up here instead of silently shifting every page.
--
-- One advance for the whole page, the large one, even though small text is fractionally
-- wider: positions have to land on a single grid, and letting each element bring its own
-- scale would put two labels on the same line a column apart.
local function advance(env)
    local large = env.LARGE_FONT_CNI
    if large and large[2] then return large[2] end
    return 0.003944
end

-- Columns as the module counts them: how many character advances a position sits from the
-- left edge of the page, and nothing else.
--
-- This used to divide by 25.05/24 to squeeze the CNI's width into the 24 columns the CDU was
-- believed to have. The panels turned out to be addressable framebuffers that take whatever
-- grid they are told, so the compression bought nothing and cost a great deal: the module
-- lays out on clean integers, and scaling dragged them off each other by up to a full column,
-- which is what stopped columns of figures lining up.
local function grid_column(env, x)
    if type(x) ~= "number" then return nil end

    local adv = advance(env)
    local left = env.lpos
    if not (left and adv > 0) then return nil end

    return math.floor((x - left) / adv + 0.5)
end

-- A numeric field that has no value yet is not empty on the CNI: it is drawn as a run of the
-- placeholder the page uses, boxes for something the crew enters and dashes for something the
-- aircraft has not computed. "]]]/" and "---T/" and "-----" are all fields waiting, and a
-- pattern built from "%d" alone rejects every one of them - which costs the element its slot
-- on exactly the pages a crew sees before takeoff.
-- It happens at two scales. A field can be partly filled - "]]]/" is a wind whose direction is
-- still to be entered - or the page can draw the whole field as placeholder, which is what
-- "-----" is where a page counter belongs. So each conversion may be a placeholder run or
-- nothing at all, and the pattern as a whole gets the same alternative.
local UNSET = "[-\\]]"

-- 25 of the 154 pages build their title through a printf format driven by a controller
-- ("INAV%d CTRL SOLN", "%sLZ %2d INIT"), so a literal title lookup misses them entirely.
-- Precomputing the pattern here keeps the .NET side free of Lua format semantics.
local function format_to_regex(fmt)
    local out, i, n = {}, 1, #fmt
    while i <= n do
        local c = fmt:sub(i, i)
        if c == "%" then
            local spec, len = fmt:match("^%%([-+ #0]*%d*%.?%d*)([diufFgGeExXocs])", i)
            if spec ~= nil then
                if len == "s" then
                    out[#out + 1] = ".*"
                elseif len == "c" then
                    out[#out + 1] = "."
                elseif len:match("[fFgGeE]") then
                    out[#out + 1] = "(?:%s*-?[0-9.]+|" .. UNSET .. "*)"
                else
                    -- width-padded integers arrive space-padded: "%2d" of 1 gives " 1",
                    -- which is why "LZ  1 INIT" carries a double space.
                    out[#out + 1] = "(?:%s*-?[0-9]+|" .. UNSET .. "*)"
                end
                i = i + 1 + #spec + 1
            else
                out[#out + 1] = "%%"
                i = i + 1
            end
        else
            if c:match("[%^%$%(%)%.%[%]%*%+%-%?%\\|{}]") then
                out[#out + 1] = "\\" .. c
            else
                out[#out + 1] = c
            end
            i = i + 1
        end
    end
    local body = table.concat(out):gsub("%%s%*", "\\s*")
    return "^(?:" .. body .. "|" .. UNSET .. "+)$"
end

-- The same conversion applied to every field, not just the title. A dynamic slot's format is
-- the only thing that distinguishes it from its neighbours: on COMM TUNE INDEX the frequency
-- "1/243.000" fits "%d/%s" and the identifier beside it only fits "%s", which is what says
-- the frequency belongs on the radio's line rather than the sub-line below it.
local function fmt_patterns(fmts)
    if not fmts then return nil end

    local out = {}
    for _, f in ipairs(fmts) do
        out[#out + 1] = format_to_regex(f)
    end
    return #out > 0 and out or nil
end

local function anchor_of(alignment)
    if type(alignment) ~= "string" then return "Left" end
    if alignment:find("^Center") then return "Center" end
    if alignment:find("^Right") then return "Right" end
    return "Left"
end

-- add_cni_el picks the line, writes init_pos, and only then nudges SMALL_FONT_CNI elements
-- 0.01 further from the centre (definitions.lua:131-137). Reading init_pos back therefore
-- lands a small label on the wrong line unless the nudge is undone first — the sign is taken
-- from the value before the shift, so it can be recovered from which side of zero it is on.
local function unshift(env, el, y)
    if type(y) ~= "number" then return y end
    if not el.__nudged or el.stringdefs ~= env.SMALL_FONT_CNI then return y end
    return y > 0 and (y - 0.01) or (y + 0.01)
end

local function describe(env, el, ordinal)
    local pos = el.init_pos or {}
    local x, y = pos[1], unshift(env, el, pos[2])
    local line, dist = nearest_line(env, y)

    local ctrl
    if type(el.controllers) == "table" then
        for _, c in ipairs(el.controllers) do
            if type(c) == "table" and type(c[1]) == "string" then ctrl = c[1] break end
        end
    end

    -- formats may be a list of alternatives the controller selects between, e.g.
    -- {"%sLEGS %d", "%sLEGS %d HIST"} in legs_progress.lua. Keeping only the first would
    -- make the page unrecognisable in its other state.
    local fmts
    if type(el.formats) == "table" then
        fmts = {}
        for _, f in ipairs(el.formats) do
            if type(f) == "string" then fmts[#fmts + 1] = f end
        end
        if #fmts == 0 then fmts = nil end
    elseif type(el.formats) == "string" then
        fmts = { el.formats }
    end

    -- A value carrying a conversion is a format the page never filled in, not a literal. Eight
    -- elements do this, all of them the "<%s" that draws a page's back-reference: the sim
    -- prints "<MISSIONS", so recording "<%s" as static text gives the matcher a landmark that
    -- can never be seen and costs the element its slot.
    local value = type(el.value) == "string" and el.value or nil
    if value and value:match("%%[-+ #0]*%d*%.?%d*[diufFgGeExXocs]") then
        fmts = fmts or {}
        fmts[#fmts + 1] = value
        value = nil
    end

    -- Identity, not size: SMALL_FONT_CNI and SMALL_FONT_CNI0 share a height but only the
    -- former is nudged vertically, so telling them apart matters for the line lookup above.
    local small = el.stringdefs == env.SMALL_FONT_CNI or el.stringdefs == env.SMALL_FONT_CNI0

    return {
        n          = ordinal,
        kind       = el.__kind,
        name       = el.name,
        value      = value,
        fmt        = fmts,
        fmtRx      = fmt_patterns(fmts),
        ctrl       = ctrl,
        anchor     = anchor_of(el.alignment),
        line       = line,
        lineErr    = dist and (dist > 0.0015) and dist or nil,
        col        = grid_column(env, x),
        x          = type(x) == "number" and x or nil,
        y          = type(y) == "number" and y or nil,
        small      = small or nil,
        -- One name is enough. cni_font_invert and cni_font_inv are named in the module's
        -- scripts too, but no element is ever assigned them: widening the test to any material
        -- containing "inv" leaves the output byte for byte identical. Some selections carry no
        -- material at all and are marked by font size alone — POWER UP's two NAV DB lines are
        -- built with SMALL_FONT_CNI against LARGE_FONT_CNI and nothing else.
        invert     = el.material == "cni_font_green_invert" or nil,
        parent     = el.parent_element,
    }
end

-- ---------------------------------------------------------------------------

local function page_catalogue()
    local rec = {}
    local env = new_env(rec)
    local ok, err = run_file(MOD .. "CNI_MU/init_gen.lua", env)
    if not ok then error("init_gen.lua: " .. tostring(err), 0) end

    -- page_subsets is keyed by the numeric page constants; recover each constant's name by
    -- scanning the environment, so the schema carries "COMM_TUNE_1" rather than 1.
    local name_of = {}
    for k, v in pairs(env) do
        if type(k) == "string" and type(v) == "number" and k:upper() == k then
            name_of[v] = name_of[v] or k
        end
    end

    local pages = {}
    for id, path in pairs(env.page_subsets or {}) do
        pages[#pages + 1] = { id = id, name = name_of[id] or ("PAGE_" .. tostring(id)), path = path }
    end
    table.sort(pages, function(a, b) return a.id < b.id end)
    return pages
end

local function extract_page(page)
    local rec = {}
    local env = new_env(rec)
    local ok, err = run_file(page.path, env)

    -- A page that dies part-way still yields everything Add()ed before the fault, and that
    -- partial layout is worth keeping: comm_tune_1.lua walks els[0..5] and asks for
    -- cni_lines_[13], one past the table's end, yet the running sim clearly renders the whole
    -- page. Until that is explained, record what was built and flag it rather than dropping
    -- the page entirely.
    local slots, title, counter, title_res = {}, nil, nil, nil
    for i, el in ipairs(rec) do
        local d = describe(env, el, i)
        slots[#slots + 1] = d
        if d.name == "cni_title" then
            if d.value then
                title = d.value
            elseif d.fmt then
                title_res = {}
                for _, f in ipairs(d.fmt) do title_res[#title_res + 1] = format_to_regex(f) end
            end
        end
        if not counter and d.value and d.value:match("^%s*%d+/%d+%s*$") then counter = d.value end
    end

    return {
        id         = page.id,
        name       = page.name,
        file       = basename(page.path),
        title      = title,
        titleRegex = title_res,
        counter    = counter,
        partial = (not ok) or nil,
        error   = (not ok) and err or nil,
        slots   = slots,
    }
end

local pages = page_catalogue()
local out, failed = {}, 0

for _, p in ipairs(pages) do
    local entry = extract_page(p)
    if entry.partial then failed = failed + 1 end
    out[#out + 1] = entry
end

io.stderr:write(string.format("pages: %d, partielles: %d\n", #out, failed))
for _, e in ipairs(out) do
    if e.error then io.stderr:write(string.format("  FAIL %-28s %s\n", e.name, e.error)) end
end

print(to_json({ columns = COLUMNS, lines = 14, pages = out }))
