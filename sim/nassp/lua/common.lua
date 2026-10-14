-- apollo-c test helpers for NASSP scenarios (run on stock and on the C AGC alike)
ac = {}
function ac.open(vname, logname)
  ac.v = vessel.get_interface(vname)
  ac.f = io.open(logname, 'w')
  ac.cur_warp = 1
  oapi.set_tacc(1)
  -- hold the warp: a changed time acceleration (by hand) is logged and undone
  proc.bg(function()
    while true do
      local w = oapi.get_tacc()
      if math.abs(w - ac.cur_warp) > 0.01 then
        ac.log(string.format('WARP CHANGED to %g (script holds %g): re-asserted', w, ac.cur_warp))
        oapi.set_tacc(ac.cur_warp)
      end
      proc.wait_sysdt(0.5)
    end
  end)
end
ac.WAIT_WARP = 20   -- waiting stretches (20x proved safe on both builds)
-- set and hold the time acceleration, logged
ac.warp_ok = nil  -- optional predicate: false holds 1x (e.g. while a job our build runs hot is active)
function ac.warp(w, why)
  if w > 1 and ac.warp_ok and not ac.warp_ok() then why = why .. '; held at 1x: ' .. ac.warp_why; w = 1 end
  if w ~= ac.cur_warp then
    ac.cur_warp = w
    oapi.set_tacc(w)
    ac.log(string.format('warp %gx (%s)', w, why))
  end
end
function ac.log(s)
  ac.f:write(string.format('%10.2f %8.1f %s\n', oapi.get_simtime(), oapi.get_systime(), s)); ac.f:flush()
end
-- abort the run loudly: logged, then the "end" line the host poller closes Orbiter on
function ac.die(msg)
  ac.log('FATAL ' .. msg)
  ac.log('end')
  error(msg)
end
-- set a panel switch NASSP knows (get_panelswitch is -1 for an unknown name; NASSP's
-- set_panelswitch would only print "Switch not found" on screen and carry on)
function ac.set(name, pos)
  if ac.v:get_panelswitch(name) == -1 then ac.die('switch not found: ' .. name) end
  ac.v:set_panelswitch(name, pos)
end
function ac.key(k)
  ac.set('DskySwitch' .. k, 1); proc.wait_simdt(0.2)
  ac.set('DskySwitch' .. k, 0); proc.wait_simdt(0.3)
end
function ac.keys(seq)  -- e.g. "V37E02E", R = RSET, P = PRO, K = KEY REL, C = CLR
  local map = {V='Verb', N='Noun', E='Enter', R='Reset', P='Proceed', K='KeyRel', C='Clear', ['+']='Plus', ['-']='Minus',
    ['0']='Zero', ['1']='One', ['2']='Two', ['3']='Three', ['4']='Four', ['5']='Five', ['6']='Six', ['7']='Seven', ['8']='Eight', ['9']='Nine'}
  for c in seq:gmatch('.') do ac.key(map[c]) end
end
-- erasable word by flat address (bank = a / 256)
function ac.e(a) return ac.v:get_agcerasable(math.floor(a / 256), a % 256) end
-- vessel position (planet-relative longitude, latitude in degrees, radius in m)
function ac.where()
  local ok, e = pcall(function() return ac.v:get_equpos() end)
  if ok and e then return string.format('lng %.4f lat %.4f rad %.0f alt %.0f', math.deg(e.lng or e[1] or 0), math.deg(e.lat or e[2] or 0), e.rad or e[3] or 0, ac.v:get_altitude()) end
  return 'pos ?'
end
-- osculating elements of the vessel (Orbiter): semi-major axis m, eccentricity, inclination deg
function ac.orbit()
  local ok, el = pcall(function() return ac.v:get_elements() end)
  if ok and el then
    local a, e = el.a or el[1], el.e or el[2]
    local ref = oapi.get_objhandle('Moon')
    local rp = a and e and a * (1 - e) or 0
    return string.format('a %.0f e %.5f i %.3f peri-rad %.0f apo-rad %.0f', a or 0, e or 0, math.deg(el.i or el[3] or 0), rp, a and e and a * (1 + e) or 0)
  end
  return 'orbit ?'
end
function ac.dump(tag, cm)
  local s = string.format('%s MODREG=%o FAILREG=%o,%o,%o ch11=%o ch12=%o ch14=%o ch30=%o ch33=%o TIME=%o,%o', tag, ac.e(521),
    ac.e(253), ac.e(254), ac.e(255), ac.v:get_agcchannel(9), ac.v:get_agcchannel(10), ac.v:get_agcchannel(12),
    ac.v:get_agcchannel(24), ac.v:get_agcchannel(27), ac.e(20), ac.e(21))
  local d = ' DSPTAB='
  for i = 0, 11 do d = d .. string.format('%o ', ac.e(531 + i)) end
  ac.log(s .. d)
end

-- ---- DSKY state from DSPTAB (relay words) and channel 11 ----
local DIG = {[0]=' ', [21]='0', [3]='1', [25]='2', [27]='3', [15]='4', [30]='5', [28]='6', [19]='7', [29]='8', [31]='9'}
local function dd(w) return (DIG[math.floor(w / 32) % 32] or '?') .. (DIG[w % 32] or '?') end
function ac.vn()  -- verb, noun, prog strings and the V/N flash
  local flash = math.floor(ac.v:get_agcchannel(9) / 32) % 2 == 1
  return dd(ac.e(540)), dd(ac.e(539)), dd(ac.e(541)), flash
end
ac.get0 = 0       -- GET at sim time 0 (scenario MISSNTIME)
function ac.get() return oapi.get_simtime() + ac.get0 end
ac.tigaddr = 1802 -- CSM TIG 03412; LM 03441 = 1825
function ac.tig_minus_now()  -- cs from the AGC clock to TIG
  local tig = ac.e(ac.tigaddr) * 16384 + ac.e(ac.tigaddr + 1)
  local now = ac.e(20) * 16384 + ac.e(21)
  return tig - now
end
function ac.wait_vn(v, n, flash, timeout)
  local t0 = oapi.get_simtime()
  while oapi.get_simtime() - t0 < timeout do
    local vv, nn, pp, fl = ac.vn()
    if vv == v and nn == n and (flash == nil or fl == flash) then return true end
    proc.wait_simdt(0.5)
  end
  ac.dump(string.format('TIMEOUT waiting for V%sN%s', v, n))
  return false
end
function ac.wait_until(fn, timeout)
  local t0 = oapi.get_simtime()
  while oapi.get_simtime() - t0 < timeout do
    if fn() then return true end
    proc.wait_simdt(0.5)
  end
  return false
end
-- PRO: hold the key 0.6 s (T4RUPT samples it); if the flashing display is still the same
-- one second later, PROCEED by V33E.
function ac.pro()
  local v0, n0, p0, f0 = ac.vn()
  ac.set('DskySwitchProg', 1); proc.wait_simdt(0.6)
  ac.set('DskySwitchProg', 0); proc.wait_simdt(1.0)
  local v1, n1, p1, f1 = ac.vn()
  if f0 and f1 and v1 == v0 and n1 == n0 then ac.log('  PRO not taken: V33E'); ac.keys('V33E') end
end
-- log the DSKY once a second while fn() is true (pacing comparisons), as a background branch
function ac.track(fn)
  proc.bg(function()
    while true do
      if fn() then ac.dump('  dsky1s') end
      proc.wait_simdt(1)
    end
  end)
end
-- V37 to program n (decimal), retried every 10 s up to 6 times while the AGC refuses it
-- (alarm 1520: V37 not permitted yet, e.g. right after V37E00E from a running P40)
function ac.select(n)
  for try = 1, 6 do
    ac.keys('V37E' .. string.format('%02d', n) .. 'E'); proc.wait_simdt(3)
    if ac.e(521) == n then ac.dump(string.format('  V37 %02d taken (try %d)', n, try)); return true end
    ac.dump(string.format('  V37 %02d refused (try %d)', n, try)); ac.keys('R'); proc.wait_simdt(7)
  end
  ac.die(string.format('V37 %02d refused 6 times', n))
end
-- ---- checklist executor (CL tables compiled from NASSP's checklist xls) ----
ac.skip = {}      -- sheets not to run (call rows)
ac.subst = {}     -- DSKY rows whose leading digits the crew types by hand: keys -> full keys
ac.manual = {}    -- manual rows met, for the report
local function note(step)
  local tx = step.text
  if ac.hook[tx] then ac.hook[tx](step); return end
  local v, n = tx:match('^F (%d%d) (%d%d)')
  if v then ac.wait_vn(v, n, true, 900); proc.wait_simdt(1); ac.dump('  seen F ' .. v .. ' ' .. n); return end
  v = tx:match('^F (%d%d)$')
  if v then ac.wait_until(function() local vv, nn, pp, fl = ac.vn(); return vv == v and fl end, 900); ac.dump('  seen F ' .. v); return end
  v, n = tx:match('^(%d%d) (%d%d) ')
  if v then
    -- a steady display the crew only reads (it may come and go between samples): 30 s at most
    if ac.wait_vn(v, n, nil, 30) then proc.wait_simdt(1); ac.dump('  seen ' .. v .. ' ' .. n) else ac.dump('  not seen ' .. v .. ' ' .. n) end
    return
  end
  -- "(PDI -10)": the PDI sheet's timeline marks, N minutes before TIG (as T-N:00)
  local pm = tx:match('^%(PDI %-(%d+)%)$')
  local m, s = tx:match('^T%-(%d+):(%d%d)')
  if pm then m, s = pm, '00' end
  if m then
    local lead = (tonumber(m) * 60 + tonumber(s)) * 100
    -- 20x while more than 60 s (and 15 s before this row) to go; 1x from TIG-60 s on
    if ac.tig_minus_now() - lead > 1500 and ac.tig_minus_now() > 7500 then
      ac.warp(ac.WAIT_WARP, 'wait for ' .. tx)
      ac.wait_until(function() local d = ac.tig_minus_now(); return d - lead <= 1500 or d <= 7500 end, 7200)
    end
    ac.warp(1, 'TIG window / ' .. tx)
    ac.wait_until(function() return ac.tig_minus_now() <= lead end, 7200)
    ac.dump('  at ' .. tx .. ' (TIG - now = ' .. ac.tig_minus_now() .. ' cs)')
    if lead == 500 then
      ac.tig0 = ac.e(ac.tigaddr) * 16384 + ac.e(ac.tigaddr + 1) -- ignition time (TIG may move later)
      ac.wait_until(function() local vv, nn, pp, fl = ac.vn(); return vv == '99' and fl end, 20); ac.dump('  V99 flash')  -- V99N40 (P40) or V99N62 (P63)
    end
    return
  end
  if tx == 'Engine Cutoff' then
    local t0 = oapi.get_simtime()
    while oapi.get_simtime() - t0 < 120 and math.floor(ac.v:get_agcchannel(9) / 4096) % 2 == 0 do proc.wait_simdt(0.1) end
    ac.dump('  engine on (or 120 s passed)')
    while oapi.get_simtime() - t0 < 900 and math.floor(ac.v:get_agcchannel(9) / 4096) % 2 == 1 do proc.wait_simdt(0.1) end
    ac.dump('  engine off')
    return
  end
  -- "At TIG +2m30s", "At TIG +26s": wait on the AGC clock against the TIG in erasable
  local am, asec = tx:match('^At TIG %+(%d+)m(%d*)')
  if not am then am = tx:match('^At PDI %+(%d+)'); if am then asec = '0' end end
  if not am then asec = tx:match('^At TIG %+(%d+)s'); if asec then am = '0' end end
  if am then
    local lead = (tonumber(am) * 60 + (tonumber(asec) or 0)) * 100
    local function since_ign()
      if ac.tig0 == nil then return -ac.tig_minus_now() end
      return ac.e(20) * 16384 + ac.e(21) - ac.tig0
    end
    ac.wait_until(function() return since_ign() >= lead end, 1200)
    ac.dump('  at ' .. tx)
    return
  end
  -- "Wait Until EI -25m", "At EI -15m", "Wait Until EI -1:35h": wait on GET when ac.ei is set
  if ac.ei then
    local hh, mm = tx:match('EI %-(%d+):(%d%d)h')
    local m2 = tx:match('EI %-(%d+)m')
    local lead = hh and (tonumber(hh) * 3600 + tonumber(mm) * 60) or (m2 and tonumber(m2) * 60)
    if lead then
      local t = ac.ei - lead
      if ac.get() < t - 60 then ac.warp(ac.WAIT_WARP, 'wait for ' .. tx) end
      while ac.get() < t - 60 do proc.wait_simdt(1) end
      ac.warp(1, tx)
      while ac.get() < t do proc.wait_simdt(0.5) end
      ac.dump('  at ' .. tx)
      return
    end
  end
  if step.manual then table.insert(ac.manual, tx) end
end
-- ac.run(sheet [, depth [, from [, to]]]): from = the text of the first row to run (a scenario that
-- starts part way through the sheet's timeline)
-- every switch a sheet (and the sheets it calls, unless skipped) sets must exist
local function preflight(sheet, seen)
  if seen[sheet] or ac.skip[sheet] or not CL[sheet] then return end
  seen[sheet] = true
  for _, st in ipairs(CL[sheet]) do
    if st.t == 'switch' and ac.v:get_panelswitch(st.item) == -1 then ac.die('switch not found: ' .. st.item .. ' (sheet ' .. sheet .. ')') end
    if st.t == 'call' then preflight(st.sheet, seen) end
  end
end
-- to = the text of the row to stop before (the script takes over from there)
ac.hook = {}      -- note text -> function run instead of the default handling
function ac.run(sheet, depth, from, to)
  depth = depth or 0
  if depth == 0 then preflight(sheet, {}) end
  local steps = CL[sheet]
  if not steps then ac.log('NO SHEET ' .. sheet); return end
  ac.log(string.rep('  ', depth) .. '[' .. sheet .. ']' .. (from and (' from "' .. from .. '"') or ''))
  local on = from == nil
  for _, st in ipairs(steps) do
    if not on and st.text == from then on = true end
    if on and to and st.text == to then ac.log('stop before "' .. to .. '"'); return end
    if on then
    if st.delay then proc.wait_simdt(st.delay) end
    if st.met then
      if ac.get() < st.met - 30 then ac.warp(ac.WAIT_WARP, 'wait for MET ' .. st.met) end
      while ac.get() < st.met - 30 do proc.wait_simdt(1) end
      ac.warp(1, 'MET ' .. st.met)
      while ac.get() < st.met do proc.wait_simdt(1) end
    end
    if st.t == 'switch' then
      ac.set(st.item, st.pos); proc.wait_simdt(0.3)
    elseif st.t == 'dsky' then
      local k = ac.subst[st.keys] or st.keys
      -- an ENTER / PRO answers a flashing display: during an attitude maneuver (V06N18,
      -- steady) wait for the flash (R60 shows F50N18 again when the maneuver ends)
      -- the crew answers a flashing display: a lone ENTER / PRO waits for the flash (an R60
      -- maneuver shows V06N18 steady until F50N18 comes back; a display being rewritten reads
      -- garbled for a moment), at most 180 s
      if k == 'P' or k == 'E' then
        local vv, nn, pp, fl = ac.vn()
        if not fl then
          ac.log('  no flashing display (V' .. vv .. 'N' .. nn .. '): wait for the flash before ' .. st.text)
          if not ac.wait_until(function() local a, b, c, f2 = ac.vn(); return f2 end, 180) then ac.dump('  no flash in 180 s') end
          proc.wait_simdt(0.5)
        end
      end
      if k == 'P' then ac.pro() else ac.keys(k:gsub('P', 'V33E')) end
      ac.dump(string.rep('  ', depth) .. 'DSKY ' .. st.keys .. '  (' .. st.text .. ')')
    elseif st.t == 'call' then
      if ac.skip[st.sheet] then ac.log(string.rep('  ', depth) .. 'skip ' .. st.sheet) else ac.run(st.sheet, depth + 1) end
    else
      note(st)
    end
    end
  end
  if not on then ac.log('NO ROW "' .. from .. '" in ' .. sheet) end
end

-- ---- edits: a CSV of selected erasables, channels and vessel truth vs GET (both builds alike) ----
-- ac.edits(path, dt, extra): every dt sim-seconds one row: GET, sim time, MODREG, FAILREG 0-2,
-- channels 11 12 14 30 33, TIME2/TIME1, DSPTAB 0-11 (octal words), then the extra columns
-- ({name, function returning a number or string}, in order). Compared by tools/edits_diff.py.
function ac.edits(path, dt, extra)
  local f = io.open(path, 'w')
  local h = 'get,t,modreg,failreg0,failreg1,failreg2,ch11,ch12,ch14,ch30,ch33,time2,time1'
  for i = 0, 11 do h = h .. ',dsptab' .. i end
  for _, x in ipairs(extra or {}) do h = h .. ',' .. x[1] end
  f:write(h .. '\n'); f:flush()
  proc.bg(function()
    while true do
      local r = string.format('%.2f,%.2f,%o,%o,%o,%o,%o,%o,%o,%o,%o,%o,%o', ac.get(), oapi.get_simtime(), ac.e(521), ac.e(253), ac.e(254), ac.e(255),
        ac.v:get_agcchannel(9), ac.v:get_agcchannel(10), ac.v:get_agcchannel(12), ac.v:get_agcchannel(24), ac.v:get_agcchannel(27), ac.e(20), ac.e(21))
      for i = 0, 11 do r = r .. string.format(',%o', ac.e(531 + i)) end
      for _, x in ipairs(extra or {}) do
        local ok, v = pcall(x[2])
        if not ok or v == nil then v = '' end
        if type(v) == 'number' then v = string.format('%.6g', v) end
        r = r .. ',' .. tostring(v)
      end
      f:write(r .. '\n'); f:flush()
      proc.wait_simdt(dt)
    end
  end)
end
