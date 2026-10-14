-- Apollo 11 lunar liftoff and rendezvous (scenario 11 with Columbia, focus MCC, headless). Liftoff as
-- liftoffh.lua; then NASSP's LM checklist "Rendezvous" (P20, CSI P32, CDH P33, TPI P34; MCC1/2 and
-- the P52 / AGS sheets skipped) with the burns nulled through the TTCA translation bits
-- (channel 31, set_agcinputbit) against N85, the "At XXX-Nm" rows timed on the LGC's TIG.
-- Lunar liftoff part (scenario 11, T-1m: Eagle on the surface, P12 loaded and waiting at
-- V06N74), unattended, 1x: NASSP's own LM checklist "Lunar Liftoff" from its "T-0:35" row (AVE G
-- on, V99 at TIG-5 s, PRO, APS burn to insertion, N85, V82 HA/HP). The scenario's P12 is used as
-- it is (no RSET / V37). Logs every 2 s from TIG-40 s: altitude, vertical and horizontal speed,
-- the APS level, attitude, MODREG; then the insertion orbit (Orbiter elements). Needs the LEM
-- Lua test hook (patches/0002).
dofile('Script/apollo-c/common.lua')
dofile('Script/apollo-c/cl_lm.lua')
ac.open('Eagle', 'apollo-c-rendezvous.log')
ac.get0 = 448249.40
ac.tigaddr = 1825 -- LM TIG 03441
local function words(a, n) local s = '' for i = 0, n - 1 do s = s .. string.format('%o ', ac.e(a + i)) end return s end
local eng
local function find_eng()  -- the largest thruster (the APS once staged; the DPS before)
  local best = 0
  pcall(function()
    for i = 0, ac.v:get_thrustercount() - 1 do
      local h = ac.v:get_thrusterhandle(i)
      local m = ac.v:get_thrustermax0(h)
      if m > best then best, eng = m, h end
    end
  end)
  return best
end
local function vel()
  local ok, w = pcall(function() return ac.v:get_airspeedvector(REFFRAME.HORIZON) end)
  if ok and w then return w.y or 0, math.sqrt((w.x or 0) ^ 2 + (w.z or 0) ^ 2) end
  return 0, 0
end
local function probe()
  local lvl = -1
  find_eng()
  if eng then pcall(function() lvl = ac.v:get_thrusterlevel(eng) end) end
  local hd, vh = vel()
  local att = ''
  pcall(function() att = string.format(' pby %.1f %.1f %.1f cdu %o %o %o', math.deg(ac.v:get_pitch()), math.deg(ac.v:get_bank()), math.deg(ac.v:get_yaw()), ac.e(26), ac.e(27), ac.e(28)) end)
  return string.format('eng %.3f mass %.1f hdot %.2f vh %.2f', lvl, ac.v:get_mass(), hd, vh) .. att
end
local function sv() return string.format('GET=%.0f TIG=%s %s %s %s', ac.get(), words(1825, 2), ac.where(), ac.orbit(), probe()) end
local function since_tig() return (ac.e(20) * 16384 + ac.e(21) - (ac.e(1825) * 16384 + ac.e(1826))) / 100 end
ac.skip['AGS Update'] = true
proc.wait_simdt(2)
ac.dump('liftoff start ' .. sv())
local function el() local e = ac.v:get_elements(); return e end
local csm = oapi.get_objhandle('Columbia')
local function range() local ok, p = pcall(function() return ac.v:get_relativepos(csm) end); if ok and p then return math.sqrt(p.x ^ 2 + p.y ^ 2 + p.z ^ 2) end; return -1 end
local function rrate() local ok, v = pcall(function() return ac.v:get_relativevel(csm) end); local ok2, p = pcall(function() return ac.v:get_relativepos(csm) end); if ok and ok2 and v and p then local r = math.sqrt(p.x ^ 2 + p.y ^ 2 + p.z ^ 2); return (p.x * v.x + p.y * v.y + p.z * v.z) / r end; return 0 end
ac.edits('apollo-c-rendezvous-edits.csv', 1, {
  {'range', range}, {'rrate', rrate},
  {'galt', function() return ac.v:get_altitude(1) end}, {'hdot', function() local a, b = vel(); return a end}, {'vh', function() local a, b = vel(); return b end},
  {'lng', function() return math.deg(ac.v:get_equpos().lng) end}, {'lat', function() return math.deg(ac.v:get_equpos().lat) end},
  {'mass', function() return ac.v:get_mass() end}, {'eng', function() find_eng(); return ac.v:get_thrusterlevel(eng) end},
  {'pitch', function() return math.deg(ac.v:get_pitch()) end}, {'bank', function() return math.deg(ac.v:get_bank()) end},
  {'yaw', function() return math.deg(ac.v:get_yaw()) end},
  {'cdux', function() return ac.e(26) end}, {'cduy', function() return ac.e(27) end}, {'cduz', function() return ac.e(28) end},
  {'apo', function() local e = el(); return e.a * (1 + e.e) end}, {'peri', function() local e = el(); return e.a * (1 - e.e) end},
  {'asceng', function() return words(1202, 6) end}, {'asctgt', function() return words(1210, 8) end},
  {'ascacc', function() return words(1260, 6) end}, {'rcoyco', function() return words(1944, 4) end},
  {'vgvect', function() return words(1957, 6) end}, {'vgbody', function() return words(1857, 6) end},
  {'tgo', function() return words(1870, 2) end}, {'ascwrd', function() return words(1244, 16) end},
  {'tbup', function() return words(1208, 2) end}, {'geff', function() return words(1228, 2) end},
  {'hdotdisp', function() return words(1851, 2) end}})
proc.bg(function()
  local mm = ac.e(521)
  while true do
    ac.dump(string.format('track TIG%+.1f %s', since_tig(), sv()))
    local t = oapi.get_simtime()
    while oapi.get_simtime() - t < 2 do
      if ac.e(521) ~= mm then mm = ac.e(521); ac.dump(string.format('  MODREG -> %o at TIG %+.1f s %s', mm, since_tig(), sv())) end
      proc.wait_simdt(0.2)
    end
  end
end)
-- displays the sheet marks as flashing that P12 shows steady (or may not show): bounded waits
local function seen(v, n, t) return function(st) if ac.wait_vn(v, n, nil, t) then proc.wait_simdt(1); ac.dump('  seen ' .. v .. ' ' .. n .. ' ' .. sv()) end end end
ac.hook['F 06 63 VI, Hdot, H'] = seen('06', '63', 20)
ac.hook['F 16 63 VI Hdot, H'] = seen('16', '63', 30)
ac.hook['F 16 85 VG XYZ (LM)'] = seen('16', '85', 30)
ac.hook['F 16 44 HA, HP, TFF'] = seen('16', '44', 30)
ac.hook['F 37'] = function(st) ac.wait_until(function() local v, n, p, fl = ac.vn(); return v == '37' end, 30); ac.dump('  F 37 ' .. sv()) end
ac.hook['At TIG +2m'] = function(st) ac.wait_until(function() return since_tig() >= 120 end, 300); ac.dump('  at TIG +2m ' .. sv()) end
-- the scenario's running P12, as loaded (as liftoffh.lua)
ac.run('Lunar Liftoff', 0, 'T-0:35')
ac.dump('after Lunar Liftoff ' .. sv())
proc.wait_simdt(10)
ac.dump('final orbit ' .. sv())
local ins_get = ac.get()
-- ---- rendezvous ----
local cs = vessel.get_interface('Columbia')
local function cset(n, p) if cs:get_panelswitch(n) ~= -1 then cs:set_panelswitch(n, p) else ac.log('CSM switch not found: ' .. n) end end
cset('RNDZXPNDRFLTBusCB', 1); cset('Panel100RNDZXPDRSwitch', 2); cset('RNDZXPDRSwitch', 0)  -- CSM transponder on (CSM checklist)
ac.log('CSM transponder on')
-- DSKY register Rn as a number (5 digits, sign), nil when blank
local DIG = {[21]=0, [3]=1, [25]=2, [27]=3, [15]=4, [30]=5, [28]=6, [19]=7, [29]=8, [31]=9}
local function reg(n)
  local w = {}
  for i = 0, 11 do w[i + 1] = ac.e(531 + i) end
  local function c(r) return DIG[math.floor(w[r] / 32) % 32] end
  local function d(r) return DIG[w[r] % 32] end
  local function b(r) return math.floor(w[r] / 1024) % 2 == 1 end
  local digs, sp, sm
  if n == 1 then digs = {d(8), c(7), d(7), c(6), d(6)}; sp, sm = b(7), b(6)
  elseif n == 2 then digs = {c(5), d(5), c(4), d(4), c(3)}; sp, sm = b(5), b(4)
  else digs = {d(3), c(2), d(2), c(1), d(1)}; sp, sm = b(2), b(1) end
  local v = 0
  for _, x in ipairs(digs) do if x == nil then return nil end; v = v * 10 + x end
  if sm and not sp then v = -v end
  return v
end
-- null N85 (VG body X, Y, Z in 0.1 ft/s) with the TTCA translation bits (+X 6, -X 7, +Y 8, -Y 9, +Z 10, -Z 11)
local function null_vg(why)
  ac.dump('  null VG (' .. why .. ') start')
  local bits = {{6, 7}, {8, 9}, {10, 11}}
  local on = {0, 0, 0}
  local t0 = oapi.get_simtime()
  while oapi.get_simtime() - t0 < 300 do
    local v, n = ac.vn()
    if n ~= '85' then ac.keys('V16N85E'); proc.wait_simdt(1) end
    local done = true
    for i = 1, 3 do
      local g = reg(i)
      local want = 0
      if g and g > 3 then want = 1 elseif g and g < -3 then want = -1 end
      if g and math.abs(g) > 2 then done = false end
      if want ~= on[i] then
        if on[i] ~= 0 then ac.v:set_agcinputbit(25, bits[i][on[i] > 0 and 1 or 2], 0) end
        if want ~= 0 then ac.v:set_agcinputbit(25, bits[i][want > 0 and 1 or 2], 1) end
        on[i] = want
      end
    end
    if done then break end
    proc.wait_simdt(0.2)
  end
  for i = 1, 3 do if on[i] ~= 0 then ac.v:set_agcinputbit(25, bits[i][on[i] > 0 and 1 or 2], 0) end end
  ac.dump(string.format('  null VG (%s) done after %.0f s %s', why, oapi.get_simtime() - t0, sv()))
end
ac.hook['Burn +Z With TTCA'] = function(st) null_vg('burn') end
ac.hook['Null dVs'] = function(st) null_vg('burn') end
ac.hook['Null Residuals'] = function(st) null_vg('residuals') end
ac.hook['Insertion +20m'] = function(st)
  while ac.get() < ins_get + 1200 do proc.wait_simdt(1) end
  ac.dump('  at insertion +20m ' .. sv())
end
-- "At CSI-22m", "At CDH-30m Or After PC", "At TPI-26m": the LGC's TIG less N minutes
local function at_tig(st)
  local m = tonumber(st.text:match('%-(%d+)m'))
  ac.wait_until(function() return ac.tig_minus_now() <= m * 6000 end, 7200)
  ac.dump('  ' .. st.text .. ' (TIG - now = ' .. ac.tig_minus_now() .. ' cs)')
end
for _, sh in ipairs({'CSI', 'CDH', 'TPI'}) do
  for _, st in ipairs(CL[sh] or {}) do
    if st.text:match('^At %u%u%u%-%d+m') then ac.hook[st.text] = at_tig end
  end
end
ac.skip['P52 IMU Realign (REFSMMAT)'] = true
ac.skip['MCC1'] = true
ac.skip['MCC2'] = true
ac.skip['PC'] = true
ac.run('Rendezvous', 0, nil, 'Perform MCC1')
ac.dump('after TPI ' .. sv())
for _, m in ipairs(ac.manual) do ac.log('manual: ' .. m) end
ac.log('end')
