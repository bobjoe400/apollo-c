-- Apollo 11 entry from scenario 14 (Entry Prep, EI - 2:15h), headless (fixed step, no warp),
-- unattended: RSET, V37E00E, then NASSP's own checklists "Entry" and "CM-SM-Separation" (P61, CM/SM
-- sep, P62, P63 .. P67 by the CMC; "Wait Until EI -.." rows timed on GET, EI 195:03:06 = 702186 s),
-- the Splashdown sheet's ELS switches (PYRO ARM at 50,000 ft, ELS LOGIC / AUTO at 40,000 ft),
-- V16N67E after the drogues. Writes apollo-c-entry.log and the edit CSV apollo-c-entry-edits.csv
-- (DSKY, MODREG, CMC words, altitude, speeds, deceleration, position, vessel count).
dofile('Script/apollo-c/common.lua')
dofile('Script/apollo-c/cl_csm.lua')
ac.open('Columbia', 'apollo-c-entry.log')
ac.get0 = 694022.18
ac.ei = 702186
ac.WAIT_WARP = 1 -- headless fixed step: no warp
local function words(a, n) local s = '' for i = 0, n - 1 do s = s .. string.format('%o ', ac.e(a + i)) end return s end
local function vel()
  local ok, w = pcall(function() return ac.v:get_airspeedvector(REFFRAME.HORIZON) end)
  if ok and w then return w.y or 0, math.sqrt((w.x or 0) ^ 2 + (w.z or 0) ^ 2), math.sqrt(w.x ^ 2 + w.y ^ 2 + w.z ^ 2) end
  return 0, 0, 0
end
local lastv, lastt, g = nil, nil, 0
local function decel()
  local hd, vh, v = vel()
  local t = oapi.get_simtime()
  if lastv and t > lastt then g = (lastv - v) / (t - lastt) / 9.80665 end
  lastv, lastt = v, t
  return g
end
local function nobj() local n = -1; pcall(function() n = oapi.get_objcount() end); return n end
local function sv() local hd, vh, v = vel(); return string.format('GET=%.0f %s hdot %.1f vh %.1f vair %.1f nobj %d', ac.get(), ac.where(), hd, vh, v, nobj()) end
ac.skip['P52 IMU Realign (REFSMMAT)'] = true
ac.skip['P27 Update'] = true
ac.skip['Splashdown'] = true
proc.wait_simdt(5)
ac.edits('apollo-c-entry-edits.csv', 1, {
  {'alt', function() return ac.v:get_altitude() end}, {'hdot', function() local a, b = vel(); return a end},
  {'vh', function() local a, b = vel(); return b end}, {'vair', function() local a, b, c = vel(); return c end},
  {'g', decel}, {'lng', function() return math.deg(ac.v:get_equpos().lng) end},
  {'lat', function() return math.deg(ac.v:get_equpos().lat) end}, {'mass', function() return ac.v:get_mass() end},
  {'pitch', function() return math.deg(ac.v:get_pitch()) end}, {'bank', function() return math.deg(ac.v:get_bank()) end},
  {'yaw', function() return math.deg(ac.v:get_yaw()) end}, {'nobj', nobj}})
ac.dump('entry start ' .. sv())
ac.keys('R')
ac.keys('V37E00E')
proc.wait_simdt(5)
-- the sheet's event rows the crew watches for: wait for them (both builds alike)
local function wait_mm(mm, why) return function(st)
  ac.wait_until(function() return ac.e(521) == mm end, 3600); ac.dump('  ' .. why .. ' ' .. sv())
end end
local function wait_alt(m, why) return function(st)
  ac.wait_until(function() return ac.v:get_altitude() < m end, 3600); ac.dump('  ' .. why .. ' ' .. sv())
end end
ac.hook['At 0.05g'] = wait_mm(64, 'at 0.05 g (P64)')
ac.hook['At P64'] = wait_mm(64, 'at P64')
ac.hook['At P67'] = wait_mm(67, 'at P67')
ac.hook['Proceed when passing 90,000 feet'] = wait_alt(27432, 'passing 90,000 ft')
ac.hook['Proceed when passing 50,000 feet'] = wait_alt(15240, 'passing 50,000 ft')
ac.hook['Monitor altimeter, proceed when passing 40,000 feet'] = wait_alt(12192, 'passing 40,000 ft')
ac.run('Entry')
ac.dump('after Entry ' .. sv())
ac.run('CM-SM-Separation')
ac.dump('after CM-SM-Separation ' .. sv())
-- to the water: ELS switches as the Splashdown sheet sets them, V16N67E after the drogues
local armed, els, drogue, n0 = false, false, false, nobj()
local t0 = oapi.get_simtime()
while oapi.get_simtime() - t0 < 1800 do
  local a = ac.v:get_altitude()
  if not armed and a < 15240 then armed = true; ac.set('PyroArmASwitch', 1); ac.set('PyroArmBSwitch', 1); ac.dump('  50,000 ft: PYRO ARM ' .. sv()) end
  if not els and a < 12192 then els = true; ac.set('ELSLogicSwitch', 1); proc.wait_simdt(0.5); ac.set('ELSAutoSwitch', 1); ac.dump('  40,000 ft: ELS LOGIC, ELS AUTO ' .. sv()) end
  if els and not drogue and nobj() ~= n0 then
    drogue = true; ac.dump('  vessel count changed (apex cover / drogues) ' .. sv())
    ac.keys('V16N67E'); proc.wait_simdt(2); ac.dump('  N67 after the drogues ' .. sv())
  end
  if a < 3 then break end
  proc.wait_simdt(1)
end
ac.dump('SPLASHDOWN ' .. sv())
ac.keys('V16N67E'); proc.wait_simdt(3); ac.dump('final N67 ' .. sv())
proc.wait_simdt(5)
for _, m in ipairs(ac.manual) do ac.log('manual: ' .. m) end
ac.log('end')
