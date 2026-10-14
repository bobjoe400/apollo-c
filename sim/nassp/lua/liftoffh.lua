-- Apollo 11 lunar liftoff (scenario 11, T-1m: Eagle on the surface, P12 loaded and waiting at
-- V06N74), unattended, 1x: NASSP's own LM checklist "Lunar Liftoff" from its "T-0:35" row (AVE G
-- on, V99 at TIG-5 s, PRO, APS burn to insertion, N85, V82 HA/HP). The scenario's P12 is used as
-- it is (no RSET / V37). Logs every 2 s from TIG-40 s: altitude, vertical and horizontal speed,
-- the APS level, attitude, MODREG; then the insertion orbit (Orbiter elements). Needs the LEM
-- Lua test hook (patches/0002).
dofile('Script/apollo-c/common.lua')
dofile('Script/apollo-c/cl_lm.lua')
ac.open('Eagle', 'apollo-c-liftoff.log')
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
  local ok, w = pcall(function() return ac.v:get_horizonairspeedvector() end)
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
ac.edits('apollo-c-liftoff-edits.csv', 1, {
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
-- P12 is selected afresh on both builds (a scenario's running P12 does not survive our load
-- restart: its program erasables are not all mapped), with the PAD TIG and targets as loaded
ac.keys('R')
ac.select(12)
if ac.wait_vn('06', '33', true, 30) then proc.wait_simdt(1); ac.dump('  F 06 33 (TIG) ' .. sv()); ac.pro() end
if ac.wait_vn('06', '76', true, 30) then proc.wait_simdt(1); ac.dump('  F 06 76 (VH, HDOT, XRNG) ' .. sv()); ac.pro() end
ac.run('Lunar Liftoff', 0, 'T-0:35')
ac.dump('after Lunar Liftoff ' .. sv())
proc.wait_simdt(10)
ac.dump('final orbit ' .. sv())
for _, m in ipairs(ac.manual) do ac.log('manual: ' .. m) end
ac.log('end')
