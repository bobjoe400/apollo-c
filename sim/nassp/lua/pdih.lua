-- Apollo 11 PDI and landing (scenario 10, T-15m, LM Eagle), unattended: RSET, V37E00E, then
-- NASSP's own LM checklist "PDI" from its "V37E 63E" row (P63, P64; the rows "At TIG +.."
-- timed on the LGC clock) up to "At TIG +10m"; then the LGC lands on its own (P64 -> P65,
-- PGNS AUTO) or, if P65 does not come, P66: MODE CONT ATT HOLD and ROD clicks (channel 016
-- bits 6/7, as NASSP's ROD keys) to a descent rate schedule. ENG STOP at ground contact.
-- Needs the LEM Lua test hook (patches/0002: get/set_panelswitch, get_agcerasable,
-- get_agcchannel, set_ttca_throttle, set_agcinputbit).
dofile('Script/apollo-c/common.lua')
dofile('Script/apollo-c/cl_lm.lua')
ac.open('Eagle', 'apollo-c-pdi.log')
ac.get0 = 368744.39
ac.tigaddr = 1825 -- LM TIG 03441
-- warp and the LM's RCS DAP: at 20x the stock LM tumbled into gimbal lock (PDI -9 min); at
-- 10x a DAP firing turned into a continuous 4-jet rotation (PDI -9) on one run and a 0.35 deg/s
-- overshoot with 3x the drift on another; 5x (stock, load to TIG-75 s, 2026-10-13) kept body
-- rates <= 0.28 deg/s (commanded maneuvers only), no alarms, middle gimbal < 4 deg. The LM's
-- coasting waits run at 5x; 1x from TIG-75 s (common.lua) through the landing.
ac.WAIT_WARP = 1  -- headless fixed step (-f 0.02): no warp
-- our build: a C job takes no AGC time, so R31 (V83, V16N54) reruns up to 64 times a centisecond
-- (sidecar "busy"); at 5x the main thread could not keep up and Orbiter hung (runs/discarded/
-- agc_pdi_5x_hang). Waits while V16N54 shows run at 1x, on both builds.
ac.warp_why = 'R31 (V16N54) active'
ac.warp_ok = function() local v, n = ac.vn(); return not (v == '16' and n == '54') end
local function words(a, n) local s = '' for i = 0, n - 1 do s = s .. string.format('%o ', ac.e(a + i)) end return s end
-- DPS: the LM's largest thruster
local dps
local function find_dps()
  local best = 0
  pcall(function()
    for i = 0, ac.v:get_thrustercount() - 1 do
      local h = ac.v:get_thrusterhandle(i)
      local m = ac.v:get_thrustermax0(h)
      if m > best then best, dps = m, h end
    end
  end)
  return best
end
local function vel()  -- local horizon frame: x east-ish, y up, z north-ish (m/s)
  local ok, w = pcall(function() return ac.v:get_airspeedvector(REFFRAME.HORIZON) end)
  if ok and w then return w.y or 0, math.sqrt((w.x or 0) ^ 2 + (w.z or 0) ^ 2) end
  return 0, 0
end
local function galt()
  local ok, a = pcall(function() return ac.v:get_altitude(1) end)  -- ALTMODE_GROUND
  if ok and a then return a end
  return -1
end
local function probe()
  local lvl = -1
  if dps then pcall(function() lvl = ac.v:get_thrusterlevel(dps) end) end
  local hd, vh = vel()
  local gc = 0
  pcall(function() if ac.v:get_groundcontact() then gc = 1 end end)
  -- attitude: Orbiter pitch/bank/yaw (deg), thrust in the horizon frame (kN: x, up, z, along
  -- the horizontal velocity), RCS jets firing, LGC CDU X/Y/Z (octal)
  local att = ''
  pcall(function()
    local t = ac.v:horizonrot(ac.v:get_thrustvector())
    local w = ac.v:get_airspeedvector(REFFRAME.HORIZON)
    local h = math.sqrt(w.x ^ 2 + w.z ^ 2)
    local along = h > 0 and (t.x * w.x + t.z * w.z) / h or 0
    local jets = 0
    for i = 0, ac.v:get_thrustercount() - 1 do
      local th = ac.v:get_thrusterhandle(i)
      if th ~= dps and ac.v:get_thrusterlevel(th) > 0 then jets = jets + 1 end
    end
    att = string.format(' pby %.1f %.1f %.1f thr %.2f %.2f %.2f along %.2f jets %d cdu %o %o %o', math.deg(ac.v:get_pitch()),
      math.deg(ac.v:get_bank()), math.deg(ac.v:get_yaw()), t.x / 1000, t.y / 1000, t.z / 1000, along / 1000, jets,
      ac.e(26), ac.e(27), ac.e(28))
  end)
  return string.format('dps %.3f mass %.1f hdot %.2f vh %.2f galt %.1f gc %d', lvl, ac.v:get_mass(), hd, vh, galt(), gc) .. att
end
local function sv() return string.format('GET=%.0f TIG=%s %s %s', ac.get(), words(1825, 2), ac.where(), probe()) end
local SW = {'EngineArmSwitch', 'THRContSwitch', 'MANThrotSwitch', 'EngineDescentCommandOverrideSwitch',
  'SCS_DECA_PWR_CB', 'SCS_ENG_ARM_CB', 'SCS_ENG_CONT_CB', 'DECA_GMBL_AC_CB', 'CDRManualEngineStop',
  'LMPManualEngineStop', 'ModeControlPGNSSwitch', 'ModeSelSwitch', 'AbortSwitch', 'AbortStageSwitch', 'LeftTTCATranslSwitch'}
local function switches()
  local s = 'switches'
  for _, n in ipairs(SW) do s = s .. ' ' .. n .. '=' .. ac.v:get_panelswitch(n) end
  ac.log(s)
end
local function since_ign() return ac.tig0 and (ac.e(20) * 16384 + ac.e(21) - ac.tig0) / 100 or -1 end
ac.skip['AGS Update'] = true
proc.wait_simdt(5)
ac.log(string.format('DPS thruster max0 %.0f N', find_dps()))
ac.dump('pdi start ' .. sv())
switches()
local function equ() local e = ac.v:get_equpos(); return e end
ac.edits('apollo-c-pdi-edits.csv', 1, {
  {'galt', galt}, {'hdot', function() local a, b = vel(); return a end}, {'vh', function() local a, b = vel(); return b end},
  {'lng', function() return math.deg(equ().lng) end}, {'lat', function() return math.deg(equ().lat) end},
  {'mass', function() return ac.v:get_mass() end}, {'dps', function() return ac.v:get_thrusterlevel(dps) end},
  {'pitch', function() return math.deg(ac.v:get_pitch()) end}, {'bank', function() return math.deg(ac.v:get_bank()) end},
  {'yaw', function() return math.deg(ac.v:get_yaw()) end},
  {'cdux', function() return ac.e(26) end}, {'cduy', function() return ac.e(27) end}, {'cduz', function() return ac.e(28) end},
  {'gc', function() return ac.v:get_groundcontact() and 1 or 0 end},
  {'unfc', function() local t = {} for i = 0, 5 do t[#t + 1] = string.format('%o', ac.e(1707 + i)) end return table.concat(t, ' ') end},
  {'unwc', function() local t = {} for i = 0, 5 do t[#t + 1] = string.format('%o', ac.e(1713 + i)) end return table.concat(t, ' ') end}})
ac.keys('R')
ac.keys('V37E00E')
proc.wait_simdt(5)
-- log every 10 s, every 2 s from TIG-40 s on (in parallel with the checklist); MODREG changes
local saved = false  -- a scenario at TIG+5m (before V57 and the LR reads), for offline replays
proc.bg(function()
  local mm = ac.e(521)
  while true do
    if not saved and since_ign() >= 300 then
      saved = true
      local ok = pcall(function() oapi.savescenario('apollo-c PDI TIG+5m', 'apollo-c: PDI at TIG+5m') end)
      ac.log('saved scenario apollo-c PDI TIG+5m: ' .. tostring(ok))
    end
    ac.dump('track ' .. sv())
    local dt = (ac.tig0 or ac.tig_minus_now() < 4000) and 2 or 10
    local t = oapi.get_simtime()
    while oapi.get_simtime() - t < dt do
      if ac.e(521) ~= mm then mm = ac.e(521); ac.dump(string.format('  MODREG -> %o at TIG %+.1f s %s', mm, since_ign(), sv())) end
      proc.wait_simdt(0.2)
    end
  end
end)
ac.hook['T-0:05'] = nil
ac.hook['At TIG +26s'] = function(st)
  ac.wait_until(function() return since_ign() >= 26 end, 120); ac.dump('  at TIG +26s ' .. sv()); switches()
end
ac.hook['P64'] = function(st)
  ac.wait_until(function() return ac.e(521) == 64 end, 600)  -- MODREG 64
  ac.dump(string.format('  P64 at TIG %+.1f s %s', since_ign(), sv()))
end
-- V16N68 is a monitor (verb 16), not the flashing V06N68 the row text names
ac.hook['F 06 68 Slant Range, TG, dH(LR-LGC)'] = function(st)
  ac.wait_vn('16', '68', nil, 20); proc.wait_simdt(2); ac.dump('  seen 16 68 ' .. sv())
end
ac.hook['When dH Within Limits'] = function(st) proc.wait_simdt(2) end
ac.hook['dH Decreasing'] = function(st) proc.wait_simdt(2) end
ac.select(63)
ac.run('PDI', 0, 'F 06 61 TG,TFI,CR', 'At TIG +10m')
ac.dump('handover ' .. sv())
-- ---- landing: P65 (auto) if the LGC goes there, else P66 with the ROD switch ----
local function rod(bit)  -- one ROD click: bit 5 = DescendPlus (+1 ft/s), 6 = DescendMinus
  ac.v:set_agcinputbit(14, bit, 1); proc.wait_simdt(0.2)
  ac.v:set_agcinputbit(14, bit, 0); proc.wait_simdt(0.3)
end
local function contact()
  local gc = false
  pcall(function() gc = ac.v:get_groundcontact() end)
  local a = galt()
  return gc or (a >= 0 and a < 1.0)  -- NASSP's own landed test (LEM.cpp: GroundContact or < 1 m)
end
local t0 = oapi.get_simtime()
local p66 = false
while not contact() and oapi.get_simtime() - t0 < 600 do
  local mm = ac.e(521)
  local a = galt()
  if not p66 and mm ~= 65 and mm ~= 66 and (a >= 0 and a < 30) then
    -- no P65 by 30 m (P64 ends near there): take over in P66
    ac.dump('  no P65 by 30 m: MODE CONT ATT HOLD, ROD ' .. sv())
    ac.set('ModeControlPGNSSwitch', 1); proc.wait_simdt(1)
    rod(5); p66 = true
  end
  if p66 then
    -- descent rate schedule: -(alt / 15) m/s, between -0.9 and -5 m/s
    local want = -math.max(0.9, math.min(5, a / 15))
    local hd = vel()
    if hd < want - 0.3 then rod(5); ac.log(string.format('  ROD + (hdot %.2f want %.2f alt %.1f)', hd, want, a))
    elseif hd > want + 0.3 then rod(6); ac.log(string.format('  ROD - (hdot %.2f want %.2f alt %.1f)', hd, want, a))
    else proc.wait_simdt(0.5) end
    proc.wait_simdt(0.5)
  else
    proc.wait_simdt(0.2)
  end
end
ac.dump(string.format('CONTACT at TIG %+.1f s %s', since_ign(), sv()))
ac.set('CDRManualEngineStop', 1)
proc.wait_simdt(3)
ac.set('EngineDescentCommandOverrideSwitch', 0)
ac.set('EngineArmSwitch', 1)
ac.set('ModeControlPGNSSwitch', 2)
proc.wait_simdt(20)
ac.dump(string.format('LANDED (+20 s) %s', sv()))
pcall(function() oapi.savescenario('apollo-c landed', 'apollo-c: LM landed') end)
for _, m in ipairs(ac.manual) do ac.log('manual: ' .. m) end
ac.log('end')
