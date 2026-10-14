-- Apollo 11 LM rendezvous, headless, 1x, unattended. Start: "apollo-c rndz start" (scenario 11 flown
-- by stock NASSP 9 through insertion: LM in orbit, P00, CSM rendezvous transponder on; made by
-- rstart.lua and the same file on both builds). NASSP's LM checklist "Rendezvous": RR checkout
-- designate, DAP data, P20 (R60 attitude, RR lock-on, marks), then CSI (P32), CDH (P33) and TPI
-- (P34), each burn in P41 nulled through the TTCA translation bits (channel 31, set_agcinputbit)
-- against N85; MCC1/MCC2, P52, PC and the AGS/DEDA rows skipped. The crew's PAD values (CSI and
-- TPI TIG) are the MCC's CSI data card in the start scenario (MCC_AP10CSI_t_CSI / _t_TPI). Ends
-- 10 minutes after the TPI burn. Needs the LEM Lua test hook (patches/0002).
dofile('Script/apollo-c/common.lua')
dofile('Script/apollo-c/cl_lm.lua')
ac.open('Eagle', 'apollo-c-rendezvous.log')
ac.WAIT_WARP = 1          -- headless fixed step: no warp
ac.get0 = 448826.78       -- GET at sim 0 of the start scenario (its MISSNTIME)
ac.tigaddr = 1825         -- LM TIG 03441
local INS_GET = 448740.2  -- insertion cut-off GET of the stock flight that made the start scenario
local PAD = {csi = '+00125E+00029E+00646E', tpi = '+00127E+00006E+03000E'} -- 125:29:06.46, 127:06:30.00
local phase = 'coast'     -- the script's own state, an edit column (which burn is being nulled)
local function words(a, n) local s = '' for i = 0, n - 1 do s = s .. string.format('%o ', ac.e(a + i)) end return s end
local function deg(c) return c * 360 / 32768 end
local csm = oapi.get_objhandle('Columbia')
local cs = vessel.get_interface('Columbia')
local function rel()  -- range m, range rate m/s (opening > 0), LOS angle off the LM +Z axis (the RR boresight), deg
  local ok, p = pcall(function() return ac.v:get_relativepos(csm) end)
  local ok2, v = pcall(function() return ac.v:get_relativevel(csm) end)
  if not (ok and ok2 and p and v) then return -1, 0, -1 end
  local r = math.sqrt(p.x ^ 2 + p.y ^ 2 + p.z ^ 2)
  local phi = -1
  pcall(function()
    local l = ac.v:global2local(cs:get_globalpos())
    phi = math.deg(math.acos(l.z / math.sqrt(l.x ^ 2 + l.y ^ 2 + l.z ^ 2)))
  end)
  return r, (p.x * v.x + p.y * v.y + p.z * v.z) / r, phi
end
local function el() return ac.v:get_elements() end
local function sv()
  local r, rd, phi = rel()
  return string.format('GET=%.0f range %.0f rrate %.2f phi %.1f %s mass %.1f cdu %.1f %.1f %.1f', ac.get(), r, rd, phi, ac.orbit(), ac.v:get_mass(), deg(ac.e(26)), deg(ac.e(27)), deg(ac.e(28)))
end
ac.edits('apollo-c-rendezvous-edits.csv', 1, {
  {'range', function() local r = rel(); return r end}, {'rrate', function() local r, rd = rel(); return rd end},
  {'phi', function() local r, rd, p = rel(); return p end},
  {'apo', function() local e = el(); return e.a * (1 + e.e) end}, {'peri', function() local e = el(); return e.a * (1 - e.e) end},
  {'mass', function() return ac.v:get_mass() end},
  {'pitch', function() return math.deg(ac.v:get_pitch()) end}, {'bank', function() return math.deg(ac.v:get_bank()) end},
  {'yaw', function() return math.deg(ac.v:get_yaw()) end},
  {'cdux', function() return ac.e(26) end}, {'cduy', function() return ac.e(27) end}, {'cduz', function() return ac.e(28) end},
  {'cdut', function() return deg(ac.e(29)) end}, {'cdus', function() return deg(ac.e(30)) end},
  {'phase', function() return phase end}, {'radmodes', function() return string.format('%o', ac.e(72)) end},
  {'markctr', function() return ac.e(1842) end}, {'tig', function() return words(1825, 2) end},
  {'tcsi', function() return words(1947, 2) end}, {'ttpi', function() return words(1949, 2) end},
  {'tcdh', function() return words(1022, 2) end}, {'delvlvc', function() return words(1819, 6) end}})
proc.bg(function()
  local mm = ac.e(521)
  while true do
    ac.dump('track ' .. sv())
    local t = oapi.get_simtime()
    while oapi.get_simtime() - t < 60 do
      if ac.e(521) ~= mm then mm = ac.e(521); ac.dump(string.format('  MODREG -> %o %s', mm, sv())) end
      proc.wait_simdt(0.2)
    end
  end
end)
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
local function regs() return string.format('R1 %s R2 %s R3 %s', tostring(reg(1)), tostring(reg(2)), tostring(reg(3))) end
-- every display row the sheets wait for is logged with its registers (N11, N37, N55, N75, N81,
-- N82, N58, N59, N85 ...): wrap the default note handler's dump
local dump0 = ac.dump
function ac.dump(tag, cm)
  if tag:match('^%s*seen') then local v, n = ac.vn(); tag = tag .. string.format(' [V%sN%s %s]', v, n, regs()) end
  dump0(tag, cm)
end
-- Crew answers to R60 attitude requests. F50N18 is R60's "please perform maneuver": PRO maneuvers
-- (V06N18 steady while it runs), and when the maneuver ends R60 shows F50N18 again, answered with
-- ENTER (done). R61 (P20's tracking attitude, every pass while the LOS is more than 15 deg off, and
-- again after an ENTER while the previous pass's RR designate still runs) raises F50N18 at any
-- time over the sheet's own displays. Rule: an F50N18 within 30 s of a V06N18 is the end of a
-- maneuver (ENTER), any other is a request (PRO). The script's key sequences and the background
-- answers below exclude each other: busy_n counts the script's sequences (and a burn) in
-- progress, crew is set while a background answer types; the script waits for that answer.
local busy_n, crew = 0, false
local function hold() while crew do proc.wait_simdt(0.1) end; busy_n = busy_n + 1 end
local function release() busy_n = busy_n - 1 end
local keys1 = ac.keys
local function keys0(seq) hold(); keys1(seq); release() end
local pro1 = ac.pro
local function pro0() hold(); pro1(); release() end
local function f5018() local v, n, p, fl = ac.vn(); return v == '50' and n == '18' and fl end
local last0618 = -1000  -- sim time a V06N18 (an R60 maneuver running) was last seen
proc.bg(function()
  while true do
    local v, n, p, fl = ac.vn()
    if v == '06' and n == '18' and not fl then last0618 = oapi.get_simtime() end
    proc.wait_simdt(0.25)
  end
end)
local function answer5018(why)  -- the caller holds the DSKY (hold, or crew)
  if oapi.get_simtime() - last0618 < 30 then
    dump0('  crew: F50N18 after the maneuver (' .. why .. '): ENTER [' .. regs() .. ']')
    ac.set('DskySwitchEnter', 1); proc.wait_simdt(0.6); ac.set('DskySwitchEnter', 0); proc.wait_simdt(0.3)
  else
    dump0('  crew: F50N18 request (' .. why .. '): PRO, maneuver [' .. regs() .. ']')
    pro1()
  end
end
-- answer F50N18 until something else flashes (at most 8 answers), then the expected display
local function clear5018(why)
  for i = 1, 8 do
    if not f5018() then return true end
    answer5018(why)
    proc.wait_simdt(3)
    ac.wait_until(function() local a, b, c, f = ac.vn(); return f end, 300)
    proc.wait_simdt(1)
  end
  return not f5018()
end
-- a sheet PRO whose display (the note before it) is covered by an R61 request: answer that first
function ac.pro()
  if f5018() and ac.last_wait ~= '5018' then
    dump0('  sheet PRO for V' .. (ac.last_wait or '?') .. ' but F50N18 is up: answer it first')
    hold(); clear5018('before the sheet PRO'); release()
    if ac.last_wait then
      local v, n = ac.last_wait:sub(1, 2), ac.last_wait:sub(3, 4)
      ac.wait_until(function() local a, b, c, f = ac.vn(); return a == v and b == n and f end, 60)
    end
    proc.wait_simdt(1)
  end
  pro0()
end
local waitvn1 = ac.wait_vn
function ac.wait_vn(v, n, flash, timeout) ac.waiting = v .. n; ac.last_wait = v .. n; local r = waitvn1(v, n, flash, timeout); ac.waiting = nil; return r end
-- the sheet's ENTER on F50N18 (P20 after the maneuver, P41's bypass): R61 may ask again while the
-- RR designate of the pass before runs; the crew answers again every 10 s (as above), at most 12
-- The DAP mode verbs V76E / V77E (minimum impulse / rate command and attitude hold) are checked:
-- if OPR ERR comes on with the entry, the crew resets and keys it again (once). A V77 that lit OPR
-- ERR on stock left the DAP in minimum impulse, so the translation pulses of the next burn tumbled
-- the LM (CDH, stock ref_rendezvous_H under af918f3c).
local function oprerr() return math.floor(ac.v:get_agcchannel(9) / 64) % 2 == 1 end
function ac.keys(seq, n)
  local was = f5018()
  local dapverb = (seq == 'V76E' or seq == 'V77E') and not oprerr()
  keys0(seq)
  if dapverb then
    proc.wait_simdt(1)
    if oprerr() then
      dump0('  crew: OPR ERR after ' .. seq .. ': RSET, key it again')
      keys0('R'); proc.wait_simdt(1); keys0(seq); proc.wait_simdt(1)
      dump0('  ' .. seq .. ' again: OPR ERR ' .. (oprerr() and 'still on' or 'off'))
    end
  end
  if seq ~= 'E' or not was then return end
  hold()
  for i = 1, 12 do
    proc.wait_simdt(3)
    if not f5018() then break end
    proc.wait_simdt(7)
    if not f5018() then break end
    answer5018(string.format('again after the ENTER, %d', i))
    if i == 12 then dump0('  F50N18 still up after 12 answers') end
  end
  release()
end
-- RR checkout designate (V41N72 to trunnion 0, shaft 283): monitor N72 until the antenna is there
ac.hook['Monitor RR Angles'] = function(st)
  local ok = ac.wait_until(function()
    local t, s = reg(1), reg(2)
    if not (t and s) then return false end
    local dt, ds = math.abs(t) % 36000, math.abs(s - 28300) % 36000
    if dt > 18000 then dt = 36000 - dt end
    if ds > 18000 then ds = 36000 - ds end
    return dt <= 50 and ds <= 50
  end, 180)
  dump0('  RR angles ' .. (ok and 'reached' or 'NOT reached in 180 s') .. ' [' .. regs() .. ']')
end
ac.hook['When Marks=1'] = function(st)
  local ok = ac.wait_until(function() return ac.e(1842) >= 1 end, 600)
  dump0('  marks ' .. ac.e(1842) .. (ok and '' or ' (600 s timeout)') .. ' ' .. sv())
end
local function marks(st) dump0('  ' .. st.text .. ': MARKCTR ' .. ac.e(1842)) end
for _, t in ipairs({'Marks=10', 'Marks=4', 'Marks=3', 'Marks=5'}) do ac.hook[t] = marks end
ac.hook['Insertion +20m'] = function(st)
  while ac.get() < INS_GET + 1200 do proc.wait_simdt(1) end
  dump0('  at insertion +20m ' .. sv())
end
-- PAD loads: the display the sheet names, then V25E and the PAD time (hh, mm, ss.ss)
local function padload(v, n, keys)
  return function(st)
    ac.wait_vn(v, n, true, 900); proc.wait_simdt(1)
    ac.dump('  seen F ' .. v .. ' ' .. n .. ' (LGC value)')
    keys0('V25E' .. keys); proc.wait_simdt(1)
    ac.dump('  seen F ' .. v .. ' ' .. n .. ' (PAD loaded)')
  end
end
ac.hook['F 06 11 TIG of CSI (Load PAD Value)'] = padload('06', '11', PAD.csi)
ac.hook['F 06 37 TIG of TPI (Load PAD Value)'] = padload('06', '37', PAD.tpi)
-- "At CSI-22m", "At CDH-30m Or After PC", "At TPI-26m": the LGC's TIG less N minutes
local function at_tig(st)
  local m = tonumber(st.text:match('%-(%d+)m'))
  ac.wait_until(function() return ac.tig_minus_now() <= m * 6000 end, 7200)
  dump0('  ' .. st.text .. ' (TIG - now = ' .. ac.tig_minus_now() .. ' cs) ' .. sv())
end
for _, sh in ipairs({'CSI', 'CDH', 'TPI'}) do
  for _, st in ipairs(CL[sh] or {}) do
    if st.text:match('^At %u%u%u%-%d+m') then ac.hook[st.text] = at_tig end
  end
end
-- null N85 (VG body X, Y, Z in 0.1 ft/s) with the TTCA translation bits (+X 6, -X 7, +Y 8, -Y 9,
-- +Z 10, -Z 11 of channel 31). Above 2 ft/s an axis thrusts continuously; below, the crew pulses
-- (0.02 s per 0.1 ft/s, at least 0.1 s: a pulse moves VG about 4 times what the jets' nominal
-- 1 ft/s^2 would give, the DAP firing whole cycles), then waits two display updates (P41
-- refreshes N85 every 2 s). Done when every axis shows 0.2 ft/s or less for 1.6 s after the
-- settling (a display being rewritten reads wrong for a moment, so no single reading ends it).
local burn = {CSI = 'csi', CDH = 'cdh', TPI = 'tpi'}
local cur = 'csi'
local function null_vg(why)
  hold()
  phase = cur .. ' ' .. why
  ac.dump('  null VG (' .. phase .. ') start [' .. regs() .. '] ' .. sv())
  local bits = {{6, 7}, {8, 9}, {10, 11}}
  local on = {0, 0, 0}
  local function set(i, want)
    if want == on[i] then return end
    if on[i] ~= 0 then ac.v:set_agcinputbit(25, bits[i][on[i] > 0 and 1 or 2], 0) end
    if want ~= 0 then ac.v:set_agcinputbit(25, bits[i][want > 0 and 1 or 2], 1) end
    on[i] = want
  end
  local function sgn(x) return x > 0 and 1 or -1 end
  local now = oapi.get_simtime
  local t0, last_pulse, quiet, pulses = now(), -100, 0, 0
  while now() - t0 < 400 do
    local v, n = ac.vn()
    if n ~= '85' then keys1('V16N85E'); proc.wait_simdt(1) end
    local g = {reg(1), reg(2), reg(3)}
    if g[1] and g[2] and g[3] then
      local big = math.abs(g[1]) >= 20 or math.abs(g[2]) >= 20 or math.abs(g[3]) >= 20
      if big then
        for i = 1, 3 do set(i, math.abs(g[i]) >= 20 and sgn(g[i]) or 0) end
        quiet = 0
      else
        for i = 1, 3 do set(i, 0) end
        if now() - last_pulse >= 4.5 then
          if math.abs(g[1]) <= 2 and math.abs(g[2]) <= 2 and math.abs(g[3]) <= 2 then
            quiet = quiet + 1
            if quiet >= 8 then break end
          else
            quiet = 0
            local dur = {}
            for i = 1, 3 do dur[i] = math.abs(g[i]) > 2 and math.max(0.1, 0.02 * math.abs(g[i])) or 0; if dur[i] > 0 then set(i, sgn(g[i])) end end
            local p0 = now()
            while on[1] ~= 0 or on[2] ~= 0 or on[3] ~= 0 do
              proc.wait_simdt(0.02)
              for i = 1, 3 do if on[i] ~= 0 and now() - p0 >= dur[i] then set(i, 0) end end
            end
            pulses = pulses + 1
            last_pulse = now()
          end
        end
      end
    end
    proc.wait_simdt(0.2)
  end
  for i = 1, 3 do set(i, 0) end
  ac.dump(string.format('  null VG (%s) done after %.1f s, %d pulses [%s] %s', phase, now() - t0, pulses, regs(), sv()))
  phase = 'coast'
  release()
end
-- V06N86 (keyed by the crew before a P41 burn: dV in local vertical) is a steady display
ac.hook['F 06 86 dVX, dVY, dVZ'] = function(st)
  local ok = ac.wait_vn('06', '86', nil, 10); proc.wait_simdt(1)
  ac.dump('  seen 06 86' .. (ok and '' or ' (not seen in 10 s)'))
end
-- after a P41 burn the sheet waits for F16N85; it may stay steady: 60 s at most
ac.hook['F 16 85 VG XYZ (LM)'] = function(st)
  local ok = ac.wait_vn('16', '85', true, 60); proc.wait_simdt(1)
  ac.dump('  seen 16 85 after the burn' .. (ok and ' (flashing)' or ' (not flashing in 60 s)'))
end
ac.hook['Burn +Z With TTCA'] = function(st) null_vg('burn') end
ac.hook['Null dVs'] = function(st) null_vg('burn') end
ac.hook['Null Residuals'] = function(st) null_vg('residuals') end
-- crew answers the sheets do not list, from a background branch while no key sequence runs:
-- (none while the crew is loading data: V21-V25 flashing)
-- KEY REL when its lamp has been on 10 s (an internal display waits behind a crew monitor verb);
-- F06N49 (R22: a mark's state-vector change above the thresholds, please verify) is accepted
-- with PRO; F06N99 (a W-matrix load) still flashing 20 s while the sheet waits for another
-- display (its PRO went to an R61 request) gets its PRO; an F50N18 the sheet is not waiting for
-- (also during the sheet's clock waits) is answered as above after 10 s.
-- Every answer is logged.
proc.bg(function()
  local krel, n49, a5018, n99 = 0, 0, 0, 0
  while true do
    proc.wait_simdt(0.5)
    local v, n, p, fl = ac.vn()
    local load = fl and (v == '21' or v == '22' or v == '23' or v == '24' or v == '25')
    if busy_n > 0 or load then krel, n49, a5018, n99 = 0, 0, 0, 0 else
      krel = (math.floor(ac.v:get_agcchannel(9) / 16) % 2 == 1) and krel + 1 or 0
      n49 = (v == '06' and n == '49' and fl) and n49 + 1 or 0
      a5018 = (f5018() and ac.waiting ~= '5018') and a5018 + 1 or 0
      n99 = (v == '06' and n == '99' and fl and ac.waiting and ac.waiting ~= '0699') and n99 + 1 or 0
      if krel >= 20 then
        crew = true; dump0('  crew: KEY REL (lamp on 10 s)'); keys1('K'); crew = false; krel = 0
      elseif n49 >= 4 then
        crew = true; dump0('  crew: F06N49 accepted, PRO [' .. regs() .. '] ' .. sv()); pro1(); crew = false; n49 = 0
      elseif a5018 >= 20 then
        crew = true; answer5018('while the sheet waits for ' .. (ac.waiting and ('V' .. ac.waiting) or 'its clock')); crew = false; a5018 = 0
      elseif n99 >= 40 then
        crew = true; dump0('  crew: F06N99 left up while the sheet waits for V' .. ac.waiting .. ': PRO [' .. regs() .. ']'); pro1(); crew = false; n99 = 0
      end
    end
  end
end)
local run0 = ac.run
function ac.run(sheet, depth, from, to)
  if burn[sheet] then cur = burn[sheet]; dump0('  ' .. sheet .. ' sheet ' .. sv()) end
  return run0(sheet, depth, from, to)
end
-- TPI sheet: "POSS F 50 18" (a possible R61 attitude request) followed by PRO, 06 18, F 50 18,
-- "When attitude OK:", ENTR. A lone PRO there would answer P34's F16N45 instead (an early final
-- computation) when no request is up, so the five rows become one conditional step: if F50N18
-- comes within 30 s, PRO (auto maneuver), wait for F50N18 again, ENTER.
do
  local t, out, skip = CL['TPI'], {}, 0
  for _, st in ipairs(t) do
    if skip > 0 then skip = skip - 1 else table.insert(out, st) end
    if st.text == 'POSS F 50 18 Request Maneuver to FDAI RPY angles' then skip = 5 end
  end
  if #out ~= #t - 5 then ac.die('TPI sheet: POSS F 50 18 rows not as expected') end
  CL['TPI'] = out
end
-- Rendezvous sheet: "MODE CONT PGNS - AUTO" comes before "V37E 20E" (the sheet has it right after).
-- R61's first G+N,AUTO test (channel 31 bit 14) runs after LPS20.1, which takes the AGC about 4 s of
-- interpreter time: the sheet's order works on yaAGC because the switch lands inside that time. The
-- C executive runs the job in zero AGC time, so the test saw ATT HOLD, skipped R60 and designated the
-- RR off the LOS (alarm 503). With the switch first both builds see AUTO at the test (P00 in AUTO
-- holds the attitude it has).
do
  local t, iv, im = CL['Rendezvous'], nil, nil
  for i, st in ipairs(t) do
    if st.keys == 'V37E20E' then iv = i end
    if iv and not im and st.item == 'ModeControlPGNSSwitch' and st.pos == 2 then im = i end
  end
  if not iv or im ~= iv + 1 then ac.die('Rendezvous sheet: V37E 20E / MODE CONT AUTO rows not as expected') end
  t[iv], t[im] = t[im], t[iv]
end
ac.hook['POSS F 50 18 Request Maneuver to FDAI RPY angles'] = function(st)
  if not ac.wait_until(f5018, 30) then dump0('  no F50N18 within 30 s (P34 attitude request not made)'); return end
  dump0('  F50N18 in P34: PRO [' .. regs() .. ']'); pro0()
  ac.wait_until(f5018, 300); proc.wait_simdt(1)
  dump0('  F50N18 after the maneuver: ENTER [' .. regs() .. ']'); ac.keys('E')
end
ac.skip['AGS Update'] = true
ac.skip['P52 IMU Realign (REFSMMAT)'] = true
ac.skip['MCC1'] = true
ac.skip['MCC2'] = true
ac.skip['PC'] = true
proc.wait_simdt(2)
ac.dump('start ' .. sv())
ac.run('Rendezvous', 0, nil, 'Perform MCC1')
ac.dump('after TPI ' .. sv())
local t = oapi.get_simtime()
while oapi.get_simtime() - t < 600 do proc.wait_simdt(60); ac.dump('post-TPI ' .. sv()) end
for _, m in ipairs(ac.manual) do ac.log('manual: ' .. m) end
ac.log('end')
