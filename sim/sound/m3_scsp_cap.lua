-- SPDX-License-Identifier: GPL-3.0-or-later
-- Sega Model 3 core for MiSTer FPGA
--
-- m3_scsp_cap.lua: the sound 68000's writes to both SCSPs and to both sound
-- RAMs, logged from MAME 0.289's model3 driver without patching it (write
-- taps), in the replay format of sim/sound/tb_m2_scsp.cpp:
--   <dir>/c0, <dir>/c1   one directory a chip
--     regs.bin  {u32 n, u16 byte offset, u16 data, u16 mask, u16 pad = 0}
--     ramw.bin  {u32 n, u32 byte address, u32 data | mask << 16}
-- n is the 44.1 kHz sample the access lands before: floor(t * 44100) + 1,
-- t the emulated time of the access (the chip's stream is brought up to t
-- by MAME before a register write). The bench applies writes between samples.
--
-- Board map (model3.cpp model3_snd): 0x000000 RAM 1, 0x100000 SCSP 1,
-- 0x200000 RAM 2, 0x300000 SCSP 2. Coins up and plays as census/play.lua.
--
--   M3SCSP_OUT=<dir> M3SCSP_S=<seconds> mame vf3 ... -autoboot_script m3_scsp_cap.lua
-- Output stays under sm3-builds/: it is derived from the game's program.

local OUT = os.getenv("M3SCSP_OUT") or error("M3SCSP_OUT")
local SECS = tonumber(os.getenv("M3SCSP_S") or "120")
local COIN_FRAME = tonumber(os.getenv("M3_COIN_FRAME") or "2400")

local cpu = manager.machine.devices[":audiocpu"]
local sp = cpu.spaces["program"]

local function opn(c, name) return assert(io.open(OUT .. "/c" .. c .. "/" .. name, "wb")) end
local f = { [0] = { regs = opn(0, "regs.bin"), ramw = opn(0, "ramw.bin") },
            [1] = { regs = opn(1, "regs.bin"), ramw = opn(1, "ramw.bin") } }
local buf = { [0] = { regs = {}, ramw = {} }, [1] = { regs = {}, ramw = {} } }
local cnt = { [0] = { regs = 0, ramw = 0 }, [1] = { regs = 0, ramw = 0 } }

local function flush(c, k)
	f[c][k]:write(table.concat(buf[c][k]))
	buf[c][k] = {}
end
local function put(c, k, s)
	local b = buf[c][k]
	b[#b + 1] = s
	cnt[c][k] = cnt[c][k] + 1
	if #b >= 8192 then flush(c, k) end
end
local function sample_n()
	return math.floor(manager.machine.time:as_double() * 44100.0) + 1
end

m3scsp_taps = {}   -- global: a collected tap is removed
local function reg_tap(c, base)
	m3scsp_taps[#m3scsp_taps + 1] = sp:install_write_tap(base, base + 0xfff, "scsp" .. c, function(off, data, mask)
		put(c, "regs", string.pack("<I4I2I2I2I2", sample_n(), off - base, data & 0xffff, mask & 0xffff, 0))
	end)
end
local function ram_tap(c, base)
	m3scsp_taps[#m3scsp_taps + 1] = sp:install_write_tap(base, base + 0x7ffff, "sram" .. c, function(off, data, mask)
		put(c, "ramw", string.pack("<I4I4I4", sample_n(), off - base, (data & 0xffff) | ((mask & 0xffff) << 16)))
	end)
end
reg_tap(0, 0x100000); reg_tap(1, 0x300000)
ram_tap(0, 0x000000); ram_tap(1, 0x200000)

-- inputs, as census/play.lua
local fields = {}
local function find(pattern)
	for _, port in pairs(manager.machine.ioport.ports) do
		for name, field in pairs(port.fields) do
			if string.lower(name):find(pattern) then return field end
		end
	end
end
local function hold(fl, on) if fl then fl:set_value(on and 1 or 0) end end
fields.coin = find("^coin 1")
fields.start = find("^1 player start") or find("^p1 start") or find("start 1")
fields.p = {}
for _, pat in ipairs({ "^p1 button 1", "^p1 button 2", "^p1 button 3", "^p1 up", "^p1 down", "^p1 left", "^p1 right" }) do
	local fl = find(pat)
	if fl then table.insert(fields.p, fl) end
end

local frame = 0
local done = false
local function finish()
	if done then return end
	done = true
	for c = 0, 1 do flush(c, "regs"); flush(c, "ramw"); f[c].regs:close(); f[c].ramw:close() end
	local s = assert(io.open(OUT .. "/count.txt", "w"))
	s:write(string.format("samples %d\n", sample_n()))
	for c = 0, 1 do s:write(string.format("c%d regs %d ramw %d\n", c, cnt[c].regs, cnt[c].ramw)) end
	s:close()
end
cap_frame = emu.add_machine_frame_notifier(function()
	frame = frame + 1
	local t = frame - COIN_FRAME
	if t >= 0 and t < 10 then hold(fields.coin, true) else hold(fields.coin, false) end
	if t >= 120 and t < 130 then hold(fields.start, true)
	elseif t >= 400 and (t % 300) < 10 then hold(fields.start, true)
	else hold(fields.start, false) end
	if t >= 200 then
		for i, fl in ipairs(fields.p) do hold(fl, ((t // 7 + i * 3) % 5) == 0) end
	end
	if manager.machine.time:as_double() >= SECS then
		finish()
		manager.machine:exit()
	end
end)
cap_stop = emu.add_machine_stop_notifier(finish)
