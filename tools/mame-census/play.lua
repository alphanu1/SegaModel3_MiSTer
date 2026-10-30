-- Coin up at COIN_FRAME, start, then mash player 1 inputs so the census
-- covers gameplay as well as boot and attract.
local COIN_FRAME = tonumber(os.getenv("M3_COIN_FRAME") or "5200")
local frame = 0
local fields = {}

local function find(pattern)
	for _, port in pairs(manager.machine.ioport.ports) do
		for name, field in pairs(port.fields) do
			if string.lower(name):find(pattern) then
				return field
			end
		end
	end
end

local function hold(f, on)
	if f then f:set_value(on and 1 or 0) end
end

local function init()
	fields.coin = find("^coin 1")
	fields.start = find("^1 player start") or find("^p1 start") or find("start 1")
	fields.p = {}
	for _, pat in ipairs({ "^p1 button 1", "^p1 button 2", "^p1 button 3", "^p1 up", "^p1 down", "^p1 left", "^p1 right" }) do
		local f = find(pat)
		if f then table.insert(fields.p, f) end
	end
	local names = {}
	for _, port in pairs(manager.machine.ioport.ports) do
		for name, _ in pairs(port.fields) do table.insert(names, name) end
	end
	print("census fields: " .. table.concat(names, ", "))
end
init()

census_sub = emu.add_machine_frame_notifier(function()
	frame = frame + 1
	local t = frame - COIN_FRAME
	if t >= 0 and t < 10 then hold(fields.coin, true) else hold(fields.coin, false) end
	if t >= 120 and t < 130 then hold(fields.start, true)
	elseif t >= 400 and (t % 300) < 10 then hold(fields.start, true)
	else hold(fields.start, false) end
	if t >= 200 then
		for i, f in ipairs(fields.p) do
			hold(f, ((t // 7 + i * 3) % 5) == 0)
		end
	end
end)
