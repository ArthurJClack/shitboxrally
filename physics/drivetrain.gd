class_name Drivetrain
extends RefCounted
## Engine (torque curves measured on Engine Simulator's dyno), friction clutch, gearbox,
## differentials (open / LSD / viscous / locked) and the auto-clutch & automatic gearbox assists.

signal gear_changed(gear: int)
signal stalled
signal started
signal blow_off(boost: float)
## The driveline's slack was taken up with a bang (on / off the throttle, a clutch dump): 0..2.
signal clunk(strength: float)
## A shift has finished: the new gear is in (g != 0). `work` = how hard the box had to work to get it in
## (0 = the revs already matched, 1 = a big mismatch): a synchro's thunk, a dog box's crack (car_sfx.gd).
signal engaged(g: int, work: float)

const RPM_TO_RAD := TAU / 60.0
const ENGINE_BUILD := preload("res://src/data/engine_build.gd")
const NL := preload("res://src/car/native/layout.gd")

# --- engine
var curve_rpm := PackedFloat32Array()
var curve_on := PackedFloat32Array()
var curve_off := PackedFloat32Array()
static var friction_scaled := true   # (bench A/B: SBR_FRICTION_SCALE=0)
var torque_scale := 1.0
var idle_rpm := 850.0
var rev_limit := 6500.0
var redline := 6500.0
var engine_inertia := 0.16
var omega_e := 0.0
var running := true
var throttle_eff := 0.0
# Intake manifold filling: the throttle plate opens at once but the air behind it takes a couple of
# engine revolutions to fill the manifold, so torque follows the pedal with a lag of about
# 120 V_manifold / (eta_v V_disp rpm) s (manifold ~ the engine's own size): ~0.18 s at idle, ~0.05 s at
# 3,000, ~0.02 s at 7,000. Why a clutch dropped at idle bogs before a stab of throttle can catch it.
const MANIFOLD_K := 140.0         # s x rpm (the 120 / eta_v above with V_manifold ~ V_disp)
static var manifold_on := true    # (static: kill switch / benches, SBR_MANIFOLD=0)
var _man := 0.0                   # air in the manifold as a share of the throttle's (0..1)
# Idle control: an idle air valve / drive-by-wire idle loop, not a foot on the pedal. It only has a
# limited bypass to play with (IDLE_AUTH over what idling takes) and it reacts through the manifold
# like any other air, so a dropped clutch drags the revs down before it catches them (or stalls).
const IDLE_AUTH := 0.4            # most extra load the idle valve can add past steady idle (its bypass at idle revs: a
                                  # small opening fills a slow-turning engine's manifold to ~2/3 atmosphere)
const IDLE_BW := 6.0              # rad/s: how quick the loop is (scaled by the engine's own inertia)
const IDLE_TI := 1.0              # s: integral time (holds idle creeping in gear)
var _idle_i := 0.0                # the loop's integral, rad/s x s
var _off_floor := 0.0             # closed-throttle drag never drops under this at low revs (setup)
var engine_torque := 0.0
var _limiter := 0.0
var _fuel_cut := false          # the last substep's throttle was cut by the limiter, not the driver
var _starter := 0.0
var _restart_assist := false   # after a restart, slip the clutch for the driver until moving
var _auto_crank := -1.0        # >0: seconds until the game restarts an engine physics stalled
# MASTER_PLAN 7.7 G: clutch heat, turbo heat soak, money shifts; F: fuel.
var clutch_temp := 40.0          # C: slipping it heats it; past ~250 C it fades
var over_rev := 0.0              # this tick: how far past the limiter a downshift dragged it (0 = fine)
var fuel_out := false            # the tank is dry: it won't run or start
var _soak := 0.0                 # turbo heat soak 0..1: long spells on boost cost a little of it
# Driveline backlash (MASTER_PLAN 7.7 G): free play in the gears, diff and joints. When the torque
# through the driveline changes direction (on the throttle / off it) nothing is transmitted until
# the slack is taken up the other way, then it all arrives at once - the shunt and clunk of a
# worn old car. Measured at the wheels; the engine side has to turn the overall ratio times that.
var lash_deg := 1.5              # degrees of free play at the wheels (worn cars more)
var _lash_side := 1.0            # which way the slack is taken up: +1 drive, -1 overrun
var _lash_left := 0.0            # engine-side radians still to turn before it bites (0 = in contact)

# Driveline torsion: the shafts between the gearbox and the wheels (prop shaft, half shafts, the
# tire's sidewall winding up) aren't rigid. They wind up under torque like a spring and spring back:
# the shudder of a hard launch on grip, a RWD axle hopping when the tires bite and let go, the
# shunt of an off-throttle snap. Solved implicitly (backward Euler) in each 960 Hz substep, so it's
# stable at any stiffness; the gearbox side has next to no inertia of its own, so with the clutch
# slipping the shafts just carry what the clutch passes. Measured at the wheels.
static var torsion_on := true     # (static: kill switch / benches, SBR_TORSION=0)
var torsion := true               # this car right now (off for far-away cars: Car._apply_sim_level)
var shaft_k := 12000.0            # N m per radian of wind-up at the wheels, all driven wheels together
var shaft_zeta := 0.12            # damping ratio: lightly damped, a shaft rings for a few cycles
var wind := 0.0                   # wind-up now (rad at the wheels, + = driving)
var shaft_torque := 0.0           # what the shafts carry this substep (N m at the wheels)
var _wind_v := 0.0                # wind-up rate (rad/s)

# --- turbo (optional): boost 0..1 builds with lag once the exhaust flow (rpm x throttle) is high enough
var turbo := {}
var boost := 0.0
var _boost_thr := 0.0
var _t_spool0 := 2500.0   # turbo numbers, read once (see set_turbo)
var _t_spool1 := 4000.0
var _t_lag := 0.5
var _t_gain := 0.4
var _bov_cool := 0.0
var _t_soak := 1.0        # how hard heat soak bites (an intercooler upgrade lowers it)
var _wg := 0.0            # wastegate opening 0..1 (_boost_step)
var _shaft := 0.0         # turbo shaft speed, as the boost it could make 0..1 (boost is the pressure)

# --- ECU features (src/data/engine_build.gd, def "ecu"): launch control, anti-lag, flat shift
var lc_rpm := 0.0         # launch control: >0 = below LAUNCH_V the limiter holds the revs here
var als := 0.0            # anti-lag: boost held off the throttle (0 = none)
var flat_shift := false   # throttle held through a shift: the ECU cuts spark instead of the driver lifting
var als_active := false   # anti-lag firing right now (car_sfx bangs, exhaust flames)
var launching := false    # launch control holding the revs right now (HUD / dash)
var _road_v := 0.0        # driven wheels' road speed (m/s), from prep_substeps
var _shift_rpm := 0.0     # revs when the shift started (flat shift holds them)
const LAUNCH_V := 3.0

# --- gearbox
var gears := PackedFloat32Array([3.5, 2.1, 1.4, 1.0, 0.8])
var reverse_ratio := 3.4
var final_drive := 4.0
var efficiency := 0.9
# Torque converter (def "converter": {stall_rpm, stall_tr}): an automatic's fluid coupling instead of
# a clutch. 0 = a clutch.
var tc_k := 0.0           # capacity factor K (rad/s per sqrt(N m)): the impeller takes (w_e / K)^2
var tc_tr := 2.0          # torque ratio at stall
var tc_ratio := 1.0       # torque multiplication right now (HUD / benches)
var shift_time := 0.25
## What kind of box (car def "gearbox_kind", upgrades.gd gearbox part): "synchro" (every road car's
## manual), "dog" (straight-cut dog box: no synchros, takes clutchless shifts, quick, whines),
## "seq" (sequential dog box: one gear at a time, quicker still) or "auto" (torque converter).
var box_kind := "synchro"
## This shift's real length (s): a synchro box takes longer when the revs are far from the new gear's
## (the synchro rings have to drag the input shaft to speed). = shift_time on the other boxes.
var shift_len := 0.25
## Output shaft speed (rad/s, the prop / final drive side): the gear whine's pitch (car_sfx.gd).
var box_out := 0.0
var shift_from := 0          # the gear this shift started from
var gear := 0
var pending_gear := 0
var shift_timer := 0.0
var since_shift := 10.0

# --- clutch
var clutch_capacity := 300.0
const BITE_START := 0.3   # pedal lift (0 = floored, 1 = up) where the plates start to touch
const BITE_END := 0.75    # fully clamped
var engagement := 1.0
var clutch_torque := 0.0
var _auto_engagement := 0.0
var clutch_assist := true      # low-speed clutch assist for the manual clutch (setting)
# Lugging (owner, 2026-11-02: "let me lug a bit more, just kick in at super low rpm"): once the clutch
# is home the engine can be dragged well under idle before the game touches the clutch. x idle_rpm.
const LUG_FLOOR := 0.55        # auto clutch: a locked clutch stays locked down to here
const ANTI_STALL := 0.5        # auto clutch: below this it dips the clutch so it doesn't stall
const ASSIST_IN := 0.6         # manual clutch assist: takes over below this road rpm (crawling / stopping)
const ASSIST_OUT := 1.1        # ... and hands back to the pedal once rolling past this
var _auto_locked := false
# The driver's foot on the clutch with auto clutch on (owner, 2026-11-03: "auto clutch ruins my clutch
# kicks and slides"): from the moment the pedal goes in, the pedal is the clutch - let it out fast and
# it bites as hard as it physically can (a kick), the auto clutch only steps back in to stop a stall.
var pedal_own := false
const PEDAL_OWN_IN := 0.12     # pedal travel that hands the clutch to the driver
var pedal_hands := false       # set by the car: a player's own pedal (not the AI, the grid, the pace car)
var assisting := false         # the assist is slipping the clutch right now (HUD)
var _assist_e := 1.0

# --- differentials
var layout := "FWD"
var front_diff := {"type": "open", "preload": 0.0, "ramp": 0.0, "viscous": 0.0}
var rear_diff := {"type": "open", "preload": 0.0, "ramp": 0.0, "viscous": 0.0}
var center_diff := {type = "viscous", split = 0.5, viscous = 60.0}
# Active diffs (type "active": a factory ACD / DCCD / ATTESA centre, or the garage's ECU-mapped
# clutch diff on any axle). An ECU sets the clutch pack's lock every tick from the pedals, the
# handbrake, the steering and a map picked by the ground under the car - like the Tarmac / Gravel /
# Snow switch on an Evo, but it switches itself. Tarmac: little lock, opens on the brakes and with
# lock on so the car turns in. Gravel: locks up on the throttle so the car drives out of a slide,
# gentler off it. Snow: nearly locked. The handbrake frees the centre completely (the rally trick:
# the rears lock without dragging the fronts with them). The car feeds it (set_active_inputs).
const ACTIVE_MAPS := {
	#          centre: preload (Nm), lock on throttle, lock off it   axle: preload, on, off
	"tarmac": [25.0, 0.28, 0.06, 35.0, 0.35, 0.1],
	"gravel": [70.0, 0.5, 0.18, 60.0, 0.5, 0.22],
	"snow":   [140.0, 0.7, 0.32, 80.0, 0.55, 0.3],
}
var active_map := "gravel"      # the map in use right now (HUD / debug): the heaviest of _map_w
var _map_w := Vector3(0.0, 1.0, 0.0)   # tarmac / gravel / snow weights, eased between surfaces
var _ai_thr := 0.0
var _ai_brk := 0.0
var _ai_hb := 0.0
var _ai_steer := 0.0
var _ai_v := 0.0


func setup(def: Dictionary, engine: Dictionary) -> void:
	# The engine's dyno curve with the car's bolt-ons / custom map on it (engine_build.gd).
	var pts: Array = ENGINE_BUILD.shaped_points(def, engine)
	if pts.is_empty():
		# Fallback curve (generic 1.6 l four) when no Engine Simulator data is present.
		pts = [{rpm = 1000, torque_on = 95, torque_off = -6}, {rpm = 3000, torque_on = 125, torque_off = -30},
			{rpm = 5000, torque_on = 120, torque_off = -45}, {rpm = 6500, torque_on = 95, torque_off = -60}]
	for p in pts:
		curve_rpm.append(float(p.rpm))
		curve_on.append(float(p.torque_on))
		curve_off.append(float(p.torque_off))
	redline = float(engine.get("redline", curve_rpm[curve_rpm.size() - 1]))
	torque_scale = def.get("torque_scale", 1.0)
	# Friction goes with the size of the engine. A car's engine is one of the dyno'd ones scaled to
	# its rated power, and the overrun curve used to stay the full-size engine's: a 2.8 V6 built
	# as 0.29 x the truck V8 dragged like the 5.7 (its idle took 36 % throttle, so the first third
	# of the pedal did nothing, and it engine-braked like a truck). Torque per litre varies about
	# as much as size does between the cars sharing a base engine, so the square root of the scale
	# is the size share of it (2.8 / 5.7 = 0.49, sqrt 0.29 = 0.54; 7.0 / 5.7 = 1.23, sqrt 1.54 = 1.24).
	if friction_scaled:
		var fk := sqrt(clampf(torque_scale, 0.25, 2.5))
		for i in curve_off.size():
			curve_off[i] *= fk
	idle_rpm = def.get("idle_rpm", 850.0)
	rev_limit = def.get("rev_limit", redline)
	engine_inertia = def.get("engine_inertia", 0.16)
	var g: Array = def.get("gears", [3.5, 2.1, 1.4, 1.0, 0.8])
	gears = PackedFloat32Array(g)
	reverse_ratio = def.get("reverse", 3.4)
	final_drive = def.get("final_drive", 4.0)
	shift_time = def.get("shift_time", 0.25)
	shift_len = shift_time
	box_kind = String(def.get("gearbox_kind", "synchro"))
	clutch_capacity = def.get("clutch_torque", 300.0)
	var conv: Dictionary = def.get("converter", {})
	tc_k = 0.0
	if not conv.is_empty():
		# K from the stall speed: flat foot against the brakes the engine settles where its torque
		# equals what the impeller takes, T = (w / K)^2.
		var stall := float(conv.get("stall_rpm", 2000.0))
		tc_tr = float(conv.get("stall_tr", 2.0))
		var t_stall := 0.0
		for i in curve_rpm.size():
			if curve_rpm[i] >= stall or i == curve_rpm.size() - 1:
				t_stall = curve_on[i] * torque_scale
				break
		tc_k = stall * RPM_TO_RAD / sqrt(maxf(t_stall, 20.0))
		box_kind = "auto"
	# Free play grows with wear (body_wear is the car's age / abuse in the garage).
	lash_deg = float(def.get("driveline_lash", 1.2 + 2.6 * float(def.get("body_wear", 0.0))))
	# Two half shafts in parallel (~14 kN m/rad each); a RWD prop shaft in series with them softens it
	# (an AWD car has two of those, and a shaft to each axle, so about the same again).
	var lay: String = def.get("drivetrain", "FWD")
	shaft_k = float(def.get("shaft_k", 28000.0 if lay == "FWD" else 20000.0 if lay == "RWD" else 24000.0))
	wind = 0.0
	_wind_v = 0.0
	layout = def.get("drivetrain", "FWD")
	if def.has("front_diff"):
		front_diff.merge(def.front_diff, true)
	if def.has("rear_diff"):
		rear_diff.merge(def.rear_diff, true)
	if def.has("center_diff"):
		center_diff.merge(def.center_diff, true)
	turbo = def.get("turbo", {})
	_t_spool0 = float(turbo.get("spool_start", 2500.0))
	_t_spool1 = float(turbo.get("spool_full", 4000.0))
	_t_lag = float(turbo.get("lag", 0.5))
	_t_gain = float(turbo.get("gain", 0.4))
	_t_soak = float(turbo.get("soak", 1.0))
	var ecu: Dictionary = def.get("ecu", {})
	lc_rpm = float(ecu.get("launch_rpm", 0.0))
	als = float(ecu.get("als", 0.0)) if not turbo.is_empty() else 0.0
	flat_shift = bool(ecu.get("flat", false))
	# Closed-throttle drag at low revs: friction (rings, bearings, valve train) has a big constant part
	# and pumping against a shut throttle is at its worst at idle, so the drag near idle is ~3/4 of what
	# it is at 2,500, not the near-zero (or even pushing) values some renders give under 1,500.
	_off_floor = 0.0
	_ff_idle = -1.0   # the idle feed-forward is worked out again on this curve
	_off_floor = 0.75 * minf(_interp(curve_off, 2500.0, false), -4.0)
	omega_e = idle_rpm * RPM_TO_RAD
	_man = _idle_air()   # settled at idle from the first tick
	_idle_i = 0.0
	gear = 0


## The throttle share an engine idling in neutral holds (what the idle loop's feed-forward gives).
func _idle_air() -> float:
	var on_i := torque_on(idle_rpm)
	var off_i := torque_off(idle_rpm)
	return clampf(-off_i / maxf(on_i - off_i, 1.0), 0.0, 1.0)


func rpm() -> float:
	return omega_e / RPM_TO_RAD


func gear_ratio(g: int = gear) -> float:
	if g > 0:
		return gears[mini(g, gears.size()) - 1] * final_drive
	if g < 0:
		return -reverse_ratio * final_drive
	return 0.0


func is_shifting() -> bool:
	return shift_timer > 0.0


## How far the revs at the start of this shift are from what gear `g` wants at this road speed,
## 0 (matched) .. 1 (3000 rpm or more out). With the clutch in the box's input shaft is still turning
## at about the revs it had when the old gear came out: that's what the synchro (or the dogs) must catch.
func shift_work(g: int) -> float:
	if g == 0:
		return 0.0
	var want := box_out * absf(gear_ratio(g)) / maxf(final_drive, 0.01) * 60.0 / TAU
	return clampf(absf(want - _shift_rpm) / 3000.0, 0.0, 1.0)


func shift_to(g: int, forward_speed: float) -> bool:
	g = clampi(g, -1, gears.size())
	# A stripped gear won't go in: the automatic picks the next one along, the manual stays out
	# (car_damage.gd grinds it and refuses before it gets here).
	while g > 0 and g in broken_gears:
		g += 1 if g > (pending_gear if is_shifting() else gear) else -1
		if g < 1 or g > gears.size():
			return false
	var target := pending_gear if is_shifting() else gear
	if g == target:
		return false
	if g == -1 and forward_speed > 2.5:
		return false
	if g > 0 and forward_speed < -2.5:
		return false
	shift_from = target
	pending_gear = g
	gear = 0
	_shift_rpm = rpm()
	shift_len = shift_time
	if box_kind == "synchro" and g > 0:
		# Synchro rings drag the box's input shaft to the new gear's speed before the sleeve can slide
		# over. shift_time is an ordinary one-gear shift (gaps up to ~2000 rpm); past that (a 1-2 off the
		# limiter, a big skip like 5th to 2nd at speed) the rings need a beat longer, up to +35 %.
		shift_len *= 1.0 + 0.35 * clampf((shift_work(g) * 3000.0 - 2000.0) / 3000.0, 0.0, 1.0)
	shift_timer = shift_len if g != 0 else 0.0
	if shift_timer <= 0.0:
		gear = g
	since_shift = 0.0
	gear_changed.emit(g)
	return true


func _interp(arr: PackedFloat32Array, r: float, power: bool) -> float:
	var n := curve_rpm.size()
	if r <= curve_rpm[0]:
		return arr[0] * clampf(r / curve_rpm[0], 0.0, 1.0)
	for i in range(1, n):
		if r <= curve_rpm[i]:
			var t := (r - curve_rpm[i - 1]) / (curve_rpm[i] - curve_rpm[i - 1])
			return lerpf(arr[i - 1], arr[i], t)
	# Past the last measured point: power falls off, friction keeps rising.
	var over := r - curve_rpm[n - 1]
	if power:
		return arr[n - 1] * maxf(0.0, 1.0 - over / 1500.0)
	return arr[n - 1] * (1.0 + over / curve_rpm[n - 1])


var health := 1.0     # engine condition / heat (EngineFluids.power_factor)
var no_start := false # drowned ignition / flat battery (car_damage.gd): the starter can't fire it
var starter_k := 1.0  # how hard the starter turns it over (battery voltage, car_damage.gd)
var iat := 1.0        # intake air density from its temperature (car.gd _intake_air): hot air, less power
var broken_gears: Array[int] = []   # stripped forward gears (car_damage.gd): they won't go in
var air := 1.0        # thin air at altitude (MASTER_PLAN 7.7 G): set by the car from the stage's region
var seized := false


func torque_on(r: float) -> float:
	return _interp(curve_on, r, true) * torque_scale * health * air * iat


func torque_off(r: float) -> float:
	return _low_drag(r, _interp(curve_off, r, false))


## The overrun curve with the low-rev drag floor (_off_floor) under 2,500 rpm.
func _low_drag(r: float, off: float) -> float:
	if r >= 2500.0 or _off_floor == 0.0:
		return off
	return minf(off, _off_floor * clampf(r / curve_rpm[0], 0.0, 1.0))


## Advances the turbo and returns the extra torque fraction it currently provides.
## The turbine runs on exhaust gas: revs x the air the throttle lets in x how hard the engine is
## working (load). Part throttle and free revving in neutral make little gas, so little boost; and
## the boost already in the manifold pushes more air through, which pushes more gas at the turbine
## (T_FEED) - why a big turbo sits there doing nothing and then comes on all at once. It spools
## quicker the more gas there is. The wastegate opens a beat late, so a fast build spikes past the
## set boost (WG_SPIKE) before it settles.
const T_FEED := 0.2         # share of the exhaust flow that is the boost feeding itself
const WG_SPIKE := 0.1       # how far past the set boost a fast build overshoots with the gate shut
const T_COAST := 1.4        # s: the turbo shaft running down with the throttle shut
const T_PRESS := 0.08       # s: the pressure filling the intake from a spinning turbo
func _boost_step(dt: float, r: float, thr: float) -> float:
	if turbo.is_empty():
		return 0.0
	# The turbo's numbers are read once in setup, not looked up by name eight substeps a tick.
	# Load: what the clutch is carrying against what the engine could make here. A slipping launch
	# clutch is full load; free revving in neutral or with the pedal down is next to none.
	var load_k := lerpf(0.45, 1.0, clampf(absf(clutch_torque) / maxf(_c_on * torque_scale * 0.8, 20.0), 0.0, 1.0))
	var flow := r * clampf((thr - 0.2) / 0.6, 0.0, 1.0) * load_k * (1.0 - T_FEED + T_FEED * minf(boost, 1.0))
	var target := smoothstep(_t_spool0 * (1.0 - T_FEED), _t_spool1 * (1.0 - 0.5 * T_FEED), flow)
	# Wastegate: shut below the set boost, opening over the last bit of it with a lag.
	_wg += (clampf((boost - 0.88) / 0.1, 0.0, 1.0) - _wg) * (1.0 - exp(-dt / 0.15))
	target *= 1.0 + WG_SPIKE * (1.0 - _wg)
	# Anti-lag: off the throttle above the spool point the ECU retards the spark (and cracks the
	# throttle / opens the air valve), so the fire in the manifold keeps the turbo spinning.
	als_active = false
	if als > 0.0 and running and thr < 0.25 and r > _t_spool0 * 0.85:
		var held := als * smoothstep(_t_spool0 * 0.85, _t_spool0 + 300.0, r)
		if held > target:
			target = held
			als_active = true
	# Flat shift: boost stays in the pipes through the shift (the throttle never closed).
	if flat_shift and is_shifting() and _boost_thr > 0.6:
		target = maxf(target, boost)
	# The shaft and the pressure are two things. The shaft spools with the lag; lift off and the
	# blow-off valve dumps the pressure in a blink, but the wheel keeps spinning (a 100k rpm rotor
	# takes a second or two to run down), so the boost is back almost at once after a gear change.
	# Measured turbo cars lose about a tenth of their boost through a quick shift, not all of it.
	var tau := _t_lag * clampf(1.15 - 0.35 * flow / maxf(_t_spool1, 1.0), 0.8, 1.15) if target > _shaft else T_COAST
	_shaft += (target - _shaft) * (1.0 - exp(-dt / tau))
	var p_tgt := _shaft * smoothstep(0.15, 0.45, thr)
	boost += (p_tgt - boost) * (1.0 - exp(-dt / (T_PRESS if p_tgt > boost else 0.12)))
	# Heat soak: a minute flat out on boost costs up to ~15 % of it until it's had time to cool
	# (anti-lag cooks the turbo much faster; a bigger intercooler shrugs more of it off).
	_soak = clampf(_soak + (boost * (0.03 if als_active else 0.012) - 0.008) * dt, 0.0, 1.0)
	return _t_gain * boost * (1.0 - 0.15 * _soak * _t_soak)


## The car was just put back on the road at a standstill (reset / recovery): engine running at idle
## (unless it's seized), any shift finished, a forward gear above 1st swapped for 1st, and the
## clutch feathered for the driver until moving - so it can't stall the moment it lands.
func on_reset() -> void:
	shift_timer = 0.0
	wind = 0.0
	_wind_v = 0.0
	if pending_gear != gear and pending_gear != 0:
		gear = pending_gear
	pending_gear = gear
	if gear > 1:
		gear = 1
		pending_gear = 1
		gear_changed.emit(1)
	_auto_crank = -1.0
	if seized:
		running = false   # a blown engine stays blown; gears and shift state still reset
		omega_e = 0.0
		return
	running = not fuel_out
	_starter = 0.0
	omega_e = idle_rpm * TAU / 60.0 if running else 0.0
	_man = _idle_air() if running else 0.0
	_idle_i = 0.0
	_restart_assist = gear != 0
	_auto_engagement = 0.0
	_auto_locked = false
	pedal_own = false


## Revs the clutch assists hold while pulling away: just above idle off the throttle, up towards
## the torque (or for a turbo, the spool) as the throttle goes down.
func launch_rpm(throttle: float) -> float:
	var top := 1900.0
	if not turbo.is_empty():
		top = maxf(top, float(turbo.get("spool_start", 2500.0)) + 400.0 - idle_rpm)
	return idle_rpm + 250.0 + clampf(throttle, 0.0, 1.0) * top


## Clutch engagement for pulling away: the plates carry about as much torque as the engine makes
## at these revs, scaled by how far the revs are towards the launch revs - below them the engine
## has torque to spare and climbs, above them the clutch bites harder and pulls them back. So the
## revs settle at launch_rpm() instead of being dragged down into a bog.
## `t_off_r` / `t_on_r` = the torques at `r`, already looked up (step()).
func _launch_eng(r: float, throttle: float, t_off_r: float, t_on_r: float) -> float:
	var lo := idle_rpm * 0.92
	var frac := clampf((r - lo) / maxf(launch_rpm(throttle) - lo, 1.0), 0.0, 2.0)
	var t_eng := maxf(lerpf(t_off_r, t_on_r, clampf(throttle, 0.0, 1.0)), 15.0)
	return clampf(t_eng * frac * 1.1 / maxf(clutch_capacity, 1.0), 0.0, 1.0) if frac < 1.6 else 1.0


var _c_on := 0.0      # _curves_at: the drive curve at those revs (before scale / health / air)
var _c_off := 0.0     # the overrun (friction) curve


## Both torque curves at `r` from one search of the rpm points: what _interp gives for each.
func _curves_at(r: float) -> void:
	var cr := curve_rpm
	var n := cr.size()
	var r0 := cr[0]
	if r <= r0:
		_c_on = curve_on[0] * clampf(r / r0, 0.0, 1.0)
		_c_off = _low_drag(r, curve_off[0] * clampf(r / r0, 0.0, 1.0))
		return
	for i in range(1, n):
		var ri := cr[i]
		if r <= ri:
			var a := cr[i - 1]
			var t := (r - a) / (ri - a)
			_c_on = lerpf(curve_on[i - 1], curve_on[i], t)
			_c_off = _low_drag(r, lerpf(curve_off[i - 1], curve_off[i], t))
			return
	# Past the last measured point: power falls off, friction keeps rising.
	var last := cr[n - 1]
	var over := r - last
	_c_on = curve_on[n - 1] * maxf(0.0, 1.0 - over / 1500.0)
	_c_off = curve_off[n - 1] * (1.0 + over / last)


## Turn the key: works when the engine is off, or dragged so low it's as good as stalled.
func crank() -> void:
	if seized or fuel_out or no_start:
		return
	if not running or rpm() < 300.0:
		running = false
		_starter = 1.5


## Per physics tick: shift timing, the stall auto-restart (it calls crank(); the starter motor
## itself turns the engine in step()) and the blow-off valve.
func update(dt: float) -> void:
	since_shift += dt
	if _auto_crank > 0.0:
		_auto_crank -= dt
		if _auto_crank <= 0.0 and not running:
			crank()
	if not turbo.is_empty():
		_bov_cool -= dt
		# A sharp lift with boost in the pipes (or a gear change on full boost) dumps it.
		# Only the driver lifting counts: the rev limiter cutting the fuel (and a stall) also
		# drops throttle_eff to 0, so sitting on the limiter fired the valve every 0.3 s.
		var cut := _fuel_cut or not running or als_active or (flat_shift and is_shifting())
		if not cut and boost > 0.2 and _boost_thr - throttle_eff > 0.35 and _bov_cool <= 0.0:
			blow_off.emit(boost)
			boost *= 0.3
			_bov_cool = 0.3
		# Frame-rate independent: a fixed 0.5 made the blow-off trigger differently at 120 Hz
		# than at 60, so the same lift could dump boost on one machine and not on another.
		if not cut:
			_boost_thr += (throttle_eff - _boost_thr) * (1.0 - exp(-dt * 60.0))
	if shift_timer > 0.0:
		shift_timer -= dt
		if shift_timer <= 0.0:
			gear = pending_gear
			if gear != 0:
				engaged.emit(gear, shift_work(gear))


var _up_key := -1.0
var _up_at := PackedFloat32Array()


## The flat-out upshift out of gear `g` (rpm): where the next gear, landing lower in the revs, pulls
## harder at the wheels (engine torque x its ratio) than this one still does, else just short of the
## limiter. That's what a quick driver (and a kickdown map) does: most engines want taking to the
## limiter, a peaky one or a wide gap shifts earlier. The old fixed 94 % of min(redline, limiter)
## short-shifted every car whose dyno curve stops before its real limiter (an Evo shifted at 6,100 of
## its 7,000). Worked out again when the box, final drive, limiter or tune changes.
func up_rpm_full(g: int = gear) -> float:
	var n := gears.size()
	if n == 0 or curve_rpm.is_empty():
		return minf(redline, rev_limit) * 0.94
	var key := torque_scale + rev_limit * 0.001 + final_drive
	for i in n:
		key += gears[i] * float(i + 2)
	if key != _up_key:
		_up_key = key
		_up_at.resize(n)
		var lim := rev_limit * 0.97
		for i in n:
			var at := lim
			if i < n - 1 and gears[i] > 0.0:
				var drop := gears[i + 1] / gears[i]
				var r := maxf(minf(redline, rev_limit) * 0.6, idle_rpm * 2.0)
				while r < lim:
					if _interp(curve_on, r, true) * gears[i] < _interp(curve_on, r * drop, true) * gears[i + 1]:
						at = r
						break
					r += 50.0
			_up_at[i] = at
	return _up_at[clampi(g - 1, 0, n - 1)]


## Automatic gearbox decisions (forward gears only, reverse is handled by the car).
func update_auto(throttle: float, road_omega := -1.0) -> void:
	if is_shifting() or since_shift < 0.8 or gear < 1:
		return
	# Shift on road speed (output shaft), not engine revs, so wheelspin doesn't make it hunt.
	var r := rpm() if road_omega < 0.0 else maxf(road_omega * gear_ratio() * 60.0 / TAU, idle_rpm)
	var top := minf(redline, rev_limit)
	var up_rpm := lerpf(top * 0.5, up_rpm_full(), throttle)
	if r > up_rpm and gear < gears.size() and engagement > 0.95:
		shift_to(gear + 1, 10.0)
		return
	if gear > 1:
		var lower_rpm := r * gear_ratio(gear - 1) / gear_ratio(gear)
		var down_rpm := lerpf(idle_rpm * 1.6, top * 0.45, throttle)
		if r < down_rpm and lower_rpm < top * 0.85:
			shift_to(gear - 1, 10.0)


## How much the automatic box (update_auto) wants to shift right now: [up, down], each 0..1, with
## 0.5 exactly at its shift point (it shifts once past it). The same rule and numbers as
## update_auto; the AI Lab teaches brains to shift from it (a number every tick, not just the
## rare moments it actually shifts).
func shift_wish(throttle: float, road_omega := -1.0) -> Vector2:
	if gear < 1:
		return Vector2.ZERO
	var r := rpm() if road_omega < 0.0 else maxf(road_omega * gear_ratio() * 60.0 / TAU, idle_rpm)
	var top := minf(redline, rev_limit)
	var band := top * 0.1
	var up := 0.0
	if gear < gears.size():
		up = clampf(0.5 + (r - lerpf(top * 0.5, up_rpm_full(), throttle)) / band, 0.0, 1.0)
	var down := 0.0
	if gear > 1 and r * gear_ratio(gear - 1) / gear_ratio(gear) < top * 0.85:
		down = clampf(0.5 + (lerpf(idle_rpm * 1.6, top * 0.45, throttle) - r) / band, 0.0, 1.0)
	return Vector2(up, down)


# What the substeps share within one tick (prep_substeps), and the idle feed-forward.
var _driven: Array[Wheel] = []
var _split := 0.0
var _lay := 0                    # layout: 0 FWD, 1 RWD, 2 AWD
# health, air, iat, torque_scale and idle revs the feed-forward was worked out for
var _ff_h := -1.0
var _ff_a := -1.0
var _ff_i := -1.0
var _ff_s := -1.0
var _ff_idle := -1.0
var _ff := 0.0


## Called by the car once a tick before its substeps: which wheels the box drives (a shaft can only
## snap between ticks), the front / rear split and the diffs' settings.
func prep_substeps(wheels: Array) -> void:
	_dead_f = _axle_dead(wheels[0], wheels[1], front_diff) if wheels.size() > 1 else false
	_dead_r = _axle_dead(wheels[2], wheels[3], rear_diff) if wheels.size() > 3 else false
	_driven.clear()
	for w: Wheel in wheels:
		if w.driven and not w.shaft_broken and not (_dead_f if w.front else _dead_r):
			_driven.append(w)
	var split := _front_split()
	if _dead_f and layout != "FWD" and layout != "RWD":
		split = 0.0
	elif _dead_r and layout != "FWD" and layout != "RWD":
		split = 1.0
	_split = split
	var v := 0.0
	for w: Wheel in _driven:
		v += absf(w.omega) * w.radius
	_road_v = v / maxf(_driven.size(), 1)
	_lay = 0 if layout == "FWD" else 1 if layout == "RWD" else 2
	_diff_params(D_FRONT, front_diff)
	_diff_params(D_REAR, rear_diff)
	_diff_params(D_CENTER, center_diff)
	for slot in 3:
		_dty[slot] = _dp[slot * 5]   # (the type as the car file has it: an active diff becomes a plated one below)
		if _dp[slot * 5] == DT_ACTIVE:
			_active_params(slot)


## What the active diffs' ECU reads, once a tick before prep_substeps: pedals 0..1, steering -1..1,
## road speed (m/s), the ground under the car (a Surfaces id) and the tick length.
func set_active_inputs(dt: float, thr: float, brk: float, hb: float, steer: float, v: float, surf: String) -> void:
	_ai_thr = thr
	_ai_brk = brk
	_ai_hb = hb
	_ai_steer = absf(steer)
	_ai_v = absf(v)
	var want := Vector3(0.0, 1.0, 0.0)
	if surf.begins_with("tarmac"):
		want = Vector3(1.0, 0.0, 0.0)
	elif surf.begins_with("snow") or surf == "ice":
		want = Vector3(0.0, 0.0, 1.0)
	# The ECU doesn't flick maps on every patch of gravel on a tarmac stage: about a second to swap.
	_map_w = _map_w.lerp(want, 1.0 - exp(-dt / 0.8))
	active_map = "tarmac" if _map_w.x >= maxf(_map_w.y, _map_w.z) else ("snow" if _map_w.z > _map_w.y else "gravel")


## An active diff's clutch settings this tick (see ACTIVE_MAPS), into _dp as a plated diff's.
func _active_params(slot: int) -> void:
	var o := slot * 5
	var c := 0 if slot == D_CENTER else 3
	var w := _map_w / maxf(_map_w.x + _map_w.y + _map_w.z, 0.001)
	var mt: Array = ACTIVE_MAPS.tarmac
	var mg: Array = ACTIVE_MAPS.gravel
	var ms: Array = ACTIVE_MAPS.snow
	var pre: float = w.x * float(mt[c]) + w.y * float(mg[c]) + w.z * float(ms[c])
	var on: float = w.x * float(mt[c + 1]) + w.y * float(mg[c + 1]) + w.z * float(ms[c + 1])
	var off: float = w.x * float(mt[c + 2]) + w.y * float(mg[c + 2]) + w.z * float(ms[c + 2])
	# On the throttle it locks up as the pedal goes down; on the brakes it lets go so the car turns
	# in; winding lock on at low speed opens it too (tarmac most: no crow-hopping round a hairpin).
	on *= lerpf(0.55, 1.0, smoothstep(0.05, 0.7, _ai_thr))
	pre *= lerpf(1.0, 0.4, clampf(_ai_brk, 0.0, 1.0))
	pre *= 1.0 - 0.5 * _ai_steer * (1.0 - smoothstep(8.0, 20.0, _ai_v)) * (0.4 + 0.6 * w.x)
	# The handbrake frees the centre (an axle diff it leaves alone: the lever locks those wheels anyway).
	if slot == D_CENTER and _ai_hb > 0.1:
		var free := 1.0 - smoothstep(0.1, 0.4, _ai_hb)
		pre *= free
		on *= free
		off *= free
	_dp[o] = DT_LSD
	_dp[o + 1] = pre
	_dp[o + 2] = on
	_dp[o + 3] = off


## The C++ core (native/src/core.cpp runs step() and the wheels' substeps in one call; car.gd
## `_native_substeps`). npack: everything step() reads, into the car block of its buffer (native/
## gen_layout.py: the CAR list); prep_substeps() must have run this tick. The statics ride along.
func npack(b: PackedFloat64Array, n_sub: int, h: float, dt: float, throttle: float, clutch_pedal: float, auto_clutch: bool, allow_stall: bool) -> void:
	b[NL.C_N_SUB] = n_sub
	b[NL.C_H] = h
	b[NL.C_DT] = dt
	b[NL.C_THROTTLE] = throttle
	b[NL.C_CLUTCH_PEDAL] = clutch_pedal
	b[NL.C_AUTO_CLUTCH] = 1.0 if auto_clutch else 0.0
	b[NL.C_ALLOW_STALL] = 1.0 if allow_stall else 0.0
	b[NL.C_S_MU_BRAKE] = Wheel.mu_brake
	b[NL.C_S_BERM_TAN] = Wheel.berm_tan
	b[NL.C_S_BERM_CAP] = Wheel.berm_cap
	b[NL.C_S_SLIDE_MU_DROP] = Wheel.slide_mu_drop
	b[NL.C_S_SLIDE_LAT] = Wheel.slide_lat
	b[NL.C_S_SLIDE_V0] = Wheel.slide_v0
	b[NL.C_S_SLIDE_HEAT] = Wheel.slide_heat
	b[NL.C_S_LOOSE_LINEAR] = 1.0 if Wheel.loose_linear else 0.0
	b[NL.C_S_MANIFOLD_ON] = 1.0 if manifold_on else 0.0
	b[NL.C_S_TORSION_ON] = 1.0 if torsion_on else 0.0
	b[NL.C_TORQUE_SCALE] = torque_scale
	b[NL.C_IDLE_RPM] = idle_rpm
	b[NL.C_REV_LIMIT] = rev_limit
	b[NL.C_ENGINE_INERTIA] = engine_inertia
	b[NL.C_HEALTH] = health
	b[NL.C_AIR] = air
	b[NL.C_IAT] = iat
	b[NL.C_OFF_FLOOR] = _off_floor
	b[NL.C_LC_RPM] = lc_rpm
	b[NL.C_ALS] = als
	b[NL.C_FLAT_SHIFT] = 1.0 if flat_shift else 0.0
	b[NL.C_ROAD_V] = _road_v
	b[NL.C_SHIFT_RPM] = _shift_rpm
	b[NL.C_FINAL_DRIVE] = final_drive
	b[NL.C_REVERSE_RATIO] = reverse_ratio
	b[NL.C_EFFICIENCY] = efficiency
	b[NL.C_TC_K] = tc_k
	b[NL.C_TC_TR] = tc_tr
	b[NL.C_GEAR] = gear
	b[NL.C_PENDING_GEAR] = pending_gear
	b[NL.C_SHIFT_TIMER] = shift_timer
	b[NL.C_SINCE_SHIFT] = since_shift
	b[NL.C_CLUTCH_CAPACITY] = clutch_capacity
	b[NL.C_CLUTCH_ASSIST] = 1.0 if clutch_assist else 0.0
	b[NL.C_PEDAL_HANDS] = 1.0 if pedal_hands else 0.0
	b[NL.C_LASH_DEG] = lash_deg
	b[NL.C_TORSION] = 1.0 if torsion else 0.0
	b[NL.C_SHAFT_K] = shaft_k
	b[NL.C_SHAFT_ZETA] = shaft_zeta
	b[NL.C_HAS_TURBO] = 1.0 if not turbo.is_empty() else 0.0
	b[NL.C_T_SPOOL0] = _t_spool0
	b[NL.C_T_SPOOL1] = _t_spool1
	b[NL.C_T_LAG] = _t_lag
	b[NL.C_T_GAIN] = _t_gain
	b[NL.C_T_SOAK] = _t_soak
	b[NL.C_STARTER_K] = starter_k
	b[NL.C_NO_START] = 1.0 if no_start else 0.0
	b[NL.C_SEIZED] = 1.0 if seized else 0.0
	b[NL.C_SPLIT] = _split
	b[NL.C_LAY] = _lay
	b[NL.C_DEAD_F] = 1.0 if _dead_f else 0.0
	b[NL.C_DEAD_R] = 1.0 if _dead_r else 0.0
	b[NL.C_OMEGA_E] = omega_e
	b[NL.C_RUNNING] = 1.0 if running else 0.0
	b[NL.C_THROTTLE_EFF] = throttle_eff
	b[NL.C_MAN] = _man
	b[NL.C_IDLE_I] = _idle_i
	b[NL.C_ENGINE_TORQUE] = engine_torque
	b[NL.C_LIMITER] = _limiter
	b[NL.C_FUEL_CUT] = 1.0 if _fuel_cut else 0.0
	b[NL.C_STARTER] = _starter
	b[NL.C_RESTART_ASSIST] = 1.0 if _restart_assist else 0.0
	b[NL.C_AUTO_CRANK] = _auto_crank
	b[NL.C_CLUTCH_TEMP] = clutch_temp
	b[NL.C_OVER_REV] = over_rev
	b[NL.C_LASH_SIDE] = _lash_side
	b[NL.C_LASH_LEFT] = _lash_left
	b[NL.C_WIND] = wind
	b[NL.C_SHAFT_TORQUE] = shaft_torque
	b[NL.C_WIND_V] = _wind_v
	b[NL.C_BOOST] = boost
	b[NL.C_BOOST_THR] = _boost_thr
	b[NL.C_WG] = _wg
	b[NL.C_SHAFT] = _shaft
	b[NL.C_SOAK] = _soak
	b[NL.C_ALS_ACTIVE] = 1.0 if als_active else 0.0
	b[NL.C_LAUNCHING] = 1.0 if launching else 0.0
	b[NL.C_TC_RATIO] = tc_ratio
	b[NL.C_BOX_OUT] = box_out
	b[NL.C_ENGAGEMENT] = engagement
	b[NL.C_CLUTCH_TORQUE] = clutch_torque
	b[NL.C_AUTO_ENGAGEMENT] = _auto_engagement
	b[NL.C_AUTO_LOCKED] = 1.0 if _auto_locked else 0.0
	b[NL.C_PEDAL_OWN] = 1.0 if pedal_own else 0.0
	b[NL.C_ASSISTING] = 1.0 if assisting else 0.0
	b[NL.C_ASSIST_E] = _assist_e
	b[NL.C_FF_H] = _ff_h
	b[NL.C_FF_A] = _ff_a
	b[NL.C_FF_I] = _ff_i
	b[NL.C_FF_S] = _ff_s
	b[NL.C_FF_IDLE] = _ff_idle
	b[NL.C_FF] = _ff
	b[NL.C_C_ON] = _c_on
	b[NL.C_C_OFF] = _c_off
	for i in 15:
		b[NL.C_DP0 + i] = _dp[i]
	b[NL.C_DTY0] = _dty[0]
	b[NL.C_DTY0 + 1] = _dty[1]
	b[NL.C_DTY0 + 2] = _dty[2]
	b[NL.C_EV_N] = 0.0


## ... and everything step() writes, back out; then the signals it would have emitted on the way
## (stalled / started / clunk), in the order they happened.
func nunpack(b: PackedFloat64Array) -> void:
	omega_e = b[NL.C_OMEGA_E]
	running = b[NL.C_RUNNING] != 0.0
	throttle_eff = b[NL.C_THROTTLE_EFF]
	_man = b[NL.C_MAN]
	_idle_i = b[NL.C_IDLE_I]
	engine_torque = b[NL.C_ENGINE_TORQUE]
	_limiter = b[NL.C_LIMITER]
	_fuel_cut = b[NL.C_FUEL_CUT] != 0.0
	_starter = b[NL.C_STARTER]
	_restart_assist = b[NL.C_RESTART_ASSIST] != 0.0
	_auto_crank = b[NL.C_AUTO_CRANK]
	clutch_temp = b[NL.C_CLUTCH_TEMP]
	over_rev = b[NL.C_OVER_REV]
	_lash_side = b[NL.C_LASH_SIDE]
	_lash_left = b[NL.C_LASH_LEFT]
	wind = b[NL.C_WIND]
	shaft_torque = b[NL.C_SHAFT_TORQUE]
	_wind_v = b[NL.C_WIND_V]
	boost = b[NL.C_BOOST]
	_wg = b[NL.C_WG]
	_shaft = b[NL.C_SHAFT]
	_soak = b[NL.C_SOAK]
	als_active = b[NL.C_ALS_ACTIVE] != 0.0
	launching = b[NL.C_LAUNCHING] != 0.0
	tc_ratio = b[NL.C_TC_RATIO]
	box_out = b[NL.C_BOX_OUT]
	engagement = b[NL.C_ENGAGEMENT]
	clutch_torque = b[NL.C_CLUTCH_TORQUE]
	_auto_engagement = b[NL.C_AUTO_ENGAGEMENT]
	_auto_locked = b[NL.C_AUTO_LOCKED] != 0.0
	pedal_own = b[NL.C_PEDAL_OWN] != 0.0
	assisting = b[NL.C_ASSISTING] != 0.0
	_assist_e = b[NL.C_ASSIST_E]
	_ff_h = b[NL.C_FF_H]
	_ff_a = b[NL.C_FF_A]
	_ff_i = b[NL.C_FF_I]
	_ff_s = b[NL.C_FF_S]
	_ff_idle = b[NL.C_FF_IDLE]
	_ff = b[NL.C_FF]
	_c_on = b[NL.C_C_ON]
	_c_off = b[NL.C_C_OFF]
	var n := int(b[NL.C_EV_N])
	for i in n:
		var kind := int(b[NL.EV_AT + i * 2])
		if kind == NL.EV_STALLED:
			stalled.emit()
		elif kind == NL.EV_STARTED:
			started.emit()
		elif kind == NL.EV_CLUNK:
			clunk.emit(b[NL.EV_AT + i * 2 + 1])


## One drivetrain substep. `wheels` must already hold this substep's tire forces, and
## prep_substeps() must have run this tick.
func step(dt: float, wheels: Array, throttle: float, clutch_pedal: float, auto_clutch: bool, allow_stall: bool) -> void:
	var r := omega_e / RPM_TO_RAD
	# Both torque curves at these revs, looked up once for the whole substep (the engine, the
	# launch control and the clutch assist all ask): the same values torque_on(r) / torque_off(r) give.
	_curves_at(r)
	var tq_on := _c_on * torque_scale * health * air * iat
	var tq_off := _c_off

	# --- engine torque
	var thr := throttle
	if running:
		# The idle feed-forward only changes with the engine's condition: worked out again when that does.
		if health != _ff_h or air != _ff_a or iat != _ff_i or torque_scale != _ff_s or idle_rpm != _ff_idle:
			_ff_h = health
			_ff_a = air
			_ff_i = iat
			_ff_s = torque_scale
			_ff_idle = idle_rpm
			var t_on_idle := torque_on(idle_rpm)
			var t_off_idle := torque_off(idle_rpm)
			_ff = -t_off_idle / maxf(t_on_idle - t_off_idle, 1.0)
		# Idle loop (PI on the revs, in torque so every engine's loop is as quick: I x bandwidth),
		# limited to the idle valve's bypass. The integral only winds while it's near idle.
		var err := (idle_rpm - r) * RPM_TO_RAD
		var span := maxf(torque_on(idle_rpm) - torque_off(idle_rpm), 1.0)
		if r < idle_rpm * 1.5 and throttle < 0.05:
			var wind_max := IDLE_AUTH * span / maxf(engine_inertia * IDLE_BW, 0.01) * IDLE_TI
			_idle_i = clampf(_idle_i + err * dt, -wind_max, wind_max)
		else:
			_idle_i = move_toward(_idle_i, 0.0, absf(_idle_i) * dt * 4.0)
		var idle_t := engine_inertia * IDLE_BW * (err + _idle_i / IDLE_TI)
		thr = maxf(thr, clampf(_ff + idle_t / span, 0.0, _ff + IDLE_AUTH))
		# The manifold: what the cylinders breathe follows the throttle a couple of revs behind.
		if manifold_on:
			var tau := minf(MANIFOLD_K / maxf(r, 300.0), 0.4)
			_man += (thr - _man) * (1.0 - exp(-dt / tau))
		else:
			_man = thr
		thr = _man
		# Launch control: sat still with the throttle down, the limiter holds the launch revs.
		var lim := rev_limit
		launching = lc_rpm > 0.0 and _road_v < LAUNCH_V and throttle > 0.5 and gear >= 0
		if launching:
			lim = minf(lim, lc_rpm)
		# Flat shift: foot flat through the shift, the ECU holds the revs where the shift began.
		elif flat_shift and is_shifting() and throttle > 0.6:
			lim = minf(lim, maxf(_shift_rpm, idle_rpm + 500.0))
		if r > lim:
			_limiter = 0.07 if lim == rev_limit else 0.02
		_fuel_cut = _limiter > 0.0
		if _limiter > 0.0:
			_limiter -= dt
			thr = 0.0
		# Floor near zero revs: the curve goes to 0 Nm at 0 rpm, so without it an engine dragged
		# to a standstill could never climb back on its own.
		var t_on := tq_on if r >= 400.0 else maxf(tq_on, 8.0)
		engine_torque = lerpf(tq_off, t_on * (1.0 + _boost_step(dt, r, thr)), thr)
	else:
		thr = 0.0
		_man = 0.0
		_idle_i = 0.0
		_boost_step(dt, r, 0.0)
		engine_torque = tq_off
		if _starter > 0.0:
			_starter -= dt
			engine_torque += 90.0 * starter_k if r < 400.0 else 0.0
			if r > 300.0 and not no_start:
				running = true
				_starter = 0.0
				# Restarted in gear with the clutch pedal up: the game feathers the clutch for
				# you until the car's moving (otherwise it would stall again straight away).
				_restart_assist = true
				_auto_engagement = 0.0
				started.emit()
	# (What the sound / exhaust / replays read: the driver's share, steady idle air reads as closed.)
	throttle_eff = clampf((thr - _ff) / maxf(1.0 - _ff, 0.1), 0.0, 1.0) if running else 0.0

	# --- driveline kinematics (speed at the gearbox input side of the clutch)
	var g := gear_ratio()
	# Every driven wheel cut off (a snapped shaft on an open diff): the box turns nothing.
	if _driven.is_empty():
		g = 0.0
	var carrier := 0.0
	var inertia_sum := 0.0
	var load_sum := 0.0
	var split := _split
	for w in _driven:
		var share: float = (split if w.front else 1.0 - split) * 0.5
		carrier += w.omega * share
		inertia_sum += w.inertia
		load_sum += -w.fx * w.radius
	var omega_d := carrier * g
	box_out = absf(carrier) * final_drive

	# --- clutch
	# The pedal isn't linear: nothing happens over the first part of the lift, the clutch bites
	# over a short band (the bite point) and it's fully home well before the pedal is all the way up.
	var e := smoothstep(BITE_START, BITE_END, 1.0 - clutch_pedal)
	if _restart_assist and (clutch_pedal > 0.6 or gear == 0 or not running):
		_restart_assist = false
	var road_rpm := absf(carrier) * absf(gear_ratio(pending_gear if is_shifting() else gear)) * 60.0 / TAU
	if auto_clutch or _restart_assist:
		var synced := absf(omega_e - omega_d) < 15.0
		# The driver's working the pedal: it's theirs until it's back up and the plates have caught up
		# (or the revs are about to die - then the auto clutch takes it back, it can't stall).
		if pedal_hands and auto_clutch and not _restart_assist and clutch_pedal > PEDAL_OWN_IN and running:
			pedal_own = true
		if pedal_own and (not pedal_hands or not running or gear == 0 or (clutch_pedal < 0.02 and synced)):
			pedal_own = false
			_auto_locked = synced and running and gear != 0
		# Like a driver launching: slip the clutch to hold the revs where the engine pulls - more
		# throttle, more revs (a turbo engine needs to be near its spool point or it just bogs).
		var target := _launch_eng(r, throttle, tq_off, tq_on)
		if synced and r > idle_rpm + 50.0:
			_auto_locked = true
		elif r < idle_rpm * LUG_FLOOR:
			# Only a stall threat lets go of a clutch that's home. (It used to let go whenever the
			# engine and the wheels got out of step - every wheel hop, landing and handbrake flick in a
			# slide - and then picked up again like a standing start, slipping the drive away.)
			_auto_locked = false
		if _auto_locked or pedal_own:
			target = 1.0   # home: it lugs under idle like a real one, the clutch stays in
		elif omega_e <= omega_d and road_rpm >= idle_rpm * ASSIST_OUT and since_shift > 0.6:
			# Rolling with the engine slower than the box (revs dropped off in a slide, a near-stall
			# dip): letting it in drags the revs up, it can't bog, so it just goes in.
			target = 1.0
		if is_shifting() or gear == 0:
			target = 0.0
			_auto_locked = false
		# Anti-stall: dip the clutch only when the revs are really falling away (stuck in a ditch...).
		if r < idle_rpm * ANTI_STALL:
			target = 0.0
			_auto_locked = false
			pedal_own = false
		var rate := 12.0 if target < _auto_engagement else 3.5
		if pedal_own:
			_auto_engagement = target   # the pedal sets the pace (a dump is a dump)
		else:
			_auto_engagement = move_toward(_auto_engagement, target, rate * dt)
		e = minf(e, _auto_engagement)
		if _restart_assist and _auto_engagement >= 0.999 and absf(omega_e - omega_d) < 5.0:
			_restart_assist = false
	else:
		pedal_own = false
	# Low-speed clutch assist (manual clutch): below the speed where the clutch has to slip anyway,
	# the game feathers it like a driver would - it never lets the plates clamp harder than the
	# engine can take at these revs, so dumping the pedal or stopping in gear doesn't stall it.
	# More throttle holds more revs (a quick getaway / a spinny launch on gravel).
	assisting = false
	if clutch_assist and not auto_clutch and running and g != 0.0 and not is_shifting():
		if road_rpm < idle_rpm * ASSIST_IN or _assist_e < 0.999:
			var tgt := _launch_eng(r, throttle, tq_off, tq_on)
			if road_rpm >= idle_rpm * ASSIST_OUT and absf(omega_e - omega_d) < 8.0:
				tgt = 1.0   # rolling and the clutch has caught up: hand it back to the pedal
			_assist_e = move_toward(_assist_e, tgt, dt * (14.0 if tgt < _assist_e else 4.0))
			if _assist_e < e:
				e = _assist_e
				assisting = true
	else:
		# Waiting in neutral / mid-shift: ready to pick up gently if we're crawling, else clamped.
		_assist_e = 0.0 if road_rpm < idle_rpm * ASSIST_IN else 1.0
	if g == 0.0 or is_shifting():
		e = 0.0
	# Cranking: the starter can't turn the whole car over, so (like a clutch-pedal interlock)
	# the clutch stays out until the engine catches. Rolling fast enough still bump-starts it.
	# (Only rolling forwards in a forward gear: rolling back would turn the engine backwards.)
	if not running and _starter > 0.0 and omega_d * 60.0 / TAU < 500.0:
		e = 0.0
	tc_ratio = 1.0
	# (`gear` reads 0 for the length of a shift, so the gear on its way in counts too. Without that
	# the box sat in neutral for the whole shift, the engine ran up on the throttle, and the new gear
	# landed on a converter 2,500 rpm out: the driven tires spun and let go sideways on every
	# upshift under power - a Crown Vic at 0.2 g kicked its tail out 70 % on the 1-2.)
	var conv := tc_k > 0.0 and (gear != 0 or (is_shifting() and pending_gear != 0)) and _driven.size() > 0 and not _tc_locked(omega_d)
	if conv:
		# An automatic pulls through its shifts (one clutch hands over to the next).
		g = gear_ratio(pending_gear if is_shifting() else gear)
		omega_d = carrier * g
		e = 0.0
	# (A converter's "engagement" is its speed ratio: how much of the engine the wheels are keeping up with.)
	engagement = clampf(omega_d / maxf(omega_e, 1.0), 0.0, 1.0) if conv else e
	# A cooked clutch slips: capacity fades from 250 C, to about half by 550 C.
	var fade := clampf(1.0 - (clutch_temp - 250.0) / 600.0, 0.5, 1.0)
	var capacity := clutch_capacity * e * fade
	if conv:
		_converter(omega_d, g)
		capacity = 0.0

	var tors := torsion and torsion_on and g != 0.0 and inertia_sum > 0.0 and not conv
	var locked := false
	if tors and capacity > 0.0:
		# Engine (reflected to the wheels) - shafts (spring + damper) - wheels, clutch held: the
		# shaft torque that leaves the wind-up consistent at the end of the substep.
		var ge := g * efficiency
		var kk := shaft_k
		var i_r := 1.0 / (g * ge * engine_inertia) + 1.0 / inertia_sum
		var cc := 2.0 * shaft_zeta * sqrt(kk / i_r)
		var a := omega_e / g - carrier + dt * (engine_torque / (g * engine_inertia) - load_sum / inertia_sum)
		var dd := kk * dt + cc
		var ts := (kk * wind + dd * a) / (1.0 + dd * dt * i_r)
		var cl := ts / ge
		locked = absf(cl) <= capacity
		clutch_torque = clampf(cl, -capacity, capacity)
	elif g != 0.0 and capacity > 0.0:
		var i_d := inertia_sum / (g * g) + 0.01
		var inv := dt * (1.0 / engine_inertia + 1.0 / i_d)
		var lock := (omega_e - omega_d + dt * (engine_torque / engine_inertia - load_sum / g / i_d)) / inv
		clutch_torque = clampf(lock, -capacity, capacity)
	elif not conv:
		clutch_torque = 0.0
	var cl0 := clutch_torque
	_lash_step(dt, g, capacity, omega_d)
	if conv:
		wind = 0.0
		_wind_v = 0.0
		shaft_torque = clutch_torque * tc_ratio * g * efficiency
	elif tors:
		_wind_step(dt, g, carrier, load_sum, inertia_sum, locked and clutch_torque == cl0)
	else:
		wind = 0.0
		_wind_v = 0.0
		shaft_torque = clutch_torque * g * efficiency

	# Clutch heat: the power it turns into heat while slipping (a few kJ per degree), cooling to
	# the bellhousing all the time.
	if not conv:
		clutch_temp += absf(clutch_torque * (omega_e - omega_d)) * dt / 3500.0
	clutch_temp -= (clutch_temp - 40.0) * 0.04 * dt
	omega_e += (engine_torque - clutch_torque) / engine_inertia * dt
	if omega_e < 0.0:
		omega_e = 0.0
	# Money shift: the clutch went in on a gear far too low and the wheels spun the engine past
	# the limiter (the limiter cuts fuel; it can't stop that).
	var r_now := omega_e / RPM_TO_RAD
	over_rev = maxf(r_now / maxf(rev_limit, 1000.0) - 1.12, 0.0) if engagement > 0.8 else 0.0
	# The driver can stall it only with "allow stall" on; physics can always kill it (a hard stop
	# in gear dragging it to nothing). Then, if the driver opted out of stalling, the game
	# restarts it after a beat - it feels like a hiccup instead of a dead car.
	if running and (r_now < 150.0 or (allow_stall and r_now < 300.0)):
		running = false
		stalled.emit()
		if not allow_stall and not seized:
			_auto_crank = 0.4

	# --- distribute wheel torque through the differentials
	for w in wheels:
		w.drive_torque = 0.0
	var t_wheels := shaft_torque
	match _lay:
		0:
			_axle(dt, wheels[0], wheels[1], t_wheels, front_diff, D_FRONT)
		1:
			_axle(dt, wheels[2], wheels[3], t_wheels, rear_diff, D_REAR)
		_:
			if _dead_f:
				_axle(dt, wheels[2], wheels[3], t_wheels, rear_diff, D_REAR)
				return
			if _dead_r:
				_axle(dt, wheels[0], wheels[1], t_wheels, front_diff, D_FRONT)
				return
			var t_front := t_wheels * split
			var t_rear := t_wheels - t_front
			var dw: float = (wheels[0].omega + wheels[1].omega - wheels[2].omega - wheels[3].omega) * 0.5
			var transfer := _lock_torque(dt, dw, D_CENTER, t_wheels, wheels[0].inertia * 2.0)
			t_front -= transfer
			t_rear += transfer
			_axle(dt, wheels[0], wheels[1], t_front, front_diff, D_FRONT)
			_axle(dt, wheels[2], wheels[3], t_rear, rear_diff, D_REAR)


## Torque converter, one substep: the impeller (engine side) takes (w_e / K)^2, falling to nothing as
## the turbine catches up past the coupling point (speed ratio ~0.87); below it the stator turns the
## returning oil and the turbine gets tc_tr times that at stall, easing to 1:1 at coupling. Driven
## backwards (coasting) it couples more loosely and multiplies nothing. Sets clutch_torque (what the
## engine feels) and tc_ratio.
const TC_COUPLE := 0.87
func _converter(omega_d: float, g: float) -> void:
	var we := maxf(omega_e, 0.0)
	if g == 0.0:
		clutch_torque = 0.0
		return
	if omega_d <= we:
		var sr := clampf(omega_d / maxf(we, 1.0), -1.0, 1.0)
		var f := 1.0 if sr < TC_COUPLE else clampf((1.0 - sr) / (1.0 - TC_COUPLE), 0.0, 1.0)
		clutch_torque = pow(we / tc_k, 2.0) * f
		tc_ratio = lerpf(tc_tr, 1.0, clampf(sr / TC_COUPLE, 0.0, 1.0))
	else:
		# Overrun: the wheels drive the engine through the oil (engine braking, weaker).
		var sr2 := clampf(we / maxf(omega_d, 1.0), 0.0, 1.0)
		var f2 := 1.0 if sr2 < TC_COUPLE else clampf((1.0 - sr2) / (1.0 - TC_COUPLE), 0.0, 1.0)
		clutch_torque = -pow(omega_d / (tc_k * 1.6), 2.0) * f2


## Lockup: in the top two gears (3rd and up), once the turbine turns past 1400 rpm, the converter's
## clutch locks it solid (no slip losses on the highway). Below that it's the fluid coupling.
func _tc_locked(omega_d: float) -> bool:
	return gear >= gears.size() - 1 and gear >= 3 and omega_d > 1400.0 * RPM_TO_RAD and not is_shifting()


## The shafts' wind-up over this substep, once the clutch (and the lash) have settled what goes
## through. Clutch held: the shafts' own torque from the implicit solve. Slipping, open or crossing
## the lash: the light gearbox side passes exactly what the clutch gives it, and the wind-up relaxes
## to suit (implicitly too).
func _wind_step(dt: float, g: float, carrier: float, load_sum: float, inertia_sum: float, locked: bool) -> void:
	var ge := g * efficiency
	var kk := shaft_k
	var i_r := 1.0 / (g * ge * engine_inertia) + 1.0 / inertia_sum
	var cc := 2.0 * shaft_zeta * sqrt(kk / i_r)
	var w0 := wind
	if locked:
		var ts := clutch_torque * ge
		# The relative speed at the end of the substep, from the same solve.
		var u := omega_e / g - carrier + dt * (engine_torque / (g * engine_inertia) - load_sum / inertia_sum) - dt * ts * i_r
		wind = w0 + dt * u
		shaft_torque = kk * wind + cc * u
	else:
		var ts := clutch_torque * ge
		wind = (ts + cc * w0 / dt) / (kk + cc / dt)
		shaft_torque = ts
	_wind_v = (wind - w0) / dt
	# (Never more wound than the clutch could ever have put in: a bound against a bad tick.)
	var lim := clutch_capacity * absf(ge) * 1.5 / kk
	wind = clampf(wind, -lim, lim)


## Driveline backlash: see lash_deg. Runs after the clutch has worked out what it would pass on.
func _lash_step(dt: float, g: float, capacity: float, omega_d: float) -> void:
	if g == 0.0 or capacity <= 0.0 or lash_deg <= 0.0:
		_lash_left = 0.0     # clutch out / neutral: the slack sits wherever it was
		return
	var gap := deg_to_rad(lash_deg) * 2.0 * absf(g)
	if _lash_left <= 0.0:
		# In contact. The torque wants to go the other way (and not just a flicker round zero):
		# the slack has to be crossed first.
		if signf(clutch_torque) != _lash_side and absf(clutch_torque) > 6.0:
			_lash_side = signf(clutch_torque)
			_lash_left = gap
		else:
			return
	# Crossing the gap: nothing goes through, the engine side turns freely relative to the wheels.
	clutch_torque = 0.0
	_lash_left -= (omega_e - omega_d) * _lash_side * dt
	if _lash_left > gap:
		# Went back the way it came (a blip and straight off again): it's against the old side.
		_lash_side = -_lash_side
		_lash_left = 0.0
	elif _lash_left <= 0.0:
		# Taken up: the next solve couples the two speeds in one go - that's the shunt.
		_lash_left = 0.0
		clunk.emit(clampf(absf(omega_e - omega_d) / 40.0, 0.0, 2.0))


func _front_split() -> float:
	match layout:
		"FWD": return 1.0
		"RWD": return 0.0
	return center_diff.split


var _dead_f := false
var _dead_r := false


## An axle with a snapped shaft and an open diff: the diff sends everything to the broken side,
## which spins freely, so nothing drives through either wheel.
static func _axle_dead(l: Wheel, r: Wheel, diff: Dictionary) -> bool:
	if not (l.shaft_broken or r.shaft_broken):
		return false
	return (l.shaft_broken and r.shaft_broken) or String(diff.get("type", "open")) == "open"


func _axle(dt: float, l: Wheel, r: Wheel, torque: float, diff: Dictionary, slot: int) -> void:
	if l.shaft_broken or r.shaft_broken:
		# One side snapped: what reaches the other is what the diff can lock across to it.
		var good := r if l.shaft_broken else l
		var bad := l if l.shaft_broken else r
		bad.drive_torque = 0.0
		var t := 0.0
		match String(diff.get("type", "open")):
			"locked":
				t = torque
			"lsd", "active":
				# (Through _dp: an active diff's settings are worked out each tick, a plated one's read.)
				t = signf(torque) * minf(absf(torque), _dp[slot * 5 + 1] + _dp[slot * 5 + 2] * absf(torque))
			"viscous":
				t = signf(torque) * minf(absf(torque), float(diff.get("viscous", 60.0)) * absf(l.omega - r.omega) * 0.5)
		good.drive_torque = t
		return
	var transfer := _lock_torque(dt, l.omega - r.omega, slot, torque, l.inertia)
	l.drive_torque = torque * 0.5 - transfer * 0.5
	r.drive_torque = torque * 0.5 + transfer * 0.5


## Torque moved from the faster to the slower side, limited so it can never over-correct.
## `input_torque` is signed: a plated diff locks on its drive ramp under power and its coast ramp
## off it (MASTER_PLAN 7.7 G: a 1.5-way diff - strong on throttle, gentler on the overrun).
## `slot` is the diff's settings in _dp (D_FRONT / D_REAR / D_CENTER, read once a tick).
func _lock_torque(dt: float, dw: float, slot: int, input_torque: float, side_inertia: float) -> float:
	var o := slot * 5
	var ty := _dp[o]
	if ty == DT_OPEN:
		return 0.0
	var limit := absf(dw) * side_inertia / dt * 0.9
	var t := 0.0
	if ty == DT_LOCKED:
		t = limit
	elif ty == DT_LSD:
		t = _dp[o + 1] + (_dp[o + 2] if input_torque >= 0.0 else _dp[o + 3]) * absf(input_torque)
	elif ty == DT_VISCOUS:
		# A viscous coupling's grip grows with the speed difference but levels off (the silicone
		# shears thin and heats up), so a big difference - rears held by the handbrake, one axle
		# spinning on ice - doesn't lock the car solid the way a straight line would.
		var a := absf(dw)
		t = _dp[o + 4] * a / (1.0 + a / VISCOUS_SAT)
	return signf(dw) * minf(t, limit)


## Each diff's settings for the substeps, as numbers: type, preload, ramp, coast ramp, viscous.
const D_FRONT := 0
const D_REAR := 1
const D_CENTER := 2
const DT_OPEN := 0.0
const DT_LOCKED := 1.0
const DT_LSD := 2.0
const DT_VISCOUS := 3.0
const DT_OTHER := 4.0
const DT_ACTIVE := 5.0           # worked out each tick (_active_params) into a DT_LSD
const VISCOUS_SAT := 6.0         # wheel rad/s where a viscous coupling is at half its straight-line grip
var _dp := PackedFloat64Array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
var _dty := PackedFloat64Array([0, 0, 0])


func _diff_params(slot: int, diff: Dictionary) -> void:
	var o := slot * 5
	var ty: Variant = diff.get("type", "open")
	_dp[o] = DT_OPEN if ty == "open" else DT_LOCKED if ty == "locked" else DT_LSD if ty == "lsd" \
		else DT_VISCOUS if ty == "viscous" else DT_ACTIVE if ty == "active" else DT_OTHER
	_dp[o + 1] = float(diff.get("preload", 0.0))
	_dp[o + 2] = float(diff.get("ramp", 0.0))
	_dp[o + 3] = float(diff.get("ramp_decel", diff.get("ramp", 0.0)))
	_dp[o + 4] = float(diff.get("viscous", 60.0))
