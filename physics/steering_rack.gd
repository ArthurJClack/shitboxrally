## The steering between the front tires and the driver's hands (MASTER_PLAN §7.25): what a real
## column carries, per car. car.gd owns one (`car.rack`) and steps it every physics tick for the
## player's car; the torque goes to a force-feedback wheel (InputManager.set_wheel_ffb), and losing
## the power steering also slows pad / keyboard steering (`heavy()`).
##
##   front tires --(trail, jacking, scrub radius, load spikes)--> kingpin torque
##     --> suspension + rack mounts (per car: strut / wishbone, bushing stiffness, kickback)
##     --> rack-and-pinion or recirculating-ball box (ratio, back-drive efficiency, friction)
##     --> power assist (hydraulic pump on the engine / electric motor / none) --> the rim
##
## Car json, all optional (defaults by mass):
##   "steering": {"box": "rack"|"ball", "assist": "manual"|"hydraulic"|"electric", "assist_level": 0..0.9,
##                "rim_deg": lock-to-lock, "front": "strut"|"wishbone"|"beam", "kickback": 0..1.5,
##                "bushing_hz": 6..40, "friction_nm": rack/column friction at the rim, "scrub_radius": m,
##                (garage steering parts, src/data/steering_parts.gd:) "stock_assist" / "stock_rim_deg": what the
##                car came with, "eps_feel" / "eps_speed": 0..1 electric map}
## The older flat keys (power_steering, steering_wheel_deg, scrub_radius) are still read.
extends RefCounted

const JACK_BASE := 0.004         # m: lift arm every front end has (patch migration across the tread as it steers)
const PNEU_TRAIL := 0.25         # pneumatic trail at zero slip, share of the patch length: the brush model's
                                 # 1/6 on a rigid carcass, about 1/4 measured (the carcass lets the patch trail)
const TIRE_CURVE := preload("res://src/car/tire_curve.gd")
const BUMP_ARM := 0.012          # m: how much a vertical load spike turns into a rack kick
const PATCH_SHIFT_MAX := 0.03    # m
const GYRO_I_MAX := 1.2          # kg m^2: spin inertia of a front wheel + tire + disc (cap: a driven
                                 # wheel's figure can carry the driveline)
const SCRUB_ARM := 0.004         # m: how far the lateral force's line of action jumps about while a front tire
                                 # slides on a hard road (stick-slip in the patch: the judder of a scrubbing tire)
# --- what the wheel helper plays on top of the torque (v5: tools/ffb/ffb_model.h), see _vibration()
const GRAIN_SHARE := 0.5         # share of a surface's `rough` that is finer than the physics can carry
const STONE_ARM := 0.001         # m: loose stones under a front tire, as an arm on its load
const WHEEL_HOP_Q := 3.0         # how much the front end's own shake (wheel hop / shimmy, ~13 Hz) magnifies
const UNBALANCE_KG := 0.005      # kg at the rim: what a freshly balanced wheel is still out by (+ wear)
const UNBALANCE_ARM := 0.03      # m: the out-of-balance force's arm on the kingpin
const JUDDER_ARM := 0.02         # m: brake force's arm on the kingpin, on top of the scrub radius
const TRAMP_SHARE := 0.1         # share of a spinning driven front tire's force that comes and goes as it hops
const ENGINE_COUPLING := 0.02    # share of the engine's shake on its mounts that ends up as torque at the rim
const MOUNT_HZ := 9.0            # Hz: the engine on stock rubber mounts
const FIRING_NM := 30.0          # Nm: torque pulse of an idling four (+ what it is making: gas pressure)
## Surface grain: the size (m) of the roughness the rim's texture is made of, by what the surface is.
## It sits around speed / size. At 60 mph the 120 Hz physics can't carry anything shorter than about
## three feet (and the suspension has the long stuff), a tire's patch swallows anything under a few
## inches: this is the band in between, where a surface's `rough` lives - chip seal and patches on
## tarmac, the ripples and stones of a gravel road, the ruts of packed snow.
const GRAIN := {"hard": 0.45, "gravel": 0.4, "soft": 0.6}
const GRAIN_BY_ID := {"tarmac": 0.5, "tarmac_bumpy": 0.42, "tarmac_washed": 0.45, "tarmac_broken": 0.35,
	"gravel_hard": 0.42, "ice": 0.8, "snow_packed": 0.55}
const BOOST_KNEE := 1.5          # Nm at the rim: where a 0.65 valve / EPS map starts boosting hard (lighter
                                 # systems boost sooner: the numb, one-finger feel of a big 90s sedan)

# What the front suspension does to the road reaching the rack.
#   kick: load spikes -> rack;  tau: how long a kick rings (s);  fric: strut top-mount bearing friction (Nm)
#   kpi: kingpin inclination (deg): with the scrub radius it lifts the nose as the wheels turn (centring by weight)
const FRONT := {
	"strut": {"kick": 1.0, "tau": 0.035, "fric": 0.12, "kpi": 13.5},     # the strut turns with the wheel: you feel it all
	"wishbone": {"kick": 0.7, "tau": 0.03, "fric": 0.0, "kpi": 10.0},    # cleaner geometry, less of the road comes back
	"beam": {"kick": 1.45, "tau": 0.06, "fric": 0.05, "kpi": 8.0},       # solid axle: every hit at one wheel steers both
}
# Box: back-drive efficiency (how much of the tires' torque gets back up), column friction (Nm),
# viscous drag at the rim (Nm per rad/s: grease, seals, the column's joints), free play at the rim (deg, each way).
const BOX := {
	"rack": {"eff": 0.88, "fric": 0.25, "visc": 0.10, "play": 0.6},
	"ball": {"eff": 0.6, "fric": 0.6, "visc": 0.25, "play": 3.0},     # recirculating ball: numb, sticky, kind on the hands
}

# --- this car (from its json)
var box := "rack"
var assist_type := "hydraulic"   # "manual", "hydraulic" (pump on the engine), "electric" (EPS)
var assist_max := 0.65           # share of a heavy load the assist takes away at full pressure
var rim_deg := 1080.0            # steering wheel lock to lock
var front := "strut"
var kickback := 1.0
var bushing_hz := 22.0
var friction_nm := 0.35
var scrub_r := 0.015
var kpi := 13.5                  # kingpin inclination (deg)
var kss := 20000.0               # Nm per rad about the kingpins, both wheels together: how stiffly the rack,
                                 # its mounts, the tie rods and the arms hold the front wheels where the rim
                                 # put them (car.gd _kingpin_steer: what the tires' side force steers out of them)
var cylinders := 4.0
# --- garage steering parts (src/data/steering_parts.gd); with none fitted these are the car's own
var stock_assist := 0.65         # the assist the car was designed with (a delete leaves the arms to do it)
var stock_rim := 1080.0          # lock to lock it came with (a quick rack is fewer)
var _mass_k := 1.0               # heavier cars are heavier to park on a manual rack
var _eps_road := 1.0             # electric map: how much of the road the motor lets through
var _eps_knee := 1.0             # ... how soon it boosts (later = more tire feel around the centre)
var mount_k := 1.0               # engine mount stiffness against stock rubber (garage Mounts & bushings)
var worn := 0.0                  # 0..1: how tired the car is (wheel balance, discs, mounts)
var tbar := 110.0                # Nm per rad at the rim: the column's own twist (the valve's torsion bar /
                                 # the EPS torque sensor on assisted cars, joints and shaft on a manual one)
var _eps_floor := 0.88           # ... share of the assist left at highway speed (a factory map keeps most of
                                 # it: a Fit is ~3 Nm in a bend, not 6; a tuned column sets its own, eps_speed)

## How the car's steering is played to a force-feedback wheel (InputManager.ffb_bite(), option ffb_feel):
## 0 = the column as the factory built it (rubber mounts, a valve that takes more away the harder
## you lean on it: true, and flat in a corner), 1 = the way a rally car's is set up - what makes a
## stage car's wheel bite. More caster (the tires' side force pulls on a longer arm, so the rim
## follows the front grip: it loads up on turn-in and under braking, snaps to opposite lock in a
## slide), a rack bolted solid (nothing smeared), and an assist that takes one fixed share instead
## of flattening the big loads. Only what the hands feel: the car drives the same.
var bite := 0.0
## This car's own share of that (the "By car" feel): how much of a driver's car its steering is, from
## the hardware it has right now, garage parts included. A manual or lightly assisted rack on firm
## mounts with a quick ratio bites (a Miata, a GTI, a WRX with the pump deleted and a race rack); a
## heavily assisted recirculating-ball box on soft rubber doesn't, however you play it (a C10, a
## Crown Vic). Worked out in setup().
var car_bite := 0.5
const BITE_CASTER := 0.9         # x(1 + this) on the caster trail
const BITE_MOUNT := 1.5          # x(1 + this) on the mounts' frequency
const BITE_ASSIST := 0.8         # share of the full boost a fixed-share assist takes
const BITE_LEVEL := 0.6          # overall, so the bite is in the changes and not just more weight

## Levelling (InputManager.ffb_norm): every car plays at about the same strength, whatever it is.
## `ref_nm` is this car's own cornering weight - the most its steering carries winding lock on in a
## steady bend on dry pavement, worked out from what it is right now (front weight, tire patch,
## caster, ratio, box, assist, feel) - and `norm` brings that to NORM_NM. So a Crown Vic and a Miata
## pull about as hard in a corner, and fitting a quick rack or deleting the pump changes how the car
## feels (how fast the weight builds, what gets through, the parking effort) instead of how strong the
## wheel is. (NORM_SHARE under 1 would leave some of the real difference in.) Losing the assist on the
## road is not levelled (the reference is the car as built): the wheel still goes to lead.
const NORM_NM := 7.6   # (6.5 until Nov 20 2026: owner asked for a bit more everywhere)
const NORM_SHARE := 1.0
var ref_nm := 6.0
var norm := 1.0
var _fz_avg := 0.0               # the front axle's load, slow average (N)
var _n_tick := 0

# --- state
var pressure := 1.0              # 0..1 of the assist actually there (pump pressure / EPS current)
var fault := ""                  # "" | "belt" (pump belt off) | "leak" (fluid going) | "eps" (motor dead)
var fluid := 1.0                 # power steering fluid left (leak)
var torque := 0.0                # last torque at the rim (Nm, + turns the wheel right)
var friction := 0.0              # last extra column friction (Nm at the rim)
var damper := 0.0                # last extra damping (0..1 of the base)
## What the wheel helper's fast loop needs (InputManager.set_wheel_ffb -> sbr_ffb.exe): the column as a
## spring and a damper around where it is right now, so the rim's own movement is answered within a
## millisecond or two instead of after the trip through the game (~30-50 ms).
var stiff := 0.0                 # Nm per rad of rim: how much the torque changes as the rim turns, right now
var damp_nms := 0.0              # Nm per rad/s at the rim: the column's own viscous damping
var lag_s := 0.03                # s: how long the tires + mounts take to answer a steering move
var _mz := 0.0                   # kingpin torque after the mounts
var _road := 0.0                 # ringing road kick (Nm at the kingpins)
var _road_f := 0.0               # ... after the mounts
var _grav := 0.0                 # gravel chatter (Nm at the kingpins)
var _scrub_ph := 0.0             # tire scrub judder: phase, and its wandering strength
var _scrub_a := 1.0
var _jolt := 0.0                 # impact yank (Nm at the rim)
## What the helper adds to the torque itself, every millisecond (v5): steady vibrations as [hz, Nm at
## the rim] x 6 (0 engine firing, 1 engine half order, 2 wheel balance, 3 brake judder, 4 axle tramp,
## 5 spare) and texture bands as [hz, Nm rms, q] x 2 (0 the road's grain, 1 loose stones).
var osc := PackedFloat32Array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
var noise := PackedFloat32Array([30.0, 0.0, 1.5, 60.0, 0.0, 1.5])
var up_gain := 0.0                # kingpin torque -> rim right now, for what comes up through the rack
var engaged := 1.0                # 0 = the rim is inside the box's free play (nothing on the other end), 1 = taken up
var pump_short := 0.0            # 0..1: how far the pump is behind a fast steering move right now
var _steer_prev := 0.0
var _rim_w := 0.0                # rim speed (rad/s), filtered
var _fz := [0.0, 0.0]
var _rng := RandomNumberGenerator.new()


func setup(def: Dictionary) -> void:
	var s: Dictionary = def.get("steering", {})
	var mass := float(def.get("mass", 1200.0))
	var ps: Variant = def.get("power_steering", null)
	# Assist: the steering block, else the old power_steering key, else by mass (light cars were manual).
	var lvl := 0.65
	if s.has("assist"):
		assist_type = String(s.assist)
	elif ps is bool:
		assist_type = "hydraulic" if ps else "manual"
	elif ps != null:
		lvl = clampf(float(ps), 0.0, 0.9)
		assist_type = "hydraulic" if lvl > 0.0 else "manual"
	else:
		assist_type = "manual" if mass < 1000.0 else "hydraulic"
	assist_max = 0.0 if assist_type == "manual" else clampf(float(s.get("assist_level", lvl)), 0.0, 0.9)
	box = String(s.get("box", "rack"))
	if not BOX.has(box):
		box = "rack"
	front = String(s.get("front", "strut"))
	if not FRONT.has(front):
		front = "strut"
	rim_deg = float(s.get("rim_deg", def.get("steering_wheel_deg", 1080.0 if assist_max > 0.0 else 1260.0)))
	kickback = clampf(float(s.get("kickback", 1.0 if box == "rack" else 0.35)), 0.0, 2.0)
	bushing_hz = clampf(float(s.get("bushing_hz", 22.0)), 4.0, 60.0)
	# Worn cars: tired bushings smear the road, a dry rack sticks.
	var wear := clampf(float(def.get("body_wear", 0.0)), 0.0, 1.0)
	bushing_hz *= 1.0 - 0.35 * wear
	friction_nm = float(s.get("friction_nm", float(BOX[box].fric) + float(FRONT[front].fric))) * (1.0 + 0.8 * wear)
	# How stiffly the linkage holds the wheels (measured steering system stiffness, Gillespie ch. 8 /
	# Milliken: 10-30 kNm/rad at the kingpins on a rack, about half that through a steering box with
	# its idler and drag link), following the mounts: a rack on soft rubber gives twice what one bolted
	# solid does. The tires' aligning torque works against it in every bend.
	kss = clampf(float(s.get("kss", (20000.0 if box == "rack" else 12000.0) * bushing_hz / 22.0)), 8000.0, 45000.0)
	scrub_r = float(s.get("scrub_radius", def.get("scrub_radius", 0.015)))
	# Wheels with less offset / spacers push the patch out from the kingpin: half the extra track each side.
	if def.has("track_front") and def.has("stock_track_front"):
		scrub_r += (float(def.track_front) - float(def.stock_track_front)) * 0.5
	kpi = clampf(float(s.get("kpi", FRONT[front].kpi)), 0.0, 20.0)
	cylinders = float(def.get("cylinders", Game.load_engine(String(def.get("engine", ""))).get("cylinders", 4.0)))
	stock_assist = clampf(float(s.get("stock_assist", assist_max)), 0.0, 0.9)
	stock_rim = maxf(float(s.get("stock_rim_deg", rim_deg)), 180.0)
	_mass_k = clampf(mass / 1200.0, 0.75, 1.3)
	mount_k = clampf(float(def.get("engine_mount_k", 1.0)), 0.5, 6.0)
	worn = wear
	# The column's twist. A hydraulic valve works by it (a thin bar, ~1.5-2 Nm a degree; a low-assist
	# valve is a stiffer bar), an EPS torque sensor is one too; a manual column only has its joints.
	if s.has("tbar"):
		tbar = float(s.tbar)
	elif assist_max <= 0.0:
		tbar = 420.0 if box == "rack" else 260.0
	elif assist_type == "electric":
		tbar = 140.0
	else:
		tbar = clampf(95.0 + 170.0 * (0.65 - assist_max), 70.0, 220.0)
	_eps_road = 1.0
	_eps_knee = 1.0
	_eps_floor = 0.88
	if s.has("eps_feel"):
		var feel := clampf(float(s.eps_feel), 0.0, 1.0)
		_eps_road = 0.35 + 1.3 * feel
		_eps_knee = 0.5 + feel
	if s.has("eps_speed"):
		_eps_floor = 1.0 - 0.9 * clampf(float(s.eps_speed), 0.0, 1.0)
	var assist_s := 1.0 - assist_max
	if assist_type == "electric":
		assist_s *= clampf(0.5 + 0.5 * _eps_road, 0.4, 1.2)      # a factory EPS map filters; a tuned one less
	var mount_s := clampf((bushing_hz - 8.0) / 22.0, 0.0, 1.0)
	var quick_s := clampf((1250.0 - rim_deg) / 500.0, 0.0, 1.0)
	car_bite = clampf((0.15 + 0.35 * assist_s + 0.25 * mount_s + 0.25 * quick_s) * (1.0 if box == "rack" else 0.5), 0.0, 1.0)
	_rng.seed = hash(String(def.get("id", "car")))
	pressure = 0.0 if assist_type == "hydraulic" else 1.0   # the pump comes up with the engine
	fault = ""
	fluid = 1.0


## Rim degrees per road-wheel degree.
func ratio(steer_lock: float) -> float:
	return deg_to_rad(rim_deg * 0.5) / maxf(steer_lock, 0.1)


## How much of the designed assist is missing right now (0 = all there or a manual rack, 1 = all gone).
func lost() -> float:
	return 0.0 if assist_max <= 0.0 else 1.0 - pressure


## How much heavier than designed the steering is for a pad / keyboard driver: 0 normal, up to 1
## (power steering gone at parking speed). At speed the tires turn the wheel far more easily.
## A power steering delete or a low-assist valve counts too (against what the car came with), mostly
## when parked: at speed a depowered rack is close to what the assist left.
func heavy(speed: float) -> float:
	var fade := smoothstep(2.0, 20.0, absf(speed))
	var h := lost() * clampf(assist_max / 0.65, 0.0, 1.3) * lerpf(1.0, 0.3, fade)
	if stock_assist > assist_max + 0.01:
		h += clampf((stock_assist - assist_max) / 0.65, 0.0, 1.3) * _mass_k * lerpf(1.0, 0.15, fade)
	return minf(h, 1.3)


## Steering ratio against the car's factory one (quick racks < 1). Pad / keyboard steering is
## shaped by it in car.gd (_rack_ratio_steer); a real wheel just gets the new lock to lock.
func ratio_k() -> float:
	return rim_deg / stock_rim


## How strongly the tires pull a pad / mouse wheel back (car.gd _steering_feedback), against the
## car as it came: less assist and a quicker ratio pass more of the tires to the hands. 1 = stock.
func pad_feel() -> float:
	var k := (1.0 - 0.45 * assist_max) / (1.0 - 0.45 * stock_assist)
	if assist_type == "electric":
		k *= 0.6 + 0.4 * _eps_road
	if not is_equal_approx(rim_deg, stock_rim):
		k *= sqrt(stock_rim / rim_deg)
	return k


func fault_text() -> String:
	match fault:
		"belt": return "POWER STEERING BELT GONE"
		"leak": return "POWER STEERING LEAK"
		"eps": return "POWER STEERING FAULT"
	return ""


## A collision, summed over car.gd's 0.25 s window (car-local |dv| per axis, m/s). Returns true when
## it broke something (car.gd raises the warning).
func hit(local_dv: Vector3) -> bool:
	if assist_max <= 0.0 or fault != "" or not Game.damage_on("engine_damage"):
		return false
	var chance := 0.0
	var kind := ""
	if local_dv.z >= 10.0:       # into something nose first: the pump, its pulley and the lines sit up front
		chance = clampf((local_dv.z - 10.0) / 20.0, 0.0, 0.4)
		kind = "belt" if _rng.randf() < 0.5 else "leak"
	if local_dv.x >= 9.0:        # sideways into a rock / a ditch edge: a tie rod hit cracks a rack seal
		var c := clampf((local_dv.x - 9.0) / 25.0, 0.0, 0.3)
		if c > chance:
			chance = c
			kind = "leak"
	if local_dv.y >= 11.0:       # grounded hard on the crossmember: the low line
		var c := clampf((local_dv.y - 11.0) / 30.0, 0.0, 0.15)
		if c > chance:
			chance = c
			kind = "leak"
	if assist_type == "electric":
		chance *= 0.35           # no belt or fluid: only a dead motor / sensor
		kind = "eps"
	if chance <= 0.0 or _rng.randf() >= chance:
		return false
	fault = kind
	return true


## One physics tick of the column (after update). Reads the front wheels of `car` (car.gd) and leaves torque / friction / damper.
func step(car: RigidBody3D, dt: float, v: Vector3, prev_v: Vector3, road_k: float) -> void:
	var speed: float = car.speed
	var steer_lock: float = car._steer_lock
	var subs := float(car._substeps)
	var running: bool = car.drivetrain.running
	var fr: Dictionary = FRONT[front]
	var bx: Dictionary = BOX[box]
	var rt := ratio(steer_lock)
	var vsign := -1.0 if speed < -0.5 else 1.0
	var mz := 0.0          # Nm about the kingpins, + turns the wheel right (positive steer_input)
	var _n_fz := 0.0
	var kick := 0.0
	var scrub_nm := 0.0
	var loose := 0.0
	var k_kp := 0.0        # Nm per rad of road-wheel angle: the kingpin torque as a spring, right now
	var k_al := 0.0        # ... the part of it that works against the linkage's give (car.gd _aligning_kp)
	var c_kp := 0.0        # Nm per rad/s of road-wheel angle
	var relax := 0.0       # m: the fronts' lateral relaxation length right now
	var scrub_kp := 0.0    # Nm about the kingpins: how hard the sliding fronts judder
	var scrub_x := 0.0     # how far past the peak they are (0..1)
	# Beam axle: one wheel rising tilts the whole axle (both wheels precess together).
	var beam_roll := 0.0
	var gyro := 0.0        # Nm about the kingpins: a road input, so it rides the road channel (detail slider, kickback)
	if front == "beam":
		var trk: float = maxf(car.wheels[0].position.distance_to(car.wheels[1].position), 1.0)
		beam_roll = (float(car.wheels[0].hub_v) - float(car.wheels[1].hub_v)) * (1.0 if car.wheels[0].left else -1.0) / trk
	for i in 2:
		var w = car.wheels[i]
		var fz: float = w.load if w.contact else 0.0
		var dfz: float = fz - float(_fz[i])
		_fz[i] = fz
		_n_fz += fz
		var arm := 1.0 if w.left else -1.0     # a rearward pull on the left wheel turns the wheels left
		if not w.contact:
			continue
		var fy: float = w.fy_sum / subs
		var fx: float = w.fx_sum / subs
		# Pneumatic trail (brush model): big at small slip, gone at the grip peak - the wheel goes light.
		var pk: float = maxf(w._peak_angle, 0.02)
		var x: float = absf(atan(w.slip_lat)) / pk
		# (On a loose layer: where the stress really sits along the patch, Wheel.trail_mix.)
		var t0: float = w.patch_len * PNEU_TRAIL
		var tp: float = w.trail_mix(_trail(x, t0, 1.0 + bite), t0)
		var cas := deg_to_rad(clampf(w.caster, 0.0, 12.0))
		var tm: float = w.radius * tan(cas) * (1.0 + BITE_CASTER * bite)
		mz += -fy * (tp + tm) * vsign
		# Combined slip: Fy pushes the patch sideways in the carcass, and the longitudinal force then
		# acts off-centre - drive adds aligning torque, braking takes it away (and can reverse it near
		# the limit: the wheel goes light / tugs into a braking turn).
		mz += -fy * clampf(fx / maxf(float(w.carcass_k_lat), 40000.0), -PATCH_SHIFT_MAX, PATCH_SHIFT_MAX)
		# Weight: the kingpin leans in (inclination) and back (caster), so turning the wheels lifts the
		# nose, and the weight on them turns them back: scrub radius x sin(kpi) (Gillespie 8.3), the
		# caster trail's own lift, and the bit every tire has. More weight, more caster, wheels with
		# less offset: more centring at parking-lot speeds, where the tires give none.
		var jack: float = JACK_BASE + absf(scrub_r) * sin(deg_to_rad(kpi)) * cos(deg_to_rad(kpi)) + tm * sin(cas)
		mz += -fz * jack * sin(-w.steer_angle)
		mz += fx * scrub_r * arm
		# The same as a spring: how much the kingpin torque changes per radian of road-wheel angle
		# right now. Cornering stiffness (the tire curve's slope where this tire sits on it) times the
		# trail, less what the collapsing pneumatic trail takes back (it goes negative just before the
		# peak: the torque falls as you wind on more lock), plus the weight term.
		var sx: float = w.kappa / maxf(w._t_sxd, 1e-4)
		var sy: float = w.slip_lat / maxf(w._t_syd, 1e-4)
		var sn := maxf(sqrt(sx * sx + sy * sy), 0.01)
		var f0: float = TIRE_CURVE.eval(sn, w._tc_c, w._tc_b, w._tc_e)
		var fp: float = (TIRE_CURVE.eval(sn + 0.02, w._tc_c, w._tc_b, w._tc_e) - f0) / 0.02
		var r2 := clampf(sy * sy / (sn * sn), 0.0, 1.0) if sn > 0.011 else 1.0
		var ca: float = w._t_fmax * w._al_lat * (fp * r2 + f0 / sn * (1.0 - r2)) / maxf(w._t_syd, 1e-4) * (1.0 - 0.6 * w.cover)
		var dtp: float = (float(w.trail_mix(_trail(x + 0.02, t0, 1.0 + bite), t0, 0.02 * pk)) - tp) / 0.02 / pk
		k_kp += ca * (tp + tm) + absf(fy) * dtp + fz * jack * cos(w.steer_angle)
		k_al += ca * (float(w.trail_mix(_trail(x, t0), t0)) + w.radius * tan(cas))
		# Steering rate itself: the patch sits a trail behind the kingpin and has length, so turning the
		# wheel drags it sideways / twists it on the road. Small at speed, it is why a parked wheel is stiff.
		c_kp += ca * (tm * (tm + tp) + w.patch_len * w.patch_len / 12.0) / maxf(absf(speed), 3.0)
		relax += float(w.relax_lat) * clampf(absf(w.v_long) / 6.0, 0.25, 1.0) * 0.5
		# Gyroscopic: a spinning wheel whose plane tilts (bump camber as it rides a bump, or a beam axle
		# rocking) precesses about the kingpin - the sharp kick through the rim when one front wheel hits
		# something at speed. phi_dot: rotation about the car's forward axis, + = left side up.
		var phi_dot := 0.0
		if front == "beam":
			phi_dot = beam_roll
		else:
			var cam_rate := -deg_to_rad(float(w.bump_camber)) * float(w.hub_v)    # camber rate, rad/s (- = top in)
			phi_dot = -cam_rate if w.left else cam_rate
		gyro += minf(float(w.inertia), GYRO_I_MAX) * float(w.omega) * phi_dot
		# A load spike is a hit from the road; the rearward part of it twists the wheel about the kingpin.
		kick += -dfz * BUMP_ARM * arm * clampf(absf(speed) / 3.0, 0.0, 1.0)
		loose += w.cover * 0.5
		# Past the grip peak on a hard road the tread sticks and lets go in the patch many times a
		# second, and the force's line of action jumps with it: the rim shivers while the fronts push
		# wide, and stops the moment they hook up. Loose surfaces have their own chatter instead.
		var sl: float = smoothstep(0.85, 1.5, x) * (1.0 - float(w.cover)) * clampf(absf(speed) / 8.0, 0.0, 1.0)
		scrub_kp += absf(fy) * SCRUB_ARM * sl
		scrub_x = maxf(scrub_x, sl)
		# Parked or crawling: the patch has to twist on the road to steer. A patch of area A on friction
		# mu turns about its middle with an arm of sqrt(A) / 3 (Gough / Taborek's parking torque,
		# mu Fz^1.5 / (3 sqrt(p))): wide, soft or heavily loaded tires are the heavy ones.
		scrub_nm += fz * minf(float(w.mu_now()), 1.1) * sqrt(maxf(w.width * w.patch_len, 0.0)) / 3.0 * (1.0 - 0.6 * w.cover)
	# The suspension: kicks ring for as long as this geometry lets them, then everything goes through
	# the rack mounts / bushings - a low-pass at this car's bushing frequency. Soft old rubber smears
	# the road into a vague hum; a tight rack passes the edges of every stone.
	var mount := 1.0 - exp(-dt * TAU * bushing_hz * (1.0 + BITE_MOUNT * bite))
	_road = _road * exp(-dt / float(fr.tau)) + kick * float(fr.kick)
	var chatter := loose * clampf(absf(speed) / 15.0, 0.0, 1.0) * 14.0
	_grav = lerpf(_grav, _rng.randfn(0.0, chatter), 1.0 - exp(-dt / 0.02))
	# The scrub: 22 Hz as the tires start to slide, towards 34 well past the peak, never quite even.
	_scrub_ph = fmod(_scrub_ph + dt * TAU * (22.0 + 12.0 * scrub_x), TAU)
	_scrub_a = clampf(lerpf(_scrub_a, _rng.randfn(1.0, 0.6), 1.0 - exp(-dt / 0.05)), 0.3, 1.8)
	var scrub := sin(_scrub_ph) * _scrub_a * scrub_kp
	_road_f = lerpf(_road_f, (_road + _grav + gyro + scrub) * kickback, mount)
	_mz = lerpf(_mz, mz, mount)
	# The box: the tires' torque comes back up through the gears at its back-drive efficiency.
	var manual := (_mz + _road_f * road_k * _eps_road) * float(bx.eff) / rt
	# Assist: the valve / EPS map boosts in proportion to how hard the column is twisted, so around
	# the centre the driver still feels most of the tires, and a heavy load is mostly taken away.
	_level(car, dt, _n_fz, rt)
	var a := assist_max * pressure
	# Sawing at the wheel faster than the pump can fill the rack: the assist falls behind and the rim
	# goes heavy for a moment (worst at idle, where the pump turns slowest: the car park shuffle).
	var half_rim := deg_to_rad(rim_deg * 0.5)
	var rw: float = (float(car.steer_input) - _steer_prev) * half_rim / maxf(dt, 1e-4)
	_steer_prev = float(car.steer_input)
	_rim_w = lerpf(_rim_w, rw, 1.0 - exp(-dt / 0.03))
	pump_short = 0.0
	if assist_type == "hydraulic" and a > 0.0:
		var idle: float = maxf(float(car.drivetrain.idle_rpm), 300.0)
		var flow := lerpf(9.0, 15.0, smoothstep(idle, idle * 2.2, float(car.rpm())))    # rad/s of rim it can follow
		pump_short = clampf(1.0 - flow / maxf(absf(_rim_w), 0.01), 0.0, 0.8)
		a *= 1.0 - pump_short
	if assist_type == "electric":
		a *= lerpf(1.0, _eps_floor, smoothstep(5.0, 30.0, absf(speed)))    # speed-sensitive map
	var knee := BOOST_KNEE * clampf((1.0 - assist_max) / 0.35, 0.3, 2.0) * _eps_knee
	var boost := lerpf(a * (1.0 - exp(-absf(manual) / knee)), a * BITE_ASSIST, bite)
	# Hitting something: the sideways part of the hit at the front yanks the rim (the assist fights it,
	# not much: it's too quick for the valve).
	var loc := car.global_transform.basis.inverse() * (v - prev_v)
	if absf(loc.x) > 1.5:
		_jolt += clampf(-loc.x * 1.6, -12.0, 12.0) * road_k * (0.5 if kickback < 0.5 else 1.0)
	_jolt *= exp(-dt / 0.07)
	# Free play: inside the box's slack (car.gd _steer_wheels: the rack only follows once the rim has
	# taken it up) the tires are not on the other end of the column - turn back a little out of a bend
	# and for those few degrees the rim carries nothing but its own friction. Half a degree on a good
	# rack, a few on a steering box, a handful on a worn-out car.
	var band: float = (float(car.steer_play) + (float(car.damage.rack) * 0.04 if car.damage else 0.0)) * (1.0 - absf(float(car.steer_input)))
	var pull: float = float(car.damage.rack_pull) if car.damage else 0.0
	engaged = 1.0 if band < 1e-5 else smoothstep(0.6, 0.98, absf(float(car.steer_input) + pull - float(car._rack)) / band)
	# ... but only while nothing presses on it. The slack is between the gear faces: as soon as the
	# tires push back with any weight (a bend, a camber, a crosswind) they hold the rack against the
	# pinion and the rim carries them, whichever way it is being turned. And a kick from the road
	# crosses the gap by itself: that is the knock of a worn rack. (Multiplying the lot by `engaged`
	# left the rim dead on every straight: no road at all with the wheel held still.)
	var m_road := _road_f * road_k * _eps_road * float(bx.eff) / rt
	var m_tire := manual - m_road
	var seated := maxf(engaged, smoothstep(0.3, 1.2, absf(m_tire)))
	var level := lerpf(1.0, BITE_LEVEL, bite)
	torque = (m_tire * seated + m_road * lerpf(0.7, 1.0, engaged)) * (1.0 - boost) * level + _jolt
	# The column as a spring / damper at the rim, for the helper's fast loop: through the box (ratio
	# twice: the torque comes down by it and the angle is geared up by it), and the assist takes its
	# share of any change the way it takes its share of the load.
	var em := exp(-absf(manual) / knee)
	var pass_k := clampf(lerpf(1.0 - a * (1.0 - em) - absf(manual) * a * em / knee, 1.0 - a * BITE_ASSIST, bite), 0.05, 1.0) * level
	var rolling := smoothstep(0.5, 3.0, absf(speed))
	# The linkage gives under the tires' aligning torque (car.gd _kingpin_steer: the wheels turn a
	# little less than the rack asked), so a move of the rim only gets this share of its angle to
	# the tires: a spring in series, as the car now drives.
	if bool(car.steer_compliance_on) and k_al > 0.0:
		k_kp /= 1.0 + k_al / maxf(kss, 1000.0)
	stiff = maxf(k_kp, 0.0) * float(bx.eff) / (rt * rt) * pass_k * rolling
	# ... through the column's own twist: two springs in a row are softer than either. On a power
	# steered car the bar is most of why the first few degrees feel elastic.
	if stiff > 0.0:
		stiff = stiff * tbar / (stiff + tbar) * engaged
	damp_nms = float(bx.visc) + c_kp * float(bx.eff) / (rt * rt) * pass_k
	if assist_type == "hydraulic":
		damp_nms += 0.25 * pressure * (1.0 - 0.5 * bite)       # fluid through the valve
	elif assist_type == "electric":
		damp_nms += 0.12 * pressure       # the motor geared to the column
	lag_s = clampf(relax / maxf(absf(speed), 2.0) + 1.0 / (TAU * bushing_hz * (1.0 + BITE_MOUNT * bite)), 0.005, 0.25)
	# Parked: the tires scrubbing on the road become column friction - all of it on a manual rack or
	# with the pump dead, a little with assist (that's what power steering was for).
	var park := 1.0 - smoothstep(0.3, 4.0, absf(speed))
	# (Parked, the valve is wide open whatever the car's feel at speed: 85-93 % of the scrub is the pump's.)
	friction = friction_nm + scrub_nm / rt * float(bx.eff) * (1.0 - minf(0.72 + 0.25 * a, 0.95) if a > 0.01 else 1.0) * park
	# Fluid in the valve damps the wheel; a pump fighting low fluid shudders.
	damper = 0.08 * park + (0.05 * pressure if assist_type == "hydraulic" else 0.0)
	if assist_type == "hydraulic" and fault == "leak" and fluid < 0.35 and running:
		torque += _rng.randfn(0.0, 0.25) * (1.0 - fluid / 0.35)
	up_gain = _eps_road * kickback * float(bx.eff) / rt * (1.0 - boost)
	_vibration(car, speed, running, road_k * up_gain)


## Through the rack's mounts at `hz`: the same low-pass the road kicks get, as a gain.
func _mount_gain(hz: float) -> float:
	return 1.0 / sqrt(1.0 + (hz / bushing_hz) * (hz / bushing_hz))


## Everything too fast for the physics tick, as numbers the wheel helper turns into the waveform at its
## own rate (tools/ffb/ffb_model.h): each one a real source with a size in newton metres.
##   `up`: kingpin torque -> rim, for what comes up through the rack (kickback, box, ratio, assist).
func _vibration(car: RigidBody3D, speed: float, running: bool, up: float) -> void:
	var v := absf(speed)
	var rolling := smoothstep(1.5, 8.0, v)
	# --- road grain (band 0) and loose stones (band 1): what the front tires are rolling over.
	var grain2 := 0.0
	var grain_len := 0.0
	var stone2 := 0.0
	var wheel_hz := 0.0
	var judder := 0.0
	var tramp := 0.0
	var hop_hz := 13.0
	for i in 2:
		var w = car.wheels[i]
		wheel_hz += absf(float(w.omega)) / TAU * 0.5
		if not w.contact:
			continue
		var sd: Dictionary = Surfaces.get_def(String(w.surface))
		var size: float = GRAIN_BY_ID.get(String(w.surface), GRAIN.get(String(sd.get("sound", "hard")), 0.34))
		# A height change of `rough` under a tire is a load change of tire stiffness x rough; the patch
		# swallows anything much shorter than itself. Under a loose layer the base's grain is buried.
		var env: float = clampf(size / (2.0 * maxf(float(w.patch_len), 0.05)), 0.2, 1.0)
		var g: float = float(w.tyre_k) * float(sd.get("rough", 0.0)) * GRAIN_SHARE * BUMP_ARM * env * (1.0 - 0.7 * float(w.cover))
		grain2 += g * g
		grain_len += size * 0.5
		var st: float = float(w.load) * float(w.cover) * STONE_ARM
		stone2 += st * st
		# Brake judder: no disc is perfectly even, and a hot one less so (thickness variation, hot spots):
		# the brake force rises and falls twice a turn, each side in its own time.
		var btv: float = 0.004 + 0.012 * worn + 0.03 * smoothstep(350.0, 650.0, float(w.brake_temp))
		var jd: float = float(w.brake_torque) / maxf(float(w.radius), 0.1) * btv * (absf(scrub_r) + JUDDER_ARM)
		judder += jd * jd
		# Axle tramp: a driven front tire spinning on a hard road hops on its own springiness, and its
		# drive force comes and goes with it. Soft engine mounts let the whole lump join in.
		if absf(float(w.drive_torque)) > 1.0:
			var spin: float = smoothstep(1.5, 4.0, absf(float(w.kappa)) / maxf(float(w._peak_slip), 0.02)) * (1.0 - 0.8 * float(w.cover))
			tramp += absf(float(w.fx_sum)) / float(car._substeps) * TRAMP_SHARE * spin * (absf(scrub_r) + UNBALANCE_ARM) / sqrt(mount_k)
		hop_hz = sqrt((float(w.tyre_k) + float(w.spring_k)) / maxf(float(w.unsprung_mass), 5.0)) / TAU
	var g_hz := clampf(v / maxf(grain_len, 0.1), 8.0, 140.0)
	noise[0] = g_hz
	noise[1] = sqrt(grain2) * _mount_gain(g_hz) * up * rolling
	noise[2] = 1.5
	var s_hz := clampf(v / 0.18, 20.0, 160.0)
	noise[3] = s_hz
	noise[4] = sqrt(stone2) * _mount_gain(s_hz) * up * rolling
	noise[5] = 2.0
	# --- a front wheel out of balance: a weight going round, the force grows with speed squared and
	# the front end answers it most at its own shake (the 60-70 mph shimmy of a tired car).
	var om := wheel_hz * TAU
	var r_w: float = car.wheels[0].radius
	var ratio_hz := wheel_hz / maxf(hop_hz, 4.0)
	var mag := 1.0 / sqrt((1.0 - ratio_hz * ratio_hz) * (1.0 - ratio_hz * ratio_hz) + (ratio_hz / WHEEL_HOP_Q) * (ratio_hz / WHEEL_HOP_Q))
	osc[4] = wheel_hz
	osc[5] = (UNBALANCE_KG + 0.04 * worn) * r_w * 0.65 * om * om * UNBALANCE_ARM * minf(mag, WHEEL_HOP_Q) * up
	osc[6] = wheel_hz * 2.0
	osc[7] = sqrt(judder) * up * rolling
	osc[8] = hop_hz
	osc[9] = tramp * up
	# --- the engine, through its mounts and the body to the column (not through the rack: the assist
	# has no say). Firing pulses: what an idling engine rocks with plus what it is making, passed by the
	# mounts less and less above their own frequency; and the shake of the moving parts themselves, which
	# grows with revs squared and which a four has most of. Solid mounts pass the lot.
	osc[0] = 30.0
	osc[1] = 0.0
	osc[2] = 15.0
	osc[3] = 0.0
	if running:
		var rpm: float = car.rpm()
		var f_hz := clampf(rpm / 60.0 * cylinders * 0.5, 6.0, 300.0)
		var fn := MOUNT_HZ * sqrt(mount_k)
		var tr := clampf((fn / f_hz) * (fn / f_hz), 0.004, 1.5)
		var cyl_k := pow(4.0 / maxf(cylinders, 2.0), 1.5)
		var pulse := (FIRING_NM + 1.2 * absf(float(car.drivetrain.engine_torque))) * cyl_k
		var red: float = maxf(float(car.drivetrain.redline), 3000.0)
		var shake := (0.05 if cylinders <= 4.5 else 0.012) * (rpm / red) * (rpm / red) * sqrt(mount_k)
		osc[0] = f_hz
		osc[1] = pulse * tr * ENGINE_COUPLING + shake
		# Half order: no two cylinders are the same, least of all at idle on a tired engine.
		osc[2] = f_hz * 0.5
		osc[3] = pulse * clampf((fn / (f_hz * 0.5)) * (fn / (f_hz * 0.5)), 0.004, 1.5) * ENGINE_COUPLING * (0.12 + 0.3 * worn) \
			* (1.0 - smoothstep(float(car.drivetrain.idle_rpm) * 1.3, float(car.drivetrain.idle_rpm) * 2.5, rpm))


## Works out ref_nm / norm (see NORM_NM), a few times a second.
func _level(car: RigidBody3D, dt: float, fz_front: float, rt: float) -> void:
	if fz_front > 100.0:
		_fz_avg = fz_front if _fz_avg <= 0.0 else lerpf(_fz_avg, fz_front, 1.0 - exp(-dt / 4.0))
	_n_tick += 1
	if _n_tick % 30 != 1 or _fz_avg <= 0.0:
		return
	var w = car.wheels[0]
	var t0: float = float(w.patch_len) * PNEU_TRAIL
	var tm: float = float(w.radius) * tan(deg_to_rad(clampf(float(w.caster), 0.0, 12.0))) * (1.0 + BITE_CASTER * bite)
	var a := assist_max
	if assist_type == "electric":
		a *= lerpf(1.0, _eps_floor, 0.6)
	var knee := BOOST_KNEE * clampf((1.0 - assist_max) / 0.35, 0.3, 2.0) * _eps_knee
	var best := 0.0
	for i in range(1, 21):
		var x := float(i) / 16.0
		var th := minf(x, 1.0)
		var fy := _fz_avg * (1.0 - pow(1.0 - th, 3.0))          # both fronts, grip 1.0, the brush model's build-up
		var manual := fy * maxf(_trail(x, t0, 1.0 + bite) + tm, 0.0) * float(BOX[box].eff) / rt
		var boost := lerpf(a * (1.0 - exp(-manual / knee)), a * BITE_ASSIST, bite)
		best = maxf(best, manual * (1.0 - boost) * lerpf(1.0, BITE_LEVEL, bite))
	ref_nm = maxf(best, 0.3)
	var want := clampf(pow(NORM_NM / ref_nm, NORM_SHARE), 0.4, 4.0)
	norm = want if _n_tick <= 1 else lerpf(norm, want, 0.2)


## Pneumatic trail (m) at `x` = slip angle / the peak's: the brush model's shape from t0 at zero slip to
## nothing at the peak, then negative past it (`neg` times as far: the rally feel doubles it, so the
## rim still drops when the fronts let go although the longer caster arm keeps pulling) (measured aligning torque curves cross zero near the
## peak and settle around a quarter of their own peak the other way, a tenth of t0 as trail): with the
## caster trail still pulling, that is the wheel going clearly light when the fronts let go.
static func _trail(x: float, t0: float, neg := 1.0) -> float:
	var th := minf(x, 1.0)
	var tp := t0 * pow(1.0 - th, 3.0) / maxf(1.0 - th + th * th / 3.0, 0.05)
	if x > 1.0:
		tp -= t0 * 0.1 * neg * minf((x - 1.0) / 0.4, 1.0)
	return tp


## Free play at the rim (deg each way) for this box, before wear.
func play_deg() -> float:
	return float(BOX[box].play)


## Pump pressure / EPS current. Hydraulic: driven off the crank, so a stalled engine is a manual rack
## (the classic: stall in a corner and the wheel goes to lead) and it's a bit weak below idle.
func update(dt: float, running: bool, rpm: float, idle: float) -> void:
	if fault == "leak":
		fluid = maxf(fluid - dt / 60.0, 0.0)          # about a minute from a cracked line to dry
	var want := 0.0
	match assist_type:
		"hydraulic":
			if running and fault != "belt":
				want = smoothstep(0.3, 0.85, rpm / maxf(idle, 300.0)) * smoothstep(0.0, 0.35, fluid)
		"electric":
			want = 1.0 if running and fault != "eps" else 0.0
	pressure = move_toward(pressure, want, dt / (0.12 if want > pressure else 0.3))
