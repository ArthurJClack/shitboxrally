class_name Wheel
extends Node3D
## One corner of the car: raycast suspension + brush-style combined-slip tire with relaxation
## length, wheel spin dynamics and lockable brakes. The node itself sits at the suspension mount;
## the visual model is moved along the strut.

const TIRE_CURVE := preload("res://src/car/tire_curve.gd")
const NATIVE := preload("res://src/car/native/native.gd")
const NL := preload("res://src/car/native/layout.gd")
const GS := preload("res://src/track/ground_shape.gd")

# --- configuration (set by Car.setup)
const TYRES := preload("res://src/data/tyres.gd")
var radius := 0.3
var width := 0.18
var inertia := 0.9
var max_length := 0.25          # full droop distance from mount to wheel centre
var spring_k := 25000.0
var bump_damp := 2200.0
var rebound_damp := 3200.0
# Dampers aren't linear: real ones have a bleed circuit for slow shaft speeds (body roll, pitch, the
# car settling after a landing) and a blow-off valve that opens past a knee for the fast hits
# (crests, potholes, washboard). Below `damp_knee` the force is damp_slow x the car's figure, above
# it each extra m/s adds bump_fast / rebound_fast x. Tuned on src/tools/damper_test: a softer fast
# side (the textbook digressive valve) let the hub hop more on washboard and the body bounce longer
# after a landing, so the gain is all in the slow circuit - the body is held tighter while the fast
# side stays the car's own figure: landings settle ~0.1-0.3 s sooner with a bounce less, tires spend
# less time unloaded on washboard, the body ~0.01 g busier there. (static: SBR_DAMP=0 / the benches
# put back the old straight line; car files can override per axle.)
static var damp_linear := false
var damp_knee := 0.15           # m/s of shaft speed
var damp_slow := 1.5            # x bump / rebound below the knee
var bump_fast := 1.0            # x bump_damp per m/s above it
var rebound_fast := 1.0         # x rebound_damp per m/s above it
var bumpstop_k := 120000.0
# Bump and droop stops (progressive_stops): the bump stop is a rubber jounce bumper that starts
# soft and stiffens as it squashes (x^3), with some hysteresis; packers (m) bring it in earlier.
# The droop side is a top-out rubber in the strut: the hub runs a little past full droop into
# it and its momentum goes into the body, instead of being thrown away at a hard clamp (which
# let a car leave a crest with its hubs' momentum still pushing it up). preload_n: spring preload
# (N): the spring still pushes this hard at full droop (Car.setup takes it off the droop travel,
# so the car sits at the same height).
var packer := 0.0
var preload_n := 0.0
var stop_start := 0.22          # compression where the bump stop starts (m), set each tick
var droop_pull := 0.0           # the top-out pulling the body down this tick (N, >= 0)
static var progressive_stops := true   # (static: the bench comparison turns it off, SBR_STOPS=0)
const TOPOUT_LEN := 0.012       # top-out rubber (m): how far past full droop the hub can go
const TOPOUT_K := 250000.0      # its initial rate (N/m), x4 when fully squashed
const STOP_C := 2.5             # rubber damping, x the bump damper, full when the stop's squashed
const STOP_REB := 0.8           # ... and on the way back out (rubber hysteresis)
const STOP_CAP := 4.0           # most the hub's stopped momentum passes per substep, x this corner's weight
var _stop_debt := 0.0           # hub momentum stopped at a hard end not passed to the body yet (N s)
var front := true
var left := true
var driven := false
var grip_scale := 1.0
var nominal_load := 3000.0
var tyre: Dictionary = {}       # tire type multipliers (data/tyres.gd)
var ref_load := 3000.0          # tire design load (same for all four tires: the car's average wheel load)
var relax_long := 0.20          # tire relaxation lengths (m)
var relax_lat := 0.45

# --- state
var omega := 0.0                # wheel spin (rad/s), positive = rolling forward
var kappa := 0.0                # longitudinal slip state
var slip_lat := 0.0             # lateral slip state (tan alpha)
var steer_angle := 0.0
var contact := false
var contact_point := Vector3.ZERO
var contact_normal := Vector3.UP
var surface := "tarmac"
var compression := 0.0
var prev_compression := 0.0
var susp_length := 0.25
var load := 0.0                 # Fz (N): what the tire presses on the road (grip)
var body_force := 0.0           # what the strut pushes into the body (N)
# Unsprung mass: the hub (wheel, tire, brake, knuckle, half the links) is a mass between the tire
# (a stiff spring against the road) and the suspension. Heavy wheels follow bumps worse: the
# tire's load swings more, and at worst the wheel hops. It only adds inertia (the body's mass is
# the whole car), so standing still everything sits exactly where it did with a rigid tire.
var unsprung_mass := 35.0       # kg
var tyre_k := 200000.0          # tire vertical stiffness (N/m)
var tyre_c := 350.0             # tire damping (N s/m)
var static_load := 3000.0       # this corner's share of the car's weight (N)
var hub_v := 0.0                # compression rate of the hub (m/s)
var _c_ground := 0.0            # where the hub would be with a rigid tire (compression, m)
var _c_ground_prev := 0.0
var _fresh := true
var fx := 0.0                   # tire forces of current substep (N)
var fy := 0.0
var f_max := 0.0                # grip limit of the current substep (friction circle radius, N)
var fx_sum := 0.0
var fy_sum := 0.0
var v_long := 0.0               # contact patch velocity in wheel frame
var v_lat := 0.0
var drive_torque := 0.0
var brake_torque := 0.0
var rub_torque := 0.0           # a crumpled fender rubbing the tire (body_panels.gd)
var arb_force := 0.0
var spin_angle := 0.0
const MU_LONG := 1.08
# The force curve itself is the Magic Formula in normalized slip (tire_curve.gd): its shape comes from
# the surface's sliding ratio and the tire's stiffness (shape cache `_tc_*`, set when either changes).
# Under power the force is shared out round a real friction circle in normalized slip; the only
# difference between driving and cornering is MU_LONG.
# Braking grip on top of MU_LONG: a tire pulls harder slowing down than driving (measured braking
# peaks sit ~5-15 % over traction peaks: the tread blocks lean back into the patch instead of
# tipping out of it, and the patch lengthens as the nose dips). Only the braking side, so launches,
# power slides and cornering are unchanged, and so are handbrake turns: the share of a rear's
# braking that comes from the handbrake (hb_share, set by Car) doesn't get it, so a flick throws the
# tail out exactly as before. (static var: src/tools/brake_test sweeps it.)
static var mu_brake := 1.15
var hb_share := 0.0
# Rubber grips less the faster it slides over the road (the hysteresis peak sits at a few cm/s of
# sliding; a locked, spinning or sideways tire flashes its surface hot and the friction falls away).
# Measured on how fast the patch slides past slide_v0, and it builds with the tread's heat over
# ~slide_heat s, so a short slide costs nearly nothing: a held one at 10 m/s ~9 %, a big one at
# 25 m/s ~18 %. A locked wheel at 60 mph brakes ~15 % worse than one held at the peak (ABS / a good
# foot), a power drift on spinning rears loses drive, and a drift you keep holding scrubs grip and
# wants catching early. (Owner's call Oct 2026: the realistic version. slide_lat 0 = fore-aft only,
# the forgiving variant: the 30 m/s open-loop swerve on the handling bench spins more cars with it on.)
# Hard base only: the loose layer has its own shear model.
# (static: SBR_SLIDEMU / SBR_SLIDEV0 / SBR_SLIDELAT / SBR_SLIDEHEAT in the benches.)
static var slide_mu_drop := 0.35
static var slide_v0 := 3.0       # m/s of patch sliding before it starts (SBR_SLIDEV0)
const SLIDE_V1 := 20.0           # m/s past that for half the drop
static var slide_lat := 1.0      # how much sideways sliding counts (0..1)
static var slide_heat := 0.8     # s: how fast the sliding loss builds / goes (SBR_SLIDEHEAT)
var _slide_ex := 0.0             # slide speed past slide_v0, through the tread's heat lag (m/s)
# The berm a sideways tire banks up on loose ground (compute_tire_force): the slip (tan) where it
# spills over the shoulder and stops growing, and the most it holds, x load. (static: the gravel bench.)
static var berm_tan := 0.36
static var berm_cap := 0.3
static var abs_loose := 0.45       # share of the way to a 25 % slip target the ABS goes on loose ground
# The seated bed (compute_tire_force): stones that haven't moved are interlocked and hold more than
# stones that are flowing (dense peak vs critical-state strength, surfaces.gd `bed_mu`), until about
# half a stone's width of shear (`grain`) breaks them loose. And the bed only jams if it gets the time:
# the inertial number of the patch's dwell, I = d sqrt(rho / P) * v / L (grain size, grain density,
# contact pressure, ground speed, patch length). Granular packings are dense and jammed for small I
# and dilate linearly with it (da Cruz et al. 2005, GDR MiDi 2004), so the extra hold fades to
# nothing by I = bed_i0. Crawling: full bite. Fast: the tire rides on stones that never locked up.
static var bed_on := true          # (SBR_BED=0: the bench comparison)
static var bed_i0 := 0.2           # inertial number where the bed has no time left to jam (SBR_BEDI0)
static var bed_soft := 1.2         # how much softer in shear the unseated layer is than the half-seated one (SBR_BEDSOFT)
# Studs (tyres.gd `studs`: 1 = road-legal studs, 3 = long rally studs). A stud is a steel pin that
# scratches a groove in ice; it has nothing to do with rubber, so it gets its own law. A tire's
# ice / snow rating stays what it holds at -5 C at its design load (what the planners and the garage
# read); under the wheel that splits into the rubber's share and the studs' (stud_share), and:
#   * temperature: rubber on ice is at its worst at 0 C (a film of melt) and keeps gaining as it
#     gets colder; studs are the other way round, best in soft ice near freezing and scratching less
#     and less as the ice hardens. Studded tires halve the stop on wet ice at 0 C (VTI) and are about
#     level with a good studless tire by -20 C.
#   * load: the pins are pressed flush at the car's own weight, so more load only puts more of them
#     on the ground (patch length ~ sqrt of load), not more force on each.
#   * sliding: a pin scratches the same locked or rolling, so there's no drop past the peak for the
#     studs' share, and the peak sits further out (the curve's shape, prepare_surface).
#   * snow: short road studs barely reach anything in packed snow; long ones go through it. Loose
#     snow on top (the layer's cover) keeps them all off the hard base - the first car through fresh
#     snow on studs has less than the tenth one.
# (static: SBR_STUDS=0 puts the old plain multiplier back.)
static var studs_on := true
const STUD_HALF := 2.33            # `studs` at which they are half the tire's hold on ice (road studs: 30 %)
const STUD_SLIDE := 0.95           # sliding / peak for the studs' share
const GRAIN_RHO := 2650.0          # kg/m^3, the stone itself (quartz / granite)
const LOOSE_LIN := 0.975
static var loose_linear := true    # (SBR_LOOSELIN=0: the rubber's load curve on the loose layer too, as before)
static var loose_plane_v := 30.0   # m/s: rolling this fast halves the sinkage into a loose layer
# The tire and the ground in series. A patch of tread on stones is two springs end to end: the
# rubber (tread blocks + carcass, the same give that sets the tire's cornering stiffness on a hard
# road) and the layer of stones under it. The slip the patch sees is shared between them, so the
# pair is always softer than either alone, and a tire can never answer more sharply on gravel than
# it does on tarmac. (Before, the layer's own shear length was used as if the tire were rigid: a
# gravel tire on hardpack built side force 2.2x quicker than on tarmac, and on dirt 3.3x.)
# The tire's share: a brush with cornering stiffness C per unit load carries a ground stress of
# mu * p at a tread shear of mu * L / (2 C) (`tire_give`). The flowing layer's exponential is solved
# in series with that spring in compute_tire_force, and the seated bed's half a stone has it added,
# so soft tires, low pressures and long patches all give more.
# Stones are not a smooth film. A patch stands on a few hundred of them and the load goes through
# chains of a few stones each, so how much one patch of road holds differs from the next by about
# sqrt(GRAIN_CHAIN / stones under the patch): 12 mm gravel under an ordinary tire +-14 %, 5 mm
# hardpack scatter +-6 %, dirt +-5 %; a wide tire or a long, soft patch averages over more of them.
# A new sample every patch length, fixed to the spot on the road (the same for every car and every
# replay). At speed the car's mass averages it away and the rim just fizzes; crawling, it shuffles.
static var grain_noise_on := true  # (SBR_GRAINS=0: every patch of a surface holds the same)
const GRAIN_CHAIN := 4.0           # stones in a force chain (the patch counts chains, not stones)
const GRAIN_CELL := 0.25           # m between samples
static var series_on := true       # (SBR_SERIES=0: the layer on a rigid tire, as before)
const SERIES_PEAK := 0.14          # the hard road the tire's own stiffness is read on (tarmac's peak_angle)
const LOAD_SENS := 0.18          # friction falls ~18% per +100% load
const CORNER_SENS := 0.45        # peak slip angle grows with load^0.45 (stiffness per newton falls)
var visual_out := 0.0           # visual-only outward offset so the tire sits flush with the body side
var kp_off := Vector3.ZERO      # visual-only: centre -> steering axis at hub height (set_kingpin)
var kp_axis := Vector3.ZERO     # ... and the axis (ZERO = a typical one, kingpin_in)
var fwd_dir := Vector3.FORWARD
var side_dir := Vector3.RIGHT
# --- ground / loose layer (refreshed every tick)
var loose_mat := ""             # material of the loose layer under the patch
var loose_depth := 0.0          # loose material under the patch (m)
var cover := 0.0                # fraction of the patch riding on loose material
var patch_len := 0.2            # contact patch length (m)
var feature_water := 0.0        # water depth under the tire (m): drag + spray
var feature_water_size := 0.0   # how big that puddle / ford is across (m): the splash sound
var water_vel := Vector3.ZERO    # how fast the water under the tire is running (world, m/s; water_sim.gd)
const WATER := preload("res://src/track/water_sim.gd")
var aquaplane := 0.0           # 0..1 how much this tire is riding on water
# Temperatures. Brakes: the disc heats with braking power and cools with airflow; past its pads'
# limit it fades. Tires: the tread heats from slip and rolling and cools with speed; grip is best
# inside the tire's working window (semi-slicks need heat, winter tires go greasy when warm).
var brake_temp := 20.0          # C
# Tire pressure. Set cold in the garage (at 20 C); it follows the air temperature in the tire
# (Gay-Lussac: p ~ absolute temperature), so it reads low on a cold morning and climbs as the tire
# works. Effects relative to the placard pressure (psi_rec), see pressure_effects().
# Alignment (degrees, set from the car's def.alignment): static camber (negative = top in), toe
# (positive = toe-in, applied as a fixed steer angle), caster (fronts: the wheels lean into the turn
# as they steer). roll_gain: how much of the body's roll the wheel follows (struts / wishbones),
# 0 for a beam axle.
var camber := 0.0
var toe := 0.0
var caster := 0.0
var roll_gain := 0.7
# Suspension geometry (src/data/suspension_geo.gd, set by Car._setup_geo): camber gained per metre
# of bump (deg/m, top in), the compression the car sits at, and the push the links put between
# body and hub this tick (N, + lifts the body / loads the tire; worked out by the car).
var bump_camber := 0.0
var comp0 := 0.0
# Bump steer + compliance toe (same file): toe gained per metre of bump (deg/m, + = toe-in) and toe
# gained when the tire brakes with a force equal to this corner's weight (deg, + = in; the bushings
# give, drive pushes the other way). _dyn_toe is the sum, settled through the rubber bushings, that
# toe_steer() adds; worked out once a tick before the steering (update_dyn_toe).
var bump_toe := 0.0
var brake_toe := 0.0
var _dyn_toe := 0.0
var lat_toe := 0.0              # rear only: deg of steer into the side force at a side force of the corner's weight (suspension_geo ltoe)
static var lat_toe_on := true
const DYN_TOE_MAX := 0.35        # deg either way
const LAT_TOE_SIGN := 1.0
var geo_jack := 0.0
var geo_rc_k := 0.0             # roll centre height / half track
var geo_tan := 0.0              # anti-dive / anti-squat force line slope
var body_roll := 0.0            # deg, car up tilting towards +x against the road (set by the car every tick)
## Camber thrust: a tire leaning over pushes towards the lean with no slip angle at all (a rolling
## cone), about as hard as CAMBER_STEER degrees of slip angle would per degree of lean (camber
## stiffness over cornering stiffness: 0.05-0.1 on radial car tires, Milliken ch. 2 / Pacejka's
## measured sets). Carried as that much extra steer where the tire's slip is measured (car.gd
## _prepare_wheels), so it runs out with the tire's grip like the real thing. The outside tires of
## a rolling car lean out of the bend and give some of their side force away: struts and beam
## fronts lose the most, which is a good part of why they push; negative camber buys it back.
const CAMBER_STEER := 0.07
var camber_steer := 0.0         # rad, + = left (as steer_angle)
static var camber_thrust_on := true
var tilt := 0.0                 # deg, wheel top leaning towards +x, relative to the road
var _al_lat := 1.0
var _al_long := 1.0
var _ps_lat := 1.0
var _ps_long := 1.0
var psi_cold := 32.0            # set in the garage, at 20 C
# --- damage picked up during a stage (see damage_hit / _wear_step)
var tread_mm := 8.0             # depth left (mm); wears with slip energy
var leak := 0.0                 # air going out through a cut (psi/s)
var flat := false               # too soft to drive on
var rim_bend := 0.0             # 0..1 out of true: vibration that rises with road speed
var bent_toe := 0.0             # degrees knocked into the toe on this corner
var bent_camber := 0.0
# --- mechanical damage (src/car/car_damage.gd pushes these)
var arm := 0.0                  # suspension arm: 0 straight .. 1 broken
var arm_camber := 0.0           # degrees the bent arm adds
var arm_toe := 0.0
var flat_mm := 0.0              # flat spot ground into the tread by a lock-up
var shaft_broken := false       # this corner's driveshaft has snapped: no drive through it
var flex_off := 0.0             # m: this mount moved up / down by the chassis twisting (car._chassis_twist)
var struck := 0.0               # severity of the last kerb / rock strike, for the car to read
var struck_landing := false     # ...and whether that was a jump landing rather than an edge
var air_t := 0.0                # s this tire has been off the ground (0 while touching)
var since_land := 9.0           # s since it last came down after a real bit of air
var _flat_at := 0.0             # where round the tire the flat is (rad)
var _flat_prev := 0.0           # spin angle last tick
var _tread_bite := 1.0          # how much the remaining tread can dig into a loose surface
var _hit_sev := 0.0             # a kerb / pothole strike waiting to be judged
var _hit_landing := false       # that strike was the tire coming back down off a jump
var _dmg_rng: RandomNumberGenerator
var psi_rec := 32.0             # placard / design pressure
var psi := 32.0                 # current (hot) pressure
var psi_set := 0.0              # the cold pressure the car was set up with (a new wheel at a reset gets it again; 0 = the placard's)
var air_temp := 15.0             # ambient (C)
var tyre_k_base := 200000.0
var _k_taken := false              # tyre_k_base read from tyre_k yet (once: see apply_carcass)
var _p_mu_hard := 1.0
var _p_mu_loose := 1.0
var _p_corner := 1.0
var _p_roll := 1.0
var _p_heat := 1.0
var _p_aqua := 1.0
var tyre_temp := 20.0            # C: the tread as the grip feels it (the loaded part of the three below)
## Tread surface temperatures across the tire: inner shoulder (towards the car's middle), centre,
## outer shoulder - what a pyrometer reads in the service park - and the carcass under them.
var tread_t := PackedFloat32Array([20.0, 20.0, 20.0])
var carcass_t := 20.0
var _tread_w := Vector3(1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0)   # inner / centre / outer share of the patch
var brake_cap := 2300.0         # J/K: effective (the disc's surface heats first; big brake kits more)
var brake_fade := Vector2(450.0, 750.0)   # starts fading .. 60 % bite (C)
var tyre_window := Vector2(10.0, 95.0)    # grip window (C)
var feature_grip := 1.0         # grip multiplier from road features under the tire (puddle, tar snake, paint)
var _road_i := -1               # last centre-line index, as a search hint
var thrown := 0.0               # loose material moved by this tire (m^3/s), for effects
var plough := 0.0               # 0..1: how hard the sideways tire is banking stones up (of the most it can), for effects
var abs_slip := 0.11            # slip ratio the ABS regulates to
var abs_gain := 1.0             # share of the pedal's pressure the ABS lets through to this caliper
var rolling_torque := 0.0
var _mu := 1.0
var _peak_slip := 0.11
var _peak_angle := 0.14
var _tc_c := 1.41                # Magic Formula shape of this tire on this surface (tire_curve.gd)
var _tc_b := 2.0
var _tc_e := -1.0
var _mu_l := 0.0
var _shear_k := 0.01
var _stud_mu := 0.0             # the studs' part of _mu (prepare_surface), for prep_substeps
var stud_share := 0.0           # studs' share of this tire's hold on this surface at -5 C (0 = none)
var _bed := 1.0                 # what the seated bed holds at its peak, x the flowing layer's limit
var _bed_k := 0.06              # shear at the back of the patch (m) where the whole seated patch slides
var no_bed := false             # this wheel alone without the seated bed (bed_curve's comparison)
var bed_seat := 0.0             # 0..1: how seated the bed under the patch is (for the HUD / benches)
var tire_give := 0.0            # m of the tire's own shear in series with the layer (series_on), for the benches
## How well this tire on this material banks a berm up against its shoulder: big open blocks cut
## in and hold one, a worn touring tire skates over the top of it. Set in pressure_effects().
var _berm := 1.0
var _coh := 0.0
var _bull := 0.0
var _spin_fade := 0.0

var visual: Node3D
# Tire deformation (looks): the carcass is a spring between rim and road, sideways and fore / aft.
const TYRE_DEFORM := preload("res://src/render/tyre_deform.gd")
const DEFORM_SHOW := 1.6         # sideways / fore-aft carcass bend, exaggerated so the chase cam sees it
## (The flattening is NOT exaggerated: it has to meet the road exactly, see _update_deform.)
var rim_ratio := 0.65            # bead radius / outer radius
var carcass_k_lat := 160000.0    # N/m: how far the patch is dragged sideways per newton
var carcass_k_long := 260000.0
var _deform_meshes: Array = []   # [MeshInstance3D, rim lip r, outer r, half width] (local units)
var _deform_dirty := false       # _update_deform ran since push_deform last put it on the shader
var _shift := Vector3.ZERO       # smoothed patch shift (world, m)
var _twist := 0.0                # smoothed patch twist (tan of the tread's angle to the wheel, along side_dir per m of fwd_dir)
var _squash := 0.0


## Ray positions along the footprint, as fractions of the radius (0 = straight down).
const RAY_OFFSETS := [0.0, -0.3, 0.3, -0.6, 0.6, -0.88, 0.88]
## Which of those a car uses at each Car.Sim level: all of them for the car you're driving and
## anything near you, three for the middle distance, one for cars far enough away that a wheel is
## a few pixels. Ride quality drops with the count, and nothing you can see is on the short lists.
const RAY_SETS := [[0, 1, 2, 3, 4, 5, 6], [0, 3, 4], [0]]
var rays: Array = RAY_SETS[0]
var susp_substeps := 8           # vertical dynamics steps per tick (4 on the Potato preset)
var sim_water := true            # read the live water (off for cars far enough to be a few pixels)
var showroom := false            # its car is the loading screen's (car.gd): the stage's GroundMap isn't under it
## Road features (ice, puddles, washboard, paint...) are re-read once the tire has moved this far.
const FEATURE_STEP := 0.07
const FEATURE_STEP_SQ := FEATURE_STEP * FEATURE_STEP
var _feat_at := Vector3(1e9, 1e9, 1e9)   # where the features were last read
var _feat_surface := ""
var _feat_grip := 1.0
var _feat_bump := 0.0
var _feat_water := 0.0
var _feat_water_size := 0.0
var _feat_rut := Vector3.ZERO    # frozen rut wall under the tire: horizontal push per newton of load
## Sideways shove from the wall of a frozen rut the tire is in (per newton of load; car.gd applies
## it with the load). The wall's slope tips the road's push on the tire towards the groove bottom.
var rut_push := Vector3.ZERO
const RUT_WALL := 0.7            # how much of the wall's slope shows up as a sideways push
const ROAD_DETAIL := preload("res://src/track/road_detail.gd")
var _feat_bumps: Array = []      # washboard / cattle grids found at _feat_at (road_detail q_bumps)
var _feat_s := 0.0               # distance along the road at _feat_at
## Collider instance id -> its surface name. Shared by every wheel on every car; cleared when a
## stage is torn down (TrackGenerator._exit_tree).
static var _surfaces := {}


static func _surface_of(col: Object) -> String:
	if col == null:
		return "tarmac"
	var id := col.get_instance_id()
	var s: Variant = _surfaces.get(id)
	if s == null:
		s = String(col.get_meta("surface", "tarmac"))
		_surfaces[id] = s
	return s


## Collider instance id -> its surface's small-scale roughness (m, ground_shape.gd fine). Same lifetime.
static var _roughs := {}


static func _rough_of(col: Object) -> float:
	if col == null:
		return 0.0
	var id := col.get_instance_id()
	var r: Variant = _roughs.get(id)
	if r == null:
		r = float(Surfaces.get_def(_surface_of(col)).get("rough", 0.0))
		_roughs[id] = r
	return r


## Collider instance id -> whether it is road (carries the road-feature lookup). Same lifetime as _surfaces.
static var _roads := {}


static func _road_of(col: Object) -> bool:
	if col == null:
		return false
	var id := col.get_instance_id()
	var r: Variant = _roads.get(id)
	if r == null:
		r = col.has_meta("road")
		_roads[id] = r
	return r


## Per-ray offset, lift (how far the tire circle sits above its lowest point there) and normal weight,
## worked out once per radius instead of a sqrt per ray per tick.
var _lift_r := -1.0
var _ray_x: PackedFloat64Array
var _ray_lift: PackedFloat64Array
var _ray_w: PackedFloat64Array


var _rc: Array = []                # RayCast3D per RAY_OFFSETS entry, on _rc_holder
var _rc_holder: Node3D
var _rc_reach := -1.0
var _rc_ex: Array[RID] = []


func _ray_cache() -> void:
	_lift_r = radius
	_rc_reach = max_length + radius
	if _rc_holder == null:
		_rc_holder = Node3D.new()
		_rc_holder.name = "SuspensionRays"
		add_child(_rc_holder)
		for k in RAY_OFFSETS.size():
			var rc := RayCast3D.new()
			rc.enabled = false                 # cast on demand (force_raycast_update), never on its own
			# Everything (as a fresh PhysicsRayQueryParameters3D) but a reset ghost's shell: you drive
			# through a ghost, not up onto it. The ground carries layer 1 as well, so it is still hit.
			rc.collision_mask = 0xFFFFFFFF & ~Car.GHOST_LAYER
			rc.collide_with_areas = false
			rc.collide_with_bodies = true
			rc.hit_back_faces = true
			rc.hit_from_inside = false
			_rc_holder.add_child(rc)
			_rc.append(rc)
		_rc_ex = []
	var n := RAY_OFFSETS.size()
	_ray_x.resize(n)
	_ray_lift.resize(n)
	_ray_w.resize(n)
	for k in n:
		var x: float = RAY_OFFSETS[k] * radius
		_ray_x[k] = x
		_ray_lift[k] = radius - sqrt(maxf(radius * radius - x * x, 0.0))
		_ray_w[k] = 1.0 - absf(RAY_OFFSETS[k]) * 0.6
		var rc: RayCast3D = _rc[k]
		rc.position = Vector3(0.0, 0.0, -float(_ray_x[k]))
		rc.target_position = Vector3(0.0, -_rc_reach, 0.0)


## Surface name -> its tire class (tyres.gd surface_class), worked out once per name.
static var _classes := {}
const LOOSE_CLASSES := ["gravel", "dirt", "mud", "sand", "snow"]
# prepare_surface's lookups, kept until the surface / loose material / tire type changes.
var _ps_surf := "\n"
var _ps_tyre: Dictionary = {}
var _ps_sd: Dictionary = {}
var _ps_tk := ""
var _ps_tgrip := 1.0
var _ps_loose := false
var _ps_twet := 1.0
var _ps_stiff := 1.0
var _ps_water := 6.0
var _ps_mud := 1.0
var _ps_lmat := "\n"
var _ps_ld: Dictionary = {}
var _ps_lbite := 1.0
var _ps_drag := 0.35


static func _class_of(s: String) -> String:
	var c: Variant = _classes.get(s)
	if c == null:
		c = TYRES.surface_class(s)
		_classes[s] = c
	return c


## The studs' share of a tire's hold on a surface at the reference -5 C (see studs_on): on ice by how
## many / how long they are, on snow by how far they reach into it as well. 0 for anything else.
static func stud_share_of(t: Dictionary, surf: String) -> float:
	var st := float(t.get("studs", 0.0))
	if st <= 0.0 or not studs_on:
		return 0.0
	var sh := st / (st + STUD_HALF)
	if surf == "ice":
		return sh
	if surf == "snow" or surf == "snow_packed":
		return sh * clampf(0.2 + 0.3 * (st - 1.0), 0.2, 0.8)
	return 0.0


## What the air temperature does to rubber on ice and on snow, and to studs in ice (x the -5 C figure).
static func ice_rubber_temp(t_air: float) -> float:
	# Slipperiest right at 0 C, steeply better to -10 C, then still gaining slowly: cold dry ice is
	# sticky (measured rubber-on-ice friction about triples from 0 to -25 C).
	return clampf(0.72 + 0.056 * -t_air, 0.6, 1.25) + 0.015 * clampf(-t_air - 9.5, 0.0, 20.0)


static func snow_temp(t_air: float) -> float:
	# Wet, greasy snow at 0 C; best around -8 to -15 C.
	return clampf(0.86 + 0.028 * -t_air, 0.86, 1.0) + 0.011 * clampf(-t_air - 5.0, 0.0, 7.0)


static func stud_temp(t_air: float) -> float:
	return clampf(1.25 + 0.05 * t_air, 1.0, 1.25) if t_air > -5.0 else clampf(1.0 + 0.0267 * (t_air + 5.0), 0.4, 1.0)


## Called when a stage is unloaded: the ids of its colliders are no longer meaningful.
static func forget_surfaces() -> void:
	_bed_curves.clear()
	_surfaces.clear()
	_roads.clear()
	_roughs.clear()

func _init() -> void:
	pass


## Floating origin: the world (and the car) moved by `d`.
func shift_origin(d: Vector3) -> void:
	contact_point += d
	_feat_at += d


func reset_state() -> void:
	omega = 0.0
	kappa = 0.0
	slip_lat = 0.0
	compression = 0.0
	prev_compression = 0.0
	hub_v = 0.0
	_stop_debt = 0.0
	_fresh = true
	fx = 0.0
	_road_i = -1
	_feat_at = Vector3(1e9, 1e9, 1e9)
	fy = 0.0


## Raycast the suspension. Returns false when the wheel is in the air.
func cast(space: PhysicsDirectSpaceState3D, exclude: Array[RID], car_up: Vector3, dt: float) -> bool:
	var from := global_position
	# Several rays along the tire's footprint, not one: the tire is a circle, so a kerb, rock, rut
	# edge or mesh seam ahead or behind lifts it smoothly instead of the hub snapping up or down
	# when a single ray steps across. Each ray's hit is turned into "where the hub would sit if the
	# circle touched that point"; the highest of those wins (that's the point the tire rests on).
	# The rays are RayCast3D nodes on a little holder that is turned to the tire's heading each tick
	# and force-updated: the same rays as a space query, cast in the engine, without building a
	# result dictionary per ray (about a quarter cheaper; this runs 7 x 4 wheels x 120 Hz a car).
	var best_eff := INF
	var best_k := -1
	var best_n := Vector3.UP
	var best_col: Object = null
	var nrm_sum := Vector3.ZERO
	var fwd_flat := (fwd_dir - car_up * fwd_dir.dot(car_up)).normalized()
	if _lift_r != radius or _rc_reach != max_length + radius or _rc.is_empty():
		_ray_cache()
	if exclude != _rc_ex:
		_rc_ex = exclude.duplicate()
		for rc in _rc:
			(rc as RayCast3D).clear_exceptions()
			for rid in exclude:
				(rc as RayCast3D).add_exception_rid(rid)
	_rc_holder.global_transform = Transform3D(Basis(car_up.cross(-fwd_flat), car_up, -fwd_flat), from)
	# The ground's small-scale shape (ground_shape.gd) and any washboard / cattle grid under the tire
	# are read at each ray's own point: the circle settles on whatever is highest across its
	# footprint. (Road features are the ones found for this patch of road, see below.)
	var bumps: Array = _feat_bumps if _feat_at.x < 1e8 else ROAD_DETAIL.NO_BUMPS
	var tan_flat := Vector3.ZERO
	if not bumps.is_empty() and TrackGenerator.current:
		var tg: Vector3 = TrackGenerator.current.flat_tangents[clampi(_road_i, 0, TrackGenerator.current.flat_tangents.size() - 1)]
		tan_flat = tg
	for k in rays:
		var rc: RayCast3D = _rc[k]
		rc.force_raycast_update()
		if not rc.is_colliding():
			continue
		var o := from + fwd_flat * _ray_x[k]
		var hp := rc.get_collision_point()
		var eff: float = o.distance_to(hp) + _ray_lift[k]
		var hc := rc.get_collider()
		var gh := GS.fine(hp.x, hp.z, _rough_of(hc))
		for f in bumps:
			gh += ROAD_DETAIL.bump_at(f, _feat_s + (hp - _feat_at).dot(tan_flat))
		eff -= gh
		var hn := rc.get_collision_normal()
		nrm_sum += hn * _ray_w[k]
		if eff < best_eff:
			best_eff = eff
			best_k = k
			best_n = hn
			best_col = hc
	prev_compression = compression
	if best_k < 0:
		contact = false
		air_t += dt
		load = 0.0
		_c_ground = -1.0
		_c_ground_prev = -1.0
		return false
	contact = true
	# Coming down off a jump (or a crest) after a real bit of air: the spike that follows is a
	# landing, not an edge (see the strike test in suspension_force).
	since_land = 0.0 if air_t > 0.12 else since_land + dt
	air_t = 0.0
	contact_normal = nrm_sum.normalized() if nrm_sum.length_squared() > 1e-6 else best_n
	# Contact point straight below the hub at the resting height (keeps forces and ground queries
	# centred on the tire even when an off-centre ray decided the height).
	contact_point = from - car_up * best_eff
	var col: Object = best_col
	# A collider's surface never changes, so it's looked up once per body instead of every tick.
	surface = _surface_of(col)
	var on_road: bool = _road_of(col)
	# Local road features: ice / grass patches swap the surface, puddles / snakes / paint cut grip,
	# washboard / cattle grids add bumps.
	feature_grip = 1.0
	feature_water = 0.0
	feature_water_size = 0.0
	rut_push = Vector3.ZERO
	if on_road and TrackGenerator.current:
		# Features are metres across, so the answer is only worked out again once the tire has
		# actually moved along the road. At 120 Hz a wheel covers a few centimetres a tick, and
		# asking four times a car a tick was the most expensive thing in the physics. The shortest
		# feature there is - washboard, 0.55 m a wave - still gets eight samples a wave.
		if _feat_at.distance_squared_to(contact_point) > FEATURE_STEP_SQ:
			var d = TrackGenerator.current.detail
			d.query(contact_point, _road_i)
			_road_i = d.q_index
			_feat_at = contact_point
			_feat_surface = d.q_surface
			_feat_grip = d.q_grip
			_feat_bump = d.q_bump
			_feat_water = d.q_water
			_feat_water_size = d.q_water_size
			_feat_bumps = d.q_bumps
			_feat_s = d.q_s
			_feat_rut = Vector3.ZERO
			if d.q_rut_slope != 0.0:
				var rr: Vector3 = TrackGenerator.current.rights[d.q_index]
				_feat_rut = Vector3(rr.x, 0.0, rr.z).normalized() * (-d.q_rut_slope * RUT_WALL)
		if _feat_surface != "":
			surface = _feat_surface
		feature_grip = _feat_grip
		feature_water = _feat_water
		feature_water_size = _feat_water_size
		rut_push = _feat_rut
	else:
		_feat_at = Vector3(1e9, 1e9, 1e9)
		_feat_bumps = ROAD_DETAIL.NO_BUMPS
	# The live water (water_sim.gd): whatever has actually collected or is running here - on the
	# road, in a ditch, through a ford. A film over the tread costs grip; deeper, the aquaplaning,
	# drag and splash below take over (prepare_surface), and the current pushes (car.gd).
	water_vel = Vector3.ZERO
	var ws: Node = WATER.current
	if ws != null and sim_water:
		ws.query(contact_point, self)
		var qd: float = ws.q_depth
		if qd > 0.0005:
			if qd > feature_water:
				feature_water = qd
				feature_water_size = maxf(feature_water_size, 3.0)
			feature_grip *= lerpf(1.0, 0.78, clampf(qd / 0.008, 0.0, 1.0))
			water_vel = ws.q_vel
	var dist := best_eff
	var sd := Surfaces.get_def(surface)
	var ld_def: Dictionary
	var nominal: float = sd.get("loose_depth", 0.0)
	loose_mat = surface
	loose_depth = nominal
	if GroundMap.current and not showroom:
		loose_depth = GroundMap.current.sample_patch(contact_point, fwd_dir, patch_len, surface)
		loose_mat = GroundMap.current.sampled_mat
	# The road mesh sits at the nominal top of the loose layer; ruts lower the wheel, berms lift it.
	ld_def = sd if loose_mat == surface else Surfaces.get_def(loose_mat)
	var ld_sink: float = ld_def.get("sink", 0.0)
	var rut: float = (maxf(loose_depth, 0.0) - nominal) * (1.0 - ld_sink)
	# Below zero the tire has dug a hole into the base itself (GroundMap dig): it drops right into it.
	if loose_depth < 0.0:
		rut += loose_depth
	# Sinking into the soft layer under load (mud, snow, sand): the tire sits down in it.
	if loose_depth > 0.0 and ld_sink > 0.0:
		rut -= minf(loose_depth, 0.2) * ld_sink * clampf(load / maxf(ref_load, 1.0), 0.4, 1.6) * 0.6
	# (The ground's shape - heave, small bumps, washboard, cattle grids - is already in `dist`: the
	# heave is in the road mesh itself, the rest was read ray by ray above.)
	var bump := 0.0
	# A flat spot: once a revolution the flat comes round to the road and the hub drops by its depth
	# for a moment - the thump of a tire that locked up. Caught on the tick the flat passes the bottom.
	if flat_mm > 0.05:
		var a0 := wrapf(_flat_prev - _flat_at, -PI, PI)
		var a1 := wrapf(spin_angle - _flat_at, -PI, PI)
		if signf(a0) != signf(a1) and absf(a0 - a1) < PI:
			bump -= flat_mm * 0.001
		_flat_prev = spin_angle
	var c_new := max_length - (dist - radius - bump - rut) + flex_off
	if _fresh or not contact_prev_ok():
		_c_ground_prev = c_new
	else:
		_c_ground_prev = _c_ground
	_c_ground = c_new
	if _fresh:
		compression = clampf(c_new, 0.0, max_length)
		hub_v = 0.0
		_stop_debt = 0.0
		_fresh = false
	return true


func contact_prev_ok() -> bool:
	return _c_ground > -0.5


## How much each loose material turns to slurry when it's soaked (0 = it doesn't: sand firms up,
## mud and snow are what they are already).
const SLURRY := {"dirt": 1.0, "gravel_hard": 0.85, "gravel": 0.7, "gravel_packed": 0.6, "grass": 0.6,
	"tarmac_broken": 0.3}
const SLURRY_COH := 1400.0       # Pa of cohesion full slurry adds
var _slurry := 0.0               # 0..1 under this tire right now


## Per-tick tire parameters for the surface under the wheel: base layer, loose layer coverage,
## contact patch length, cohesion, bulldozing coefficient and rolling resistance.
func prepare_surface(wetness: float) -> void:
	# What only depends on the surface names and the tire type is looked up when one of them changes,
	# not four times a car every tick (same numbers either way).
	if surface != _ps_surf or not is_same(tyre, _ps_tyre):
		_ps_surf = surface
		_ps_tyre = tyre
		_ps_sd = Surfaces.get_def(surface)
		_ps_tk = _class_of(surface)
		_ps_tgrip = tyre.get(_ps_tk, 1.0)
		_ps_loose = _ps_sd.has("loose_depth") or _ps_tk in LOOSE_CLASSES
		_ps_twet = float(tyre.get("wet", 1.0))
		_ps_stiff = float(tyre.get("stiff", 1.0))
		stud_share = stud_share_of(tyre, surface)
		var tc := TIRE_CURVE.shape(lerpf(float(_ps_sd.get("slide", 0.8)), STUD_SLIDE, stud_share), _ps_stiff)
		_tc_c = tc.c
		_tc_b = tc.b
		_tc_e = tc.e
		_ps_water = float(tyre.get("water", 6.0))
		_ps_mud = float(tyre.get("mud", 1.0))
		_ps_lat = float(tyre.get("lat", 1.0))     # tire balance sheet: lateral / longitudinal grip (1.0 = as ever)
		_ps_long = float(tyre.get("long", 1.0))
		_ps_lmat = "\n"   # (the loose layer's values below depend on the tire too)
	if loose_mat != _ps_lmat:
		_ps_lmat = loose_mat
		_ps_ld = Surfaces.get_def(loose_mat)
		# (Studs do little in a loose layer: there, the tire is mostly its tread and rubber.)
		_ps_lbite = float(tyre.get(_class_of(loose_mat), 1.0)) * (1.0 - 0.85 * stud_share_of(tyre, loose_mat))
		_ps_drag = float(_ps_ld.get("drag", 0.35))
	var sd := _ps_sd
	var wet := clampf(wetness, 0.0, 1.0)
	# The dry line: where tires have run, the water's been squeezed out of the ruts (GroundMap).
	if wet > 0.05 and contact and GroundMap.current and not showroom:
		wet *= 1.0 - 0.85 * GroundMap.current.sample_dry(contact_point)
	var tk := _ps_tk
	var t_grip: float = _ps_tgrip
	var t_wet := lerpf(1.0, _ps_twet, wet)
	_mu = sd.grip * lerpf(1.0, sd.wet_factor, wet) * feature_grip * t_grip * t_wet * tyre_temp_grip()
	_mu *= _p_mu_loose if _ps_loose else _p_mu_hard
	# The air temperature (weather.gd): ice is at its slipperiest right at 0 C, with a film of melt on
	# it, and bites better the colder it gets (the figures are for about -5 C); packed snow a little
	# the same. Wet tarmac at freezing goes to black ice.
	_stud_mu = 0.0
	if surface == "ice":
		_mu *= ice_rubber_temp(air_temp) if studs_on else clampf(0.72 + 0.056 * -air_temp, 0.6, 1.25)
	elif surface == "snow" or surface == "snow_packed":
		_mu *= snow_temp(air_temp) if studs_on else clampf(0.94 + 0.012 * -air_temp, 0.9, 1.06)
	elif tk == "tarmac" and air_temp < 1.0 and wet > 0.2:
		var frost := clampf((1.0 - air_temp) / 4.0, 0.0, 1.0) * clampf((wet - 0.2) / 0.4, 0.0, 1.0)
		_mu *= 1.0 - 0.62 * frost
	_peak_slip = sd.peak_slip
	_peak_angle = sd.peak_angle / (_ps_stiff * _p_corner)
	if stud_share > 0.0:
		# The studs' share (see studs_on): out of the rubber's figure, back in on the pins' own terms.
		# No wetness, tread temperature or pressure in it; the peak moves out with it.
		_stud_mu = float(sd.grip) * t_grip * stud_share * stud_temp(air_temp) * (_p_mu_loose if _ps_loose else _p_mu_hard)
		_mu = _mu * (1.0 - stud_share) + _stud_mu
		_peak_slip *= 1.0 + stud_share
		_peak_angle *= 1.0 + stud_share
	# Aquaplaning in standing water: the tread can only pump so much water out of the way. The speed
	# it starts at depends on the tire's water evacuation, its tread depth, its width (wide tires
	# plane earlier) and the depth of the water.
	aquaplane = 0.0
	if feature_water > 0.003:
		var tread := clampf(0.6 + tread_mm / 20.0, 0.65, 1.2)
		var v_ap := (14.0 + 2.2 * _ps_water) * tread * pow(0.195 / maxf(width, 0.12), 0.3) * _p_aqua \
			* clampf(pow(0.01 / feature_water, 0.25), 0.7, 1.3)
		aquaplane = smoothstep(v_ap * 0.7, v_ap * 1.1, absf(v_long)) * clampf(feature_water / 0.008, 0.0, 1.0)
		_mu *= 1.0 - 0.8 * aquaplane
	# Worn tread has no blocks left to dig into loose ground with: a bald tire on gravel just
	# polishes it. (On tarmac the opposite is true, which is why slicks exist, so this only
	# touches the loose-layer grip.)
	_tread_bite = clampf(0.78 + tread_mm / 26.0, 0.78, 1.08)
	var ld := _ps_ld
	cover = 0.0
	bed_seat = 0.0
	tire_give = 0.0
	_coh = 0.0
	_bull = 0.0
	_berm = 0.0
	var sink := 0.0
	var plane := 1.0
	var pg := 1.0
	if loose_depth > 0.0005 and ld.has("loose_depth"):
		# Ground pressure: a narrow or hard tire presses harder and cuts down through the loose
		# stuff to the firm base under it, a wide soft one floats on top (why snow and deep-gravel
		# tires are narrow, and why a fat summer tire skates on marbles). Measured against a 195
		# at its design load and pressure (~90 kPa over the patch).
		pg = clampf(load / maxf(patch_len * width, 0.01) / 90000.0, 0.5, 2.0)
		cover = 1.0 - exp(-loose_depth / (float(ld.cover) * pg))
		# Dynamic sinkage: granular stuff has no time to give way under a fast tire, so it rides
		# higher in the layer the faster it rolls (measured in sand and gravel, Wong ch. 2). It
		# sinks less, compacts less and has less depth in front of it to plough: the grip stays
		# about level with speed instead of climbing with the plough's v^2. (loose_plane_v: bench.)
		plane = 1.0 / (1.0 + absf(v_long) / loose_plane_v)
		sink = minf(loose_depth, 0.2) * float(ld.sink) * plane
		_mu_l = ld.loose_mu * lerpf(1.0, ld.wet_factor, wet) * _ps_lbite * t_wet * _p_mu_loose * _tread_bite
		if grain_noise_on and not showroom and ld.has("grain"):
			var gr: float = ld.grain
			var sig := minf(sqrt(GRAIN_CHAIN * gr * gr / maxf(patch_len * width, 0.005)), 0.2)
			_mu_l *= 1.0 + sig * 2.3 * _grain_at(contact_point.x / GRAIN_CELL, contact_point.z / GRAIN_CELL)
		_shear_k = ld.shear_k
		# How much traction a fast-spinning tire loses on this material (mud tires clear their
		# tread, so they lose less).
		_spin_fade = float(ld.get("spin_fade", 0.3)) * lerpf(1.0, 0.55, clampf((_ps_mud - 1.0) / 0.5, 0.0, 1.0))
		# Wet dirt turns to slurry (physics list 9): the fines and the water make a paste on top. The
		# stones lose their bite, the paste holds together (cohesion: a little more grip at small
		# slip, a car can lean on it), and it packs a spinning tread solid sooner - a rainy dirt stage
		# drives like a different road. Where the cars have run the line dries first (`wet` above).
		_slurry = smoothstep(0.25, 0.85, wet) * float(SLURRY.get(loose_mat, 0.0))
		if _slurry > 0.0:
			_mu_l *= 1.0 - 0.12 * _slurry
			_spin_fade = minf(_spin_fade + 0.25 * _slurry * lerpf(1.0, 0.55, clampf((_ps_mud - 1.0) / 0.5, 0.0, 1.0)), 0.85)
	else:
		_slurry = 0.0
	# Patch length from tire deflection (radial stiffness, softer when the pressure's down) plus
	# sinkage: L = 2 sqrt(2 r d).
	patch_len = clampf(2.0 * sqrt(2.0 * radius * (load / tyre_k + sink)), 0.08, 0.6)
	if cover > 0.0:
		_coh = (float(ld.cohesion) + SLURRY_COH * _slurry) * patch_len * width
		if bed_on and not no_bed and ld.has("bed_mu"):
			# The seated bed (see bed_i0). A long patch and a hard-pressing tire seat it better, so
			# letting air out helps at speed and a narrow tire keeps its bite longer; slurry greases
			# the stones' contacts, so a soaked road has less of it. It yields at about half a stone
			# of shear.
			var grain: float = ld.grain
			# (x 6: a brush's patch is all sliding once the shear at its back edge is 6x the give
			# that carries the peak stress, tread and stones together.)
			_bed_k = 6.0 * 0.5 * grain
			var p_c := maxf(load / maxf(patch_len * width, 0.01), 30000.0)
			var inert := grain * sqrt(GRAIN_RHO / p_c) * sqrt(v_long * v_long + v_lat * v_lat) / patch_len
			bed_seat = clampf(1.0 - inert / bed_i0, 0.0, 1.0)
			_bed = 1.0 + (float(ld.bed_mu) - 1.0) * (1.0 - 0.6 * _slurry)
			# And the other way: stones that never seated are a dilated, rolling layer, which takes
			# more shear to carry the same force (a loose packing is softer than a dense one; Wong's
			# K runs larger for loose soils). The surface's shear_k is the half-seated middle.
			_shear_k *= 1.0 + bed_soft * (0.5 - bed_seat)
		if series_on:
			var cc := float(tyre.get("tarmac", 1.0)) * _tc_b * _tc_c / tan(SERIES_PEAK / (_ps_stiff * _p_corner))
			tire_give = (_mu_l * LOOSE_LIN + _coh / maxf(load, 0.2 * ref_load)) * patch_len / (2.0 * cc)
			if bed_seat > 0.0:
				_bed_k += 6.0 * tire_give * _bed
		# The berm against the tire's side: it needs material to pile (cover), blocks to cut with
		# (tread depth and the tire's rating on this stuff) and a long patch to pile it along, which
		# is why a soft tire at low pressure bites on gravel and a hard one at 40 psi skitters.
		var bite: float = _ps_lbite
		_berm = cover * clampf(bite, 0.5, 1.6) * _tread_bite * clampf(patch_len / 0.22, 0.6, 1.6)
		# Material ahead of the sliding tread (a berm or wedge it has built up) is what it bulldozes.
		var ahead := loose_depth
		var vs := fwd_dir * (v_long - omega * radius) + side_dir * v_lat
		if GroundMap.current and not showroom and vs.length_squared() > 0.25:
			ahead = maxf(ahead, GroundMap.current.sample(contact_point + vs.normalized() * (patch_len * 0.5 + 0.12), surface))
		# Momentum flux of the material shoved aside: F = C rho b d v^2 (C ~ 0.5).
		# (x ground pressure: a narrow or heavily loaded tire sinks deeper into what it's shoving,
		# so a narrow one pushes as much stone as a wide one and a loaded one bites harder.)
		_bull = 0.5 * float(ld.density) * width * minf(ahead, 0.25) * plane * pg
	# The ABS's slip target: a road car's ABS (everything up to 2014) can't tell it's on gravel. It
	# lets a little more slip on loose stuff, because the wheels slow differently, but it never
	# lets them dig the wedge a locked tire builds, which is why NHTSA's cars all stopped ~27 %
	# longer on loose gravel with it on than locked (gravel_bench refs). abs_loose = how far it goes.
	abs_slip = lerpf(_peak_slip, 0.25, cover * abs_loose)
	# Compaction resistance from sinking into the layer, growing with speed.
	# (`drag`: how hard the material is to squash a rut into. Sand and mud flow out sideways and
	# rebuild in front of the tire instead of packing down once: Wong / Taborek have a car at 0.1-0.3
	# of its weight in loose sand and soft ground, against 0.02 on rolled gravel.)
	var crr: float = sd.rolling * _p_roll + _ps_drag * sink / patch_len * (1.0 + absf(v_long) / 40.0)
	rolling_torque = crr * load * radius
	# Ploughing through water: F ~ 1/2 rho Cd (width x depth) v^2, as a drag on the wheel.
	if feature_water > 0.003:
		rolling_torque += 0.5 * 1000.0 * 1.1 * width * minf(feature_water, 0.3) * v_long * v_long * radius


## One tick of the corner's vertical dynamics, sub-stepped: tire spring against the road, the hub's
## mass, spring / damper / bump stop / anti-roll bar into the body. Sets `load` (tire on the road,
## for grip) and `body_force` / returns the force into the body.
func suspension_force(dt: float) -> float:
	var n := susp_substeps
	# The loop itself: the C++ core when it's there (native/src/core.cpp, the same lines), else the script.
	if not _susp_native(dt, n):
		_susp_script(dt, n)
		if _scheck:
			_scheck = false
			NATIVE.same("susp compression", _sbuf[NL.S_COMPRESSION], compression)
			NATIVE.same("susp hub_v", _sbuf[NL.S_HUB_V], hub_v)
			NATIVE.same("susp stop_debt", _sbuf[NL.S_STOP_DEBT], _stop_debt)
			NATIVE.same("susp ft_sum", _sbuf[NL.S_FT_SUM], _ft_sum)
			NATIVE.same("susp fs_sum", _sbuf[NL.S_FS_SUM], _fs_sum)
			NATIVE.same("susp stop_start", _sbuf[NL.S_STOP_START], stop_start)
	var ss := stop_start
	load = _ft_sum / n
	susp_length = max_length - compression
	body_force = maxf(_fs_sum / n, 0.0)
	droop_pull = maxf(-_fs_sum / n, 0.0)
	# A hit hard enough to put the suspension on its stop, at speed, is the kind that cuts a tire,
	# bends a rim or knocks the corner out of line: the far edge of a pothole, a kerb, a rock in
	# the road. The load spike relative to what this corner normally carries says how hard.
	# A jump landing is different: the whole tire comes down on a flat road, spread over the patch
	# and the stroke, so it takes a spike of 3-5x every time without a scratch (rally cars do it all
	# stage). Within LAND_WINDOW of touching down it only counts from LAND_SPIKE (a flat landing off
	# a big jump, onto the stops), and it never cuts the tire.
	if contact and absf(v_long) > 8.0 and compression > ss:
		var spike := load / maxf(nominal_load, 1.0)
		var landing := since_land < LAND_WINDOW
		var base := LAND_SPIKE if landing else 3.0
		if spike > base:
			var sev := (spike - base) * 0.55 * clampf(absf(v_long) / 18.0, 0.4, 1.6)
			if sev > _hit_sev:
				_hit_sev = sev
				_hit_landing = landing
	return body_force


var _ft_sum := 0.0               # the tick's summed tire force / strut force over the substeps
var _fs_sum := 0.0
var _sbuf := PackedFloat64Array()
var _scheck := false


## suspension_force's loop in the C++ core. False = it isn't there (or refused): nothing was changed.
func _susp_native(dt: float, n: int, with: RefCounted = null) -> bool:
	var nat: RefCounted = with if with != null else NATIVE.core()
	if nat == null:
		return false
	var b := _sbuf
	if b.size() != NL.S_SIZE:
		b.resize(NL.S_SIZE)
	b[NL.S_N] = n
	b[NL.S_DT] = dt
	b[NL.S_STATIC_LOAD] = static_load
	b[NL.S_TYRE_K] = tyre_k
	b[NL.S_TYRE_C] = tyre_c
	b[NL.S_CONTACT] = 1.0 if contact else 0.0
	b[NL.S_C_GROUND] = _c_ground
	b[NL.S_C_GROUND_PREV] = _c_ground_prev
	b[NL.S_MAX_LENGTH] = max_length
	b[NL.S_ARM] = arm
	b[NL.S_PACKER] = packer
	b[NL.S_SPRING_K] = spring_k
	b[NL.S_COMPRESSION] = compression
	b[NL.S_HUB_V] = hub_v
	b[NL.S_BUMP_DAMP] = bump_damp
	b[NL.S_REBOUND_DAMP] = rebound_damp
	b[NL.S_DAMP_KNEE] = damp_knee
	b[NL.S_DAMP_SLOW] = damp_slow
	b[NL.S_BUMP_FAST] = bump_fast
	b[NL.S_REBOUND_FAST] = rebound_fast
	b[NL.S_DAMP_LINEAR] = 1.0 if damp_linear else 0.0
	b[NL.S_ARB_FORCE] = arb_force
	b[NL.S_BUMPSTOP_K] = bumpstop_k
	b[NL.S_UNSPRUNG_MASS] = unsprung_mass
	b[NL.S_GEO_JACK] = geo_jack
	b[NL.S_PROGRESSIVE_STOPS] = 1.0 if progressive_stops else 0.0
	b[NL.S_PRELOAD_N] = preload_n
	b[NL.S_STOP_DEBT] = _stop_debt
	var r: PackedFloat64Array = nat.call(&"suspension", b)
	if r.size() != NL.S_SIZE:
		return false
	_sbuf = r
	if NATIVE.check and with == null:
		_scheck = true   # (the scripts do this tick; suspension_force compares the two after)
		return false
	compression = r[NL.S_COMPRESSION]
	hub_v = r[NL.S_HUB_V]
	_stop_debt = r[NL.S_STOP_DEBT]
	_ft_sum = r[NL.S_FT_SUM]
	_fs_sum = r[NL.S_FS_SUM]
	stop_start = r[NL.S_STOP_START]
	return true


## ... and in script (the fallback, and what the core is checked against: native_parity_test).
func _susp_script(dt: float, n: int) -> void:
	var h := dt / n
	var pre := static_load / tyre_k
	var gv := (_c_ground - _c_ground_prev) / dt if contact else 0.0
	stop_start = maxf(max_length * (0.88 if arm < 1.0 else 0.45) - packer, max_length * 0.35)
	var ss := stop_start
	var k_spring := spring_k * (1.0 if arm < 1.0 else 0.35)   # broken: the corner sags onto its stop
	var ft_sum := 0.0
	var fs_sum := 0.0
	# (The state and settings in locals for the loop: same arithmetic, less script overhead.)
	var c := compression
	var v := hub_v
	var on := contact
	var cg0 := _c_ground_prev
	var cg1 := _c_ground
	var tk := tyre_k
	var tc := tyre_c
	var bd := bump_damp
	var rd := rebound_damp
	# Two-stage dampers (see damp_knee): slow coefficient, the force at the knee, fast coefficient.
	var kn := damp_knee
	var bds := bd
	var rds := rd
	var bdf := bd
	var rdf := rd
	if not damp_linear:
		bds = bd * damp_slow
		rds = rd * damp_slow
		bdf = bd * bump_fast
		rdf = rd * rebound_fast
	var bkn := bds * kn
	var rkn := rds * kn
	var arb := arb_force
	var bsk := bumpstop_k
	var mu := unsprung_mass
	var ml := max_length
	var gj := geo_jack if on else 0.0
	var prog := progressive_stops
	var pn := preload_n
	var bl := maxf(ml - ss, 0.01)
	var inv_bl2 := 1.0 / (bl * bl)
	var bsc := bd * STOP_C
	var debt := _stop_debt
	var cap := STOP_CAP * static_load * h
	for i in n:
		var ft := 0.0
		if on:
			var cg := lerpf(cg0, cg1, float(i + 1) / n)
			var defl := cg - c + pre
			if defl > 0.0:
				ft = maxf(tk * defl + tc * (gv - v), 0.0)
		var fd: float
		if v > 0.0:
			fd = bds * v if v < kn else bkn + bdf * (v - kn)
		else:
			fd = rds * v if v > -kn else -rkn + rdf * (v + kn)
		var fs := 0.0
		if prog:
			# (A coil spring only pushes: past full droop it's sat on its seat, or held by the preload.)
			fs = maxf(k_spring * c + pn, 0.0) + fd + arb + gj
			if c > ss:
				var x := c - ss
				fs += bsk * x * (0.6 + 1.4 * x * x * inv_bl2) + bsc * x / bl * v * (1.0 if v > 0.0 else STOP_REB)
			elif c < 0.0:
				var x := -c
				var xr := x / TOPOUT_LEN
				fs -= TOPOUT_K * x * (1.0 + 3.0 * xr * xr) - bsc * xr * v * (1.0 if v < 0.0 else STOP_REB)
		else:
			fs = k_spring * c + fd + arb + gj
			if c > ss:
				fs += bsk * (c - ss)
		v += (ft - fs) / mu * h
		c += v * h
		if prog:
			# Fully squashed either way: whatever speed the hub still had is stopped by the body,
			# so it goes into the body, not nowhere. Paid out over a few substeps (the last of the
			# rubber, the mounts), at most STOP_CAP x the corner's weight at a time: all of it
			# arrives, just not as a one-substep spike.
			if c < -TOPOUT_LEN:
				c = -TOPOUT_LEN
				var vs := maxf(v, 0.0)
				debt += mu * (v - vs)
				v = vs
			elif c > ml:
				c = ml
				var vs := minf(v, 0.0)
				debt += mu * (v - vs)
				v = vs
			if debt != 0.0:
				var pay := clampf(debt, -cap, cap)
				fs += pay / h
				debt -= pay
		elif c < 0.0:
			c = 0.0
			v = maxf(v, 0.0)
		elif c > ml:
			c = ml
			v = minf(v, 0.0)
			fs = maxf(fs, ft)   # solid against the stop: the road pushes straight into the body
		ft_sum += ft
		fs_sum += fs
	compression = c
	hub_v = v
	_stop_debt = debt
	_ft_sum = ft_sum
	_fs_sum = fs_sum


# The parts of the tire substeps that only change once a tick (load, grip, surface, brakes, road
# speed), worked out by prep_substeps() instead of in each of the eight substeps. Same expressions
# in the same order, so the forces come out to the last digit as they did.
var _t_on := false               # in contact with load on it
var _t_fmax := 0.0               # friction circle radius on the hard base (N)
var _t_sxd := 1.0                # kappa at the peak at this load
var _t_syd := 1.0                # tan(slip angle at the peak) at this load
var _t_fl := 0.0                 # loose-layer shear limit (N)
var _t_rk := 1.0                 # loose-layer stiffness load factor
var _t_load2 := 0.0              # 0.2 load: most the wedge ahead of the tread holds
var _i_cap := 0.0                # brakes + rolling + rub, as spin lost per substep (rad/s)
var _i_k := 0.0                  # substep length (s)
var _i_al := 0.0                 # substep / relax_long
var _i_dl := 1.0                 # kappa's implicit damping divisor
var _i_ny := 0.0                 # slip_lat's push this substep
var _i_dy := 1.0                 # slip_lat's implicit damping divisor
var _i_dk := 0.0                 # low-speed tread damping: kappa added per m/s of slip-state change (s/m)
var _i_dv := 1.0                 # the speed kappa's relaxation runs at (m/s)


const LAT_SETTLE_V := 2.0         # m/s: the slowest the sideways slip state settles at (prep_substeps)
const LOW_V := 4.0                # m/s: below this the tread's own damping shows (prep_substeps)
const LOW_ZETA := 0.6             # damping ratio of the wheel's spin on its tread at a standstill


## What the seated bed does to a surface's usable cornering grip against speed, for the planners
## (speed_profile.bed_k: the AI's corner speeds, the co-driver's calls): at each of BED_VS (m/s) the
## best side force inside bed_win deg of slip, with the bed over without it, on an ordinary road tire
## at an ordinary load. Worked out from the tire model itself the first time a surface is asked
## for, so it follows the surface numbers. Surfaces with no bed: all 1.
const BED_VS: Array[float] = [2.0, 5.0, 8.0, 12.0, 16.0, 20.0, 25.0, 30.0, 38.0, 48.0]
static var _bed_curves := {}
static var bed_win := 20           # deg of slip the planners' curve looks inside (12 before the tire and the stones were in series: the same grip now sits further out)


static func bed_curve(surf: String) -> PackedFloat32Array:
	if _bed_curves.has(surf):
		return _bed_curves[surf]
	var out := PackedFloat32Array()
	out.resize(BED_VS.size())
	out.fill(1.0)
	if bed_on and Surfaces.get_def(surf).has("bed_mu"):
		var w := Wheel.new()
		w.tyre = TYRES.TYRES["road"]
		w.radius = 0.31
		w.width = 0.195
		w.ref_load = 3500.0
		w.load = 3500.0
		w.nominal_load = 3500.0
		w.static_load = 3500.0
		w.contact = true
		w.surface = surf
		w.loose_mat = surf
		w.loose_depth = Surfaces.loose_depth(surf)
		w.showroom = true
		w.tread_t = PackedFloat32Array([60.0, 60.0, 60.0])
		w.tread_mm = float(w.tyre.get("tread_mm", 8.0))
		for vi in BED_VS.size():
			var best := [0.0, 0.0]
			for pass_i in 2:
				w.no_bed = pass_i == 1
				for d in range(2, bed_win + 1):
					var a := deg_to_rad(float(d))
					w.v_long = BED_VS[vi] * cos(a)
					w.v_lat = BED_VS[vi] * sin(a)
					w.omega = w.v_long / w.radius
					w.kappa = 0.0
					w.slip_lat = tan(a)
					w.prepare_surface(0.0)
					w.prep_substeps(1.0 / 960.0, 0.0)
					w.compute_tire_force()
					best[pass_i] = maxf(best[pass_i], absf(w.fy))
			out[vi] = best[0] / maxf(best[1], 1.0)
		w.free()
	_bed_curves[surf] = out
	return out


## The tire's pneumatic trail on a loose layer (m), from the same picture of the patch the force comes
## from. `tb`: the trail of the share of the patch on the hard base (steering_rack.gd `_trail`), `t0`:
## the trail at no slip, `ds`: extra slip (for a slope). A side force acts behind the middle of the
## patch only while the front of the patch is still building stress and the back has let go; how far
## behind follows how much of the patch is used up, the brush model's
##   t / t0 = (1 - psi)^3 / (1 - psi + psi^2 / 3),  psi = the sliding share of the patch
##   * seated stones (the stick / slide brush): psi is its own
##   * flowing stones (Janosi): no stuck and sliding zones as such, so the psi of a brush carrying the
##     same share g of its limit, 1 - (1 - g)^(1/3). The layer never reaches a peak, so its trail
##     fades out over the whole slide instead of ending at one
## weighted by the force each share carries (the berm's push has none). (Before: the hard road's trail, halved by the cover, gone
## at the base's 8-10 deg peak: the rim went dead early in every gravel slide.)
## Smooth noise fixed to the ground, about 0.43 rms (bilinear between hashed corners, -1..1).
static func _grain_at(x: float, z: float) -> float:
	var ix := floori(x)
	var iz := floori(z)
	var fx := x - ix
	var fz := z - iz
	fx = fx * fx * (3.0 - 2.0 * fx)
	fz = fz * fz * (3.0 - 2.0 * fz)
	return lerpf(lerpf(_grain_h(ix, iz), _grain_h(ix + 1, iz), fx), lerpf(_grain_h(ix, iz + 1), _grain_h(ix + 1, iz + 1), fx), fz)


static func _grain_h(ix: int, iz: int) -> float:
	var n := (ix * 374761393 + iz * 668265263) & 0xFFFFFFFF
	n = ((n ^ (n >> 13)) * 1274126177) & 0xFFFFFFFF
	n = n ^ (n >> 16)
	return float(n & 0xFFFF) / 32767.5 - 1.0


## The flowing layer's share of its limit with the back of the patch sheared `dd` (m): compute_tire_force's own.
func _flow_g(dd: float) -> float:
	if tire_give <= 0.0:
		var r := dd / _shear_k
		return r * 0.5 if r < 1e-3 else 1.0 - (1.0 - exp(-r)) / r
	var u := dd / (_shear_k + tire_give)
	if u < 1e-3:
		return 0.5 * u
	for i in 3:
		u += (dd - _shear_k * u - tire_give * (1.0 - exp(-u))) / (_shear_k + tire_give * exp(-u))
	var eu := exp(-u)
	var sg := 1.0 - eu
	return sg - (_shear_k * (sg - u * eu) + 0.5 * tire_give * sg * sg) / dd


static var trail_patch := true     # (SBR_TRAILPATCH=0: the halved hard-road trail, as before)
func trail_mix(tb: float, t0: float, ds := 0.0) -> float:
	if cover <= 0.0:
		return tb
	if not trail_patch:
		return tb * (1.0 - 0.5 * cover)
	var sl := sqrt(kappa * kappa + slip_lat * slip_lat) + ds
	var g := _flow_g(patch_len * sl * _t_rk)
	var ps := 1.0 - pow(maxf(1.0 - g, 0.0), 1.0 / 3.0)
	var sh := (1.0 - g) / maxf(1.0 - ps + ps * ps / 3.0, 0.05)
	if bed_seat > 0.0:
		var rb := minf(patch_len * sl / _bed_k * _t_rk, 1.0)
		var q := 1.0 - rb
		var gb := 3.0 * _bed * rb * q * q + rb * rb * (3.0 - 2.0 * rb)
		var shb := q * q * q / maxf(q + rb * rb / 3.0, 0.05)
		var gm := lerpf(g, gb, bed_seat)
		sh = lerpf(g * sh, gb * shb, bed_seat) / maxf(gm, 1e-9)
		g = gm
	var wl := cover * _t_fl * g
	var wb := (1.0 - cover) * _t_fmax * TIRE_CURVE.eval((absf(slip_lat) + ds) / maxf(_t_syd, 1e-3), _tc_c, _tc_b, _tc_e)
	if wl + wb < 1e-6:
		return lerpf(tb, t0, cover)
	# (The berm pushes on the tire's side all along the patch: side force with no trail at all.)
	return (wb * tb + wl * t0 * sh) / (wb + wl + plough * load * berm_cap)


## The tire's peak slip angle at its load right now (rad), for the steering assist (Car._steer_assist).
func peak_angle_now() -> float:
	return atan(_t_syd) if _t_on else _peak_angle


## The tire's friction on this surface right now (before load sensitivity), for the same assist.
func mu_now() -> float:
	return _mu * grip_scale


## Once a tick, after the suspension, prepare_surface() and the brakes: everything the substeps
## (compute_tire_force / integrate, `h` long) would otherwise work out again each time.
func prep_substeps(h: float, dt := 0.0) -> void:
	# Real tires: grip and especially cornering stiffness grow more slowly than the load. A heavily
	# loaded tire needs a bigger slip angle, which is what makes front-heavy road cars (and stiff
	# front anti-roll bars) understeer gently instead of being twitchy and neutral.
	_t_on = contact and load > 0.0
	if _t_on:
		var lr := clampf(load / ref_load, 0.2, 3.0)
		var load_sens := clampf(1.0 - LOAD_SENS * (lr - 1.0), 0.65, 1.15)
		_t_fmax = _mu * grip_scale * load_sens * load
		if slide_mu_drop > 0.0:
			var vsx := v_long - omega * radius
			var ex := maxf(sqrt(vsx * vsx + slide_lat * v_lat * v_lat) - slide_v0, 0.0)
			# The loss is heat in the tread surface, so it builds over ~slide_heat s of sliding and
			# cools as fast: a flick or a swerve barely feels it, a held drift or a locked wheel does.
			_slide_ex = lerpf(_slide_ex, ex, 1.0 - exp(-dt / slide_heat)) if dt > 0.0 else ex
			_t_fmax *= 1.0 - slide_mu_drop * _slide_ex / (_slide_ex + SLIDE_V1)
		if _stud_mu > 0.0:
			# The studs' share on its own terms: no rubber load curve and no sliding loss, the pins'
			# force grows with the patch's length (sqrt of load) instead.
			var rub := _t_fmax * (1.0 - minf(_stud_mu / maxf(_mu, 1e-6), 1.0))
			_t_fmax = rub + _stud_mu * grip_scale * load * clampf(pow(lr, -0.5), 0.6, 1.5)
		_t_sxd = _peak_slip * pow(lr, 0.15)
		_t_syd = tan(_peak_angle * pow(lr, CORNER_SENS))
		# The loose layer is soil, not rubber: its shear strength is c + p tan phi, straight in the
		# load (Mohr-Coulomb; Wong ch. 2), so it gets none of the rubber's load sensitivity. Weight put
		# on a tire on gravel buys grip pound for pound, which is what left-foot braking and a lift on
		# turn-in live on; the cohesion is per area, so mud and snow still favour the light corner.
		# (LOOSE_LIN: the surfaces' numbers were set on whole-car stops with the rubber curve taking
		# ~2.5 % off across a braking car's four tires; this keeps those stops where they were.)
		_t_fl = (_mu_l * (load_sens if not loose_linear else LOOSE_LIN) * load + _coh) * grip_scale
		_t_rk = pow(lr, -CORNER_SENS)
		_t_load2 = 0.2 * load
	else:
		_slide_ex = 0.0
	_i_cap = (brake_torque + rolling_torque + rub_torque) * h / inertia
	_i_k = h
	# Implicit relaxation-length slip dynamics: stable at any speed, damped at standstill.
	var damp_v := maxf(absf(v_long), 1.0)
	_i_al = h / relax_long
	_i_dl = 1.0 + h * damp_v / relax_long
	var relax_y := relax_lat * clampf(absf(v_long) / 6.0, 0.25, 1.0)
	_i_ny = h / relax_y * v_lat
	# Sideways it settles from LAT_SETTLE_V, not 1 m/s: parked on a stiff-walled tire (semi-slicks,
	# summer performance) the short standstill relaxation length made the lateral force a stiff
	# spring that lagged the body, and the car rocked itself side to side on its tires, harder and
	# harder (the menu / loading screen car, parked with the brakes held). Only matters below 2 m/s.
	_i_dy = 1.0 + h * maxf(damp_v, LAT_SETTLE_V) / relax_y
	# Low-speed damping (Pacejka, "Tire and Vehicle Dynamics" 8.7; MF-Tyre's low-speed damping). A
	# wheel spinning on the tread's fore-aft spring is a mass on a spring, and the only damping the
	# relaxation-length model gives it is the rolling speed: at a standstill it rang at ~20 Hz with
	# almost none. A wheel off the clutch at a launch (or a player feathering it) spun forwards and
	# backwards against the road and the car rocked on the spot instead of pulling away. Real tread
	# rubber is lossy, so the force also follows how fast the slip is changing, enough to bring the
	# ringing to LOW_ZETA, faded out by LOW_V where the rolling already damps it. (The rate, not the
	# slip speed itself: that also added a steady push to every sliding tire, and a winter tire's
	# ice stop came up 10 % short.)
	_i_dv = damp_v
	_i_dk = 0.0
	var vf := absf(v_long) / LOW_V
	if _t_on and vf < 1.0:
		var ck := _t_fmax * MU_LONG * _tc_b * _tc_c / maxf(_t_sxd, 1e-3)
		var r2i := radius * radius / inertia
		var wn := sqrt(ck * r2i / relax_long)
		var z := LOW_ZETA - damp_v / relax_long / (2.0 * wn)
		if z > 0.0:
			_i_dk = 2.0 * z * wn / (ck * r2i) * 0.5 * (1.0 + cos(PI * vf))


## Combined-slip tire force from the current slip states: a blend of rubber on the hard base
## (Magic Formula curve, tire_curve.gd) and shear of the loose layer (Janosi-Hanamoto averaged
## over the patch, keeps building with slip), plus the bulldozing force of material the sliding
## tread pushes ahead of it. One substep's worth (the per-tick part is in prep_substeps), added
## into fx_sum / fy_sum.
func compute_tire_force() -> void:
	plough = 0.0
	if not _t_on:
		fx = 0.0
		fy = 0.0
		f_max = 0.0
		return
	var k := kappa + _i_dk * (omega * radius - v_long - _i_dv * kappa)
	var sla := slip_lat
	var fm := _t_fmax
	var fxv := 0.0
	var fyv := 0.0
	var sx := k / _t_sxd
	var sy := sla / _t_syd
	var s := sqrt(sx * sx + sy * sy)
	if s > 1e-6:
		var f := TIRE_CURVE.eval(s, _tc_c, _tc_b, _tc_e)
		# The friction circle in normalized slip: the force points the way the tire slides, and a
		# tire drives / brakes MU_LONG harder than it corners (the tread blocks are stiffest fore-aft).
		var mx := MU_LONG * (1.0 + (mu_brake - 1.0) * (1.0 - hb_share)) if k * v_long < 0.0 else MU_LONG
		fxv = fm * mx * f * sx / s * _al_long
		fyv = -fm * f * sy / s * _al_lat
	var cv := cover
	if cv <= 0.0:
		fx = fxv
		fy = fyv
		f_max = fm
		fx_sum += fxv
		fy_sum += fyv
		return
	# Granular shear: shear displacement grows linearly along the patch, j(x) = slip * x, and
	# tau = tau_max (1 - e^(-j/K)); averaged over the patch length L.
	var fl := _t_fl
	var sl := sqrt(k * k + sla * sla)
	var fxl := 0.0
	var fyl := 0.0
	if sl > 1e-6:
		var dd := patch_len * sl * _t_rk
		var r := dd / _shear_k
		var g := r * 0.5 if r < 1e-3 else 1.0 - (1.0 - exp(-r)) / r
		if tire_give > 0.0:
			# The tire's own give in series with the layer (series_on). A bit of tread that has been
			# dragged d along the ground shares it with the stones under it: d = K u + G (1 - e^-u),
			# u = the stones' shear / K, G = tire_give, stress = 1 - e^-u. No formula for u, so three
			# Newton steps from the soft end (it closes in from one side, the curve bends one way);
			# the patch average then has one, by parts:
			#   g = s - [K (s - u e^-u) + G s^2 / 2] / d,  s = the stress at the back of the patch
			var u := dd / (_shear_k + tire_give)
			if u < 1e-3:
				g = 0.5 * u
			else:
				u += (dd - _shear_k * u - tire_give * (1.0 - exp(-u))) / (_shear_k + tire_give * exp(-u))
				u += (dd - _shear_k * u - tire_give * (1.0 - exp(-u))) / (_shear_k + tire_give * exp(-u))
				u += (dd - _shear_k * u - tire_give * (1.0 - exp(-u))) / (_shear_k + tire_give * exp(-u))
				var eu := exp(-u)
				var sg := 1.0 - eu
				g = sg - (_shear_k * (sg - u * eu) + 0.5 * tire_give * sg * sg) / dd
		if bed_seat > 0.0:
			# The seated share of the bed, as a brush: each bit of tread stretches (tire and stones
			# in series, see series_on) until the stones under it break loose at the bed's peak,
			# then slides on at the flowing strength. Under a patch pressed hardest in the middle
			# that is the textbook brush model with a static and a sliding friction:
			#   F / F_flow = 3 bed psi (1 - psi)^2 + psi^2 (3 - 2 psi),  psi = slip / slip where it all slides
			# The stuck part at the front is always on stones that haven't moved yet; past psi = 1
			# the whole patch is flowing.
			var rb := dd / _bed_k
			var gb := 1.0
			if rb < 1.0:
				var q := 1.0 - rb
				gb = 3.0 * _bed * rb * q * q + rb * rb * (3.0 - 2.0 * rb)
			g = lerpf(g, gb, bed_seat)
		fxl = fl * g * k / sl
		fyl = -fl * g * sla / sl
	# A tire spinning hard in loose stuff digs itself a hole, polishes the bottom and (in mud and
	# snow) packs its tread solid: past a few m/s of slip the drive it finds falls away, so the
	# engine revs up instead of dragging the car through. Wet grass and mud are the worst.
	# Measured on wheelspin alone. It used to include the sideways slip, which meant a car taken
	# through a corner sideways - the normal way round a loose corner - had its cornering force cut
	# as if it were sitting still spinning a hole. It is the spinning that polishes the bottom of
	# the hole and packs the tread, not the sliding.
	# Only while the tread keeps working the same patch of ground, though: a car that's moving runs
	# onto fresh material all the time, so it's the slip against the car's speed that digs, not the
	# slip speed alone (8 % slip in 4th at 70 mph is already 2.5 m/s, which used to start the fade).
	var wr := omega * radius
	var vl := v_long
	var v_spin := absf(wr - vl)
	var fade := _spin_fade * smoothstep(1.5, 12.0, v_spin) * smoothstep(0.25, 1.0, v_spin / maxf(absf(vl), 1.0))
	fxl *= 1.0 - fade
	fyl *= 1.0 - fade * 0.35
	fxv = lerpf(fxv, fxl, cv)
	fyv = lerpf(fyv, fyl, cv)
	f_max = lerpf(fm, fl, cv)
	var vsx := vl - wr
	if _bull > 0.0:
		# Fore and aft: the tread ploughs the material lying in front of it. A locked or sliding
		# tire shoves a wedge along; a spinning one mostly throws it clear behind.
		if absf(vsx) > 0.1:
			var kx := 0.3 if absf(wr) > absf(vl) else 1.0
			fxv -= minf(_bull * vsx * vsx, _t_load2) * kx * signf(vsx)
		# Sideways: a different thing entirely, and the one that was missing. A tire running at a
		# big slip angle cuts into the layer and the stones or snow bank up against its shoulder
		# and sidewall. That bank is what a rally car actually turns on once the rubber has let go,
		# so it grows with the angle and is worth most exactly where the brush model has nothing
		# left. It holds a lot more than the wedge in front of the tread before it shears away.
		# The bank only grows until it's as tall as it can stand: past ~berm_tan of slip the stones
		# spill over the shoulder and the tire runs on what's spilled, so the push stops growing
		# with the angle (measured gravel side force goes flat past 15-25 deg, it doesn't keep
		# climbing). Before the spill a 40 deg slide pushed harder than the tire's peak on tarmac.
		var vt := v_lat
		if absf(vt) > 0.1:
			var vref := maxf(absf(vl), 3.0)
			var ang := absf(vt) / vref
			var build := clampf(ang / 0.3, 0.0, 1.0) * _berm
			var vtb := minf(absf(vt), vref * berm_tan)
			var fb := minf(_bull * build * vtb * vtb, load * berm_cap)
			fyv -= fb * signf(vt)
			plough = fb / maxf(load * berm_cap, 1.0)
	fx = fxv
	fy = fyv
	fx_sum += fxv
	fy_sum += fyv


## Integrates wheel spin for one substep with a lockable friction torque (brakes + rolling), then
## (when `then_force`) works out the next substep's tire force straight away.
func integrate(then_force := false) -> void:
	var w_free := omega + _i_k * (drive_torque - fx * radius) / inertia
	var cap := _i_cap
	var om: float
	if absf(w_free) <= cap:
		om = 0.0
	else:
		om = w_free - signf(w_free) * cap
	omega = om
	# Implicit relaxation-length slip dynamics (divisors in prep_substeps).
	kappa = clampf((kappa + _i_al * (om * radius - v_long)) / _i_dl, -8.0, 8.0)
	slip_lat = clampf((slip_lat + _i_ny) / _i_dy, -8.0, 8.0)
	if then_force:
		compute_tire_force()


## This wheel into its block of the C++ core's buffer (native/gen_layout.py: the WHEEL list), at `o`.
## Everything prep_substeps / compute_tire_force / integrate read. in_driven: the box drives it this tick.
func npack(b: PackedFloat64Array, o: int, in_driven: bool) -> void:
	b[o + NL.W_CONTACT] = 1.0 if contact else 0.0
	b[o + NL.W_LOAD] = load
	b[o + NL.W_REF_LOAD] = ref_load
	b[o + NL.W_MU] = _mu
	b[o + NL.W_GRIP_SCALE] = grip_scale
	b[o + NL.W_STUD_MU] = _stud_mu
	b[o + NL.W_PEAK_SLIP] = _peak_slip
	b[o + NL.W_PEAK_ANGLE] = _peak_angle
	b[o + NL.W_MU_L] = _mu_l
	b[o + NL.W_COH] = _coh
	b[o + NL.W_BRAKE_TORQUE] = brake_torque
	b[o + NL.W_ROLLING_TORQUE] = rolling_torque
	b[o + NL.W_RUB_TORQUE] = rub_torque
	b[o + NL.W_RELAX_LONG] = relax_long
	b[o + NL.W_RELAX_LAT] = relax_lat
	b[o + NL.W_RADIUS] = radius
	b[o + NL.W_INERTIA] = inertia
	b[o + NL.W_V_LONG] = v_long
	b[o + NL.W_V_LAT] = v_lat
	b[o + NL.W_TC_C] = _tc_c
	b[o + NL.W_TC_B] = _tc_b
	b[o + NL.W_TC_E] = _tc_e
	b[o + NL.W_HB_SHARE] = hb_share
	b[o + NL.W_AL_LONG] = _al_long
	b[o + NL.W_AL_LAT] = _al_lat
	b[o + NL.W_COVER] = cover
	b[o + NL.W_PATCH_LEN] = patch_len
	b[o + NL.W_SHEAR_K] = _shear_k
	b[o + NL.W_TIRE_GIVE] = tire_give
	b[o + NL.W_BED_SEAT] = bed_seat
	b[o + NL.W_BED_K] = _bed_k
	b[o + NL.W_BED] = _bed
	b[o + NL.W_SPIN_FADE] = _spin_fade
	b[o + NL.W_BULL] = _bull
	b[o + NL.W_BERM] = _berm
	b[o + NL.W_FRONT] = 1.0 if front else 0.0
	b[o + NL.W_IN_DRIVEN] = 1.0 if in_driven else 0.0
	b[o + NL.W_SHAFT_BROKEN] = 1.0 if shaft_broken else 0.0
	b[o + NL.W_OMEGA] = omega
	b[o + NL.W_KAPPA] = kappa
	b[o + NL.W_SLIP_LAT] = slip_lat
	b[o + NL.W_SLIDE_EX] = _slide_ex
	b[o + NL.W_FX] = fx
	b[o + NL.W_FY] = fy
	b[o + NL.W_FX_SUM] = fx_sum
	b[o + NL.W_FY_SUM] = fy_sum
	b[o + NL.W_F_MAX] = f_max
	b[o + NL.W_PLOUGH] = plough
	b[o + NL.W_DRIVE_TORQUE] = drive_torque
	b[o + NL.W_T_ON] = 1.0 if _t_on else 0.0
	b[o + NL.W_T_FMAX] = _t_fmax
	b[o + NL.W_T_SXD] = _t_sxd
	b[o + NL.W_T_SYD] = _t_syd
	b[o + NL.W_T_FL] = _t_fl
	b[o + NL.W_T_RK] = _t_rk
	b[o + NL.W_T_LOAD2] = _t_load2
	b[o + NL.W_I_CAP] = _i_cap
	b[o + NL.W_I_K] = _i_k
	b[o + NL.W_I_AL] = _i_al
	b[o + NL.W_I_DL] = _i_dl
	b[o + NL.W_I_NY] = _i_ny
	b[o + NL.W_I_DY] = _i_dy
	b[o + NL.W_I_DK] = _i_dk
	b[o + NL.W_I_DV] = _i_dv


## ... and back out after the core's substeps: everything those three functions write.
func nunpack(b: PackedFloat64Array, o: int) -> void:
	omega = b[o + NL.W_OMEGA]
	kappa = b[o + NL.W_KAPPA]
	slip_lat = b[o + NL.W_SLIP_LAT]
	_slide_ex = b[o + NL.W_SLIDE_EX]
	fx = b[o + NL.W_FX]
	fy = b[o + NL.W_FY]
	fx_sum = b[o + NL.W_FX_SUM]
	fy_sum = b[o + NL.W_FY_SUM]
	f_max = b[o + NL.W_F_MAX]
	plough = b[o + NL.W_PLOUGH]
	drive_torque = b[o + NL.W_DRIVE_TORQUE]
	_t_on = b[o + NL.W_T_ON] != 0.0
	_t_fmax = b[o + NL.W_T_FMAX]
	_t_sxd = b[o + NL.W_T_SXD]
	_t_syd = b[o + NL.W_T_SYD]
	_t_fl = b[o + NL.W_T_FL]
	_t_rk = b[o + NL.W_T_RK]
	_t_load2 = b[o + NL.W_T_LOAD2]
	_i_cap = b[o + NL.W_I_CAP]
	_i_k = b[o + NL.W_I_K]
	_i_al = b[o + NL.W_I_AL]
	_i_dl = b[o + NL.W_I_DL]
	_i_ny = b[o + NL.W_I_NY]
	_i_dy = b[o + NL.W_I_DY]
	_i_dk = b[o + NL.W_I_DK]
	_i_dv = b[o + NL.W_I_DV]


func update_visual(dt: float) -> void:
	if visual == null:
		return
	spin_angle = wrapf(spin_angle - omega * dt, -TAU, TAU)
	# Camber (static + steering caster; the visible lean of body roll comes with the body itself).
	var vis_tilt := (-1.0 if left else 1.0) * camber
	var real_kp := front and kp_axis != Vector3.ZERO
	if front and caster != 0.0 and not real_kp:
		vis_tilt -= caster * sin(steer_angle - toe_steer())
	# (about the real steering axis the tilt of the axis itself leans the wheel into the turn)
	var turn := Basis(kp_axis, steer_angle) if real_kp else Basis(Vector3.UP, steer_angle)
	var b := turn * Basis(Vector3.BACK, deg_to_rad(-vis_tilt)) * Basis(Vector3.RIGHT, spin_angle)
	if left:
		b = b * Basis(Vector3.UP, PI)
	# The physics hub sits where a RIGID tire would just touch the road at the corner's static load
	# (its static deflection is folded into the tire spring, see suspension_force). A tire that is
	# drawn squashing has to sit that much lower, or its flattened patch hovers over the road.
	var drop := static_load / maxf(tyre_k, 1.0) if not _deform_meshes.is_empty() else 0.0
	var at := Vector3(-visual_out if left else visual_out, -susp_length - drop, 0.0)
	if front and steer_angle != 0.0:
		# A steered wheel turns about its kingpin, not its own middle: the axis runs down through the
		# strut top / upper ball joint to the lower ball joint, inboard of the tire's centre line, so
		# the tire swings on a short arc and its inner sidewall stays clear of the strut and spring.
		# The car's own links give the axis (car_parts.gd kingpin()), else a typical one.
		var k := kp_off if real_kp else Vector3(kingpin_in(radius) * (1.0 if left else -1.0), 0.0, 0.0)
		at += k - turn * k
	visual.transform = Transform3D(b, at)
	_update_deform(dt)


## The steering axis from the suspension's links (car_parts.gd): `off` from the wheel's centre to
## the axis at hub height, `axis` pointing up, both in this node's frame. Visual only.
func set_kingpin(off: Vector3, axis: Vector3) -> void:
	kp_off = off
	kp_axis = axis


## How far inboard of the tire's centre line the steering axis is at hub height (m): the scrub
## radius at the ground (~2 cm) plus ~13 deg of kingpin inclination up to the hub.
static func kingpin_in(r: float) -> float:
	return clampf(0.02 + r * tan(deg_to_rad(13.0)), 0.06, 0.12)


## ABS valve for this wheel (per tick): dumps pressure quickly when the wheel starts to lock (slip
## past the tire's peak), then builds it back up more slowly - the 10-15 Hz pulsing of a real
## unit, which keeps the tire near its peak (~90 % of perfect threshold braking).
func abs_modulate(dt: float) -> float:
	# Below walking pace the wheel-speed signal is too coarse: real units hand back to the pedal.
	if absf(v_long) < 1.5:
		abs_gain = 1.0
		return 1.0
	# Aim just past the tire's peak slip at its current load (the peak moves out as load grows).
	var lr := clampf(load / ref_load, 0.2, 3.0)
	var target := maxf(abs_slip, _peak_slip * pow(lr, 0.15)) * 1.05
	var err := (-kappa - target) / target     # > 0: slipping past the peak
	if err > 0.0:
		abs_gain -= dt * 20.0 * minf(err, 3.0)
	else:
		abs_gain += dt * 9.0 * minf(-err, 1.0)
	abs_gain = clampf(abs_gain, 0.1, 1.0)
	return abs_gain


## Toe as a steer offset (rad): toe-in points the front of the wheel towards the middle of the car.
## (Positive steer_angle turns the wheel left.)
func toe_steer() -> float:
	return deg_to_rad(toe + bent_toe + arm_toe + _dyn_toe) * (-1.0 if left else 1.0)


## Bump steer + compliance toe for this tick, from last tick's compression and tire force (the
## same one-tick lag as the anti-roll bars). Bushings are rubber: it follows over ~40 ms, so ABS
## pulsing the brake doesn't shake the wheel.
func update_dyn_toe(dt: float, substeps: int) -> void:
	if bump_toe == 0.0 and brake_toe == 0.0 and lat_toe == 0.0:
		_dyn_toe = 0.0
		return
	var want := bump_toe * (compression - comp0)
	if contact and absf(v_long) > 1.0:
		var brk := -fx_sum / maxf(substeps, 1) * signf(v_long) / maxf(nominal_load, 1.0)
		want += brake_toe * clampf(brk, -0.6, 1.5)
		# Lateral force compliance steer: the side force bends the wheel on its bushings. + = it
		# steers the way the force pushes (the loaded outside rear toes in: understeer, what a
		# multilink's toe link is there for), - = the other way (a semi-trailing arm, a twist beam).
		if lat_toe != 0.0:
			want += lat_toe * clampf(fy_sum / maxf(substeps, 1) / maxf(nominal_load, 1.0), -1.5, 1.5) * (LAT_TOE_SIGN if left else -LAT_TOE_SIGN)
	want = clampf(want, -DYN_TOE_MAX, DYN_TOE_MAX)
	_dyn_toe = lerpf(_dyn_toe, want, 1.0 - exp(-dt / 0.04))


## Camber against the road this tick and what it does to grip (call before the tire substeps).
## The tire grips best sideways when its top leans ~1.5 deg into the turn (like a bike) and worst
## leaning out; standing on its edge it brakes and drives a little worse.
func update_alignment() -> void:
	# Knocked out of true by a hit: the car pulls, and that tire scrubs and runs hot.
	var camber_eff := camber + bent_camber + arm_camber - bump_camber * (compression - comp0)
	var side := -1.0 if left else 1.0
	tilt = side * camber_eff + body_roll * roll_gain
	if front and caster != 0.0:
		tilt -= caster * sin(steer_angle - toe_steer())
	camber_steer = -deg_to_rad(clampf(tilt, -8.0, 8.0) * CAMBER_STEER) if camber_thrust_on else 0.0
	var dir := signf(fy_sum) if absf(fy_sum) > 50.0 else 0.0
	var lean := tilt * dir
	# (Relative to a typical factory set-up leaning ~1.5 deg the wrong way under roll, which the
	# tire grip figures were tuned with: a good camber set-up gains a few percent, a bad one loses.)
	var dl := lean - 1.5
	var f := 1.0 + 0.011 * dl if dl < 0.0 else 1.0 - 0.004 * dl * dl
	_al_lat = (clampf(f / 0.97, 0.85, 1.04) if dir != 0.0 else 1.0) * _ps_lat
	_al_long = clampf(1.0 - 0.003 * tilt * tilt, 0.85, 1.0) * _ps_long


## Carcass from the tire's sidewall stiffness (wall rating and profile): stiffer sidewalls bend
## less and build cornering force over a shorter distance (relaxation length).
func apply_carcass() -> void:
	# The base is read once: after this tyre_k is the pressure-scaled figure, and reading it again on
	# every recovery (fix_damage) softened the tire ~7 % a reset, for good.
	if not _k_taken:
		tyre_k_base = tyre_k
		_k_taken = true
	psi = psi_cold * (air_temp + 273.15) / (20.0 + 273.15)
	pressure_effects()


## Everything the pressure changes, from the current hot pressure (psi) against the placard
## pressure. A tire is mostly an air spring with a bit of carcass stiffness on top:
##   * vertical stiffness ~ 0.3 + 0.7 p: soft tires soak up bumps and have a longer contact patch
##     (more bite on gravel, snow and mud - why rally crews let some air out on loose stages),
##   * cornering stiffness and carcass bend: soft = lazier response, bigger slip angles, more bend,
##   * tarmac grip peaks around the placard pressure: over-inflated runs on the crown of the tread
##     (smaller patch), badly under-inflated folds its sidewall and the bead starts to roll,
##   * rolling resistance and flex heat grow as it goes soft; aquaplaning starts lower (~ sqrt p).
func pressure_effects() -> void:
	# The placard figure is a cold one; tires are designed to work ~8 % above it once warm.
	var pr := clampf(psi / maxf(psi_rec * 1.08, 1.0), 0.3, 1.8)
	tyre_k = tyre_k_base * (0.3 + 0.7 * pr)
	var over := maxf(pr - 1.0, 0.0)
	var under := maxf(1.0 - pr, 0.0)
	_p_mu_hard = 1.0 - 0.45 * over * over - 0.3 * under * under
	if pr < 0.6:
		_p_mu_hard *= lerpf(0.65, 1.0, clampf((pr - 0.35) / 0.25, 0.0, 1.0))   # sidewall folding over
	# Loose surfaces like it softer: best at ~80 % of placard.
	var pl := pr / 0.8
	_p_mu_loose = clampf(1.0 + 0.08 - 0.35 * (pl - 1.0) * (pl - 1.0), 0.75, 1.08) / 1.08 * (1.0 + 0.08 * clampf(1.0 - pr, 0.0, 0.25) / 0.25)
	_p_corner = pow(pr, 0.35)
	_p_roll = pow(pr, -0.5)
	_p_heat = pow(pr, -1.5)
	_p_aqua = sqrt(pr)
	var stiff := clampf(float(tyre.get("stiff", 1.0)), 0.6, 1.6) * _p_corner
	carcass_k_lat = 160000.0 * stiff
	carcass_k_long = 260000.0 * stiff
	relax_lat = 0.45 / stiff
	relax_long = 0.20 / sqrt(stiff)


## Hot pressure from the cold setting and the air in the tire (it sits on the carcass, so it follows
## the carcass temperature, not the tread surface).
func update_pressure() -> void:
	var t_air := lerpf(air_temp, carcass_t, 0.85)
	psi = psi_cold * (t_air + 273.15) / (20.0 + 273.15)
	pressure_effects()


## Swap the wheel's materials for the deforming tire material and note each mesh's size. The rim
## stays rigid out to its lip, measured off the model itself (see _tyre_bead_lip): the bead seat
## from the tire size alone missed it by up to an inch on low-profile cars, so the lip and the
## bead next to it bent with the rubber and the tire folded into the wheel lip.
## Catalogue rims (a separate "Rim" mesh) are left alone, and their lip sets where the rubber starts.
func setup_deform() -> void:
	_deform_meshes.clear()
	if visual == null:
		return
	var rim_lip := 0.0
	for mi in visual.find_children("Rim", "MeshInstance3D", true, false):
		var rm := (mi as MeshInstance3D).mesh
		if rm:
			var rb := rm.get_aabb()
			rim_lip = maxf(rim_lip, maxf(maxf(absf(rb.position.y), absf(rb.end.y)), maxf(absf(rb.position.z), absf(rb.end.z))))
	for mi in visual.find_children("*", "MeshInstance3D", true, false):
		var m := mi as MeshInstance3D
		if m.mesh == null or m.name == "Rim":
			continue
		var bb := m.mesh.get_aabb()
		var outer := maxf(maxf(absf(bb.position.y), absf(bb.end.y)), maxf(absf(bb.position.z), absf(bb.end.z)))
		if outer <= 0.0:
			continue
		var bl := _tyre_bead_lip(m.mesh)
		var bead: float = bl.x if bl.x > 0.0 else outer * rim_ratio
		var lip: float = maxf(bl.y if bl.x > 0.0 else bead * 1.02, rim_lip if rim_lip < outer * 0.95 else 0.0)
		lip = minf(maxf(lip, bead), outer * 0.95)
		m.set_meta("retro_skip", true)
		if m.material_override:
			m.material_override = TYRE_DEFORM.material(m.material_override)
		else:
			for k in m.mesh.get_surface_count():
				var cur := m.get_active_material(k)
				if cur:
					m.set_surface_override_material(k, TYRE_DEFORM.material(cur))
		var half := maxf(absf(bb.position.x), absf(bb.end.x))
		_deform_meshes.append([m, lip, outer, half])
		m.set_instance_shader_parameter("tyre_geo", Vector4(lip, outer, half, bead))


## Bead and lip radius of a wheel mesh (local units), from its vertices: the bead is the inside of
## the tire surfaces (material named Tire / Tyre), the lip the furthest out any other surface (the
## rim) reaches before the bead. Vector2.ZERO when the mesh has no tire surface. Cached on the mesh.
static func _tyre_bead_lip(mesh: Mesh) -> Vector2:
	if mesh.has_meta("tyre_bead_lip"):
		return mesh.get_meta("tyre_bead_lip")
	var bead := INF
	var tyre_s: Array = []
	for k in mesh.get_surface_count():
		var mat := mesh.surface_get_material(k)
		var nm := (mat.resource_name if mat else "").to_lower()
		if nm.contains("tire") or nm.contains("tyre"):
			tyre_s.append(k)
			for v in mesh.surface_get_arrays(k)[Mesh.ARRAY_VERTEX] as PackedVector3Array:
				bead = minf(bead, Vector2(v.y, v.z).length())
	var out := Vector2.ZERO
	if not tyre_s.is_empty():
		var lip := bead
		for k in mesh.get_surface_count():
			if k in tyre_s:
				continue
			for v in mesh.surface_get_arrays(k)[Mesh.ARRAY_VERTEX] as PackedVector3Array:
				var r := Vector2(v.y, v.z).length()
				if r <= bead * 1.06:
					lip = maxf(lip, r)
		out = Vector2(bead, lip)
	mesh.set_meta("tyre_bead_lip", out)
	return out


func _update_deform(dt: float) -> void:
	if _deform_meshes.is_empty():
		return
	# A car at the FAR sim level (sim_water off, see Car._apply_sim_level) is a few pixels, and a
	# hidden wheel shows nothing: neither needs three shader parameters a tire a tick.
	if not sim_water or not visual.is_visible_in_tree():
		return
	# Vertical: the tire's own spring deflection. Sideways / fore-aft: the carcass is dragged
	# along with the road force (it leads the rim in the force's direction).
	var squash := 0.0
	var shift := Vector3.ZERO
	var twist := 0.0
	if contact:
		# The tire's real deflection: with the hub drawn its static deflection lower (update_visual)
		# this puts the flat patch exactly on the road surface, whatever the load.
		squash = clampf(load / tyre_k, 0.0, 0.08)
		shift = side_dir * clampf(fy / carcass_k_lat, -0.05, 0.05) + fwd_dir * clampf(fx / carcass_k_long, -0.03, 0.03)
		# Slip angle: the tread stuck to the road lies along the patch's real path (v_lat / v_long,
		# which slip_lat follows over the relaxation length - the same lag the carcass has). Past the
		# tire's peak slip the tread slides rather than bending further, so the skew tops out
		# there. On loose ground part of the slip is the ground shearing, not the rubber (the tire's share: tire_give).
		var sat := maxf(_t_syd, 0.02)
		twist = sat * tanh(slip_lat / sat) * signf(v_long) * lerpf(1.0, tire_give / maxf(_shear_k + tire_give, 1e-4) if series_on else 0.5, cover)
	var k := 1.0 - exp(-dt * 30.0)
	_squash = lerpf(_squash, squash, k)
	_shift = _shift.lerp(shift, k)
	_twist = lerpf(_twist, twist, k)
	_deform_dirty = true


## The squash onto the tire's shader, once per drawn frame (Car._process): the physics runs 2-4
## ticks a frame and only the last one is ever seen, so pushing it every tick was wasted
## RenderingServer calls (4 tires x every car near enough to deform).
func push_deform() -> void:
	if not _deform_dirty:
		return
	_deform_dirty = false
	for e in _deform_meshes:
		var m: MeshInstance3D = e[0]
		if not is_instance_valid(m):
			continue
		var inv := m.global_transform.basis.inverse()
		var down_l: Vector3 = inv * -contact_normal if contact else inv * -global_transform.basis.y
		var s := down_l.length()
		var dn := Vector3(0.0, down_l.y, down_l.z).normalized()
		m.set_instance_shader_parameter("tyre_down", Vector4(dn.x, dn.y, dn.z, _squash * s))
		var sh := inv * (_shift * DEFORM_SHOW)
		# Twist: the rolling direction in the wheel's plane at the patch, the patch's half length,
		# and the tread's offset across the tire (local x) per local unit along it (local units
		# may be scaled, hence the trip through the world vectors).
		var roll_l := inv * fwd_dir
		roll_l.x = 0.0
		roll_l -= dn * roll_l.dot(dn)
		var per_m := roll_l.length()
		var tw := 0.0
		var roll := Vector4.ZERO
		if per_m > 1e-5 and _squash > 1e-4:
			var outer: float = e[2]
			var d := _squash * s
			var half_len := sqrt(maxf(2.0 * outer * d - d * d, 0.0))   # chord of the flat patch
			tw = (inv * side_dir).x * _twist * DEFORM_SHOW / per_m
			roll = Vector4(roll_l.x / per_m, roll_l.y / per_m, roll_l.z / per_m, half_len * 1.15)
		m.set_instance_shader_parameter("tyre_shift", Vector4(sh.x, sh.y, sh.z, tw))
		m.set_instance_shader_parameter("tyre_roll", roll)


## Grip from tread temperature: full inside the window, down to -10 % cold / -25 % overheated.
## Each of the three zones on its own, weighted by how much of the patch it carries: a shoulder
## cooked in a long slide goes greasy while the rest of the tread is fine.
func tyre_temp_grip() -> float:
	return _tyre_zone_grip(tread_t[0]) * _tread_w.x + _tyre_zone_grip(tread_t[1]) * _tread_w.y \
		+ _tyre_zone_grip(tread_t[2]) * _tread_w.z


func _tyre_zone_grip(t: float) -> float:
	if t < tyre_window.x:
		return 1.0 - minf((tyre_window.x - t) * 0.004, 0.10)
	if t > tyre_window.y:
		return 1.0 - minf((t - tyre_window.y) * 0.008, 0.25)
	return 1.0


## Every zone and the carcass at one temperature (a fresh set on, a reset, the start of a stage).
func set_tyre_temp(t: float) -> void:
	tread_t.fill(t)
	carcass_t = t
	tyre_temp = t


## How the patch's load (and the slide's heat) is shared across the tread: camber leaning the top in
## puts it on the inner shoulder, cornering force rolls the tire onto its outer one (the outside
## tires in a turn), over-inflated it rides on the centre, soft on the shoulders.
func _tread_share(fy_avg: float) -> Vector3:
	var inward := 1.0 if left else -1.0
	var lean_in := tilt * inward                      # deg, + = top towards the car (negative camber)
	var lat := 0.0
	if contact and load > 100.0:
		lat = clampf(fy_avg / maxf(load, 1.0) * inward, -1.3, 1.3)   # + = pushing the car to its middle
	var sh := clampf(0.06 * lean_in - 0.3 * lat, -0.7, 0.7)          # + = towards the inner shoulder
	var pr := clampf(psi / maxf(psi_rec * 1.08, 1.0), 0.6, 1.5)
	var wi := maxf(1.0 + 1.5 * sh, 0.15)
	var wo := maxf(1.0 - 1.5 * sh, 0.15)
	var wc := pr * pr
	var tot := wi + wc + wo
	return Vector3(wi / tot, wc / tot, wo / tot)


## How much of the brake's bite is left at this disc temperature.
func brake_fade_factor() -> float:
	return lerpf(1.0, 0.6, clampf((brake_temp - brake_fade.x) / maxf(brake_fade.y - brake_fade.x, 1.0), 0.0, 1.0))


## Per tick, after the tire substeps: heat in (brake power, tire slip + rolling), heat out (air).
func update_thermal(dt: float, ambient: float, speed: float, fx_avg: float, fy_avg: float) -> void:
	_wear_step(dt, fx_avg, fy_avg)
	var p_brake := brake_torque * absf(omega)
	var h_brake := 12.0 + 3.6 * absf(speed)
	brake_temp += (p_brake * 0.95 - h_brake * (brake_temp - ambient)) / brake_cap * dt
	brake_temp = clampf(brake_temp, ambient - 5.0, 1100.0)
	# Three tread zones over a carcass (MASTER_PLAN physics list 8). The sliding power (force x slip
	# speed), a share of which ends up in the rubber, goes into the zones carrying the patch; rolling
	# hysteresis (flex) heats the carcass. The zones are thin and quick - a long slide cooks the
	# surface in seconds and it cools again over a straight - the carcass is slow, so a cold set takes
	# a couple of corners to come in. Heat crosses the tread between zones and soaks into / out of the
	# carcass; the air takes it off the surface. (Same total heat capacity and cooling as the old one
	# temperature, so a tire run the same way ends up as warm as it always did.)
	var p_slide := 0.0
	var p_flex := 0.0
	if contact:
		p_slide = (absf(fx_avg * (omega * radius - v_long)) + absf(fy_avg * v_lat)) * 0.5
		p_flex = load * absf(v_long) * 0.012 * _p_heat
	_tread_w = _tread_share(fy_avg)
	var h_zone := (18.0 + 2.8 * absf(speed)) / 3.0
	var t0 := tread_t[0]
	var t1 := tread_t[1]
	var t2 := tread_t[2]
	var tc := carcass_t
	# Small steps when the quarter-rate upkeep hands over a big dt: the zones are quick.
	var n := maxi(1, ceili(dt / 0.02))
	var h := dt / n
	for _k in n:
		var q0 := p_slide * _tread_w.x + p_flex * 0.15 - h_zone * (t0 - ambient) - TREAD_G * (t0 - tc) - TREAD_SIDE * (t0 - t1)
		var q1 := p_slide * _tread_w.y + p_flex * 0.15 - h_zone * (t1 - ambient) - TREAD_G * (t1 - tc) - TREAD_SIDE * (2.0 * t1 - t0 - t2)
		var q2 := p_slide * _tread_w.z + p_flex * 0.15 - h_zone * (t2 - ambient) - TREAD_G * (t2 - tc) - TREAD_SIDE * (t2 - t1)
		var qc := p_flex * 0.55 + TREAD_G * (t0 + t1 + t2 - 3.0 * tc) - 4.0 * (tc - ambient)
		t0 += q0 / TREAD_CAP * h
		t1 += q1 / TREAD_CAP * h
		t2 += q2 / TREAD_CAP * h
		tc += qc / CARCASS_CAP * h
	tread_t[0] = clampf(t0, ambient - 5.0, 220.0)
	tread_t[1] = clampf(t1, ambient - 5.0, 220.0)
	tread_t[2] = clampf(t2, ambient - 5.0, 220.0)
	carcass_t = clampf(tc, ambient - 5.0, 200.0)
	tyre_temp = tread_t[0] * _tread_w.x + tread_t[1] * _tread_w.y + tread_t[2] * _tread_w.z
	air_temp = ambient
	update_pressure()


const LAND_WINDOW := 0.3         # s after touchdown that a load spike counts as the landing
const LAND_SPIKE := 5.5          # x static load a landing has to reach before it can knock anything
const TREAD_CAP := 650.0          # J/K per tread zone (thin and quick)
const CARCASS_CAP := 1050.0       # J/K: the carcass and the air in it (slow); 3 x zones + this = the old 3000
const TREAD_G := 22.0             # W/K: each zone <-> carcass
const TREAD_SIDE := 9.0           # W/K: zone <-> its neighbour across the tread


# ---------------------------------------------------------------- damage

## Tread wears away with the energy the tire puts into sliding, and faster when it is hot. It
## starts at the tire's new depth and never goes below the canvas. Shallow tread aquaplanes sooner
## (already) and bites loose surfaces less, because there are no blocks left to dig with.
func _wear_step(dt: float, fx_avg: float, fy_avg: float) -> void:
	if _hit_sev > 0.0:
		if _dmg_rng == null:
			_dmg_rng = RandomNumberGenerator.new()
			_dmg_rng.seed = hash([get_instance_id(), front, left])
		damage_hit(_hit_sev, _dmg_rng, _hit_landing)
		_hit_sev = 0.0
		_hit_landing = false
	if contact and Game.damage_on("tyre_damage"):
		var slide := absf(fx_avg * (omega * radius - v_long)) + absf(fy_avg * v_lat)
		# ~1 mm per 40 km of hard use; heat above 90 C eats it faster.
		var heat_k := 1.0 + clampf((tyre_temp - 90.0) / 60.0, 0.0, 1.5)
		tread_mm = maxf(tread_mm - slide * 2.2e-7 * heat_k * dt, 0.6)
	# Air leaving through a cut or an unseated bead. Below about 8 psi the sidewall is folding and
	# it is a flat: pressure_effects() already turns that into a soft, draggy, lazy tire.
	if leak > 0.0:
		psi_cold = maxf(psi_cold - leak * dt, 4.0)
		if psi_cold <= 4.01:
			leak = 0.0
		flat = psi_cold < 10.0


## Something hit this wheel hard: a rock, a kerb edge, the far side of a pothole. `sev` is roughly
## how violent it was (1 = just enough to notice). Rolls for a cut, a bent rim and knocked
## alignment, all on this corner only.
func damage_hit(sev: float, rng: RandomNumberGenerator, landing := false) -> void:
	if sev > struck:
		struck = sev                # the suspension arms and the wiring hear about it (car_damage.gd)
		struck_landing = landing
	if sev < 1.0 or not Game.damage_on("tyre_damage"):
		return
	var over := clampf((sev - 1.0) / 2.0, 0.0, 1.0)
	# A cut: at worst it is down in seconds, at best it is a slow leak you notice two corners later.
	# (Not from a landing: there's no edge to cut it on, just the road.)
	if not landing and rng.randf() < 0.10 + 0.45 * over:
		var rate := lerpf(0.25, 9.0, over * rng.randf())
		leak = maxf(leak, rate)
	# A bent rim runs out of true: a vibration that comes and goes with road speed.
	if rng.randf() < 0.06 + 0.3 * over:
		rim_bend = clampf(rim_bend + lerpf(0.05, 0.4, over), 0.0, 0.8)
	# And the corner is knocked out of alignment, so the car pulls and that tire scrubs.
	if rng.randf() < 0.12 + 0.4 * over:
		var k := lerpf(0.3, 2.2, over) * (1.0 if rng.randf() < 0.5 else -1.0)
		# Bent, not destroyed: past a few degrees the car would be undriveable, and a stage that
		# ends in a wreck is a reset anyway.
		bent_toe = clampf(bent_toe + k, -3.0, 3.0)
		bent_camber = clampf(bent_camber + k * 0.6, -2.0, 2.0)
		update_alignment()


## Puts a new wheel on at a service stop or a reset: full pressure, straight rim, fresh tread.
func fix_damage() -> void:
	leak = 0.0
	flat = false
	rim_bend = 0.0
	bent_toe = 0.0
	bent_camber = 0.0
	tread_mm = float(tyre.get("tread_mm", 8.0))
	flat_mm = 0.0
	psi_cold = psi_set if psi_set > 0.0 else psi_rec
	update_alignment()
	apply_carcass()


## How out of true this wheel runs: a bent rim or a flat spot (steering shimmy, rumble).
func shake() -> float:
	return maxf(rim_bend, flat_mm * 0.09)


## Slip speed at the contact patch (m/s) used for effects and audio.
func slip_speed() -> float:
	if not contact:
		return 0.0
	return Vector2(omega * radius - v_long, v_lat).length()
