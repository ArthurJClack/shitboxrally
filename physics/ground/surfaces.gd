class_name Surfaces
extends RefCounted
## Driving surface definitions used by the tire model, the ground map, the generator, effects and audio.
##
## Every surface is a hard BASE layer with an optional LOOSE layer of granular material on top
## (gravel stones, dust, slurry, sand, fresh snow, turf). The tire force is a blend of the two,
## weighted by how much loose material is under the contact patch (see Wheel.prepare_surface).
## Tires move the loose material around (GroundMap): wheelspin throws it back, locked or sliding
## tires push it into a wedge/berm ahead of them, rolling compacts it into ruts and repeated passes
## sweep a cleaner, grippier line down to the base.
##
## Base layer (rubber on a hard surface, brush-model curve with a peak and drop-off):
##   grip       – peak friction coefficient for a reference road tire
##   peak_slip  – longitudinal slip ratio where grip peaks
##   peak_angle – slip angle (rad) where lateral grip peaks
##   slide      – fraction of peak grip remaining when fully sliding
##   rolling    – rolling resistance coefficient
## Loose layer (granular shear, Janosi-Hanamoto: tau = (c + p tan phi)(1 - e^(-j/K)); the force
## keeps building with slip and never drops off, and material shoved aside adds a bulldozing force):
##   loose_depth – nominal depth of the loose layer (m); 0 = none
##   loose_mu    – internal friction of the material (tan phi, effective)
##   cohesion    – cohesion c (Pa), adds c * contact area (mud, snow)
##   shear_k     – shear deformation modulus K (m): how much shear the layer itself takes to build
##                 force. The tire's own give is added on top (Wheel.series_on: tread and stones
##                 are springs in series), so this is the stones alone
##   bed_mu      – how much more the seated, interlocked bed holds than the same stones flowing,
##                 locally (peak / critical-state strength; absent = none). A dense graded bed
##                 peaks 10-16 deg of friction angle over its sheared strength at tire pressures
##                 (Bolton 1986: 1.4-1.75x); a thin layer on a rough base starts moving at a much
##                 steeper angle than it stops at (Pouliquen & Forterre 2002). Over a whole patch
##                 it's worth a fifth to a quarter of that (the stuck front of the patch is still
##                 stretching while the back already flows): Wong's table has a tire on loose
##                 gravel at 0.6 peak / 0.55 sliding. Fades with speed: Wheel.bed_i0.
##   grain       – stone size (m): sets the shear that breaks the bed loose and how long it needs
##                 to seat under the tire
##   cover       – depth (m) at which the material covers ~63 % of the patch
##   density     – bulk density (kg/m^3), sets the bulldozing force of material pushed by the tire
##   sink        – fraction of the layer the tire sinks through (adds compaction rolling drag)
##   drag        – how much rolling drag that sinkage costs (0.35 if absent; sand and mud flow
##                 back in front of the tire instead of packing down, so they cost about double)
##   compact     – fraction of the depth one rolling pass compacts (ruts in snow, sand, mud)
##   rut         – fraction one rolling pass squeezes out to the sides of the tire
##   throw       – fraction of the material moved by wheelspin that is thrown clear (roost)
##   base_color  – colour of the exposed / compacted base
## Other:
##   rough      – amplitude (m) of small-scale bumps felt by the suspension
##   heave      – amplitude (m) of long-wavelength (~5-15 m) dips and lumps: old, sunken roads
##   wet_factor – grip multiplier at full wetness
##   mark       – how dark rubber marks get (hard surfaces); mark_color is their colour
##   sound      – rolling-noise family ("hard", "gravel", "soft")

const DEFS := {
	"tarmac": {name = "Tarmac", grip = 1.0, peak_slip = 0.11, peak_angle = 0.14, slide = 0.8,
		rolling = 0.013, rough = 0.002, wet_factor = 0.68, sound = "hard", mark = 0.8,
		color = Color(0.20, 0.20, 0.22), dust = Color(0.75, 0.75, 0.78, 0.35)},
	"tarmac_bumpy": {name = "Old Tarmac", grip = 0.95, peak_slip = 0.11, peak_angle = 0.14, slide = 0.8,
		rolling = 0.014, rough = 0.007, heave = 0.03, wet_factor = 0.66, sound = "hard", mark = 0.75,
		color = Color(0.25, 0.25, 0.26), dust = Color(0.72, 0.72, 0.74, 0.35)},
	# Sun-bleached, worn-smooth old asphalt: light grey, the binder faded off the stones. Grips nearly
	# like fresh tarmac in the dry (polished aggregate takes a few % off), loses more in the wet.
	"tarmac_washed": {name = "Washed-out Tarmac", grip = 0.95, peak_slip = 0.105, peak_angle = 0.135, slide = 0.8,
		rolling = 0.013, rough = 0.002, wet_factor = 0.62, sound = "hard", mark = 0.6,
		color = Color(0.47, 0.46, 0.43), base_color = Color(0.42, 0.41, 0.39),
		dust = Color(0.78, 0.76, 0.72, 0.5)},
	"tarmac_broken": {name = "Broken Tarmac", grip = 0.84, peak_slip = 0.11, peak_angle = 0.14, slide = 0.78,
		rolling = 0.015, rough = 0.010, heave = 0.045, wet_factor = 0.6, sound = "hard", mark = 0.5,
		loose_depth = 0.003, loose_mu = 0.62, cohesion = 0.0, shear_k = 0.005, cover = 0.005,
		density = 1550.0, sink = 0.08, compact = 0.0, rut = 0.03, throw = 0.6,
		color = Color(0.29, 0.28, 0.26), base_color = Color(0.22, 0.22, 0.23),
		dust = Color(0.74, 0.70, 0.62, 0.55)},
	# Hard-packed, damp-ish gravel with a thin stony top: grips almost like tarmac, still moves.
	"gravel_packed": { dig = 0.02,name = "Packed Gravel", grip = 0.9, peak_slip = 0.13, peak_angle = 0.16, slide = 0.87,
		rolling = 0.015, rough = 0.008, wet_factor = 0.9, sound = "gravel", mark = 0.3,
		loose_depth = 0.012, spin_fade = 0.2, bed_mu = 1.5, grain = 0.008, loose_mu = 0.74, cohesion = 300.0, shear_k = 0.005, cover = 0.015,
		density = 1700.0, sink = 0.2, compact = 0.03, rut = 0.03, throw = 0.5,
		color = Color(0.50, 0.42, 0.34), base_color = Color(0.38, 0.32, 0.26),
		dust = Color(0.66, 0.58, 0.48, 0.5)},
	# Dry, rolled hardpack with a few loose "marbles" on top: fast, slippery off the line.
	"gravel_hard": {name = "Hard-packed Gravel", grip = 0.86, peak_slip = 0.12, peak_angle = 0.15, slide = 0.84,
		rolling = 0.014, rough = 0.006, wet_factor = 0.8, sound = "gravel", mark = 0.35,
		loose_depth = 0.004, spin_fade = 0.18, bed_mu = 1.5, grain = 0.005, loose_mu = 0.55, cohesion = 0.0, shear_k = 0.004, cover = 0.006,
		density = 1650.0, sink = 0.1, compact = 0.01, rut = 0.04, throw = 0.6,
		color = Color(0.62, 0.57, 0.48), base_color = Color(0.52, 0.47, 0.39),
		dust = Color(0.78, 0.72, 0.6, 0.6)},
	"gravel": { dig = 0.03,name = "Gravel", grip = 0.82, peak_slip = 0.14, peak_angle = 0.17, slide = 0.86,
		rolling = 0.016, rough = 0.012, wet_factor = 0.88, sound = "gravel", mark = 0.25,
		loose_depth = 0.035, spin_fade = 0.22, bed_mu = 1.5, grain = 0.012, loose_mu = 0.62, cohesion = 0.0, shear_k = 0.006, cover = 0.02,
		density = 1600.0, sink = 0.35, compact = 0.02, rut = 0.05, throw = 0.6,
		color = Color(0.55, 0.50, 0.43), base_color = Color(0.40, 0.36, 0.31),
		dust = Color(0.72, 0.66, 0.55, 0.55)},
	"dirt": { dig = 0.08,name = "Dirt", grip = 0.80, peak_slip = 0.13, peak_angle = 0.16, slide = 0.85,
		rolling = 0.016, rough = 0.008, wet_factor = 0.65, sound = "gravel", mark = 0.3,
		loose_depth = 0.012, spin_fade = 0.3, bed_mu = 1.35, grain = 0.004, loose_mu = 0.66, cohesion = 500.0, shear_k = 0.005, cover = 0.008,
		density = 1300.0, sink = 0.4, compact = 0.06, rut = 0.04, throw = 0.7,
		color = Color(0.45, 0.34, 0.24), base_color = Color(0.33, 0.25, 0.18),
		dust = Color(0.58, 0.46, 0.33, 0.55)},
	"mud": { dig = 0.18, drag = 0.7,name = "Mud", grip = 0.55, peak_slip = 0.18, peak_angle = 0.19, slide = 0.85,
		rolling = 0.02, rough = 0.010, wet_factor = 0.9, sound = "soft", mark = 0.2,
		loose_depth = 0.06, spin_fade = 0.5, loose_mu = 0.30, cohesion = 4000.0, shear_k = 0.010, cover = 0.02,
		density = 1700.0, sink = 0.7, compact = 0.2, rut = 0.1, throw = 0.5,
		color = Color(0.30, 0.22, 0.14), base_color = Color(0.20, 0.15, 0.10),
		dust = Color(0.28, 0.20, 0.12, 0.8)},
	"sand": { dig = 0.14, drag = 0.8,name = "Sand", grip = 0.66, peak_slip = 0.16, peak_angle = 0.19, slide = 0.9,
		rolling = 0.02, rough = 0.006, wet_factor = 1.05, sound = "soft", mark = 0.15,
		loose_depth = 0.08, spin_fade = 0.3, loose_mu = 0.53, cohesion = 1000.0, shear_k = 0.008, cover = 0.02,
		density = 1600.0, sink = 0.6, compact = 0.15, rut = 0.12, throw = 0.7,
		color = Color(0.82, 0.72, 0.50), base_color = Color(0.66, 0.56, 0.38),
		dust = Color(0.86, 0.77, 0.56, 0.6)},
	# Snow, against tire tests: a studless winter tire stops from 25 mph on packed snow in 50-60 ft
	# (Tire Rack; What Car? 17.7 m; Auto Bild 16.5 m from 40 km/h) = 0.36-0.4 g, an all-season about
	# 0.25-0.3, a summer tire under 0.2. The game's cars: 0.40 / 0.29 / 0.17 (src/tools/loose_bench).
	"snow": { dig = 0.16,name = "Snow", grip = 0.30, peak_slip = 0.12, peak_angle = 0.13, slide = 0.85,
		rolling = 0.015, rough = 0.004, wet_factor = 0.95, sound = "soft", mark = 0.1,
		loose_depth = 0.07, spin_fade = 0.4, loose_mu = 0.30, cohesion = 3000.0, shear_k = 0.010, cover = 0.03,
		density = 400.0, sink = 0.75, compact = 0.45, rut = 0.05, throw = 0.5,
		color = Color(0.92, 0.94, 0.97), base_color = Color(0.64, 0.69, 0.78),
		dust = Color(0.95, 0.97, 1.0, 0.7)},
	# Snow pressed into a hard, polished layer by traffic: more grip than fresh snow, little to dig into.
	"snow_packed": { dig = 0.06,name = "Packed Snow", grip = 0.36, peak_slip = 0.11, peak_angle = 0.12, slide = 0.82,
		rolling = 0.013, rough = 0.003, wet_factor = 0.85, sound = "soft", mark = 0.15,
		loose_depth = 0.01, spin_fade = 0.35, loose_mu = 0.32, cohesion = 2500.0, shear_k = 0.008, cover = 0.012,
		density = 500.0, sink = 0.3, compact = 0.3, rut = 0.03, throw = 0.5,
		color = Color(0.84, 0.87, 0.92), base_color = Color(0.70, 0.75, 0.83),
		dust = Color(0.95, 0.97, 1.0, 0.6)},
	"ice": {name = "Ice", grip = 0.13, peak_slip = 0.07, peak_angle = 0.08, slide = 0.80,
		rolling = 0.010, rough = 0.0005, wet_factor = 0.8, sound = "hard", mark = 0.35,
		mark_color = Color(0.92, 0.96, 1.0),
		color = Color(0.72, 0.84, 0.92), dust = Color(0.9, 0.95, 1.0, 0.3)},
	# A carpet of wet leaves over a hard road: it looks like a surface and behaves like a bearing.
	# Almost no internal friction, nothing holding it together, and it goes to mush in the wet, so
	# the car goes light on the brakes and the back steps out with no warning at all. It only ever
	# lies on top of something else (road_detail spills it under the trees in autumn).
	"leaves": {name = "Leaves", grip = 0.72, peak_slip = 0.13, peak_angle = 0.16, slide = 0.72,
		rolling = 0.017, rough = 0.004, wet_factor = 0.45, sound = "soft", mark = 0.25,
		loose_depth = 0.02, spin_fade = 0.5, loose_mu = 0.3, cohesion = 0.0, shear_k = 0.012, cover = 0.008,
		density = 120.0, sink = 0.55, compact = 0.0, rut = 0.02, throw = 0.95,
		color = Color(0.45, 0.30, 0.14), base_color = Color(0.30, 0.22, 0.12),
		dust = Color(0.55, 0.38, 0.18, 0.45)},
	"grass": { dig = 0.07,name = "Grass", grip = 0.62, peak_slip = 0.15, peak_angle = 0.17, slide = 0.85,
		rolling = 0.02, rough = 0.012, wet_factor = 0.7, sound = "soft", mark = 0.2,
		loose_depth = 0.03, spin_fade = 0.45, loose_mu = 0.45, cohesion = 500.0, shear_k = 0.007, cover = 0.015,
		density = 900.0, sink = 0.5, compact = 0.12, rut = 0.03, throw = 0.4,
		color = Color(0.27, 0.42, 0.18), base_color = Color(0.30, 0.22, 0.14),
		dust = Color(0.35, 0.30, 0.20, 0.4)},
}

const ROAD_SURFACES := ["tarmac", "tarmac_bumpy", "tarmac_washed", "tarmac_broken", "gravel", "gravel_packed",
	"gravel_hard", "dirt", "mud", "sand", "snow", "snow_packed", "ice"]
## Index order is stored in GroundMap cells: only ever append.
const IDS: Array[String] = ["tarmac", "gravel", "dirt", "mud", "sand", "snow", "ice", "grass",
	"tarmac_bumpy", "tarmac_washed", "tarmac_broken", "gravel_packed", "gravel_hard", "snow_packed",
	"leaves"]


static func get_def(id: String) -> Dictionary:
	return DEFS.get(id, DEFS.tarmac)


static func loose_depth(id: String) -> float:
	return get_def(id).get("loose_depth", 0.0)


## Effective friction for the surface at a given wetness (0..1): the loose layer at nominal depth
## at high slip (blended with the base by coverage), or the base for hard surfaces.
static func grip(id: String, wetness: float) -> float:
	var d := get_def(id)
	var mu: float = d.grip
	var depth: float = d.get("loose_depth", 0.0)
	if depth > 0.0:
		mu = lerpf(mu, d.loose_mu + 0.04, 1.0 - exp(-depth / d.cover))
	return mu * lerpf(1.0, d.wet_factor, clampf(wetness, 0.0, 1.0))
