extends RefCounted
## Lightweight aero: what the air does to a car, worked out once in Car.setup so the physics tick
## is a few multiplies (car.gd, "Aero against the air").
##
##   drag          F = 1/2 rho Cd A v^2, against the air (the car files' drag_cd x frontal_area)
##   lift / down   F = 1/2 rho Cl A v_fwd^2 per axle, at the axle, along the body's up. Positive Cl is
##                 lift: a stock road car is a wing the wrong way up and gets a little lighter at speed,
##                 the rear most of all on a saloon or a hatch. A factory spoiler / lip trims the rear.
##                 Negative Cl is downforce.
##   draft         behind another car the air is already moving with you: drag and lift both shrink
##                 with the gap (Car._update_draft, a few times a second, no raycasts).
##
##   ride height   (sens(), Car._aero_attitude) the lift numbers above are at the car's rest height.
##                 Lower, less air gets under it: a body loses lift and a splitter / diffuser in
##                 ground effect makes more downforce; higher (or off the ground) the reverse. Nose-up
##                 to the air (a crest, the rear squatting) adds lift, mostly at the front; nose-down
##                 takes it off. Rolled over in a corner, one side seals and the other lifts: a little
##                 less of either. Damaged panels let air in where it shouldn't be (body_panels.gd).
##
## Per car: the body shape (BODY by car id, or a "body_type" key) picks the lift numbers, a car file can
## set its own with "aero": {"cl_front": x, "cl_rear": y}, and garage parts nudge them (part_k below).
## Real-world-ish figures: a 90s saloon is about Cl 0.3 overall, a hatch a bit more at the back, a
## pickup's bed and a wagon's long roof less; sports cars with a spoiler near 0.15.
## (No class_name on purpose: preload("res://src/car/aero.gd").)

const TRAITS := preload("res://src/data/brand_traits.gd")

const RHO := 1.225   # air density, kg/m^3
const TALL := ["hatch", "wagon", "suv", "van"]   # a wing on the roof sits in the roof's wake

## Garage AERO tab slots (aero_parts.gd builds the pieces).
const PART_SLOTS := ["aero_wing", "aero_front", "aero_rear", "aero_roof"]
## What each option does to the air: [front Cl, rear Cl, Cd], added to the car's own (times its
## frontal area; - Cl = downforce). A big rear wing pries the nose up a little through the axle, so it
## adds some front lift. Real-ish: a GT wing on a road car is about 200-250 lb at 100 mph.
const AIR := {
	"aero_wing": {
		"lip": [0.0, -0.05, 0.004], "ducktail": [0.01, -0.09, 0.006], "whale": [0.02, -0.15, 0.012],
		"bootlid": [0.02, -0.17, 0.012], "rally": [0.03, -0.30, 0.025], "pedestal": [0.03, -0.27, 0.022],
		"gt": [0.04, -0.45, 0.035], "swan": [0.04, -0.50, 0.030], "biplane": [0.05, -0.58, 0.046],
	},
	"aero_front": {
		"lip": [-0.05, 0.0, -0.003], "air_dam": [-0.08, 0.0, -0.002], "planes": [-0.07, 0.0, 0.010],
		"splitter": [-0.16, 0.01, 0.008], "canards": [-0.24, 0.01, 0.020], "fences": [-0.29, 0.01, 0.026],
	},
	"aero_roof": {
		"vortex": [0.0, -0.03, 0.003], "scoop": [0.0, 0.0, 0.006], "fin": [0.0, -0.01, 0.002], "spoiler": [0.0, -0.06, 0.006],
	},
	"aero_rear": {
		"fins": [0.0, -0.02, -0.001], "diffuser": [0.0, -0.05, -0.004], "twin": [-0.01, -0.08, -0.002],
		"race": [-0.02, -0.12, 0.003],
	},
}

## Lift coefficients by body shape, [front, rear] (times the frontal area).
const SHAPE := {
	"sedan": [0.10, 0.20],
	"coupe": [0.08, 0.18],
	"hatch": [0.10, 0.24],
	"wagon": [0.10, 0.14],
	"sports": [0.05, 0.16],   # low wedge: little front lift, the rear still lifts without a spoiler
	"muscle": [0.12, 0.16],
	"pickup": [0.12, 0.10],
	"suv": [0.14, 0.18],
	"van": [0.12, 0.14],
}

const BODY := {
	"accent": "sedan", "accord": "sedan", "corolla": "sedan", "sedan": "sedan", "impreza": "sedan",
	"impreza_gd": "sedan", "civic_fg2": "coupe", "cobalt": "coupe", "ion": "sedan", "crownvic": "sedan", "galaxie": "muscle",
	"crownvic_p71": "sedan", "taurus_sho": "sedan", "taurus_pis": "sedan", "datsun510": "sedan",
	"ae86": "coupe", "impreza_rs": "coupe", "integra": "coupe", "tiburon": "coupe", "eclipse": "coupe",
	"genesis_coupe": "coupe", "brz": "coupe",
	"civic_ef": "hatch", "civic_ek": "hatch", "civic_ep3": "hatch", "civic_g1": "hatch", "civic_g2": "hatch",
	"civic_g3": "hatch", "hatch": "hatch", "focus": "hatch", "golf_mk4": "hatch", "gti_mk5": "hatch",
	"rabbit": "hatch", "rabbit_mk5": "hatch", "mini": "hatch", "fit": "hatch", "elantra_gt": "hatch",
	"soul": "hatch",
	"wagon": "wagon", "wrx_wagon": "wagon", "impreza_wagon": "wagon", "outback_sport": "wagon",
	"volvo850": "wagon", "eagle": "wagon",
	"miata": "sports", "miata_nb": "sports", "miata_nc": "sports", "miata_ms": "sports", "mr2_aw11": "sports",
	"mr2_sw20": "sports", "mr2_zzw30": "sports", "rx7_fb": "sports", "rx7_fc": "sports", "rx7_fd": "sports",
	"fiero": "sports", "s2000_ap1": "sports", "s2000_ap2": "sports",
	"alcyone_xt": "sports", "alcyone_xt6": "sports", "alcyone_svx": "coupe",
	"camaro": "muscle", "camaro_police": "muscle", "challenger": "muscle", "challenger_police": "muscle",
	"charger": "muscle", "charger_pursuit": "muscle", "gto": "muscle",
	"truck": "pickup", "b3000": "pickup", "ranger": "pickup", "s10": "pickup", "dakota": "pickup", "baja": "pickup",
	"cherokee": "suv", "samurai": "suv", "escape": "suv", "escape_police": "suv",
	"previa": "van",
}

## Factory spoilers / lips / wings: taken off the rear lift coefficient.
const SPOILER := {
	"rx7_fd": 0.10, "mr2_sw20": 0.08, "eclipse": 0.08, "impreza_rs": 0.08, "cobalt": 0.06, "fiero": 0.06,
	"tiburon": 0.05, "civic_fg2": 0.05, "ion": 0.05, "genesis_coupe": 0.04, "miata_ms": 0.05,
	"rx7_fc": 0.04, "wrx_wagon": 0.04, "gti_mk5": 0.03, "golf_mk4": 0.02, "civic_ep3": 0.03,
	"camaro": 0.04, "camaro_police": 0.04, "challenger": 0.04, "challenger_police": 0.04,
	"charger": 0.04, "charger_pursuit": 0.04, "taurus_sho": 0.02, "brz": 0.02,
}


## {cda (m^2), cla_f, cla_r (m^2, + = lift)} for a car def with its parts already applied (upgrades.gd).
static func for_def(def: Dictionary) -> Dictionary:
	var id := String(def.get("base_id", def.get("id", "")))
	var cd: float = def.get("drag_cd", 0.38)
	var area: float = def.get("frontal_area", 1.9)
	var body := String(def.get("body_type", BODY.get(id, "sedan")))
	var shape: Array = SHAPE.get(body, SHAPE.sedan)
	var parts: Dictionary = def.get("parts", {})
	var wing := String(parts.get("aero_wing", "stock"))
	var clf := float(shape[0])
	var clr := float(shape[1]) - (float(SPOILER.get(id, 0.0)) if wing == "stock" else 0.0)   # a fitted wing replaces it
	var own: Dictionary = def.get("aero", {})
	clf = float(own.get("cl_front", clf))
	clr = float(own.get("cl_rear", clr))
	var k := part_k(parts)
	clf *= float(k.lift)
	clr *= float(k.lift)
	# Garage AERO tab (AIR above).
	for slot in PART_SLOTS:
		var a: Array = (AIR.get(slot, {}) as Dictionary).get(String(parts.get(slot, "stock")), [])
		if a.is_empty():
			continue
		var eff := 0.8 if slot == "aero_wing" and body in TALL else 1.0
		# The brand's version makes a little more or less downforce (brand_traits.gd perf); lift and drag as listed.
		var kd := 1.0 + float(TRAITS.on(parts, slot).get("perf", 0.0))
		clf += float(a[0]) * eff * (kd if float(a[0]) < 0.0 else 1.0)
		clr += float(a[1]) * eff * (kd if float(a[1]) < 0.0 else 1.0)
		cd += float(a[2])
	return {cda = cd * area * float(k.drag) + float(k.add_cda), cla_f = clf * area, cla_r = clr * area}


## Ride-height sensitivity per axle, Cl per metre of height (+ = more lift as it rises): {rh_f, rh_r}.
## The bare underbody is ~0.03 Cl per inch; anything working in ground effect at that end (a lip, an
## air dam, a splitter / a diffuser) a lot more: a splitter's downforce falls away fast as the nose
## lifts. Wings sit in clean air and don't care.
const RH_BODY_F := 1.2
const RH_BODY_R := 0.8
const RH_PART := 3.5
const PITCH_CL := 0.01          # Cl per degree of nose-up to the air (whole car)
const PITCH_FRONT := 0.65       # ... share of it at the front axle
const ROLL_K := 0.015           # lift and downforce lost per degree of roll
static func sens(def: Dictionary) -> Dictionary:
	var parts: Dictionary = def.get("parts", {})
	var df_f := 0.0
	var df_r := 0.0
	var a: Array = (AIR.aero_front as Dictionary).get(String(parts.get("aero_front", "stock")), [])
	if not a.is_empty():
		df_f = maxf(-float(a[0]), 0.0)
	a = (AIR.aero_rear as Dictionary).get(String(parts.get("aero_rear", "stock")), [])
	if not a.is_empty():
		df_r = maxf(-float(a[1]), 0.0)
	return {rh_f = RH_BODY_F + RH_PART * df_f, rh_r = RH_BODY_R + RH_PART * df_r}


## What garage parts do to the air: ride height changes how much air gets under the car (lower = less
## lift and a touch less drag; a raised gravel kit the reverse), and anything bolted to the roof or
## sticking out of the hood adds drag.
static func part_k(parts: Dictionary) -> Dictionary:
	var drag := 1.0
	var lift := 1.0
	var add := 0.0
	match String(parts.get("suspension", "stock")):
		"sport":
			drag = 0.99
			lift = 0.88
		"coilover":
			drag = 0.98
			lift = 0.82
		"gravel":
			drag = 1.03
			lift = 1.12
		"springs", "street_coil":
			drag = 0.99
			lift = 0.86
		"tarmac_rally":
			drag = 0.995
			lift = 0.92
		"lifted":
			drag = 1.05
			lift = 1.18
	match String(parts.get("lightbar", "stock")):
		"pods": add += 0.07
		"led": add += 0.05
	add += float(preload("res://src/car/hood_scoops.gd").CDA.get(String(parts.get("hood_scoop", "stock")), 0.0))
	if String(parts.get("fun_ears", "stock")) != "stock":
		add += 0.02
	return {drag = drag, lift = lift, add_cda = add}


## How much of the air ahead a car `ahead_m` behind another one is spared (0..~0.45): the wake is a
## tube about a car wide that fills back in over a few car lengths. `off_m` = sideways offset,
## `width` = the leading car's width, `v_k` = its speed / ours (a parked car hides you from nothing).
static func draft_k(ahead_m: float, off_m: float, width: float, v_k: float) -> float:
	if ahead_m <= 0.0 or ahead_m > 45.0:
		return 0.0
	var reach := width * (0.55 + ahead_m * 0.012)   # the wake spreads as it goes
	var side := clampf(1.0 - absf(off_m) / reach, 0.0, 1.0)
	return 0.45 * exp(-ahead_m / 11.0) * side * clampf(v_k, 0.0, 1.0)
