extends RefCounted
## The tire's force curve: Pacejka's Magic Formula in normalized slip (no class_name; preload).
##   f(s) = sin(C * atan(B*s - E*(B*s - atan(B*s))))      s = slip / the slip where the grip peaks
## so f(1) = 1 (the peak, grip = mu x load), whatever the tire. The three shape numbers come from
## published tire behavior, not guesses per tire:
##   C: from the sliding ratio (what the tire still has when it's fully sliding / its peak). The
##      formula's far end is sin(C*pi/2), so C = 2 - (2/pi) * asin(slide). Wong, "Theory of Ground
##      Vehicles" (peak vs sliding friction: dry asphalt ~0.85 / 0.75, wet ~0.6 / 0.5, gravel
##      ~0.6 / 0.55, snow ~0.2 / 0.15, ice ~0.1 / 0.07) is what surfaces.gd's `slide` numbers follow.
##   B: solved so the peak sits at s = 1 (the surface's peak slip / peak angle).
##   E: the curvature round the peak. Pacejka's fitted lateral curves run from about +0.5 to -2;
##      the stiffer the tire (data/tyres `stiff`) the more negative, which also keeps the initial
##      slope B*C at 1.9 to 2.4 per unit of normalized slip. That is what real road and race
##      tires measure (cornering stiffness ~10-16 per radian of load, peak at 7-9 degrees) and what
##      the brush curve the game used before had (2.0), so a tire's steering response is unchanged.

const E_SOFT := -0.5         # soft, tall, off-road rubber (stiff 0.75 and under): slope ~2.4 per peak slip
const E_SHARP := -1.6        # stiff race rubber (stiff 1.35 and over): slope ~1.9
const SLIDE_SOFT := 1.0      # the surface's `slide` x this for the soft ones (as measured: 1.04 was a forgiveness fudge, off 0.7.7)
const SLIDE_SHARP := 0.93    # ... and for the sharp ones (they let go harder)

static var _b_cache := {}


## {c, e, b, slide} for a tire of stiffness `stiff` on a surface whose sliding ratio is `slide`.
static func shape(slide: float, stiff: float) -> Dictionary:
	var k := clampf((stiff - 0.75) / 0.6, 0.0, 1.0)
	var e := lerpf(E_SOFT, E_SHARP, k)
	var sl := clampf(slide * lerpf(SLIDE_SOFT, SLIDE_SHARP, k), 0.3, 0.97)
	var c := 2.0 - (2.0 / PI) * asin(sl)
	return {c = c, e = e, b = solve_b(c, e), slide = sl}


## B so that the peak (C * atan(argument) = pi / 2) falls at s = 1. Cached by shape.
static func solve_b(c: float, e: float) -> float:
	var key := int(roundf(c * 1000.0)) * 10000 + int(roundf((e + 2.0) * 1000.0))
	if _b_cache.has(key):
		return _b_cache[key]
	var target := tan(PI / (2.0 * c))
	var lo := 0.05
	var hi := 60.0
	for _i in 48:
		var mid := 0.5 * (lo + hi)
		var arg := mid - e * (mid - atan(mid))
		if arg < target:
			lo = mid
		else:
			hi = mid
	var b := 0.5 * (lo + hi)
	_b_cache[key] = b
	return b


## The normalized curve: 0 at s = 0, 1 at s = 1, falling to `slide` as s grows.
static func eval(s: float, c: float, b: float, e: float) -> float:
	var x := b * s
	return sin(c * atan(x - e * (x - atan(x))))
