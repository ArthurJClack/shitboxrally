class_name GroundMap
extends Node3D

const PERF := preload("res://src/tools/perf.gd")
const WEATHER := preload("res://src/race/weather.gd")
## Deformable loose layer of the stage (gravel, dust, slurry, sand, snow, turf) and tire marks.
##
## A sparse grid of 0.1 m cells, allocated in 3.2 m chunks only where tires have been. Each cell
## stores the depth (m) of loose material on it, what that material is, the surface underneath and
## a mark intensity. Untouched cells hold the surface's nominal depth. Every physics tick each tire
## moves material (interact):
##   - sliding (locked brakes, lateral slides) pushes it ahead of the tread into a wedge / berm,
##   - wheelspin digs it out and throws it behind the tire (roost), spawning flying stones,
##   - rolling compacts it and squeezes some to the sides, so repeated passes cut ruts and sweep
##     a cleaner, grippier line down to the base,
##   - sliding rubber leaves dark marks on hard surfaces.
## Wheels read the depth back (sample_patch) for their grip blend, bulldozing and ride height.
## Rendered as one Decal per nearby chunk (albedo + normal map from the depth gradient), projected
## only onto road/terrain meshes (render layer 2) so cars and props stay clean.

const CELL := 0.1
const N := 32
const SIZE := CELL * N
const AREA := CELL * CELL
const UNSET := -1.0
const VIS_RANGE := 160.0
const TEX_INTERVAL := 1.0 / 15.0
const MAX_STONES := 384
const LAYER := 2

static var current: GroundMap

var sampled_mat := ""           # loose material found by the last sample()/sample_patch()

var _chunks := {}               # Vector2i -> Chunk
var _coarse := {}               # Vector2i (chunk key / COARSE) -> Array[Chunk]: the decal range scan
const COARSE := 20              # chunks per coarse cell side (64 m)
var _dirty := {}                # Chunk -> true
var _decals := Retro.decals_drawn()   # false: nothing here is painted (the renderer can't show decals)
var _decaled: Array[Chunk] = []
var _decal_queue: Array[Chunk] = []   # chunks come into range in batches; their decals are made a few a frame
const DECALS_PER_FRAME := 3
var _tex_due := 0.0             # chunks owed a repaint (a fraction carries over to the next frame)
var _turn: Array[Chunk] = []    # the chunks repainted this frame
var _vis_timer := 0.0
var _rng := RandomNumberGenerator.new()
var _ci := 0                    # cell index returned by _cell()
var _mats: Array[Dictionary] = []
# Per-material values the painter reads for every cell, looked up once instead of per pixel.
var _nom := PackedFloat32Array()       # loose_depth
var _col := PackedColorArray()         # color
var _base_col := PackedColorArray()    # base_color (or color)
var _mark_col := PackedColorArray()    # mark_color
var _dig := PackedFloat64Array()       # dig
var _loose := PackedByteArray()        # 1 = has a loose layer (loose_depth)
var _compact := PackedFloat64Array()   # compact (interact() reads these for every tire every tick)
var _rutk := PackedFloat64Array()      # rut
var _markk := PackedFloat64Array()     # mark
var _throw := PackedFloat64Array()     # throw
var _ray_q := PhysicsRayQueryParameters3D.new()   # _surface_at's ray, reused
var _ids := {}                         # surface name -> index in Surfaces.IDS
# interact(): which cells the current tire has worked (see Chunk.seen).
var _stamp := 0
var _seen_c: Array[Chunk] = []
var _seen_i := PackedInt32Array()
var _todo := PackedInt32Array()        # normals to repaint in a chunk (_timed_process)
var _last_k := Vector2i.ZERO           # _cell's last chunk
var _last_c: Chunk = null
var _soff := PackedVector3Array()      # interact(): offsets across the patch

var _mm: MultiMesh
var _sp := PackedVector3Array()
var _sv := PackedVector3Array()
var _sfloor := PackedFloat32Array()
var _slife := PackedFloat32Array()
var _snext := 0
var _salive := 0
var _slive := PackedInt32Array()   # the slots in flight or settling, in slot order (the tick walks these, not all 384)
## The stones' MultiMesh buffer (12 floats of transform + 4 of colour each), sent once a rendered
## frame when a physics tick has moved them, instead of a call per stone per tick.
var _sbuf := PackedFloat32Array()
var _sdirty := false
var _bodies: Array = []         # [{body, boxes: [[CollisionShape3D, half extents]], listener}]
var _bpos := PackedVector3Array()   # each body's position this tick (_update_stones)


class Chunk:
	var key: Vector2i
	var y := 0.0
	var depth := PackedFloat32Array()   # loose material (m), UNSET = untouched
	var mat := PackedByteArray()        # loose material type (index in Surfaces.IDS)
	var base := PackedByteArray()       # surface underneath
	var mark := PackedByteArray()       # rubber / scuff marks 0..255
	var dig := PackedFloat32Array()     # hole dug into the soft base below the loose layer (m)
	var dry := PackedFloat32Array()     # wet roads: how much water tires have pushed out here (0..1)
	var dry_r := PackedFloat32Array()   # weather.rain_seconds when `dry` was last set (rain refills it)
	var seen := PackedInt32Array()      # GroundMap._stamp of the last interact() that worked the cell
	var ck := PackedFloat32Array()      # the road condition's depth factor the cell started with (1 = nominal)
	var alpha := PackedFloat32Array()   # albedo alpha last painted (the normal map's alpha follows it)
	var nflag := PackedByteArray()      # normal already on this repaint's list
	var dirty := {}
	var listed := false                 # in GroundMap._dirty
	var decal: Decal
	var queued := false                 # waiting in _decal_queue
	var img_a: Image
	var img_n: Image
	var tex_a: ImageTexture
	var tex_n: ImageTexture

	func _init(k: Vector2i, y0: float) -> void:
		key = k
		y = y0
		depth.resize(N * N)
		depth.fill(UNSET)
		mat.resize(N * N)
		base.resize(N * N)
		mark.resize(N * N)
		dig.resize(N * N)
		dry.resize(N * N)
		dry_r.resize(N * N)
		seen.resize(N * N)
		ck.resize(N * N)
		ck.fill(1.0)


func _enter_tree() -> void:
	current = self


func _exit_tree() -> void:
	if current == self:
		current = null


## Loose material laid down before anyone drives (spilled gravel, drifted sand / snow, mud).
func seed_material(p: Vector3, depth: float, mat: int, surf: int) -> void:
	var c := _cell(p, true)
	var i := _ci
	_touch(c, i, surf)
	if depth > c.depth[i]:
		c.depth[i] = depth
		c.mat[i] = mat
	_set_dirty(c, i)


## The stage's planned spills (road_detail.spills) go in when a car or the camera comes within
## SEED_RANGE of them, checked every SEED_EVERY s, rather than all at the start: on an 11 mile stage
## seeding them all was ~5 s of the race start on a mini PC. Well past what the decals show
## (VIS_RANGE) and what a car covers between checks.
const SEED_RANGE := 260.0
const SEED_EVERY := 0.2
var _unseeded: Array = []       # spills not in the grid yet
var _seed_timer := 0.0


func _seed_from_stage() -> void:
	_unseeded.clear()
	_seed_timer = 0.0
	_setup_condition()
	if TrackGenerator.current and TrackGenerator.current.detail:
		_unseeded = TrackGenerator.current.detail.spills.duplicate()
		_seed_near()


func _seed_near() -> void:
	if _unseeded.is_empty() or not is_inside_tree():
		return
	var at := PackedVector3Array()
	for c in get_tree().get_nodes_in_group("cars"):
		if c is Node3D and c.is_inside_tree():
			at.append((c as Node3D).global_position)
	var cam := get_viewport().get_camera_3d()
	if cam:
		at.append(cam.global_position)
	if at.is_empty():
		return
	var detail = TrackGenerator.current.detail if TrackGenerator.current else null
	if detail == null:
		_unseeded.clear()
		return
	for k in range(_unseeded.size() - 1, -1, -1):
		var sp: Dictionary = _unseeded[k]
		var p: Vector3 = sp.pos
		var r: float = SEED_RANGE + maxf(float(sp.len), float(sp.wid)) * 0.5
		for q in at:
			if Vector2(q.x - p.x, q.z - p.z).length_squared() < r * r:
				detail.seed_spill(self, sp)
				_unseeded.remove_at(k)
				break


func _ready() -> void:
	for id in Surfaces.IDS:
		var def := Surfaces.get_def(id)
		_mats.append(def)
		_nom.append(float(def.get("loose_depth", 0.0)))
		var col: Color = def.color
		_col.append(col)
		_base_col.append(def.get("base_color", col) as Color)
		_mark_col.append(def.get("mark_color", Color(0.04, 0.04, 0.045)) as Color)
		_dig.append(float(def.get("dig", 0.0)))
		_loose.append(1 if def.has("loose_depth") else 0)
		_compact.append(float(def.get("compact", 0.0)))
		_rutk.append(float(def.get("rut", 0.0)))
		_markk.append(float(def.get("mark", 0.3)))
		_throw.append(float(def.get("throw", 0.0)))
		if not _ids.has(id):
			_ids[id] = _mats.size() - 1
	_seed_from_stage.call_deferred()
	_sp.resize(MAX_STONES)
	_sv.resize(MAX_STONES)
	_sfloor.resize(MAX_STONES)
	_slife.resize(MAX_STONES)
	_mm = MultiMesh.new()
	_mm.transform_format = MultiMesh.TRANSFORM_3D
	_mm.use_colors = true
	var mesh := SphereMesh.new()
	mesh.radius = 0.022
	mesh.height = 0.036
	mesh.radial_segments = 5
	mesh.rings = 2
	var m := StandardMaterial3D.new()
	m.vertex_color_use_as_albedo = true
	m.roughness = 1.0
	mesh.material = m
	_mm.mesh = mesh
	_mm.instance_count = MAX_STONES
	_sbuf.resize(MAX_STONES * 16)
	_sbuf.fill(0.0)
	_mm.buffer = _sbuf
	var mmi := MultiMeshInstance3D.new()
	mmi.multimesh = _mm
	# Set from _process, and a reused slot jumps from nothing (or an old stone) to a new one:
	# interpolated, that drew a stone flying in from the old spot for a frame.
	mmi.physics_interpolation_mode = Node.PHYSICS_INTERPOLATION_MODE_OFF
	mmi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	mmi.custom_aabb = AABB(Vector3(-1e5, -1e4, -1e5), Vector3(2e5, 2e4, 2e5))
	add_child(mmi)


## Removes all deformation, marks and flying stones (stage restart).
func clear() -> void:
	for c in _decaled:
		c.decal.queue_free()
	_decaled.clear()
	_decal_queue.clear()
	_chunks.clear()
	_coarse.clear()
	_last_c = null
	_dirty.clear()
	for i in MAX_STONES:
		_slife[i] = 0.0
	_sbuf.fill(0.0)
	_mm.buffer = _sbuf
	_sdirty = false
	_salive = 0
	_slive.clear()
	_seed_from_stage()


# --- road condition (TrackParams.road_condition) ---------------------------------------------------
# The loose layer before anyone has driven on it, as a multiplier of the surface's nominal depth by
# where across the road a cell is: worked out from the position alone (the centre line, its edges
# and a racing line cut to the inside of the bends), so every PC gets the same road and cells that
# are never touched read the same as ones that are.

var _cond := 0                         # 0 normal, 1 fresh, 2 worn
var _cond_hint := 0                    # last centre-line index, as a search hint
var _cond_line := PackedFloat32Array() # worn: the line's lateral offset per centre-line point (m)
const WORN_TRACK := 0.75               # half the distance between the two swept wheel tracks (m)
const WORN_SIGMA := 0.4                # how wide each swept track is (m)


func _setup_condition() -> void:
	_cond = 0
	_cond_line.clear()
	var gen := TrackGenerator.current
	if gen == null or gen.detail == null or gen.params == null or gen.points.size() < 4:
		return
	_cond = cond_mode(gen)
	if _cond == 2:
		_cond_line = worn_line(gen)


## 0 normal, 1 fresh, 2 worn (TrackParams.road_condition).
static func cond_mode(gen: TrackGenerator) -> int:
	match String(gen.params.road_condition):
		"fresh":
			return 1
		"worn":
			return 2
	return 0


## Worn roads' line: in towards the inside of each bend (curvature smoothed over ~30 m, so it apexes
## the middle of the corner and drifts across on the straights between), never closer than 1.4 m to
## the edge. Lateral offset (m) per centre-line point. Also used by road_detail to bake the look.
static func worn_line(gen: TrackGenerator) -> PackedFloat32Array:
	var n := gen.points.size()
	var line := PackedFloat32Array()
	line.resize(n)
	var k := gen.curvature
	for i in n:
		var sum := 0.0
		var cnt := 0
		for j in range(maxi(i - 8, 0), mini(i + 9, n)):
			sum += k[j]
			cnt += 1
		var ks := sum / maxf(cnt, 1)
		var side := signf(ks)
		var room := maxf(gen.detail.edge(i, side) - 1.4, 0.0)
		line[i] = side * clampf(absf(ks) * 45.0, 0.0, 1.0) * room
	return line


## The untouched loose depth as a multiple of the surface's nominal one, at lateral offset `x` from
## the centre line with the road edge `hw` m out on that side and the worn line at `xl`.
static func cond_factor(cond: int, x: float, hw: float, xl: float) -> float:
	var ax := absf(x)
	if cond == 0 or ax > hw + 0.3:
		return 1.0   # off the road: verges and fields stay as they are
	if cond == 1:
		# Fresh: newly graded, deep and even, a bit deeper still where the grader left its windrow.
		return 1.35 + 0.3 * smoothstep(hw - 1.2, hw, ax)
	var a := (x - xl + WORN_TRACK) / WORN_SIGMA
	var b := (x - xl - WORN_TRACK) / WORN_SIGMA
	var g := maxf(exp(-a * a), exp(-b * b))
	return 1.0 - 0.75 * g + 0.7 * smoothstep(hw - 1.6, hw - 0.2, ax)


## Loose depth multiplier at a point for this stage's road condition.
func _cond_k(p: Vector3) -> float:
	var gen := TrackGenerator.current
	if gen == null or gen.detail == null:
		return 1.0
	var i := gen.index_near(p, _cond_hint)
	_cond_hint = i
	var x := gen.lateral_offset(p, i)
	var hw: float = gen.detail.edge(i, 1.0 if x >= 0.0 else -1.0)
	if _cond == 2 and i >= _cond_line.size():
		return 1.0
	return cond_factor(_cond, x, hw, _cond_line[i] if _cond == 2 else 0.0)


# --- grid access ---------------------------------------------------------------------------------

func _cell(p: Vector3, create: bool) -> Chunk:
	var k := Vector2i(floori(p.x / SIZE), floori(p.z / SIZE))
	# Lookups come in runs on the same chunk (the cells under one tire): the last one is kept.
	var c: Chunk = _last_c if k == _last_k and _last_c != null else _chunks.get(k)
	if c == null:
		if not create:
			return null
		c = Chunk.new(k, p.y)
		_chunks[k] = c
		_index(c)
	_last_k = k
	_last_c = c
	var ix := clampi(int((p.x - k.x * SIZE) / CELL), 0, N - 1)
	var iz := clampi(int((p.z - k.y * SIZE) / CELL), 0, N - 1)
	_ci = iz * N + ix
	return c


## Initialises an untouched cell with the surface it belongs to.
func _touch(c: Chunk, i: int, surf: int) -> void:
	if c.depth[i] < 0.0:
		var nd := _nom[surf]
		if _cond != 0 and nd > 0.0:
			var k := _cond_k(Vector3((c.key.x * N + i % N + 0.5) * CELL, c.y, (c.key.y * N + i / N + 0.5) * CELL))
			c.ck[i] = k
			nd *= k
		c.depth[i] = nd
		c.mat[i] = surf
		c.base[i] = surf


func _set_dirty(c: Chunk, i: int) -> void:
	if not _decals:
		return
	c.dirty[i] = true
	if not c.listed:
		c.listed = true
		_dirty[c] = true


## Loose depth (m) at a point; sets sampled_mat.
func sample(p: Vector3, surface: String) -> float:
	var c := _cell(p, false)
	if c == null or c.depth[_ci] < 0.0:
		sampled_mat = surface
		if _cond != 0:
			var nd := Surfaces.loose_depth(surface)
			return nd * _cond_k(p) if nd > 0.0 else nd
		return Surfaces.loose_depth(surface)
	sampled_mat = Surfaces.IDS[c.mat[_ci]]
	return c.depth[_ci] - c.dig[_ci]


## The dry line: how much of the water tires have squeezed out at this point (0..1), fading back as
## more rain falls on it (weather.gd rain_seconds).
func sample_dry(p: Vector3) -> float:
	var c := _cell(p, false)
	if c == null:
		return 0.0
	var d := c.dry[_ci]
	if d <= 0.0:
		return 0.0
	return d * exp(-(_rain_now() - c.dry_r[_ci]) * 0.08)


static func _rain_now() -> float:
	var wx: Node = WEATHER.current
	if wx == null:
		return 0.0
	var r: float = wx.rain_seconds
	return r


## A tire rolling over a wet sealed road pushes the water out from under it.
func _squeegee(w: Wheel, surf: int) -> void:
	var wx: Node = WEATHER.current
	if wx == null:
		return
	var wet: float = wx.wetness
	if wet < 0.08 or absf(w.v_long) < 2.0:
		return
	var rain: float = wx.rain
	var rn := _rain_now()
	var per_pass := 0.12 * (1.0 - rain * 0.7) * clampf(w.load / maxf(w.nominal_load, 1.0), 0.4, 1.5)
	for k in 3:
		var p := w.contact_point + w.fwd_dir * (w.patch_len * (k - 1) * 0.5)
		var c := _cell(p, true)
		var i := _ci
		_touch(c, i, surf)
		var cur := c.dry[i] * exp(-(rn - c.dry_r[i]) * 0.08) if c.dry[i] > 0.0 else 0.0
		c.dry[i] = minf(cur + per_pass, 1.0)
		c.dry_r[i] = rn
		_set_dirty(c, i)


## Average loose depth over a contact patch (three points along its length).
func sample_patch(p: Vector3, fwd: Vector3, length: float, surface: String) -> float:
	var d := sample(p + fwd * (length * 0.33), surface) + sample(p - fwd * (length * 0.33), surface)
	return (d + sample(p, surface)) / 3.0


## Adds a volume (m^3) of material at a point, spilling over to neighbours past the angle of repose.
## With a wheel, the point may be metres from it (roost), so a cell nothing has touched yet asks
## what ground is really there instead of taking the thrower's: otherwise turf off the verge turns
## the dirt road beside it into a patch of grass, which the next tire picks up and throws again.
func _deposit(p: Vector3, vol: float, mat: int, surf: int, w: Wheel = null) -> void:
	var c := _cell(p, true)
	var i := _ci
	if w != null and c.depth[i] < 0.0:
		surf = _surface_at(p, w, surf)
	_touch(c, i, surf)
	var add := vol / AREA
	if c.depth[i] < 0.002 or _nom[c.mat[i]] <= 0.0:
		c.mat[i] = mat
	var cap := clampf(_nom[mat] * 4.0, 0.05, 0.3)
	var nd := c.depth[i] + add
	if nd > cap:
		var spill := nd - cap
		nd = cap
		var off := Vector3(CELL * (1 if _rng.randf() < 0.5 else -1), 0.0, CELL * _rng.randi_range(-1, 1))
		var c2 := _cell(p + off, true)
		var j := _ci
		_touch(c2, j, c.base[i])
		if c2.depth[j] < 0.002 or _nom[c2.mat[j]] <= 0.0:
			c2.mat[j] = mat
		c2.depth[j] = minf(c2.depth[j] + spill, cap * 1.5)
		_set_dirty(c2, j)
	c.depth[i] = nd
	_set_dirty(c, i)


## Surface id of the ground at a point near a wheel (the collider under it, plus road features such as
## the grass strip down the middle), or `fallback` if nothing with a surface is there.
func _surface_at(p: Vector3, w: Wheel, fallback: int) -> int:
	var up := w.contact_normal
	# From well above: the point carries the thrower's height, and the road can stand a good bit
	# above the verge (or the ground below it).
	var q := _ray_q
	q.from = p + up * 2.5
	q.to = p - up * 2.0
	q.exclude = w._rc_ex
	var hit := get_world_3d().direct_space_state.intersect_ray(q)
	if hit.is_empty() or not (hit.collider as Object).has_meta("surface"):
		return fallback
	var col: Object = hit.collider
	var s := String(col.get_meta("surface"))
	if col.has_meta("road") and TrackGenerator.current and TrackGenerator.current.detail:
		var d = TrackGenerator.current.detail
		d.query(hit.position, w._road_i)
		if d.q_surface != "":
			s = d.q_surface
	var id := Surfaces.IDS.find(s)
	return id if id >= 0 else fallback


# --- tire interaction ----------------------------------------------------------------------------

## Moves material under one tire for this physics tick (called by Car after the tire forces).
func interact(w: Wheel, dt: float) -> void:
	var surf: int = _ids.get(w.surface, -1)
	var lm: int = _ids.get(w.loose_mat, -1)
	if surf < 0 or lm < 0:
		return
	var loose := _loose[lm] == 1
	if not loose and _loose[surf] == 0:
		_squeegee(w, surf)
	var fwd := w.fwd_dir
	var side := w.side_dir
	var up := w.contact_normal
	var L := w.patch_len
	var b := w.width
	var vsx := w.v_long - w.omega * w.radius
	var vs := Vector2(vsx, w.v_lat).length()
	var load_k := clampf(w.load / w.nominal_load, 0.0, 2.0)
	var dir := (fwd * vsx + side * w.v_lat) / maxf(vs, 1e-4)
	var extent := absf(dir.dot(fwd)) * L + absf(dir.dot(side)) * b + 0.05
	# Fractions of the material under the patch moved this tick (fraction of the patch slid through).
	var push := clampf(maxf(vs - 0.3, 0.0) * dt / extent, 0.0, 0.6) * minf(load_k, 1.2) if loose else 0.0
	var roll := clampf(absf(w.v_long) * dt / L, 0.0, 1.0) * load_k
	var compact: float = roll * _compact[lm]
	var rut: float = roll * _rutk[lm]
	var mk: float = clampf(maxf(vs - 1.5, 0.0) * dt / L, 0.0, 0.2) * load_k * _markk[surf]
	if push <= 0.0 and compact <= 0.0 and mk <= 0.0 and rut <= 0.0:
		return

	var moved := 0.0
	var squeezed := 0.0
	# Each cell under the patch is worked once (the sample grid is finer than the cells). A cell
	# counts as seen when its chunk's stamp for it is this call's number: no dictionary per tire per
	# tick. The cells are kept in the order they were first met for the dig below.
	_stamp += 1
	if _stamp >= 0x7FFFFFFF:     # (weeks of driving; the stamps are 32-bit)
		for ch: Chunk in _chunks.values():
			ch.seen.fill(0)
		_stamp = 1
	var stamp := _stamp
	_seen_c.clear()
	_seen_i.clear()
	var nm := ceili(mk * 255.0) if mk > 0.0 else 0
	var keep := 1.0 - push - compact - rut
	var cp := w.contact_point
	var nl := maxi(2, ceili(L / 0.08))
	var nw := maxi(2, ceili(b / 0.08))
	# (_cell, _touch and _set_dirty written out in the loop: the same steps without three calls a cell.)
	var soff := _soff
	soff.resize(nw)
	for s in nw:
		soff[s] = side * (b * ((s + 0.5) / nw - 0.5))
	var nom_s := _nom[surf]
	var lc := _last_c
	var lkx := _last_k.x
	var lkz := _last_k.y
	for a in nl:
		var row := cp + fwd * (L * ((a + 0.5) / nl - 0.5))
		for s in nw:
			var p := row + soff[s]
			var kx := floori(p.x / SIZE)
			var kz := floori(p.z / SIZE)
			var c: Chunk = lc
			if kx != lkx or kz != lkz or lc == null:
				var k := Vector2i(kx, kz)
				c = _chunks.get(k)
				if c == null:
					c = Chunk.new(k, p.y)
					_chunks[k] = c
					_index(c)
				lc = c
				lkx = kx
				lkz = kz
			var i := clampi(int((p.z - kz * SIZE) / CELL), 0, N - 1) * N + clampi(int((p.x - kx * SIZE) / CELL), 0, N - 1)
			if c.seen[i] == stamp:
				continue
			c.seen[i] = stamp
			_seen_c.append(c)
			_seen_i.append(i)
			var d := c.depth[i]
			if d < 0.0:
				c.depth[i] = nom_s
				c.mat[i] = surf
				c.base[i] = surf
				d = c.depth[i]
			if d > 0.0 and c.mat[i] == lm:
				moved += d * push
				squeezed += d * rut
				c.depth[i] = d * keep
			if nm > 0:
				c.mark[i] = mini(255, c.mark[i] + nm)
			if _decals:
				c.dirty[i] = true
				if not c.listed:
					c.listed = true
					_dirty[c] = true
	_last_c = lc
	_last_k = Vector2i(lkx, lkz)
	# A tire spinning on soft ground (mud, sand, snow, wet dirt, turf) keeps digging once the
	# loose top is gone: it cuts a hole into the base itself and sinks into it.
	var max_dig: float = _dig[surf]
	if max_dig > 0.0:
		var spin_v := absf(w.omega * w.radius) - absf(w.v_long)
		if spin_v > 1.0:
			var rate := 0.02 * clampf((spin_v - 1.0) / 8.0, 0.0, 1.5) * load_k * dt   # ~2 cm/s flat out
			for q in _seen_c.size():
				var c2: Chunk = _seen_c[q]
				var j: int = _seen_i[q]
				if c2.depth[j] < 0.004:
					var dd := minf(rate, max_dig - c2.dig[j])
					if dd > 0.0:
						c2.dig[j] += dd
						moved += dd
						_set_dirty(c2, j)
	moved *= AREA
	squeezed *= AREA
	w.thrown = moved / dt

	if squeezed > 0.0:
		_deposit(w.contact_point + side * (-1.0 * (b * 0.5 + 0.06)), squeezed * 0.5, lm, surf)
		_deposit(w.contact_point + side * (1.0 * (b * 0.5 + 0.06)), squeezed * 0.5, lm, surf)
	if moved <= 0.0:
		return
	# Wheelspin (tread running backward faster than the car) throws material clear; sliding and
	# braking mostly bulldoze it into a wedge at the leading edge of the patch.
	var spin := clampf((absf(w.omega * w.radius) - absf(w.v_long)) / maxf(vs, 0.1), 0.0, 1.0)
	var throw_frac: float = _throw[lm] * (0.15 + 0.85 * spin)   # hard surfaces have none
	var thrown := moved * throw_frac
	var pushed := moved - thrown
	var perp := dir.cross(up).normalized()
	var spread := absf(perp.dot(fwd)) * L + absf(perp.dot(side)) * b
	var edge := w.contact_point + dir * (extent * 0.5 + 0.06)
	for k in 3:
		_deposit(edge + perp * (spread * (k / 2.0 - 0.5)), pushed / 3.0, lm, surf)
	for k in 2:
		var dist := 0.25 + vs * 0.12 * _rng.randf_range(0.4, 1.4)
		var q := w.contact_point + dir * minf(dist, 4.0) + perp * _rng.randf_range(-0.35, 0.35)
		_deposit(q, thrown * 0.5, lm, surf, w)
	# Flying stones / clods for what was thrown, fewer when the slip is small next to road speed.
	var rel := clampf(vs / maxf(maxf(absf(w.v_long), absf(w.omega * w.radius)), 2.0), 0.0, 1.0)
	var n := mini(int(thrown / 2e-5 * rel + _rng.randf()), 3)
	var col: Color = _col[lm]
	for k in n:
		var v := dir * vs * _rng.randf_range(0.25, 0.75) + up * _rng.randf_range(0.8, 3.5) * clampf(vs / 8.0, 0.3, 1.0)
		v += Vector3(_rng.randf_range(-1, 1), 0.0, _rng.randf_range(-1, 1)) * 0.8
		_emit(w.contact_point + dir * (L * 0.5) + up * 0.04, v, col.darkened(_rng.randf_range(0.3, 0.65)), w.contact_point.y)


## Thrown stones / clods bounce off this body's box collision shapes. The listener (if any) gets
## stone_hit(speed, local_pos) for each hit.
func register_body(body: PhysicsBody3D, listener: Object = null) -> void:
	var boxes := []
	for c in body.get_children():
		var cs := c as CollisionShape3D
		var bs: Vector3 = Car.shape_box(cs) if cs else Vector3.ZERO
		if bs != Vector3.ZERO:
			boxes.append([cs, bs * 0.5 + Vector3(0.02, 0.02, 0.02)])
	if not boxes.is_empty():
		_bodies.append({body = body, boxes = boxes, listener = listener})


## Pushes a stone out of any registered body it has entered and bounces it off that face.
func _collide_stone(i: int) -> void:
	var p := _sp[i]
	for k in _bodies.size():
		var e: Dictionary = _bodies[k]
		if not is_instance_valid(e.body):
			_bodies.remove_at(k)
			_bpos.remove_at(k)
			return
		var body: PhysicsBody3D = e.body
		if _bpos[k].distance_squared_to(p) > 16.0:
			continue
		for bx in e.boxes:
			var cs: CollisionShape3D = bx[0]
			var half: Vector3 = bx[1]
			var xf := cs.global_transform
			var lp := xf.affine_inverse() * p
			var pen := half - lp.abs()
			if pen.x <= 0.0 or pen.y <= 0.0 or pen.z <= 0.0:
				continue
			# Exit through the nearest face.
			var axis := 0 if pen.x < pen.y and pen.x < pen.z else (1 if pen.y < pen.z else 2)
			var n_local := Vector3.ZERO
			n_local[axis] = signf(lp[axis]) if lp[axis] != 0.0 else 1.0
			lp[axis] = n_local[axis] * half[axis]
			p = xf * lp
			var n := (xf.basis * n_local).normalized()
			var rb := body as RigidBody3D
			var bv := Vector3.ZERO
			if rb:
				bv = rb.linear_velocity + rb.angular_velocity.cross(p - rb.global_position)
			var rel := _sv[i] - bv
			var vn := rel.dot(n)
			if vn < 0.0:
				rel = (rel - n * vn) * 0.7 - n * vn * 0.35
				_sv[i] = rel + bv
				if e.listener and e.listener.has_method("stone_hit"):
					e.listener.stone_hit(-vn, body.global_transform.affine_inverse() * p)
			_sp[i] = p
			return


func _emit(p: Vector3, v: Vector3, col: Color, floor_y: float) -> void:
	var i := _snext
	_snext = (_snext + 1) % MAX_STONES
	if _slife[i] <= 0.0:
		_salive += 1
		_slive.append(i)
		if _slive.size() > 1 and _slive[-2] > i:
			_slive.sort()   # (the ring wrapped: keep slot order, so a tick visits them as before)
	_sp[i] = p
	_sv[i] = v
	_sfloor[i] = floor_y
	_slife[i] = 2.0
	var o := i * 16 + 12
	_sbuf[o] = col.r
	_sbuf[o + 1] = col.g
	_sbuf[o + 2] = col.b
	_sbuf[o + 3] = col.a


# --- visuals -------------------------------------------------------------------------------------

func _physics_process(delta: float) -> void:
	var t := PERF.t0()
	_update_stones(delta)
	PERF.add(&"stones", t)
	if not _unseeded.is_empty():
		_seed_timer -= delta
		if _seed_timer <= 0.0:
			_seed_timer = SEED_EVERY
			t = PERF.t0()
			_seed_near()
			PERF.add(&"ground_map", t)


## Timed so the F3 performance page and perf.csv can show what this system costs (see perf.gd).
func _process(delta: float) -> void:
	var t := PERF.t0()
	if _sdirty:
		_sdirty = false
		_mm.buffer = _sbuf
	_timed_process(delta)
	PERF.add(&"ground_map", t)


func _timed_process(delta: float) -> void:
	if not _decals:
		return
	var cam := get_viewport().get_camera_3d()
	if cam == null:
		return
	var cp := cam.global_position
	_vis_timer -= delta
	if _vis_timer <= 0.0:
		_vis_timer = 1.0
		for i in range(_decaled.size() - 1, -1, -1):
			var c := _decaled[i]
			if _dist(c, cp) > VIS_RANGE * 1.2:
				c.decal.queue_free()
				c.decal = null
				c.img_a = null
				c.img_n = null
				c.tex_a = null
				c.tex_n = null
				_decaled.remove_at(i)
		# Only the chunks in coarse cells within range of the camera, not every chunk ever touched
		# (that list grows the whole way down the stage, and was copied each time).
		var cs := SIZE * COARSE
		for kz in range(floori((cp.z - VIS_RANGE) / cs), floori((cp.z + VIS_RANGE) / cs) + 1):
			for kx in range(floori((cp.x - VIS_RANGE) / cs), floori((cp.x + VIS_RANGE) / cs) + 1):
				var list: Variant = _coarse.get(Vector2i(kx, kz))
				if list == null:
					continue
				for c: Chunk in list:
					if c.decal == null and not c.queued and _dist(c, cp) < VIS_RANGE:
						c.queued = true
						_decal_queue.append(c)
	# Painting a chunk's decal is 1024 cells of script: making every chunk that came into range
	# this second in one frame was a hitch once a second on a stage with spilled gravel or snow
	# along it. A few a frame instead: they are 100+ m out when they join the queue.
	var made := 0
	while made < DECALS_PER_FRAME and not _decal_queue.is_empty():
		var q: Chunk = _decal_queue.pop_back()
		q.queued = false
		if q.decal == null and _dist(q, cp) < VIS_RANGE * 1.2:
			_make_decal(q)
			made += 1
	if _dirty.is_empty():
		_tex_due = 0.0
		return
	# Every touched chunk is repainted about TEX_INTERVAL after it was worked, as before, but in turn
	# over the frames in between and not all of them on one frame: repainting every chunk under both
	# cars on the same frame was a 2-3 ms step 15 times a second (the biggest script spike of a stage
	# on loose ground), which is what uneven frame pacing looks like. A chunk worked again while it
	# waits keeps its place in the line.
	_tex_due = minf(_tex_due + _dirty.size() * delta / TEX_INTERVAL, float(_dirty.size()))
	var due := int(_tex_due)
	if due <= 0:
		return
	_tex_due -= due
	var turn := _turn
	turn.clear()
	for c: Chunk in _dirty:
		turn.append(c)
		if turn.size() >= due:
			break
	for c in turn:
		_dirty.erase(c)
		c.listed = false
		if c.decal == null:
			if _dist(c, cp) < VIS_RANGE:
				_make_decal(c)
			c.dirty.clear()
			continue
		# A cell's colour is its own business, so only the cells that changed get a new one. The normal
		# depends on the neighbours' depth too, so their normals are redone with it. Dirty cells come in
		# clumps (a contact patch is several cells across), so each normal goes on the list once.
		var flags := c.nflag
		var todo := _todo
		todo.clear()
		for i: int in c.dirty:
			_paint_albedo(c, i)
			var x := i % N
			if flags[i] == 0:
				flags[i] = 1
				todo.append(i)
			if x > 0 and flags[i - 1] == 0:
				flags[i - 1] = 1
				todo.append(i - 1)
			if x < N - 1 and flags[i + 1] == 0:
				flags[i + 1] = 1
				todo.append(i + 1)
			if i >= N and flags[i - N] == 0:
				flags[i - N] = 1
				todo.append(i - N)
			if i < N * (N - 1) and flags[i + N] == 0:
				flags[i + N] = 1
				todo.append(i + N)
		for i in todo:
			_paint_normal(c, i)
			flags[i] = 0
		c.dirty.clear()
		c.tex_a.update(c.img_a)
		c.tex_n.update(c.img_n)


func _index(c: Chunk) -> void:
	var ck := Vector2i(floori(float(c.key.x) / COARSE), floori(float(c.key.y) / COARSE))
	var list: Variant = _coarse.get(ck)
	if list == null:
		list = []
		_coarse[ck] = list
	(list as Array).append(c)


func _dist(c: Chunk, p: Vector3) -> float:
	return Vector2((c.key.x + 0.5) * SIZE - p.x, (c.key.y + 0.5) * SIZE - p.z).length()


func _make_decal(c: Chunk) -> void:
	c.img_a = Image.create(N, N, false, Image.FORMAT_RGBA8)
	c.img_n = Image.create(N, N, false, Image.FORMAT_RGBA8)
	c.img_n.fill(Color(0.5, 0.5, 1.0, 0.0))
	c.alpha.resize(N * N)
	c.nflag.resize(N * N)
	c.nflag.fill(0)
	for i in N * N:
		if c.depth[i] >= 0.0:
			_paint_albedo(c, i)
			_paint_normal(c, i)
	c.tex_a = ImageTexture.create_from_image(c.img_a)
	c.tex_n = ImageTexture.create_from_image(c.img_n)
	var d := Decal.new()
	d.size = Vector3(SIZE, 8.0, SIZE)
	d.texture_albedo = c.tex_a
	d.texture_normal = c.tex_n
	d.cull_mask = 1 << (LAYER - 1)
	d.upper_fade = 0.0
	d.lower_fade = 0.0
	d.normal_fade = 0.0
	add_child(d)
	d.global_position = Vector3((c.key.x + 0.5) * SIZE, c.y, (c.key.y + 0.5) * SIZE)
	c.decal = d
	_decaled.append(c)


## Writes the albedo pixel of one cell (and keeps its alpha for the normal map, _paint_normal).
func _paint_albedo(c: Chunk, i: int) -> void:
	var d := c.depth[i]
	var x := i % N
	var z := i / N
	if d < 0.0:
		c.img_a.set_pixel(x, z, Color(0, 0, 0, 0))
		c.alpha[i] = 0.0
		return
	var mi := c.mat[i]
	var bi := c.base[i]
	var nom := _nom[mi]
	if mi == bi:
		nom *= c.ck[i]   # against what the road started with (worn tracks / fresh windrows are baked into the road mesh)
	var col := _base_col[bi]
	var a := 0.0
	if nom > 0.0 and mi == bi:
		var r := d / nom - 1.0
		if r < 0.0:
			# Swept / dug down towards the base.
			col = _base_col[mi].lerp(_col[mi], (1.0 + r) * 0.35)
			a = clampf(-r * 1.3, 0.0, 0.92)
		else:
			# Heaped up: berm, wedge or roost.
			col = _col[mi].lightened(0.12)
			a = clampf(r * 0.7, 0.0, 0.85)
	elif nom > 0.0:
		# Foreign material spilled onto another surface (gravel thrown onto tarmac or grass).
		col = _col[mi]
		a = clampf(d / (nom * 0.5), 0.0, 0.9)
	# Wet road: the line the tires have cleared reads lighter than the water around it.
	if c.dry[i] > 0.02:
		var wx: Node = WEATHER.current
		var wet: float = wx.wetness if wx != null else 0.0
		var dk := c.dry[i] * exp(-(_rain_now() - c.dry_r[i]) * 0.08) * clampf(wet * 1.5, 0.0, 1.0)
		if dk > 0.02:
			col = col.lerp(_base_col[bi].lightened(0.22), dk / maxf(a + dk, 1e-3))
			a = maxf(a, dk * 0.45)
	var m := c.mark[i] / 255.0
	if m > 0.0:
		col = col.lerp(_mark_col[bi], m / maxf(a + m, 1e-3))
		a = maxf(a, m * 0.85)
	c.img_a.set_pixel(x, z, Color(col, a))
	c.alpha[i] = a


## Writes the normal pixel of one cell, from the depth gradient (exaggerated so small ruts read at
## a distance). Its alpha follows the cell's albedo alpha (painted first).
func _paint_normal(c: Chunk, i: int) -> void:
	var x := i % N
	var z := i / N
	var dep := c.depth
	var d := dep[i]
	if d < 0.0:
		c.img_n.set_pixel(x, z, Color(0.5, 0.5, 1.0, 0.0))
		return
	var bs := c.base
	var dg := c.dig
	# Height of the loose layer against its nominal (0 where nothing has touched it).
	var ck := c.ck
	var h := d - _nom[bs[i]] * ck[i] - dg[i]
	var hx0 := h
	var hx1 := h
	var hz0 := h
	var hz1 := h
	var j: int
	var dj: float
	if x > 0:
		j = i - 1
		dj = dep[j]
		hx0 = 0.0 if dj < 0.0 else dj - _nom[bs[j]] * ck[j] - dg[j]
	if x < N - 1:
		j = i + 1
		dj = dep[j]
		hx1 = 0.0 if dj < 0.0 else dj - _nom[bs[j]] * ck[j] - dg[j]
	if z > 0:
		j = i - N
		dj = dep[j]
		hz0 = 0.0 if dj < 0.0 else dj - _nom[bs[j]] * ck[j] - dg[j]
	if z < N - 1:
		j = i + N
		dj = dep[j]
		hz1 = 0.0 if dj < 0.0 else dj - _nom[bs[j]] * ck[j] - dg[j]
	var n := Vector3(-(hx1 - hx0) / (2.0 * CELL) * 2.5, 1.0, -(hz1 - hz0) / (2.0 * CELL) * 2.5).normalized()
	# Decal normal maps are tangent-space: R = local X, G = -local Z (Godot flips green).
	c.img_n.set_pixel(x, z, Color(n.x * 0.5 + 0.5, 0.5 - n.z * 0.5, 1.0, maxf(c.alpha[i], 0.6)))


func _update_stones(delta: float) -> void:
	if _salive <= 0:
		return
	var g := Vector3(0.0, -9.81 * delta, 0.0)
	# Where each body is, read once a tick rather than once per stone per body (nothing moves them
	# while the stones are stepped).
	_bpos.resize(_bodies.size())
	for k in _bodies.size():
		var b: Variant = _bodies[k].body
		_bpos[k] = (b as Node3D).global_position if is_instance_valid(b) else Vector3.ZERO
	var keep := 0
	for j in _slive.size():
		var i := _slive[j]
		_slife[i] -= delta
		var p := _sp[i]
		if _sv[i] != Vector3.ZERO:
			_sv[i] += g
			p += _sv[i] * delta
			if p.y < _sfloor[i] and _sv[i].y < 0.0:
				p.y = _sfloor[i]
				_sv[i] = Vector3.ZERO
				_slife[i] = minf(_slife[i], 0.5)
			_sp[i] = p
			if not _bodies.is_empty():
				_collide_stone(i)
				p = _sp[i]
		var s := 0.0
		if _slife[i] <= 0.0:
			_salive -= 1
		else:
			s = minf(_slife[i] * 2.0, 1.0)
			_slive[keep] = i
			keep += 1
		# Uniform scale s at p, in the buffer's row-major 3x4 (the zeros never change).
		var o := i * 16
		_sbuf[o] = s
		_sbuf[o + 5] = s
		_sbuf[o + 10] = s
		_sbuf[o + 3] = p.x
		_sbuf[o + 7] = p.y
		_sbuf[o + 11] = p.z
	_slive.resize(keep)
	_sdirty = true
