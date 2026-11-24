# Shitbox Rally: vehicle physics

The physics core of [Shitbox Rally](https://github.com/ArthurJClack/shitboxrally-releases), a rally game about tired old cars on procedurally generated stages. Godot 4, GDScript, Jolt.

The full game lives in a private repo (it's about 3 GB with assets). This repo is a read-only slice of the part I'd most want another engineer to look at: how a car turns throttle, steering and a patch of gravel into forces. It's copied straight from the game's `dev` branch, not cleaned up for show.

**Play it:** [latest release](https://github.com/ArthurJClack/shitboxrally-releases/releases/latest)

## What's here

| file | lines | what it does |
| --- | --- | --- |
| [`physics/wheel.gd`](physics/wheel.gd) | 2,067 | One corner of the car. Raycast suspension with non-linear dampers, a combined-slip tire with relaxation length, wheel spin dynamics, lockable brakes, plus tire pressure, temperature, wear and brake fade. |
| [`physics/tire_curve.gd`](physics/tire_curve.gd) | 57 | The tire force curve: Pacejka's Magic Formula in normalized slip. The header explains where each shape number comes from. Start here, it's short. |
| [`physics/drivetrain.gd`](physics/drivetrain.gd) | 1,308 | Engine torque curves, friction clutch, gearbox, torque converter, driveline lash, and open / LSD / viscous / locked differentials. |
| [`physics/steering_rack.gd`](physics/steering_rack.gd) | 654 | Everything between the front tires and the driver's hands: kingpin torque, rack or recirculating-ball box, power assist. Its output drives force-feedback wheels. |
| [`physics/aero.gd`](physics/aero.gd) | 198 | Drag, per-axle lift and downforce, drafting, and ride-height sensitivity. |
| [`physics/ground/surfaces.gd`](physics/ground/surfaces.gd) | 171 | Surface definitions. Every surface is a hard base layer with an optional loose layer on top. |
| [`physics/ground/ground_map.gd`](physics/ground/ground_map.gd) | 1,021 | The deformable loose layer: a sparse grid of 10 cm cells that tires dig, push and compact, so ruts form and a clean line gets swept in. |

## The parts that were hard

**Two tire models blended by what's under the tread.** On a hard surface the tire follows a peak-then-drop-off curve (`tire_curve.gd`). On loose material it follows a granular shear model (Janosi-Hanamoto) where force keeps building with slip and never drops off, which is why you drive gravel sideways. `Wheel.prepare_surface` blends the two by how much loose material is actually under the contact patch, and that amount changes as cars drive over it.

**The ground remembers.** `ground_map.gd` stores loose-material depth per 10 cm cell, allocated in 3.2 m chunks only where tires have been. Wheelspin throws material back, locked or sliding tires push it into a berm, rolling compacts it. Wheels read the depth back for grip, bulldozing force and ride height, so the stage is different on the second pass.

**Tire shape numbers from published data, not per-tire guesses.** The Magic Formula's `C` is derived from the ratio of sliding to peak friction, `B` is solved so the peak lands at normalized slip 1, and `E` scales with tire stiffness. The derivation and sources are in the header of `tire_curve.gd`.

**A driveline with slack in it.** `drivetrain.gd` models the clutch as a friction element, not a switch, and takes up driveline lash with a clunk when you get on or off the throttle.

## Running it

These files won't run on their own. They reference the rest of the game (autoloads like `Settings`, data tables, the track generator) through `res://` paths that only exist in the full project. They're here to be read.

## License

No open-source license. You're welcome to read the code; please ask before reusing it.
