# Geometry and Visualization (`src/geometry/`, `src/visualization/`)

This is how the project turns numeric dimensions into an actual 3D shape (STL meshes) and then into pictures/videos. None of this feeds back into the physics simulation except for mass/inertia properties (`rcflex_body_properties.m`, covered in `01_PHYSICS_AND_MODEL.md`) — this is otherwise a presentation layer.

## Geometry generation (`src/geometry/`)

### `generate_rcflex_assets.m` — building the whole parts list

This is a large function that constructs the entire 3D assembly as a MATLAB struct array `A`, one entry per part, each with a name, a "body" it's rigidly attached to (used later by the animator to decide which rotation/translation to apply), a category, a color, a mass, and a triangle mesh (`V` vertices, `F` faces). It builds every visible component: the handle's outer shell (upper/lower halves, generated separately so they can be hidden independently in cutaway views), the spherical bezel and moving carrier, the front cap, control panel/button/LEDs, the battery pouch, PCB and named chips (MCU, IMU, H-bridge drivers), USB-C connector, the two voice-coil actuators (winding, housing, rear cap, mount, plunger, visual coil turns), the flexure pivot sleeves (C-Flex B-10 x2, D-10 x1) and their supporting frame parts, the pushrods and rod-end/slider joints, the spoon neck and bowl, the spoon-side IMU, a bellows seal, and a static balance mass.

Several helper functions at the bottom build primitive meshes:
- `boxmesh`, `cylmesh`, `ringmesh`, `ellipsoidmesh` — simple parametric solids (box, cylinder, hollow ring/tube, ellipsoid).
- `shellmesh` — the hollow ergonomic handle shell, built as a "superellipse" cross-section (a shape between a rectangle and an ellipse, controlled by an exponent) that varies along the handle's length, interpolated (`pchip` — piecewise cubic Hermite interpolation, chosen because it doesn't overshoot between control points the way a plain spline can) from the small set of `H.shell_x_mm`/`shell_y_scale`/`shell_z_scale` control stations in `rcflex_parameters.m`.
- `sphereshell` — a hollow spherical shell segment (used for both the fixed bezel and the moving spherical carrier).
- `frontcapmesh` — the front end plate, built using MATLAB's `polyshape`/`triangulation` (2D polygon boolean operations) to cut two actual rod-clearance openings out of a superellipse-shaped cap, rather than faking the openings visually.
- `bowlmesh` — the actual spoon bowl, generated from the same analytic outline function (`rcflex_spoon_outline.m`) used elsewhere, radially blended from the outline down to a bowl floor using smoothstep-like cubic blending functions, with both an outer surface (with wall thickness subtracted) and inner surface.
- `neckmesh` — the curved connecting neck between bowl and handle, using `pchip` interpolation of neck width vs. position and a smooth (cubic) height ramp.
- `panelmesh` — the rounded control-panel strip on top of the handle.

After building all parts, the function does mesh cleanup for every part: welds near-duplicate vertices (`unique(round(V*1e9)/1e9,...)`, i.e. snapping vertices that are within a nanometre of each other — this fixes seams where separately-generated patches should connect), removes degenerate zero-area triangles, and standardizes each mesh's face-winding order so that computed enclosed volume is always positive (flipping faces if the initial winding gives a negative volume — this matters for `rcflex_mesh_mass_properties.m`'s volume-integral formula and for correct outward-facing normals in the STL). It then writes every part as a binary STL file (`write_stl`, a hand-written minimal STL binary writer — 80-byte header, triangle count, then normal+3 vertices per triangle) into `assets/geometry/`, saves the full struct array plus the hardware parameters `H` into `assembly.mat`, and writes a `parts_manifest.csv` listing every part's name, body, category, and mass in grams.

### `rcflex_mesh_mass_properties.m`

Covered in `01_PHYSICS_AND_MODEL.md` — exact tetrahedral-decomposition volume/centroid/inertia integration for a closed triangle mesh, given an assumed total mass. Used both for the moving-body mass properties feeding the physics model, and (in `generate_rcflex_assets.m`) to compute density-consistent masses for some parts from their volume.

### `rcflex_spoon_outline.m`

A single function defining the spoon bowl's plan-view half-width as a function of distance from the tip — an elliptical nose blending into a wider "shoulder" section and then a smooth cubic-Hermite blend down to the narrow neck-connection width. This one function is reused in three unrelated places: the 3D bowl mesh generator, the liquid model's bowl-geometry sampler (`rcflex_liquid_geometry.m`), and the dimension-verification preview — guaranteeing all three agree on the bowl's actual shape rather than risking three independent (and possibly inconsistent) definitions.

## Visualization (`src/visualization/`)

### `animate_rcflex_analytic.m` — the single-model animation

Loads the saved assembly, draws every part as a MATLAB `patch` (a 3D filled polygon mesh graphics object), and for each simulation-time frame updates every part's vertex positions based on the solved joint angles/actuator strokes at that instant: parts attached to `PitchCarrier` get rotated by the pitch angle, parts attached to `SpoonCarrier` by the combined pitch+roll rotation, actuator plungers get translated along their axis by the solved stroke `L.s`, and pushrods are re-oriented between their (recomputed) endpoints using a Rodrigues-rotation helper (`rod(...)`) that rotates the rod's original mesh from its neutral orientation to its new one. In "cutaway" mode, certain parts (upper shell, bezel, moving carrier's outer surface, control panel) are hidden (`alpha=0`) so the internal mechanism is visible. If a video filename is supplied, frames are captured via `getframe` and written with `VideoWriter` (MPEG-4). This function is used by both the open-loop voltage test (producing `voltage_test.mp4` / `rcflex_physical_test.png`) and the closed-loop control demo (producing `control_demo.mp4` / `rcflex_control_animation.png`) — it looks at `S.mode` to decide the on-screen footer text.

### `animate_rcflex_comparison.m` — side-by-side "with vs. without control"

Draws **two** copies of the entire assembly side by side in one figure: the left copy is always driven with `q = 0` (gimbal rigidly locked to the handle — the "without compensation" baseline), the right copy uses the actual solved joint angles from a closed-loop run (the "with compensation" case). Both are driven by the exact same base motion, so any visual difference is due only to the controller. Below the two 3D views, it also plots live-updating pitch/roll error traces with a moving marker, and (when a liquid simulation result is supplied) overlays an animated liquid surface (as a colored `surf` plot reconstructed from the saved height field) and droplet particles following ballistic trajectories from the fluid model's outflow events. It renders in two passes: full-length at real-time (30 fps), then again at quarter-speed with unchanged amplitude — explicitly so a slow-motion pass can be inspected without amplifying the apparent motion. This produces `control_comparison.mp4`/`.png` and (with a liquid argument) the `soup_*_comparison.mp4` family.

### `plot_rcflex_plant.m` — open-loop response plot

A simple 6-panel `tiledlayout` figure for one open-loop analytic run: applied voltage, solved current, solved joint angle (deg), solved actuator displacement (mm), electromagnetic force, and copper loss — saved as `rcflex_voltage_response.png`.

### `preview_rcflex_design.m` — static 3D previews

Draws the full assembly in one of several modes: `'assembled'` (everything visible), `'cutaway'` (outer shell/bezel/carrier hidden, matching the animator's cutaway logic), `'exploded'` (parts offset from their normal position by category — spoon assembly pulled forward, flexure frame pulled back, rods separated, shell halves pulled apart vertically, electronics/battery offset — a classic "exploded assembly view" for showing how parts fit together), or `'orthographic'` (a 2×2 tiled layout showing top/side/front/isometric views together). Saves to `rcflex_<viewMode>.png`.

### `preview_spoon_closeup.m`

A tighter camera view isolating just the spoon bowl, neck, spherical carrier, and quick-release collar parts — saved as `spoon_curved_neck_closeup.png`.

### `preview_spoon_dimensions.m`

Not a 3D render — a 2D dimensioned-drawing-style figure (top view, side section, and cross-section at the widest point) with dimension lines and text labels (helper function `dim(...)` draws a line with end-tick marks and a centered label), directly validating and illustrating the bowl's headline dimensions (62×40×12 mm, wall thicknesses, etc.) against the analytic outline function. Saved as `spoon_dimension_verification.png`.

## "Assembled" vs. "cutaway" — what actually differs

Both use the exact same assembly data and rendering code path; the only difference is which named parts are skipped (`continue`) or drawn fully transparent (`alpha=0`). The specific parts hidden in cutaway mode are the outer shell top half, the fixed spherical bezel, the moving spherical carrier's outer shell, a "gimbal guard" reference (not currently generated — see note below), the control panel, and the power button/status LEDs — i.e., everything that would otherwise visually block the internal flexures, actuators, and linkage from view. "Assembled" draws everything with full opacity, showing the product's real external appearance.

Note: some code (`test_rcflex_shell_clearance.m`, the cutaway-hiding list) references `H.guard_x_mm`/`Gimbal_guard`, but `generate_rcflex_assets.m` does not currently create a part named `Gimbal_guard` or set `H.guard_x_mm` in `rcflex_parameters.m` — those checks are conditioned on `isfield(...)` and simply skip themselves if absent, so this isn't a broken reference, just an optional/inactive code path from what appears to be an earlier design iteration.
