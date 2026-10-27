# MATLAB Webots Controller README

This folder contains MATLAB controllers for two e-puck robots and a movable target in a Webots simulation. The scripts generate IBVS data, plots and CSV statistics. They are controller experiments rather than a standalone MATLAB application, so they require a Webots world with the expected robot and supervisor nodes.

## Controllers

| Controller | Assign to | Purpose |
| --- | --- | --- |
| `ibvs_testing/ibvs_testing.m` | Controlled e-puck (`bot`) | Detects four target corners with Harris features and performs IBVS using the left/right wheel motors. Can save plots and summary statistics. |
| `trajecotry/trajecotry.m` | Second robot (`tarbot`) | Moves `tarbot` along a configured path. |
| `tarbot/tarbot.m` | Supervisor | Positions the target above `tarbot` and records `bot` and `target` trajectories. |
| `block_mover/block_mover.m` | Supervisor | Moves the target in a straight line or square and records `bot` and `target` trajectories. |

The misspelling `trajecotry.m` is intentional in the current files: use that exact filename and controller function name.

## Requirements

- Webots with MATLAB selected as a controller language.
- MATLAB with the Image Processing Toolbox (`detectHarrisFeatures`, `matchpairs`, `rgb2gray`, and `exportgraphics`).
- A Webots world containing:
  - a robot or device group named `bot`,
  - a second robot or supervisor named `tarbot`,
  - a supervisor-controlled target with the DEF name `target`,
  - a camera and motors named `camera`, `left wheel motor`, and `right wheel motor` on `bot`.
- Write access to the output directory specified by `file_path`.

The scripts use the Webots MATLAB API functions `wb_*`. Opening a script in MATLAB alone does not run the simulation.

## World and target texture

- [`Camera_testing.wbt`](./Camera_testing.wbt) is the supplied Webots world. Open it from Webots before assigning or running the MATLAB controllers.
- The world uses a 3 m × 3 m `RectangleArena`, a 16 ms `basicTimeStep`, and a 104 × 78-pixel camera on `bot`. The arena walls are 0.05 m high.
- `basicTimeStep` is Webots' world-level physics and sensor integration interval. It does not directly determine how often a MATLAB controller calls its API: each controller has a separate `TIME_STEP` value. The supplied controllers use 32 ms (`tarbot.m` and `trajecotry.m`) or 64 ms (`ibvs_testing.m` and `block_mover.m`), so each controller is stepped once every 2 or 4 world steps. Keep each pair in a run aligned: Webots accepts the controller call, but the MATLAB controller samples slower than the world integration. This is a controller sampling interval, not a replacement for `basicTimeStep`.
- `bot` is currently assigned the `ibvs_testing` controller in the world. `tarbot` has no controller selected, and the final supervisor `Robot` is assigned `block_mover`.
- `bot`, `tarbot`, and the target use the DEF names `bot`, `tarbot`, and `target`. The target is a 0.15 m × 0.15 m × 0.15 m box at 0.075 m above the arena floor.
- [`Block Texture.png`](./Block%20Texture.png) is the texture applied to both the visible surface and emissive surface of the target. The current `Camera_testing.wbt` references it with `../textures/Block Texture.png`. If the world file is kept directly in this folder, this relative path requires a sibling `textures/` directory containing the image; or the world URL must be updated to `../Block Texture.png` or another valid relative path.

  ![Block target texture](./Block%20Texture.png)
- The texture supplies strong, trackable image features. Replacing or repositioning it can change initial Harris corner detection and later feature matching. Keep the image at a suitable resolution and preserve a clear contrast pattern.
- A world reset returns the target to its initial 0 m/0 m/0.075 m translation, but controller assignments and output paths must still match the run being performed.
- The target and arena appearance are independent of the MATLAB controllers. Edit the world appearance only when deliberately changing lighting or feature geometry; then restart Webots and rerun from an initial state.

### Supplied environment view

The supplied `world snap shot.png` shows the Webots arena, target, and robots from the viewpoint used for the experiment:

![Camera_testing Webots environment screenshot](./world%20snap%20shot.png)

The controllers do not create or load the world. They expect the supplied node names, devices, geometry, and target appearance to exist in the open world.

## Running a Simulation

### Square or approach configuration

1. Open the Webots world in the MATLAB/Webots setup.
2. Assign `ibvs_testing.m` to `bot`.
3. Assign `trajecotry.m` to `tarbot`.
4. Assign `tarbot.m` to a supervisor controller.
5. Set `Approach` in `ibvs_testing.m` and `tarbot.m` to the same value:
   - `1` uses the short, 10-second approach experiment.
   - `0` uses the configured path duration.
6. Set `Straightline = OFF` in `tarbot.m` for the square path.
7. Reset and run the simulation.

### Line configuration

1. Move `tarbot` outside the image-processing area or omit it from the experiment.
2. Assign `ibvs_testing.m` to `bot`.
3. Assign `block_mover.m` to a supervisor controller.
4. Set `Straightline = ON` in both `ibvs_testing.m` and `block_mover.m` for a line.
5. Set `Approach = 0` in `ibvs_testing.m` unless an approach experiment is intended.
6. Reset and run the simulation.

To use a square instead, set `Straightline = OFF` in `ibvs_testing.m` and `block_mover.m`. `block_mover.m` otherwise follows a 1 m forward/backward path.

## Shared settings

Before running a test, open the relevant scripts and check the settings near the top of each file.

| Setting | Used by | Purpose |
| --- | --- | --- |
| `file_path` | All controllers | Absolute output directory. It must exist and must be updated for the local machine. |
| `TIME_STEP` | All controllers | Controller sampling interval passed to `wb_robot_step` and sensor enable calls. It is independent of the world-level `basicTimeStep` (16 ms in the supplied world). Do not confuse it with `dT`, which is just `TIME_STEP / 1000` for MATLAB calculations. |
| `Block_speed` | `ibvs_testing.m`, `block_mover.m`, `tarbot.m` | Target speed in m/s. Change it manually in every controller participating in a run. |
| `Block_Length` | `ibvs_testing.m`, `block_mover.m` | Target path length in metres. |
| `gain` | `ibvs_testing.m` | Scale applied to the IBVS control input. |
| `Straightline` | `ibvs_testing.m`, `block_mover.m`, `tarbot.m` | Selects a line (`ON`) or square (`OFF`). |
| `Approach` | `ibvs_testing.m`, `tarbot.m` | Selects the 10-second approach duration. |
| `needed` | `ibvs_testing.m` | Adds turn and feature-loss markers to plots. Keep it as `0` for the default graphs. |

The scripts use `ON = 1` and `OFF = 0` booleans. `ibvs_testing.m` calculates the default simulation duration from `Block_Length`, `Block_speed`, and `Straightline`, but an explicit approach duration overrides it.

## IBVS settings

`ibvs_testing.m` also exposes these toggles:

| Setting | Effect |
| --- | --- |
| `slip_toggle` | Uses the wheel-kinematic mapping with slip when enabled. |
| `d_bnd_toggle` | Enables or disables the wheel-speed dead band. Disable it to always send the computed speeds. |
| `plot_img` | Updates the live camera/feature view. It must remain `ON` for the initial detection in the current implementation. |
| `plot_error` | Saves point error graphs. |
| `plot_wheel_speeds` | Saves wheel speed and wheel difference graphs. |
| `plot_points` | Saves the image-plane corner paths. |
| `plot_Jp` | Saves the image-Jacobian condition number graph. |
| `norm_e_plot` | Saves the feature-error norm graph. |
| `plot_loss` | Saves a feature-loss graph. |

Four target corners are initially sorted by position and then tracked with Hungarian matching. The source comments describe dynamic output paths as a possible future improvement. At present, `file_path` is hard-coded and the destination directory must already exist.

During the main loop, Hungarian matching is used to associate detected corners with the previous locations. If fewer than four strong features are available, the controller writes invalid samples and continues until the configured duration is complete. This fallback does not cover the initial capture: the initial Harris detection is inside `if plot_img == ON`, so `plot_img` must remain `ON` or the controller will have no initial `strong` data.

## Outputs

At the end of a run, `ibvs_testing.m` exports enabled plots using names derived from `Block_speed`, for example:

- `error_Speed_0.08_mps.png`
- `norm_error_Speed_0.08_mps.png`
- `wheels_Speed_0.08_mps.png`
- `points_Speed_0.08_mps.png`
- `cond_num_Speed_0.08_mps.png`
- `point_lossSpeed_0.08_mps.png`
- `Speed_0.08_mpsIBVS_statistics.csv`

The supervisor controllers export `position_Speed_<speed>_mps.png`. The CSV contains mean and standard-deviation error and Jacobian-condition values selected by the active `idx` expression. Before the run, choose the appropriate time interval in `ibvs_testing.m`: use `idx = t >= 4 & t <= 6` for square or approach experiments, or use `idx = t >= (max(t)/2) & t <= max(t)` for the line configuration. These are alternative lines of code, not settings that run together.

The loss-output implementation is currently unreliable: in `plot_loss == ON`, it clears and exports the existing `cond_num` figure instead of the `loss` figure, and therefore will not necessarily save the feature-loss graph under the listed filename. It is not a general Webots problem.

## Example configuration changes

For a 0.08 m/s straight-line run in `ibvs_testing.m` and `block_mover.m`:

```matlab
file_path = "C:\Users\Kyle\Documents\Uni\Final year\FYP\Data\Plots\example\";
TIME_STEP = 64;
Block_speed = 0.08;
Block_Length = 1;
gain = 2;
Straightline = ON;
Approach = 0;
slip_toggle = ON;
d_bnd_toggle = OFF;
```

Make the same directory and speed available to all participating controllers. `file_path` must point to an existing directory on the machine running MATLAB.

## Comments and code limitations

The source comments identify these current limitations and should be read alongside the README:

- `ibvs_testing.m` says that file output could be made dynamic with directory creation; the README's `file_path` guidance reflects the current hard-coded implementation rather than that proposed improvement.
- `ibvs_testing.m` uses `Z = 1` in the image Jacobian, despite comments noting that an actual target depth could be used instead.
- The source marks `needed = 0` and describes the turn/loss-marker plotting as rudimentary. The README therefore recommends leaving it disabled.
- `ibvs_testing.m` comments that `t >= 4 & t <= 6` is used for square and approach runs, while `t >= (max(t)/2) & t <= max(t)` is an alternative for line runs. Swap these two active `idx` lines depending on the selected configuration.
- `trajecotry.m` retains an explicit note that the cleanup section should write data in the future. This controller does not currently export data or plots.
- `tarbot.m` and `block_mover.m` explicitly describe the tracked block speed as a shared parameter that must be kept synchronized manually. The README also calls out this requirement.
- `block_mover.m` does not contain those dynamic-output comments, but the hard-coded path and `file_path` model are consistent with the matching section in `ibvs_testing.m`.

These are source-specific observations. The general Webots tips below are operational guidance and are not claims made by the controller comments.

## General Webots help tips

- Save the world in Webots before running MATLAB controllers. A controller that appears to use the wrong nodes or settings may be attached to an older unsaved world state.
- Use **Reset** rather than repeatedly editing files between runs. Reset restores node positions and initial field values, although a clean restart may be needed after controller or MATLAB changes.
- Select **Pause** before changing a node's DEF name, supervisor state, camera, or motor configuration. Save the world, then reset and run it again.
- Assign controllers in the robot **Controller** field and inspect each node's properties if a script cannot find a device or DEF name. JavaScript and MATLAB controller fields are separate from the supervisor used to move the target.
- Keep the Webots and MATLAB process architecture (32-bit or 64-bit) compatible with the installed MATLAB integration. Restart MATLAB after changing Webots or MATLAB integration settings.
- If sensor values appear stale, confirm the camera or other sensor is enabled and that `TIME_STEP` is a positive value used consistently by its enable call and simulation.
- Use Webots' **Console** for runtime errors. For MATLAB controller failures, also check MATLAB's Command Window and avoid closing it while a controller is running.
- If output from a controller is slow or the simulation becomes unstable, use the real-time or accelerated run modes cautiously and reduce image/plot toggles for exploratory runs. Enable only the plots required for the experiment.
- Check the Webots world view at the start of a run before relying on tracked feature data. Target occlusion, poor lighting, exposure, or a target outside the camera's field of view can prevent corner detection.
- After changing Webots nodes or controller assignments, restart the simulation rather than assuming an already running controller will reload those changes.
- If MATLAB displays a function-name or API error, confirm that the controller file is being loaded by MATLAB through the Webots integration and that the required MATLAB Toolboxes are installed.

## Troubleshooting

- **The output plot or CSV cannot be written:** create or correct `file_path` before starting the simulation.
- **Controller aborts with a missing node or device:** verify the DEF names (`bot`, `tarbot`, and `target`) and the exact device names in the Webots world.
- **The target does not match the controlled robot:** ensure `Block_speed` and `Straightline` are consistent across all participating controllers.
- **No four corners are detected:** improve target contrast, lighting or scale, or check the live camera image.
- **The initial feature set is wrong:** reset the simulation and allow the initial camera view to settle before starting.
- **Webots stops unexpectedly:** reset the world and rerun. Repeated crashes can occur in some controller/Webots combinations.

For general Webots setup and MATLAB-controller instructions, see the [Webots documentation](https://cyberbotics.com/doc/guide/index).

---

This README was created by GitHub Copilot (AI assistant using Copilot SDK in VS Code).
