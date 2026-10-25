# ACFT11_a_121 — Session change log

Summary of all changes made to `ACFT11_a_121.slx` and the supporting scripts in
this session. Backups were taken at each major step (see **Backups** below).

---

## 1. Gravity consistency in the logged weight / load factor
**Problem:** the flight dynamics (`ecef_eom_fcn.m`) use the correct WGS-84 point-mass
+ J2 gravity in ECEF (verified correct). But `EqM/ExtForcesMoments` computed the
logged `ExtForces_N` / `LoadFactors` using a *flat-earth* constant `g = 9.80665`
pointing straight down in NED — off by ~0.026 m/s² and a small direction error.

**Fix:**
- New file **`ecef_weight_body.m`** — gravity body-axis force using the *same*
  point-mass + J2 model as `ecef_eom_fcn`.
- New `GravityBody` MATLAB Function block in `EqM/ExtForcesMoments`, fed by
  `ECEF_Core` outputs `r_ecef` and `DCM_b2e`; removed the old flat-earth weight
  blocks (gain `g`, `[0;0;mg]` mux, transpose, product) and the now-unused `LBE` inport.

**Result:** logged weight now matches the dynamics gravity to ~1e-10 N. Flight
dynamics (`ECEF_Core`) were **not** touched — trajectory unchanged.

---

## 2. Normal load factor < 2.5 g at aerospike ignition / zoom
**Problem:** at ACCEL→ZOOM the sequencer stepped the flight-path command to 24° at
~M0.95, slamming the pull-up into the 3.0 g limiter (γ-law had *no* Nz limiter).

**Fix:**
- **Pre-pitch during ACCEL** — `MissionManager` ACCEL state now ramps `gamma_cmd`
  toward `FCS_gamma_accel` as Mach builds (`du:` action).
- **Speed-scheduled γ-command rate limiter** in the FCS (`Rate Limiter Dynamic`),
  slew cap = `(FCS_Nz_zoom_tgt-1)*g/V` → bounds Nz by construction.
- **All-phase Nz limiter** (`NzLimiter` MATLAB Function) on the *final* pitch-rate
  command after the law-select switch — protects every phase, not just ACAH.

**Params (`fcs_params.m`):** `FCS_gamma_accel = 12`, `FCS_M_prepitch = 0.80`,
`FCS_Nz_zoom_tgt = 2.0` (tuned down from 2.4 to absorb the inner-loop overshoot).

**Result:** peak Nz through the boost = **2.42 g** (< 2.5).

---

## 3. COAST phase — controllable attitude, no negative AoA
**Problem:** COAST commanded a fixed nose-over while the vehicle was still climbing
past apogee at high qbar → γ-law slammed the elevator to +24.7° and drove α to −8.7°.

**Fix:**
- **COAST/SEPARATE guidance tracks the flight path:** `gamma_cmd = gamma + FCS_alpha_coast`
  (flight-path angle fed into the sequencer as new input `gamma_in`, from `Bus
  Selector1`). Holds a small +AoA along the ballistic arc — no slam, α stays positive.
- **`RCS_ctrl` rewritten:** holds **+3° AoA** (was 0), **4000 N·m per-axis budget**
  (was 50k/120k), lower bandwidth `wn = 0.35`, qbar fade out by ~3000 Pa (light
  low-qbar backup only).
- **1 s aerospike thrust decay** — `ThrustRamp` (Rate Limiter) on the Aerospike
  force, fast rising / 1 s falling, instead of instant cutoff.
- **0.5 s stage-mass jettison ramp** in `MassModel` (removes the inertia step).

**Params (`fcs_params.m`):** `FCS_gamma_coast = 0`, `FCS_alpha_coast = 3`.

**Result:** COAST α stays positive (+6…+16°, 0% negative); elevator at cutoff is
smooth (~4 °/s, ±5°) vs the old +24.7° slam.

---

## 4. Control authority — lateral saturation, actuator rate limits, flutter
**Findings (via the new analysis script):** elevator/aileron/rudder all reach their
deflection limits in places; the glide/approach turns run aileron+rudder to 100%;
commanded surface rates exceeded the actuator rate limits (no rate model); and the
lateral surfaces showed a ~65–77 Hz control limit-cycle ("flutter").

**Fix:**
- **Turn taming:** `FCS_bank_max 45→35°`, `FCS_p_max 60→40 °/s`, `FCS_L1_tc 15→18 s`
  (also updated the hardcoded `L1_tc` inside the `L1law` block).
- **Actuator rate limits enforced INSIDE the INDI loop** — per-step increment
  clamps `RateLim_de/da/dr` (= `FCS_*_rate * FCS_dt`) in `INDI_Inner`, so the loop
  stays aware of the limit (an *external* limiter was tried first and caused a
  max-rate bang-bang / windup — reverted).

**Result:** flutter eliminated (chatter → δe 0 / δa 4 / δr 3 Hz), RMS surface rate
cut ~3×, lateral surfaces quiet outside genuine turns, RCS roll/yaw back to small,
rate limits respected without windup.

**New file: `analyze_control_authority.m`** — run `A = analyze_control_authority(simOut)`
after any sim. Reports per-surface & per-phase deflection %, **deflection rate**
(+ amplitude-gated chatter/flutter detector), **dynamic-pressure sensitivity**
(mean |defl| by qbar bin, qbar at saturation onset), and RCS vs budget; draws the
limit-referenced time histories and a |defl|-vs-qbar plot. Needs the FCS surface
signals logged (`Elevlog/Ailog/Rudlog/RCSlog/PhaseLog/Qbarlog` — logging is saved
in the model).

---

## 5. Glide pitch oscillation (γ-guidance)
**Problem:** a sustained wings-level pitch oscillation (±7°, ~12.7 s) in the glide —
a lightly-damped guidance loop (altitude → `glide_gamma_cmd` → pitch → altitude),
not the turn and not the actuators (elevator used only 16% of its rate).

**Fix (`GlideCompute` + FCS):**
- Softened the `atan` shaping in `GlideCompute` (`half*0.3 → half*1.0`) — lower loop gain.
- Added a continuous **`1/(3s+1)` low-pass** (`GlideGammaLP`) on `glide_gamma_cmd`.
- Also corrected the glide heading command: `glide_psi_cmd = bearing_home` (was
  `bearing_home * gate`, which swept the heading and over-turned).

**Result:** sustained pitch oscillation removed — θ holds steady (~0.5°) in the glide.

---

## 6. LIFTOFF→CLIMB pitch dip + engine spool-up
**Pitch dip (t≈46 s):** at the LIFTOFF→CLIMB1 handoff the pitch law switched from a
fixed 12° climb command to TECS, but the TECS pitch integrator had wound up
open-loop during liftoff → `theta_cmd` jumped 12°→−13° and dived the nose to ~−4°.

**Fix (bumpless transfer in `TECS`):** reset/seed the pitch integrator (`pitch_int`)
while not in the TECS law — routed `law_select` into TECS, reset is `level` on
`(law_select − 1)` (nonzero unless law 1), external IC = `FCS_theta_climb − Kep·eBal`
so `theta_cmd = theta_climb` exactly at takeover. **Result:** θ holds 12° through the
handoff (min θ = 12.0°), no dip.

**Engine spool-up (relight & everywhere):** `PropulsionFCN` produced thrust
instantly from throttle. Added a first-order lag `1/(FCS_tau_spool·s + 1)`
(`SpoolLag`, τ = 2 s) on the throttle into `PropulsionFCN`, so thrust builds up
gradually instead of stepping — at relight and at every throttle change.

**Params (`fcs_params.m`):** `FCS_theta_cmd_rate` (added but reverted — a slew limit
can't fix a sustained wound-up command; left defined, unused), `FCS_tau_spool = 2`.

---

## 7. Attitude visualization (new)
Prior viz was trajectory-only. Added two attitude views driven by the logged
body->ECEF quaternion (`QuatLog`) + ECEF position (`PosLog`) — both signal-logged
in the model and saved to `attitude_data.mat`:
- **`attitude_ribbon.m`** (option E): one static 3-D figure with aircraft glyphs
  oriented by the quaternion, sampled along the path (ENU at the datum), coloured
  by altitude — shows ground/zoom/coast/glide attitude at a glance.
- **`viz_globe_attitude.m`** (option D): extends the `satelliteScenario` Cesium
  globe with an oriented **AirTransport.glb** aircraft mesh (platform `Attitude`
  from body->NED quaternion + `Visual3DModel`); press Play to watch attitude.
- **`run_viz.m`** updated to build `attitude_data.mat` from `so` and call both.
  (If the globe mesh looks mirrored, use `viz_globe_attitude(src,3000,true)`.)

New signal logging: `PosLog` (ECEF_Core r_ecef), `QuatLog` (ECEF_Core q_b2e).

- **`flight_hud.m`**: lightweight side telemetry HUD (dark, HUD-green) — altitude,
  speed, Mach, pitch, roll, heading, flight-path gamma, time — synced to the mission.
  Speed/Mach/gamma are derived from the ECEF position history (no extra logging).
  Sits on the right edge beside the globe; animates (playback-speed arg).
- **`viz_globe_attitude.m`** extended: custom model file + size (`modelFile`,
  `modelScale`; accepts .glb/.gltf/**.stl**), and a **follow camera** (`follow=true`
  -> `camtarget`/`camheight`/`campitch` chase the aircraft).
- **`run_viz.m`** has a top config block and launches ribbon + globe + camera.

Final viz capabilities:
- **`flight_hud.m`** now drives a smooth, fixed-offset **cinematic camera** on the
  Cesium globe (drives the viewer clock per-frame so the chase keeps a CONSTANT
  distance at any altitude — no camtarget zoom-out). Camera shots: `chase`,
  `wing`, `orbit`, `top`, `fpv`, `reveal`, `low`, and `cinematic` (auto-cycles).
  Data downsampled to ~30 Hz and all camera frames precomputed; 30 fps playback.
  The side HUD is optional via the final `showHUD` arg (run_viz passes `false`).
- **`viz_globe_attitude.m`**: model keyframe rate raised to 10 Hz for smoother
  model motion/attitude; custom model (`.glb/.gltf/.stl`), scale, follow, attConj.
- **`run_viz.m`** config: `AIRCRAFT_MODEL`, `AIRCRAFT_SCALE`, `ATT_CONJUGATE`,
  `CAM_MODE`, `CHASE_BACK`, `CHASE_UP`, `PLAY_SPEED`; HUD off.

---

## Open / not yet addressed
- **Glide over-turn:** the glide still holds ~160 s at the 35° bank limit (>360° of
  turning) — a heading/L1 loop issue, separate from the γ damping above.
- **Glide-entry pitch transient:** θ dips to ~−17° at the COAST→GLIDE handoff, then settles.
- **Rollout solver failure:** the mission fails to converge at **t≈1854 s during
  rollout** (stiff gear/ground contact after touchdown) — numerical, not control.
- **Lateral deflection in turns:** aileron/rudder still reach 100% during genuine
  glide/approach turns (now rate-limited & flutter-free, but no deflection margin).
- Note: scalar `Cn_rud = 0` in `fcs_params.m` is legacy/unused — the real rudder
  effectiveness comes from the `Cn_rudder` lookup table in the aero block (healthy).

---

## Backups (pre-change snapshots of the model)
- `ACFT11_a_121_pregravity_backup.slx` — before §1
- `ACFT11_a_121_prenz_backup.slx` — before §2
- `ACFT11_a_121_precoast_backup.slx` — before §3

## New / modified files
- **New:** `ecef_weight_body.m`, `analyze_control_authority.m`, this file
- **Modified:** `ACFT11_a_121.slx`, `fcs_params.m`
