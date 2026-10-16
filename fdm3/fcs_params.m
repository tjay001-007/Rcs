%% fcs_params.m
%  Central parameter set for the ACFT11 full-mission Flight Control System.
%  Run this BEFORE setup_fcs.m and before every simulation.
%  Every gain, limit, and effectiveness value the FCS uses lives here —
%  this is the single place to tune the controller.
%
%  Aircraft: supersonic UAV, ~28.5 t class, S=85.25 m2, b=9.05 m, c=15.55 m,
%            envelope M0.4-M4.  Effectiveness values taken from the aero
%            lookup tables (per radian, evaluated at alpha=2 deg).
% -----------------------------------------------------------------------
fprintf('>> Loading FCS parameters...\n');

%% ---- Sample time -------------------------------------------------------
FCS_dt = 0.01;              % s   controller update (100 Hz)

%% ---- Reference geometry & inertia (from setup_landing_gear.m / model) --
FCS_S   = 85.25;           % m^2  wing area
FCS_b   = 9.05;            % m    reference span   (roll/yaw moment arm)
FCS_c   = 15.55;           % m    reference chord  (pitch moment arm)
FCS_Ixx = 74848;           % kg m^2
FCS_Iyy = 288344;          % kg m^2
FCS_Izz = 390125;          % kg m^2
FCS_Ixz = 2511;            % kg m^2
% (if setup_landing_gear reassigned inertia for a heavier config, pull it)
if evalin('base','exist(''Ixx'',''var'')'), FCS_Ixx = evalin('base','Ixx'); end
if evalin('base','exist(''Iyy'',''var'')'), FCS_Iyy = evalin('base','Iyy'); end
if evalin('base','exist(''Izz'',''var'')'), FCS_Izz = evalin('base','Izz'); end
if evalin('base','exist(''Ixz'',''var'')'), FCS_Ixz = evalin('base','Ixz'); end

%% ---- Control effectiveness (per RAD), nominal @ alpha=2 deg -----------
%  From aero tables Cm_elev / Cl_AilT / Cn_rudder.  These scale the INDI
%  G-matrix.  Dynamic-pressure scaling is done LIVE inside the FCS;
%  these are the derivative magnitudes at the low-speed anchor (M0.4).
%  For full scheduling, replace with the (alpha,Mach) lookup later.
FCS_Cm_de = -0.144;        % /rad  d(Cm)/d(elevator)
FCS_Cl_da = -0.126;        % /rad  d(Cl)/d(aileron)
FCS_Cn_dr = -0.059;        % /rad  d(Cn)/d(rudder)

% Lower bounds on |G| (avoid divide-by-zero at very low q-bar / high alt)
FCS_Gmin_q = 0.02;         % rad/s^2 per rad, pitch
FCS_Gmin_p = 0.02;         % roll
FCS_Gmin_r = 0.01;         % yaw

% Control-authority fade-in vs dynamic pressure (anti-windup at low airspeed).
% The INDI increment is scaled by fade = sat((qbar-lo)/(hi-lo),0,1), so the
% surface loops carry no authority (and cannot wind up) when controls are
% ineffective, and fade in smoothly as speed builds.
FCS_qbar_lo = 1000.0;      % Pa  (~40 m/s at SL) fade starts
FCS_qbar_hi = 6000.0;      % Pa  (~99 m/s at SL) full authority

%% ---- INDI inner-loop rate gains (desired rate-loop bandwidth, 1/s) -----
FCS_Kp = 6.0;              % roll-rate  loop
FCS_Kq = 5.0;              % pitch-rate loop
FCS_Kr = 4.0;              % yaw-rate   loop
FCS_Nfilt = 20.0;          % rad/s  angular-accel washout/derivative filter BW

%% ---- Attitude (ACAH) outer-loop gains ---------------------------------
FCS_Ktheta = 2.0;          % 1/s  pitch attitude -> q_cmd
FCS_Kphi   = 3.0;          % 1/s  bank  attitude -> p_cmd
FCS_Kbeta  = 2.0;          % 1/s  sideslip -> r_cmd (turn-coord / yaw)
FCS_yawdamp_wo = 1.0;      % rad/s  yaw-damper washout pole

%% ---- Envelope protection limits ---------------------------------------
FCS_alpha_prot = 12.0;     % deg  soft AoA limit (start pushing nose down)
FCS_alpha_max  = 16.0;     % deg  hard AoA limit
FCS_Nz_max     = 3.0;      % g    max load factor
FCS_Nz_min     = -1.0;     % g    min load factor
FCS_bank_max   = 45.0;     % deg  normal bank limit
FCS_bank_hard  = 60.0;     % deg  hard bank limit

%% ---- Rate & command saturations ---------------------------------------
FCS_q_max = 15.0;          % deg/s  max pitch-rate command
FCS_p_max = 60.0;          % deg/s  max roll-rate  command
FCS_r_max = 15.0;          % deg/s  max yaw-rate   command
FCS_theta_max = 30.0;      % deg
FCS_theta_min = -20.0;     % deg

%% ---- Actuator limits (deg) & rate limits (deg/s) ----------------------
FCS_de_max = 25.0;   FCS_de_rate = 80.0;   % elevator
FCS_da_max = 20.0;   FCS_da_rate = 100.0;  % aileron
FCS_dr_max = 25.0;   FCS_dr_rate = 80.0;   % rudder

%% ---- TECS (total energy) longitudinal autopilot -----------------------
FCS_V_cmd     = 250.0;     % m/s  default airspeed target (~M0.75 @ SL-ish)
FCS_h_cmd     = 3000.0;    % m    default altitude target
FCS_TECS_Kti  = 0.10;      % throttle integral gain (energy rate)
FCS_TECS_Ktp  = 0.30;      % throttle prop gain
FCS_TECS_Kei  = 0.08;      % elevator/pitch integral gain (energy balance)
FCS_TECS_Kep  = 0.25;      % pitch prop gain
FCS_thr_min   = 0.0;   FCS_thr_max = 1.0;   % idle thrust floor = 0 so TECS can cut thrust fully for descent/landing

%% ---- Lateral guidance (L1 lateral-acceleration, replaces heading-hold) -
FCS_psi_cmd   = 0.0;       % deg  default heading target
FCS_Kpsi      = 1.2;       % deg bank per deg heading error (legacy heading-hold)
FCS_turn_bank = 30.0;      % deg  standard turn bank for large errors
FCS_L1_tc     = 15.0;      % s    L1 look-ahead time constant: a_lat=(2V/L1_tc)*sin(eta)
                           %      (smaller = tighter/faster turns; mirrors value hardcoded in L1law)

%% ---- Remote-pilot RCAH (attitude-command / attitude-hold) -------------
FCS_stick_detent   = 0.03;     % |stick| below this = centered = HOLD
FCS_stick_q_scale  = 15.0;     % deg/s per unit pitch-stick (out of detent)
FCS_stick_p_scale  = 60.0;     % deg/s per unit roll-stick
% Attitude reference clamps reuse envelope limits above.

%% ---- Envelope-protection command gains & aux constants ----------------
FCS_g       = 9.80665;    % m/s^2
FCS_Vmin    = 60.0;       % m/s   min speed for divide-protection
FCS_Kalpha  = 3.0;        % deg/s per deg   AoA-limiter command gain
FCS_Kalpha_lim = 2.0;     % deg/s per deg   gamma-law AoA limiter: q_cmd <= Kalpha_lim*(alpha_max-alpha)
FCS_Knz     = 20.0;       % deg/s per g     Nz-limiter command gain

%% ---- TECS auxiliary ----------------------------------------------------
FCS_Kh        = 0.10;     % 1/s   altitude error -> climb-rate demand
FCS_hdot_max  = 30.0;     % m/s   climb-rate demand limit
FCS_thr_trim  = 0.50;     % -     throttle trim (TECS integrator seed)
FCS_theta_trim= 0.0;      % deg   pitch trim

%% ---- Mode manager / datalink failsafe --------------------------------
FCS_MODE_AUTO      = 0;        % enum
FCS_MODE_STABILIZE = 1;
FCS_link_timeout   = 1.0;      % s   command age -> lost-link -> continue mission

%% =====================================================================
%  MISSION MANAGER (Stateflow phase sequencer)
%% =====================================================================
%  Longitudinal-law selector (law_select values on the GuidanceRef bus)
FCS_LAW_GROUND    = 0;   % on-ground: q_cmd=q_rot_cmd, throttle=thr_cmd
FCS_LAW_AIR       = 1;   % airborne : TECS + ACAH (speed/altitude managed)
FCS_LAW_FLARE     = 2;   % flare    : idle throttle, pitch to theta_flare
FCS_LAW_PITCHHOLD = 3;   % initial climb: hold theta_climb via ACAH, throttle MAX
FCS_theta_climb   = 12.0; % deg  initial-climb pitch attitude (pitch-hold law)

%  Phase enums (for logging / scopes) — full two-stage carrier demo (17 phases)
PH_GROUNDROLL=1; PH_ROTATION=2; PH_LIFTOFF=3; PH_CLIMB1=4; PH_CRUISE1=5;
PH_ACCEL=6; PH_ZOOM=7; PH_SEPARATE=8; PH_GLIDE=9; PH_RELIGHT=10;
PH_CRUISE_RTB=11; PH_DESCEND=12; PH_APPROACH=13; PH_FLARE=14;
PH_TOUCHDOWN=15; PH_ROLLOUT=16; PH_STOPPED=17;

%  ---- Transition thresholds & phase targets -------------------------
FCS_V_rotate   = 94.0;   % m/s   DEFAULT rotation speed (Vr is a runtime INPUT;
                          %       this only seeds the root Vr_cmd constant).
                          %       High wing-loading jet: needs ~137 m/s to fly
                          %       at 10deg AoA, so Vr set well above stall margin.
FCS_theta_LOF  = 10.0;    % deg   liftoff pitch attitude
FCS_q_rot      = 5.0;     % deg/s rotation pitch-rate command
FCS_WoW_thresh = 100.0;   % N     gear normal force below this = airborne
FCS_h_gearsafe = 50.0;    % m     gear-up / climb-established altitude
FCS_V_climb    = 180.0;   % m/s   climb speed target
FCS_h_cruise   = 6000.0;  % m     cruise altitude
FCS_V_cruise   = 180.0;   % m/s   cruise speed
FCS_V_desc     = 120.0;   % m/s   descent / steep-segment speed (decel to this on the platform leg, carried onto the 5deg segment)
FCS_h_approach = 500.0;   % m     approach-capture altitude
FCS_h_platform = 800.0;   % m     RTB level-off platform altitude (glide-slope intercept from below)
FCS_loc_tol    = 1000.0;  % m     max cross-track error to allow glide-slope capture (localizer aligned)
FCS_gs_break   = 3000.0;  % m     along-track break between steep (4deg) and shallow (2.5deg) glide-slope segments
FCS_v_decel_ref = 6000.0; % m     along-track where approach speed eases V_desc->V_app (completes by ~4 km, so the shallow final <3 km is flown at constant V_app)
FCS_V_app      = 100.0;   % m/s   approach speed
FCS_h_flare    = 30.0;    % m     flare-initiation altitude (AGL gate) - start arrest higher/gentler
FCS_theta_flare= 6.0;     % deg   flare target pitch (legacy fixed-attitude flare)
FCS_Ktheta_flr = 0.5;     % 1/s   flare pitch gain (legacy)
FCS_hdot_appr  = 3.0;     % m/s   approach sink-rate (h_cmd ramp)
% --- exponential sink-rate flare (soft-landing law) -------------------
FCS_h_gnd      = 2.24;    % m     altitude of main-gear ground contact (alt datum)
FCS_tau_flare  = 8.0;     % s     flare time-constant: hdot_cmd = -(h_agl)/tau
FCS_hdot_td    = 0.3;     % m/s   commanded touchdown sink-rate floor (soft)
FCS_Kflare     = 1.0;     % (deg/s)/(m/s)  sink-rate error -> pitch-rate gain (reduced: 3.0 porpoised/over-rotated)
FCS_mu_brake   = 0.30;    % -     rollout braking friction (mu_cmd = 0.3 full brakes)
FCS_mu_roll    = 0.03;    % -     rolling friction (no brakes)
FCS_runway_hdg = 0.0;     % deg   runway heading
FCS_V_derotate = 60.0;    % m/s   speed to lower nose after touchdown
FCS_V_stop     = 2.0;     % m/s   speed considered stopped

%% =====================================================================
%  HIGH-SPEED MISSION — GN&C stack (Sequencer / Guidance / Estimation)
%% =====================================================================
%  Guidance-mode enums (MODE bus: Sequencer -> Guidance)
GM_GROUND=0; GM_CLIMB=1; GM_ACCEL=2; GM_ZOOM=3; GM_COAST=4; ...
GM_GLIDE=5; GM_RELIGHT=6; GM_CRUISE=7; GM_APPROACH=8;

%  Propulsion-mode enums (MODE bus: Sequencer -> Propulsion Manager)
PM_OFF=0;            % all engines off (coast)
PM_AB_DRY=1;         % airbreathing, dry (no afterburner)
PM_AB_WET=2;         % airbreathing + afterburner
PM_AEROSPIKE=3;      % aerospike (rocket) only
PM_AEROSPIKE_AB=4;   % aerospike + airbreathing AB (combined boost)
PM_IDLE=5;           % airbreathing idle (ground/rollout)

%  New FCS longitudinal law: flight-path (gamma) hold  (zoom / coast / glide)
FCS_LAW_GAMMA = 4;   % q_cmd from K_gamma*(gamma_cmd - gamma); throttle from guidance

%  ---- Mission altitude / Mach set-points ----------------------------
FCS_h_ab_reignite = 6000.0;   % m    climb->accel: reignite afterburner
FCS_M_target      = 2.4;      % -    target separation Mach (demo profile)
FCS_h_apogee_tgt  = 27000.0;  % m    separation altitude: ZOOM->SEPARATE triggers at h>=this
FCS_h_relight     = 10000.0;  % m    glide: airbreathing relight altitude (relight while still fast, M>0.5)
FCS_Kgamma        = 1.5;      % 1/s  flight-path error -> q_cmd gain

%  ---- Sequencer phase-transition set-points (two-stage carrier demo) --
FCS_M_trans       = 0.95;     % -    accel->zoom: Mach at which the zoom climb starts
FCS_gamma_zoom    = 24.0;     % deg  zoom climb flight-path (tuned: 250 kN + 24deg -> ~27 km @ M2.27 at separation)
FCS_gamma_glide   = -4.0;     % deg  gliding-descent flight-path (legacy fixed value; GLIDE now energy-managed)
%  Glide energy-management: gamma points at relight target (home, h_relight), clamped:
FCS_gamma_glide_shallow = 3.0;  % deg  shallowest glide (best-glide / max homeward range when far)
FCS_gamma_glide_steep   = 18.0; % deg  steepest glide (dissipate energy when high/close to home)
%  High-altitude COAST (reentry-style): above FCS_h_coast the air is too thin for
%  aero control, so fly a gentle ballistic nose-over instead of the aggressive
%  energy-managed glide; hand off to the energy glide once below FCS_h_coast.
FCS_qbar_glide  = 5000.0;  % Pa   COAST->energy-glide handoff q-bar (adequate aero control authority)
FCS_gamma_coast = -8.0;    % deg  gentle ballistic nose-over flight-path during low-q-bar COAST
FCS_R_capture     = 100000.0; % m    RTB: range-to-home to begin the platform descent (CRUISE_RTB -> DESCEND_RTB); needs ~76 km to reach platform at the ~3.9deg idle glide
FCS_gs_capture_tol = 50.0;    % m    glide-slope beam intercept tolerance (capture when gs_alt_cmd <= h + tol)
FCS_t_cruise1     = 20.0;     % s    dwell at 6 km cruise before arming the boost
FCS_t_sep         = 2.0;      % s    settle time held after stage separation before glide
FCS_t_relight     = 5.0;      % s    airbreathing spool-up time after relight before cruise
FCS_t_burn        = 45.0;     % s    FIXED aerospike burn time in ZOOM -> separation at burnout (max-Mach ascent goal)
FCS_m_stage       = 3000.0;   % kg   scaled second-stage mass dropped at separation (plant stub)
home_N            = 0.0;      % m    start-point North coord (return-to-base origin)
home_E            = 0.0;      % m    start-point East  coord (return-to-base origin)

%  ---- Envelope / constraint limits  (>>> PLACEHOLDERS — need real data) ----
FCS_qbar_max   = 50000.0;  % Pa   structural dynamic-pressure limit   <<< TBD
FCS_qbar_min_fly = 1000.0; % Pa   min q-bar for useful aero control    <<< TBD
%  Airbreathing relight box (glide must arrive inside this to relight):
FCS_relight_M_lo = 0.35;   % -    min Mach for airbreathing relight  (matched to ~10 km glide: M~0.46)
FCS_relight_M_hi = 2.0;    % -    max Mach for airbreathing relight
FCS_relight_qbar = 3000.0; % Pa   min q-bar for airbreathing relight  (matched to ~10 km glide: q~3800 Pa)

%  ---- Propulsion performance  (>>> PLACEHOLDERS — need real data) --------
%  These are consumed by the Propulsion Manager / engine models (step 3).
FCS_AB_boost_factor = 1.5; % -    afterburner thrust multiplier vs dry  <<< TBD
FCS_aerospike_thrust= 250000.0; % N  aerospike FIXED thrust (on/off only, NO throttle) <<< TBD
FCS_aerospike_Isp   = 350.0;    % s  aerospike specific impulse         <<< TBD
%  NOTE: aerospike runs at fixed max thrust — commanded on/off via prop_mode,
%  never throttled. Zoom trajectory is shaped by pitch/gamma, not thrust.

%% =====================================================================
%  MASS / INERTIA SCHEDULE  (multi-point interpolation)
%% =====================================================================
%  The carrier's mass and inertia change throughout the mission due to
%  fuel consumption and orbiter separation.  Define values at key flight
%  points; the MassModel block interpolates in real time.

MASS_full       = 28500;     % kg   fully loaded (fuel + orbiter + structure)
MASS_empty      = 10800;     % kg   carrier dry (no fuel, no orbiter)
MASS_orbiter    = 3000;      % kg   second-stage orbiter + adapter
MASS_fuel_total = MASS_full - MASS_empty - MASS_orbiter;  % ~14700 kg

% Fuel flow rates [kg/s] by propulsion mode (PM_* enums)
FF_rates = [0.0, ...   % PM_OFF      (0): coast / glide
            2.5, ...   % PM_AB_DRY   (1): military dry thrust
            5.0, ...   % PM_AB_WET   (2): afterburner
           73.0, ...   % PM_AEROSPIKE(3): 250 kN / (g*350s) ~ 73 kg/s
           78.0, ...   % PM_AEROSPIKE_AB(4): combined boost
            0.5];      % PM_IDLE     (5): ground idle / rollout

% Inertia at key mass points [Ixx, Iyy, Izz, Ixz]  (kg*m^2)
%   Full:       takeoff configuration (fuel + orbiter aboard)
%   Separated:  orbiter just jettisoned; still have some fuel
%   Empty:      dry carrier after landing
INERTIA_full      = [74848, 288344, 390125, 2511];
INERTIA_separated = [52000, 200000, 268000, 1800];
INERTIA_empty     = [32379, 136386, 168765, 2511];

% Breakpoints for inertia lookup (ascending mass)
MASS_bp      = [MASS_empty,        MASS_full - MASS_orbiter, MASS_full];
IXX_tbl      = [INERTIA_empty(1),  INERTIA_separated(1),    INERTIA_full(1)];
IYY_tbl      = [INERTIA_empty(2),  INERTIA_separated(2),    INERTIA_full(2)];
IZZ_tbl      = [INERTIA_empty(3),  INERTIA_separated(3),    INERTIA_full(3)];
IXZ_tbl      = [INERTIA_empty(4),  INERTIA_separated(4),    INERTIA_full(4)];

%% =====================================================================
%  APPROACH PATH GUIDANCE  (ILS-style localizer + glide slope)
%% =====================================================================
%  Runway reference: threshold at NED origin (home_N, home_E), with
%  runway heading FCS_runway_hdg.  The approach path extends behind
%  the threshold along the runway centerline.

FCS_GS_angle       = 3.0;     % deg   glide-slope angle
FCS_FAF_dist       = 10000.0; % m     final-approach-fix distance from threshold
FCS_app_capture_R  = 20000.0; % m     range to switch from RTB to localizer capture
FCS_Kloc           = 0.8;     % deg heading correction per m cross-track error (capped)
FCS_loc_max_corr   = 20.0;    % deg   max localizer heading correction

fprintf('>> FCS parameters loaded.  dt=%.3f s  |  Kp/Kq/Kr = %.1f/%.1f/%.1f\n', ...
    FCS_dt, FCS_Kp, FCS_Kq, FCS_Kr);
fprintf('>> G-matrix anchors (/rad): Cm_de=%.3f  Cl_da=%.3f  Cn_dr=%.3f\n', ...
    FCS_Cm_de, FCS_Cl_da, FCS_Cn_dr);
