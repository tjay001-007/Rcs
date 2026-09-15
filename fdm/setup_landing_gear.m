%% setup_landing_gear.m
%  Generic landing gear configurator for ACFT11_a_121.slx
%
%  HOW IT WORKS
%  ─────────────────────────────────────────────────────────────────────
%  1. You fill in the USER CONFIGURATION block below.
%  2. The script computes:
%       • Gear arm vectors from CG   (body frame: x fwd, y right, z DOWN)
%       • Static load per gear       (longitudinal equilibrium)
%       • Spring stiffness k         k = N_static / delta_static_target
%       • Damper coefficient c       c = 2*zeta*sqrt(k * m_gear)
%       • CG height IC               h_CG = R(3,:)*r_MLG - delta_static
%  3. Injects new LandingGear_FCN code into the Simulink model.
%  4. Sets Simulink ICs (ZI_m, theta, u=0).
%  5. Sets Mass / Inertia in MATLAB base workspace for FCS scripts.
%  6. Saves model.
%
%  SIGN CONVENTIONS (body frame)
%  ─────────────────────────────────────────────────────────────────────
%  x  positive = FORWARD      (away from tail)
%  y  positive = RIGHT        (starboard)
%  z  positive = DOWNWARD     (standard aerospace NED body)
%  All positions measured from CG to contact point.
%
%  CG position is entered as distance from nose along fuselage (positive).
%  Gear positions likewise from nose.
% -----------------------------------------------------------------------

%% =====================================================================
%  USER CONFIGURATION  ← edit ONLY this section
%% =====================================================================

%  ── Aircraft mass & inertia ──────────────────────────────────────────
mass_kg = 28500;   % kg   total aircraft mass (24650 = original)

%  Inertia tensor (if you don't have updated values, scale from original
%  by the mass ratio as a first approximation):
mass_orig = 24650;
scale_I   = mass_kg / mass_orig;          % proportional scaling

Ixx = 74848  * scale_I;   % kg·m²  roll
Iyy = 288344 * scale_I;   % kg·m²  pitch
Izz = 390125 * scale_I;   % kg·m²  yaw
Ixz = 2511   * scale_I;   % kg·m²  cross-product

%  ── CG position ──────────────────────────────────────────────────────
xcg_from_nose_m = 9.87;    % m  CG distance from nose, along fuselage

%  ── Ground attitude ──────────────────────────────────────────────────
theta_gnd_deg = 2.0;       % deg  nose-up ground pitch angle

%  ── Nose Landing Gear (NLG) ──────────────────────────────────────────
xnlg_from_nose_m = 3.110;  % m  NLG axle x-position from nose
ynlg_m           = 0.0;    % m  lateral offset (0 = centreline, typical)
znlg_strut_m     = 2.558;  % m  CG to contact-point, body z-DOWN (rest length)

%  ── Main Landing Gear (MLG, left+right) ──────────────────────────────
xmlg_from_nose_m = 10.885; % m  MLG axle x-position from nose
ymlg_half_m      = 2.300;  % m  half-track (lateral distance, one side)
zmlg_strut_m     = 2.286;  % m  CG to contact-point, body z-DOWN (rest length)

%  ── Spring & Damper Design ───────────────────────────────────────────
delta_static_m = 0.080;    % m  target static strut deflection
                            %    Typical jet transport: 0.06–0.12 m
zeta_damping   = 5.0;      % –  damping ratio (1 = critical, 5 = oleo style)
                            %    Real oleo-pneumatic: 4–6 × critical

%  ── Friction ─────────────────────────────────────────────────────────
mu_roll  = 0.03;           % –  rolling friction
mu_brake = 0.30;           % –  full-brake friction (set by FCS in BFL mode)

%% =====================================================================
%  END USER CONFIGURATION
%% =====================================================================

g_n   = 9.80665;
W     = mass_kg * g_n;
theta = theta_gnd_deg * pi/180;

%% ── 1. Gear arm vectors (body frame: CG → contact point) ─────────────
%   x_arm = xcg - xgear  (positive = gear is FORWARD of CG)
r_NLG_x = xcg_from_nose_m - xnlg_from_nose_m;   % positive for NLG
r_MLG_x = xcg_from_nose_m - xmlg_from_nose_m;   % negative for MLG (aft)

r_NLG   = [r_NLG_x;        0;          znlg_strut_m];
r_MLG_L = [r_MLG_x;  -ymlg_half_m;     zmlg_strut_m];
r_MLG_R = [r_MLG_x;  +ymlg_half_m;     zmlg_strut_m];

fprintf('\n');
fprintf('=================================================================\n');
fprintf('  LANDING GEAR CONFIGURATOR\n');
fprintf('  Aircraft mass : %.0f kg  (W = %.0f N)\n', mass_kg, W);
fprintf('=================================================================\n');
fprintf('\n  Gear arm vectors from CG (body: x-fwd, y-right, z-down)\n');
fprintf('    NLG  : [%+.3f, %+.3f, %+.3f] m\n', r_NLG(1),   r_NLG(2),   r_NLG(3));
fprintf('    MLG-L: [%+.3f, %+.3f, %+.3f] m\n', r_MLG_L(1), r_MLG_L(2), r_MLG_L(3));
fprintf('    MLG-R: [%+.3f, %+.3f, %+.3f] m\n', r_MLG_R(1), r_MLG_R(2), r_MLG_R(3));

%% ── 2. Rotation matrix at ground attitude (phi=0, psi=0) ─────────────
%   R_BE row-3 = [-sin(theta), 0, cos(theta)]  (maps body→NED z-down)
R3 = [-sin(theta), 0, cos(theta)];   % only row we need for vertical projection

%% ── 3. Static load distribution (longitudinal equilibrium) ───────────
%   Taking moments about MLG (positive x forward):
%   N_NLG * L = W * |r_MLG_x|    → N_NLG = W * |r_MLG_x| / L
L_wheelbase = r_NLG_x + abs(r_MLG_x);   % longitudinal wheelbase [m]

N_NLG_static   = W * abs(r_MLG_x) / L_wheelbase;
N_MLG_total    = W * r_NLG_x      / L_wheelbase;
N_MLG_L_static = N_MLG_total / 2;
N_MLG_R_static = N_MLG_total / 2;

frac_NLG = N_NLG_static / W * 100;
frac_MLG = N_MLG_total  / W * 100;

fprintf('\n  Static load distribution (phi=0, theta=%g deg)\n', theta_gnd_deg);
fprintf('    Wheelbase (NLG–MLG): %.3f m\n', L_wheelbase);
fprintf('    CG split — NLG: %.1f%%   MLG total: %.1f%%\n', frac_NLG, frac_MLG);
fprintf('    N_NLG           = %.0f N  (%.0f kg)\n', N_NLG_static,   N_NLG_static/g_n);
fprintf('    N_MLG each      = %.0f N  (%.0f kg)\n', N_MLG_L_static, N_MLG_L_static/g_n);

%% ── 4. Spring stiffness ──────────────────────────────────────────────
k_NLG = N_NLG_static   / delta_static_m;
k_MLG = N_MLG_L_static / delta_static_m;

%% ── 5. Damping coefficients ──────────────────────────────────────────
%   Critical damping: c_cr = 2 * sqrt(k * m_gear)
%   m_gear = portion of aircraft mass supported by that gear
m_NLG_eff = N_NLG_static   / g_n;
m_MLG_eff = N_MLG_L_static / g_n;

c_cr_NLG = 2 * sqrt(k_NLG * m_NLG_eff);
c_cr_MLG = 2 * sqrt(k_MLG * m_MLG_eff);

c_NLG = zeta_damping * c_cr_NLG;
c_MLG = zeta_damping * c_cr_MLG;

% Round to nearest 100 for clean injection
k_NLG_r = round(k_NLG / 100) * 100;
k_MLG_r = round(k_MLG / 100) * 100;
c_NLG_r = round(c_NLG / 100) * 100;
c_MLG_r = round(c_MLG / 100) * 100;

omega_n_NLG = sqrt(k_NLG / m_NLG_eff);   % rad/s natural freq
omega_n_MLG = sqrt(k_MLG / m_MLG_eff);

fprintf('\n  Spring & Damper values  (δ_target=%.3f m, ζ=%.1f)\n', delta_static_m, zeta_damping);
fprintf('    k_NLG = %7.0f N/m   c_NLG = %7.0f N·s/m\n', k_NLG_r, c_NLG_r);
fprintf('    k_MLG = %7.0f N/m   c_MLG = %7.0f N·s/m\n', k_MLG_r, c_MLG_r);
fprintf('    ω_n_NLG = %.2f rad/s  (%.2f Hz)\n', omega_n_NLG, omega_n_NLG/(2*pi));
fprintf('    ω_n_MLG = %.2f rad/s  (%.2f Hz)\n', omega_n_MLG, omega_n_MLG/(2*pi));
fprintf('    ζ_NLG = %.1f  (c/c_cr = %.0f/%.0f)\n', c_NLG/c_cr_NLG, c_NLG_r, round(c_cr_NLG));
fprintf('    ζ_MLG = %.1f  (c/c_cr = %.0f/%.0f)\n', c_MLG/c_cr_MLG, c_MLG_r, round(c_cr_MLG));

%% ── 6. Initial condition: CG height ──────────────────────────────────
%   At static equilibrium on flat ground, MLG compression = delta_static
%   h_CG = R3 * r_MLG  (zero-compression CG height)  minus delta_static
%   ZI_m (NED z positive DOWN) = -h_CG

h_CG_zero_MLG = R3 * r_MLG_L;      % CG height when MLG just touches (zero compression)
h_CG_zero_NLG = R3 * r_NLG;        % same check for NLG

% Actual static compressions at h_CG_IC
h_CG_IC = h_CG_zero_MLG - delta_static_m;
ZI_m_IC = -h_CG_IC;

d_MLG_IC = h_CG_zero_MLG - h_CG_IC;   % = delta_static
d_NLG_IC = h_CG_zero_NLG - h_CG_IC;

N_NLG_IC   = k_NLG_r * d_NLG_IC;
N_MLG_L_IC = k_MLG_r * d_MLG_IC;
err_pct = (N_NLG_IC + 2*N_MLG_L_IC - W) / W * 100;

fprintf('\n  Initial Conditions  (static equilibrium on flat ground)\n');
fprintf('    h_CG_zero (MLG just touching) = %.4f m\n', h_CG_zero_MLG);
fprintf('    h_CG_zero (NLG just touching) = %.4f m\n', h_CG_zero_NLG);
if abs(h_CG_zero_MLG - h_CG_zero_NLG) > 0.05
    fprintf('    *** WARNING: NLG and MLG give inconsistent h_CG — check znlg_strut_m\n');
end
fprintf('    h_CG_IC  (at static load)     = %.4f m\n', h_CG_IC);
fprintf('    ZI_m_IC  (Simulink NED z)     = %.5f m\n', ZI_m_IC);
fprintf('    MLG compression at IC         = %.4f m  (target %.4f m)\n', d_MLG_IC, delta_static_m);
fprintf('    NLG compression at IC         = %.4f m\n', d_NLG_IC);
fprintf('    N_NLG  at IC = %.0f N  (static = %.0f N)\n', N_NLG_IC,   N_NLG_static);
fprintf('    N_MLG_L at IC = %.0f N  (static = %.0f N)\n', N_MLG_L_IC, N_MLG_L_static);
fprintf('    Total N vs W: err = %+.2f%%\n', err_pct);

%% ── 7. Build LandingGear_FCN code string ────────────────────────────
lg_code = sprintf([...
'function [GndForces, N_total, N_gears] = LandingGear_FCN(EulerAngles_rad, VB_mps, BodyRates_radps, pos_NED_m, mass_kg, mu_cmd)\n'...
'%%#codegen\n'...
'%% Landing gear: spring-damper, one-sided damper (no tension).\n'...
'%% Body frame: x-fwd, y-right, z-DOWN.\n'...
'%% Auto-generated by setup_landing_gear.m\n'...
'%%   mass_kg = %.0f  |  delta_static = %.4f m  |  zeta = %.1f\n'...
'%% --- Gear arm vectors from CG [x_fwd, y_right, z_down] m -----------\n'...
'r_NLG   = [%.4f; %.4f; %.4f];\n'...
'r_MLG_L = [%.4f; %.4f; %.4f];\n'...
'r_MLG_R = [%.4f; %.4f; %.4f];\n'...
'%% --- Spring-damper constants ----------------------------------------\n'...
'k_NLG = %.0f;   c_NLG = %.0f;\n'...
'k_MLG = %.0f;   c_MLG = %.0f;\n'...
'mu    = mu_cmd;   %% 0.03 rolling | 0.3 full brakes (from FCS)\n'...
'%% --- Unpack states --------------------------------------------------\n'...
'phi=EulerAngles_rad(1); theta=EulerAngles_rad(2); psi=EulerAngles_rad(3);\n'...
'cph=cos(phi); sph=sin(phi); cth=cos(theta); sth=sin(theta);\n'...
'cps=cos(psi); sps=sin(psi);\n'...
'R_BE=[cth*cps, sph*sth*cps-cph*sps, cph*sth*cps+sph*sps;\n'...
'      cth*sps, sph*sth*sps+cph*cps, cph*sth*sps-sph*cps;\n'...
'     -sth,     sph*cth,              cph*cth];\n'...
'h_CG = -pos_NED_m(3);   %% NED z positive-down; altitude = -z_NED\n'...
'%% --- Gear compressions (positive = wheel penetrating ground) --------\n'...
'd_NLG   = (R_BE(3,:)*r_NLG)   - h_CG;\n'...
'd_MLG_L = (R_BE(3,:)*r_MLG_L) - h_CG;\n'...
'd_MLG_R = (R_BE(3,:)*r_MLG_R) - h_CG;\n'...
'on_NLG   = double(d_NLG   > 0);\n'...
'on_MLG_L = double(d_MLG_L > 0);\n'...
'on_MLG_R = double(d_MLG_R > 0);\n'...
'%% --- Gear-tip velocity in NED (z-component = compression rate) ------\n'...
'vn_NLG   = R_BE*(VB_mps+cross(BodyRates_radps,r_NLG));\n'...
'vn_MLG_L = R_BE*(VB_mps+cross(BodyRates_radps,r_MLG_L));\n'...
'vn_MLG_R = R_BE*(VB_mps+cross(BodyRates_radps,r_MLG_R));\n'...
'%% --- Normal forces (one-sided damper: damping only on compression) --\n'...
'N_NLG   = max(k_NLG*d_NLG   + c_NLG*max(vn_NLG(3),  0)*on_NLG,   0);\n'...
'N_MLG_L = max(k_MLG*d_MLG_L + c_MLG*max(vn_MLG_L(3),0)*on_MLG_L, 0);\n'...
'N_MLG_R = max(k_MLG*d_MLG_R + c_MLG*max(vn_MLG_R(3),0)*on_MLG_R, 0);\n'...
'N_total = N_NLG + N_MLG_L + N_MLG_R;\n'...
'N_gears = [N_NLG; N_MLG_L; N_MLG_R];\n'...
'%% --- Friction forces ------------------------------------------------\n'...
'fwd_hat = R_BE(:,1);\n'...
'sgn_u   = VB_mps(1)/max(abs(VB_mps(1)),0.01);\n'...
'FN_NED  = fwd_hat*(-mu*N_NLG  *sgn_u)+[0;0;-N_NLG  ];\n'...
'FL_NED  = fwd_hat*(-mu*N_MLG_L*sgn_u)+[0;0;-N_MLG_L];\n'...
'FR_NED  = fwd_hat*(-mu*N_MLG_R*sgn_u)+[0;0;-N_MLG_R];\n'...
'%% --- Transform to body frame, compute moments about CG -------------\n'...
'Fb = R_BE''*(FN_NED+FL_NED+FR_NED);\n'...
'Mb = cross(r_NLG,  R_BE''*FN_NED) + ...\n'...
'     cross(r_MLG_L,R_BE''*FL_NED) + ...\n'...
'     cross(r_MLG_R,R_BE''*FR_NED);\n'...
'GndForces = [Fb; Mb];   %% [Fx;Fy;Fz;Mx;My;Mz] N / N·m\n'...
'end\n'], ...
    mass_kg, delta_static_m, zeta_damping, ...
    r_NLG(1),   r_NLG(2),   r_NLG(3), ...
    r_MLG_L(1), r_MLG_L(2), r_MLG_L(3), ...
    r_MLG_R(1), r_MLG_R(2), r_MLG_R(3), ...
    k_NLG_r, c_NLG_r, ...
    k_MLG_r, c_MLG_r);

%% ── 8. Load model and inject code ────────────────────────────────────
mdl = 'ACFT11_a_121';
if ~bdIsLoaded(mdl)
    load_system([mdl '.slx']);
    fprintf('\n  Model loaded.\n');
else
    fprintf('\n  Model already open.\n');
end

rt     = sfroot;
charts = rt.find('-isa','Stateflow.EMChart');
fcn_found = false;
for k = 1:numel(charts)
    if contains(charts(k).Path, [mdl '/Landing Gear/LandingGear_FCN'])
        charts(k).Script = lg_code;
        fcn_found = true;
        break;
    end
end
if ~fcn_found
    error('[setup_lg] LandingGear_FCN not found in model. Run original setup first.');
end
fprintf('  LandingGear_FCN code injected.\n');

%% ── 9. Update Simulink initial conditions ────────────────────────────
set_param([mdl '/EqM/InertialData/ZI_m'],     'InitialCondition', num2str(ZI_m_IC, '%.5f'));
set_param([mdl '/EqM/EulerAngles/THETA_rad'], 'InitialCondition', sprintf('%g*pi/180', theta_gnd_deg));
set_param([mdl '/EqM/BodyVelocities/u_mps'],  'InitialCondition', '0');
set_param([mdl '/EqM/BodyVelocities/v_mps'],  'InitialCondition', '0');
set_param([mdl '/EqM/BodyVelocities/w_mps'],  'InitialCondition', '0');
fprintf('  Simulink ICs set: ZI_m=%.5f m, theta=%g deg.\n', ZI_m_IC, theta_gnd_deg);

%% ── 10. Inertia tensor ───────────────────────────────────────────────
Inertia      = [Ixx 0 -Ixz; 0 Iyy 0; -Ixz 0 Izz];
Ixx1 = 32379 * scale_I; Iyy1 = 136386 * scale_I;
Izz1 = 168765 * scale_I; Ixz1 = 2511 * scale_I;
InertiaEmpty = [Ixx1 0 -Ixz1; 0 Iyy1 0; -Ixz1 0 Izz1];

%% ── 11. Export to base workspace ────────────────────────────────────
%  FCS setup scripts (setup_takeoff_fcs, setup_bfl_analysis) will pick
%  these up automatically if they are present in the workspace.
assignin('base', 'Mass',          mass_kg);
assignin('base', 'Mass_kg',       mass_kg);
assignin('base', 'g',             g_n);
assignin('base', 'Ixx',           Ixx);
assignin('base', 'Iyy',           Iyy);
assignin('base', 'Izz',           Izz);
assignin('base', 'Ixz',           Ixz);
assignin('base', 'Inertia',       Inertia);
assignin('base', 'Ixx1',          Ixx1);
assignin('base', 'Iyy1',          Iyy1);
assignin('base', 'Izz1',          Izz1);
assignin('base', 'Ixz1',          Ixz1);
assignin('base', 'InertiaEmpty',  InertiaEmpty);
assignin('base', 'Xcg',           xcg_from_nose_m);
assignin('base', 'ZI_m_IC',       ZI_m_IC);
% Store gear parameters for reference
assignin('base', 'LG_k_NLG',     k_NLG_r);
assignin('base', 'LG_c_NLG',     c_NLG_r);
assignin('base', 'LG_k_MLG',     k_MLG_r);
assignin('base', 'LG_c_MLG',     c_MLG_r);
assignin('base', 'LG_delta_stat',delta_static_m);
fprintf('  Base workspace updated: Mass=%.0f kg, ZI_m_IC=%.5f m.\n', mass_kg, ZI_m_IC);

%% ── 12. Save model ───────────────────────────────────────────────────
save_system(mdl);
fprintf('  Model saved.\n');

%% ── 13. Summary ─────────────────────────────────────────────────────
fprintf('\n=================================================================\n');
fprintf('  LANDING GEAR SETUP COMPLETE\n');
fprintf('=================================================================\n');
fprintf('\n  GEOMETRY\n');
fprintf('    CG from nose : %.3f m\n', xcg_from_nose_m);
fprintf('    NLG from nose: %.3f m  (arm x=%+.3f m)\n', xnlg_from_nose_m, r_NLG_x);
fprintf('    MLG from nose: %.3f m  (arm x=%+.3f m, track=+/-%.3f m)\n', ...
        xmlg_from_nose_m, r_MLG_x, ymlg_half_m);
fprintf('    Ground pitch : %.1f deg\n', theta_gnd_deg);
fprintf('\n  SPRING & DAMPER\n');
fprintf('    k_NLG = %7d N/m    c_NLG = %7d N·s/m\n', k_NLG_r, c_NLG_r);
fprintf('    k_MLG = %7d N/m    c_MLG = %7d N·s/m\n', k_MLG_r, c_MLG_r);
fprintf('    δ_static = %.0f mm  |  ζ = %.1f\n', delta_static_m*1000, zeta_damping);
fprintf('\n  INITIAL CONDITIONS\n');
fprintf('    ZI_m (NED z)  = %.5f m\n', ZI_m_IC);
fprintf('    h_CG above gnd = %.4f m\n', h_CG_IC);
fprintf('    theta = %.1f deg\n', theta_gnd_deg);
fprintf('\n  MASS\n');
fprintf('    Mass = %.0f kg   W = %.0f N\n', mass_kg, W);
fprintf('    Inertia scaled from original by factor %.4f\n', scale_I);
fprintf('\n  NEXT STEPS\n');
fprintf('    1. Open setup_takeoff_fcs.m   → change Mass = %.0f\n', mass_kg);
fprintf('    2. Open setup_bfl_analysis.m  → change Mass = %.0f\n', mass_kg);
fprintf('    3. Run the appropriate setup script\n');
fprintf('    4. Press Run in Simulink\n');
fprintf('=================================================================\n\n');

%% ── 14. Optional: verify with a quick static check ──────────────────
fprintf('  STATIC VERIFICATION AT IC\n');
phi_v = 0; theta_v = theta; psi_v = 0;
cph=cos(phi_v); sph=sin(phi_v); cth=cos(theta_v); sth=sin(theta_v);
cps=cos(psi_v); sps=sin(psi_v);
R_check = [cth*cps, sph*sth*cps-cph*sps, cph*sth*cps+sph*sps;
            cth*sps, sph*sth*sps+cph*cps, cph*sth*sps-sph*cps;
           -sth,     sph*cth,              cph*cth];
h_CG_check = h_CG_IC;
d_N = R_check(3,:)*r_NLG   - h_CG_check;
d_L = R_check(3,:)*r_MLG_L - h_CG_check;
d_R = R_check(3,:)*r_MLG_R - h_CG_check;
N_N = max(k_NLG_r*d_N, 0);
N_L = max(k_MLG_r*d_L, 0);
N_R = max(k_MLG_r*d_R, 0);
N_tot_check = N_N + N_L + N_R;
fprintf('    Compressions  : NLG=%.4f m  MLG_L=%.4f m  MLG_R=%.4f m\n', d_N, d_L, d_R);
fprintf('    Normal forces : NLG=%.0f N  MLG_L=%.0f N  MLG_R=%.0f N\n', N_N, N_L, N_R);
fprintf('    Total N       = %.0f N   (W = %.0f N)   err = %+.2f%%\n', ...
        N_tot_check, W, (N_tot_check-W)/W*100);
if abs((N_tot_check-W)/W) < 0.02
    fprintf('    --> Gear balanced to within 2%%: IC is good.\n\n');
else
    fprintf('    *** WARNING: balance error > 2%% — check geometry inputs!\n\n');
end
