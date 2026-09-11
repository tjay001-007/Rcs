%% Landing Gear Subsystem Creation Script
%  Creates a 3-point spring-damper landing gear model and wires it into
%  ACFT11_a_121.slx, replacing the zero Constant at GroundForcesMoments.
%
%  Aircraft geometry (body frame: x fwd, y right, z DOWN):
%    CG at Xcg = 9.87 m from nose
%    MLG: x = +1.015 m aft, y = +/-2.3 m, z = +2.286 m below CG (compressed)
%    NLG: x = -6.76 m fwd,  y = 0,        z = +2.014 m below CG (compressed)
%         (NLG z-arm adjusted from stated 2.48 m to 2.014 m for geometric
%          consistency with 2-deg nose-up ground attitude; the 2.48 m value
%          appears to be the unloaded/extended strut length)
%
%  Initial conditions set for ground start:
%    Altitude  ZI_m_IC  = -2.249 m  (z_NED, CG is 2.249 m above ground)
%    Pitch     THETA_IC = +2 deg
%    u_mps_IC = 1 m/s   (small non-zero value avoids TAS=0 singularity)

mdl = 'ACFT11_a_121';

%% ---- Load model -----------------------------------------------------------
if ~bdIsLoaded(mdl)
    load_system(fullfile(fileparts(mfilename('fullpath')), [mdl '.slx']));
    fprintf('Model loaded.\n');
else
    fprintf('Model already loaded.\n');
end

%% ---- STEP 1: Disconnect zero-constant from EqM port 2 --------------------
try
    delete_line(mdl, 'Constant/1', 'EqM/2');
    fprintf('Disconnected Constant -> EqM/GroundForcesMoments.\n');
catch e
    fprintf('Note: delete_line: %s\n', e.message);
end

%% ---- STEP 2: Add Landing Gear subsystem at root level ---------------------
lg_path = [mdl '/Landing Gear'];
if ~exist_block(mdl, 'Landing Gear')
    add_block('simulink/Ports & Subsystems/Subsystem', lg_path, ...
        'Position', [640 440 840 570]);
    fprintf('Added Landing Gear subsystem.\n');
else
    fprintf('Landing Gear subsystem already exists, clearing content.\n');
    % Clear existing content
    Simulink.SubSystem.deleteContents(lg_path);
end

%% ---- STEP 3: Input / output ports inside the subsystem -------------------
pw = 30; ph = 20;   % port width / height
ips = {
    'EulerAngles_rad',  '1',  30,  40;
    'VB_mps',           '2',  30, 100;
    'BodyRates_radps',  '3',  30, 160;
    'pos_NED_m',        '4',  30, 220;
    'mass_kg',          '5',  30, 280;
};
for k = 1:size(ips,1)
    bname = ips{k,1}; pnum = ips{k,2}; px = ips{k,3}; py = ips{k,4};
    add_block('simulink/Ports & Subsystems/In1', [lg_path '/' bname], ...
        'Port', pnum, 'Position', [px py px+pw py+ph]);
end

ops = {'GndForces_N_Nm','1',460,175; 'N_total_N','2',460,275};
for k = 1:size(ops,1)
    bname = ops{k,1}; pnum = ops{k,2}; px = ops{k,3}; py = ops{k,4};
    add_block('simulink/Ports & Subsystems/Out1', [lg_path '/' bname], ...
        'Port', pnum, 'Position', [px py px+pw py+ph]);
end
fprintf('Input/Output ports added inside Landing Gear subsystem.\n');

%% ---- STEP 4: Add MATLAB Function block inside subsystem ------------------
mlfcn_path = [lg_path '/LandingGear_FCN'];
add_block('simulink/User-Defined Functions/MATLAB Function', mlfcn_path, ...
    'Position', [180 60 390 360]);
fprintf('MATLAB Function block added.\n');

%% ---- STEP 5: Set the MATLAB function code --------------------------------
lg_code = [
"function [GndForces, N_total] = LandingGear_FCN(EulerAngles_rad, VB_mps, BodyRates_radps, pos_NED_m, mass_kg) %#ok<INUSL>"
"%#codegen"
"% Spring-damper landing gear for 3-point tricycle gear."
"% Body frame: x forward, y right, z DOWN (standard aerospace NED)"
"% Inputs:"
"%   EulerAngles_rad  [phi, theta, psi] rad"
"%   VB_mps           [u, v, w] m/s"
"%   BodyRates_radps  [p, q, r] rad/s"
"%   pos_NED_m        [x, y, z_NED] m   (z positive DOWN)"
"%   mass_kg          mass (unused; reserved for future load-dependent mu)"
"% Outputs:"
"%   GndForces  [Fx; Fy; Fz; Mx; My; Mz]  N and N.m in body frame"
"%   N_total    total normal force N  (goes to zero at lift-off)"
""
"% --- Gear arm vectors from CG [x_fwd, y_right, z_down] m -----------------"
"r_NLG   = [-6.76;  0.0;   2.014];  % Nose gear  (z adjusted for 2-deg attitude)"
"r_MLG_L = [ 1.015; -2.3;  2.286];  % Main gear LEFT"
"r_MLG_R = [ 1.015;  2.3;  2.286];  % Main gear RIGHT"
""
"% --- Strut spring-damper parameters ----------------------------------------"
"k_NLG = 150000;   % N/m   spring stiffness"
"c_NLG =   6000;   % N.s/m damping"
"k_MLG = 400000;   % N/m"
"c_MLG =  12000;   % N.s/m"
"mu    =    0.03;  % rolling friction coefficient"
""
"% --- Unpack inputs ---------------------------------------------------------"
"phi   = EulerAngles_rad(1);"
"theta = EulerAngles_rad(2);"
"psi   = EulerAngles_rad(3);"
"VB    = VB_mps(:);"
"omega = BodyRates_radps(:);"
""
"% CG height above ground (positive up)"
"h_CG = -pos_NED_m(3);   % NED z positive down, altitude = -z_NED"
""
"% --- Body-to-NED rotation matrix -------------------------------------------"
"cphi=cos(phi); sphi=sin(phi);"
"cth =cos(theta); sth=sin(theta);"
"cpsi=cos(psi);  spsi=sin(psi);"
""
"R_BE = [cth*cpsi,                   sphi*sth*cpsi-cphi*spsi,  cphi*sth*cpsi+sphi*spsi;"
"        cth*spsi,                   sphi*sth*spsi+cphi*cpsi,  cphi*sth*spsi-sphi*cpsi;"
"       -sth,                        sphi*cth,                  cphi*cth              ];"
""
"R3 = R_BE(3,:);  % row 3: maps body vector to NED-z (downward component)"
""
"% --- Ground contact heights (positive = above ground, negative = compressed)"
"h_NLG   = h_CG - R3*r_NLG;"
"h_MLG_L = h_CG - R3*r_MLG_L;"
"h_MLG_R = h_CG - R3*r_MLG_R;"
""
"% --- Compression (positive when gear presses into ground) ------------------"
"d_NLG   = max(-h_NLG,   0);"
"d_MLG_L = max(-h_MLG_L, 0);"
"d_MLG_R = max(-h_MLG_R, 0);"
""
"on_NLG   = double(h_NLG   <= 0);  % contact flags"
"on_MLG_L = double(h_MLG_L <= 0);"
"on_MLG_R = double(h_MLG_R <= 0);"
""
"% --- Velocity of gear contact points in body frame ------------------------"
"vg_NLG   = VB + cross(omega, r_NLG);"
"vg_MLG_L = VB + cross(omega, r_MLG_L);"
"vg_MLG_R = VB + cross(omega, r_MLG_R);"
""
"% Convert to NED -- z component is the strut compression rate"
"vn_NLG   = R_BE * vg_NLG;"
"vn_MLG_L = R_BE * vg_MLG_L;"
"vn_MLG_R = R_BE * vg_MLG_R;"
""
"% --- Normal forces (upward reaction; clamped to >= 0, cannot pull) --------"
"N_NLG   = max(k_NLG*d_NLG   + c_NLG*vn_NLG(3)*on_NLG,   0);"
"N_MLG_L = max(k_MLG*d_MLG_L + c_MLG*vn_MLG_L(3)*on_MLG_L, 0);"
"N_MLG_R = max(k_MLG*d_MLG_R + c_MLG*vn_MLG_R(3)*on_MLG_R, 0);"
"N_total  = N_NLG + N_MLG_L + N_MLG_R;"
""
"% --- Rolling friction (opposes longitudinal body velocity u) ---------------"
"u_body = VB(1);"
"sgn_u  = u_body / max(abs(u_body), 0.01);  % smooth sign near zero"
""
"Ff_NLG   = -mu * N_NLG   * sgn_u;"
"Ff_MLG_L = -mu * N_MLG_L * sgn_u;"
"Ff_MLG_R = -mu * N_MLG_R * sgn_u;"
""
"fwd_hat = R_BE(:,1);  % forward unit vector in NED (body x column)"
""
"% --- Gear forces in NED frame  [fwd_friction; 0; -N_up] ------------------"
"Fg_NLG_NED   = fwd_hat*Ff_NLG   + [0; 0; -N_NLG  ];"
"Fg_MLG_L_NED = fwd_hat*Ff_MLG_L + [0; 0; -N_MLG_L];"
"Fg_MLG_R_NED = fwd_hat*Ff_MLG_R + [0; 0; -N_MLG_R];"
""
"% --- Transform gear forces to body frame -----------------------------------"
"R_EB = R_BE';   % NED -> Body"
"Fg_NLG_b   = R_EB * Fg_NLG_NED;"
"Fg_MLG_L_b = R_EB * Fg_MLG_L_NED;"
"Fg_MLG_R_b = R_EB * Fg_MLG_R_NED;"
""
"% --- Moments about CG in body frame: M = r x F ----------------------------"
"M_NLG   = cross(r_NLG,   Fg_NLG_b);"
"M_MLG_L = cross(r_MLG_L, Fg_MLG_L_b);"
"M_MLG_R = cross(r_MLG_R, Fg_MLG_R_b);"
""
"% --- Assemble output vector ------------------------------------------------"
"F_total   = Fg_NLG_b + Fg_MLG_L_b + Fg_MLG_R_b;"
"M_total   = M_NLG + M_MLG_L + M_MLG_R;"
"GndForces = [F_total; M_total];   % [Fx; Fy; Fz; Mx; My; Mz] N / N.m"
"end"
];

% Join lines into a single char string
lg_code_str = strjoin(lg_code, newline);

% Set the code via the Stateflow API
sf_root = sfroot();
chart = find(sf_root, '-isa', 'Stateflow.EMChart', 'Path', mlfcn_path);
if isempty(chart)
    error('Could not find EMChart at "%s". Check that the block was added.', mlfcn_path);
end
chart.Script = lg_code_str;
fprintf('MATLAB Function code set successfully.\n');

%% ---- STEP 6: Internal wiring inside Landing Gear subsystem ---------------
add_line(lg_path, 'EulerAngles_rad/1', 'LandingGear_FCN/1', 'autorouting','on');
add_line(lg_path, 'VB_mps/1',          'LandingGear_FCN/2', 'autorouting','on');
add_line(lg_path, 'BodyRates_radps/1', 'LandingGear_FCN/3', 'autorouting','on');
add_line(lg_path, 'pos_NED_m/1',       'LandingGear_FCN/4', 'autorouting','on');
add_line(lg_path, 'mass_kg/1',         'LandingGear_FCN/5', 'autorouting','on');
add_line(lg_path, 'LandingGear_FCN/1', 'GndForces_N_Nm/1',  'autorouting','on');
add_line(lg_path, 'LandingGear_FCN/2', 'N_total_N/1',       'autorouting','on');
fprintf('Internal wiring complete.\n');

%% ---- STEP 7: Root-level wiring -------------------------------------------
% Bus Selector1 (blk_38) outputs:
%   port 1 = VB_mps        (3-element, m/s)
%   port 2 = BodyRates_radps (3-element, rad/s)
%   port 3 = EulerAngles_rad (3-element, rad)
%   port 4 = InertialPosition_m (3-element, m NED)
add_line(mdl, 'Bus Selector1/3', 'Landing Gear/1', 'autorouting','on');  % EulerAngles_rad
add_line(mdl, 'Bus Selector1/1', 'Landing Gear/2', 'autorouting','on');  % VB_mps
add_line(mdl, 'Bus Selector1/2', 'Landing Gear/3', 'autorouting','on');  % BodyRates_radps
add_line(mdl, 'Bus Selector1/4', 'Landing Gear/4', 'autorouting','on');  % InertialPosition_m (NED)
add_line(mdl, 'From8/1',         'Landing Gear/5', 'autorouting','on');  % mass_kg

% Connect landing gear output -> EqM GroundForcesMoments (port 2)
add_line(mdl, 'Landing Gear/1', 'EqM/2', 'autorouting','on');
fprintf('Root-level wiring complete.\n');

%% ---- STEP 8: Add Normal-Force scope (liftoff monitor) --------------------
if ~exist_block(mdl, 'Nf_LiftoffScope')
    add_block('simulink/Sinks/Scope', [mdl '/Nf_LiftoffScope'], ...
        'Position', [900 440 930 470], 'NumInputPorts', '1');
    add_line(mdl, 'Landing Gear/2', 'Nf_LiftoffScope/1', 'autorouting','on');
    fprintf('Normal-force liftoff scope added.\n');
end

%% ---- STEP 9: Initial conditions for ground-start -------------------------
% Place CG at 2.249 m above ground (z_NED = -2.249 m)
% Theta = 2 deg nose-up, u = 1 m/s (small: avoids TAS=0 singularity)
set_param([mdl '/EqM/EulerAngles/THETA_rad'],    'InitialCondition', '2*pi/180');
set_param([mdl '/EqM/InertialData/ZI_m'],        'InitialCondition', '-2.249');
set_param([mdl '/EqM/BodyVelocities/u_mps'],     'InitialCondition', '1.0');
set_param([mdl '/EqM/BodyRates/p_radps'],        'InitialCondition', '0');
set_param([mdl '/EqM/BodyRates/q_radps'],        'InitialCondition', '0');
set_param([mdl '/EqM/BodyRates/r_radps'],        'InitialCondition', '0');
fprintf('Initial conditions set (theta=2deg, ZI=-2.249m, u=1m/s).\n');

%% ---- STEP 10: Save -------------------------------------------------------
save_system(mdl);
fprintf('\n========== Landing Gear setup complete ==========\n');
fprintf('  NLG arm (body):  x=-6.76 m, y=0, z=+2.014 m\n');
fprintf('  MLG arm (body):  x=+1.015 m, y=+/-2.3 m, z=+2.286 m\n');
fprintf('  k_MLG=400000 N/m,  c_MLG=12000 N.s/m\n');
fprintf('  k_NLG=150000 N/m,  c_NLG=6000 N.s/m\n');
fprintf('  mu_roll = 0.03\n');
fprintf('  Initial h_CG = 2.249 m  (ZI_m IC = -2.249 m)\n');
fprintf('  Initial theta = 2 deg\n');
fprintf('\n  NOTE: NLG z-arm adjusted from stated 2.48 m (unloaded strut)\n');
fprintf('  to 2.014 m (compressed, on-ground) for geometric consistency.\n');
fprintf('================================================\n');

%% ---- Helper: check if block exists in model ------------------------------
function tf = exist_block(model, blkname)
    try
        get_param([model '/' blkname], 'BlockType');
        tf = true;
    catch
        tf = false;
    end
end
