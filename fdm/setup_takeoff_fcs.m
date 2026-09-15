%% setup_takeoff_fcs.m
%  Takeoff FCS setup — INDI pitch rate + P attitude hold
%  Run this script ONCE before pressing Run in Simulink.
%  Does NOT call sim() — start simulation manually from Simulink toolbar.
%
%  Architecture:
%    Phase 0 (GROUND_ROLL) : δe = 0,  full throttle until V ≥ 110 m/s
%    Phase 1 (ROTATION)    : INDI tracks q_cmd = 3°/s until liftoff / θ ≥ 12°
%    Phase 2 (CLIMB_HOLD)  : Outer P (Kθ=1) → q_cmd, INDI inner loop
%
%  G_δe = Cm_elev × q̄ × S × c / Iyy  (scheduled with V in real-time)
%  K_q = 5 s⁻¹,  K_θ = 1 s⁻¹,  N_filt = 15 rad/s,  dt = 0.02 s
% -------------------------------------------------------------------------

mdl = 'ACFT11_a_121';

%% ---- 0. Load model -------------------------------------------------------
if ~bdIsLoaded(mdl)
    load_system([mdl '.slx']);
    fprintf('[setup] Model loaded.\n');
else
    fprintf('[setup] Model already open.\n');
end

%% ---- 1. Workspace variables (aerodynamics + inertia) --------------------
Mass    = 28500;
g       = 9.80665;
Mass_kg = Mass;

Ixx = 74848;   Iyy = 288344;   Izz = 390125;   Ixz = 2511;
Inertia = [Ixx 0 -Ixz; 0 Iyy 0; -Ixz 0 Izz];

Ixx1=32379; Iyy1=136386; Izz1=168765; Ixz1=2511;
InertiaEmpty = [Ixx1 0 -Ixz1; 0 Iyy1 0; -Ixz1 0 Izz1];

y_cg = 0;  z_cg = 0;  Xcg  = 9.87;

V_reference_mps     = 200;
rho_reference_kgpm3 = 1.225;
nv   = 0;
nrho = 0.75;
alfaf_deg = -1.8;
xf_m = 0;   zf_m = -0.08;
Tmax = 140000;

S = 85.25;   c = 15.55;   b = 9.05;

% Aero derivatives
CL0=0.00003;  CL_alpha=-0.247;  CL_elev=0.0179;  CL_AlphaDot=0;  CL_q=-0.114;
CD0=0.00554;  CD_alpha=0.0177;  CD_elev=0.0001;
CY_beta=0.0228; CY_rud=0; CY_ail=0.00327; CY_r=0.105; CY_p=0.0305;
Cl_beta=0.0393; Cl_rud=0; Cl_ail=-0.00325; Cl_r=-0.035; Cl_p=-0.00065;
Cm0=0.00007;  Cm_alpha=-0.124;  Cm_elev=-0.0084;  Cm_AlphaDot=0;  Cm_q=-0.025;
Cn_beta=-0.0123; Cn_rud=0; Cn_ail=-0.00194; Cn_r=0.0168; Cn_p=-0.0185;

Lgrfrontx=1.92; Lgrfronty=0; Lgrfrontz=0.5;

% Push all needed variables to base workspace
vars = {'Mass','g','Mass_kg','Ixx','Iyy','Izz','Ixz','Inertia', ...
        'Ixx1','Iyy1','Izz1','Ixz1','InertiaEmpty', ...
        'y_cg','z_cg','Xcg', ...
        'V_reference_mps','rho_reference_kgpm3','nv','nrho', ...
        'alfaf_deg','xf_m','zf_m','Tmax', ...
        'S','c','b', ...
        'CL0','CL_alpha','CL_elev','CL_AlphaDot','CL_q', ...
        'CD0','CD_alpha','CD_elev', ...
        'CY_beta','CY_rud','CY_ail','CY_r','CY_p', ...
        'Cl_beta','Cl_rud','Cl_ail','Cl_r','Cl_p', ...
        'Cm0','Cm_alpha','Cm_elev','Cm_AlphaDot','Cm_q', ...
        'Cn_beta','Cn_rud','Cn_ail','Cn_r','Cn_p', ...
        'Lgrfrontx','Lgrfronty','Lgrfrontz'};
for k = 1:numel(vars)
    assignin('base', vars{k}, eval(vars{k}));
end
fprintf('[setup] Workspace variables loaded (%d vars).\n', numel(vars));

%% ---- 2. Initial conditions ----------------------------------------------
%  Static equilibrium on gear (theta=2°, from rest)
%  ZI_m = -2.23952 m  (CG 2.23952 m above ground plane)
set_param([mdl '/EqM/InertialData/ZI_m'],    'InitialCondition','-2.23952');
set_param([mdl '/EqM/EulerAngles/THETA_rad'],'InitialCondition','2*pi/180');
set_param([mdl '/EqM/BodyVelocities/u_mps'], 'InitialCondition','0');
set_param([mdl '/EqM/BodyVelocities/v_mps'], 'InitialCondition','0');
set_param([mdl '/EqM/BodyVelocities/w_mps'], 'InitialCondition','0');
fprintf('[setup] ICs set: ZI=-2.23952 m, θ=2°, V=0.\n');

%% ---- 3. Solver ----------------------------------------------------------
set_param(mdl, 'StopTime',          '130');
set_param(mdl, 'SolverName',        'ode15s');
set_param(mdl, 'SolverType',        'Variable-step');
set_param(mdl, 'RelTol',            '1e-3');
set_param(mdl, 'AbsTol',            '1e-4');
set_param(mdl, 'MaxStep',           '0.05');
set_param(mdl, 'AlgebraicLoopMsg', 'none');
fprintf('[setup] Solver: ode15s, RelTol=1e-3, AbsTol=1e-4, MaxStep=0.05, T_end=130 s.\n');

%% ---- 4. External input OFF (FCS runs in-loop) ---------------------------
set_param(mdl, 'LoadExternalInput', 'off');
fprintf('[setup] ExternalInput: OFF (FCS controls elevator + throttle).\n');

% Ensure ToWorkspace blocks write directly to base workspace (not bundled in 'out')
set_param(mdl, 'ReturnWorkspaceOutputs', 'off');
fprintf('[setup] ReturnWorkspaceOutputs: OFF (TW blocks write directly to workspace).\n');

%% ---- 5. Summary ---------------------------------------------------------
fprintf('\n');
fprintf('============================================================\n');
fprintf('  TAKEOFF FCS READY\n');
fprintf('  Aircraft : Mass=%g kg, Tmax=%g N, T/W=%.2f\n', Mass, Tmax, Tmax/(Mass*g));
fprintf('  Gear ICs : ZI=-2.23952 m, theta=2 deg, u=0 m/s\n');
fprintf('  FCS      : INDI (K_q=5), P-attitude (K_theta=1)\n');
fprintf('  Phases   : GROUND_ROLL -> ROTATION (q=3 deg/s) -> CLIMB (theta=12 deg)\n');
fprintf('  Sim time : 130 s\n');
fprintf('============================================================\n');
fprintf('\n  --> Press RUN in Simulink to start the simulation.\n\n');
