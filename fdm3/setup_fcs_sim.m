%% setup_fcs_sim.m
%  Setup for the full-mission Flight Control System (native-block FCS).
%  Replaces setup_takeoff_fcs.m for the new INDI + ACAH + TECS architecture.
%
%  What it does:
%    1. Loads the aircraft configuration (mass, inertia, geometry, aero, prop).
%    2. Loads all FCS gains/limits from fcs_params.m.
%    3. Sets an AIRBORNE cruise initial condition for a trim/stability check
%       (matching the AUTO-mode setpoints V_cmd, h_cmd in fcs_params.m).
%    4. Configures the solver and logging.
%
%  The FCS runs in AUTO by default (RC_mode constant = FCS_MODE_AUTO):
%    TECS holds airspeed + altitude, HeadingHold holds track, ACAH+INDI
%    stabilise attitude.  Envelope protection (alpha, Nz, bank) is active.
%
%  Usage:
%    run('setup_fcs_sim.m')      % configures the model
%    then press Run in Simulink, OR call sim('ACFT11_a_121').
% -----------------------------------------------------------------------
mdl = 'ACFT11_a_121';
if ~bdIsLoaded(mdl), load_system([mdl '.slx']); end

%% ---- 1. Aircraft configuration (base workspace) -----------------------
Mass = 28500;  g = 9.80665;  Mass_kg = Mass;
Ixx = 74848;  Iyy = 288344;  Izz = 390125;  Ixz = 2511;
Inertia = [Ixx 0 -Ixz; 0 Iyy 0; -Ixz 0 Izz];
Ixx1=32379; Iyy1=136386; Izz1=168765; Ixz1=2511;
InertiaEmpty = [Ixx1 0 -Ixz1; 0 Iyy1 0; -Ixz1 0 Izz1];
y_cg = 0;  z_cg = 0;  Xcg = 9.87;
V_reference_mps = 200;  rho_reference_kgpm3 = 1.225;  nv = 0;  nrho = 0.75;
alfaf_deg = -1.8;  xf_m = 0;  zf_m = -0.08;  Tmax = 140000;
S = 85.25;  c = 15.55;  b = 9.05;
CL0=0.00003;  CL_alpha=-0.247;  CL_elev=0.0179;  CL_AlphaDot=0;  CL_q=-0.114;
CD0=0.00554;  CD_alpha=0.0177;  CD_elev=0.0001;
CY_beta=0.0228; CY_rud=0; CY_ail=0.00327; CY_r=0.105; CY_p=0.0305;
Cl_beta=0.0393; Cl_rud=0; Cl_ail=-0.00325; Cl_r=-0.035; Cl_p=-0.00065;
Cm0=0.00007;  Cm_alpha=-0.124;  Cm_elev=-0.0084;  Cm_AlphaDot=0;  Cm_q=-0.025;
Cn_beta=-0.0123; Cn_rud=0; Cn_ail=-0.00194; Cn_r=0.0168; Cn_p=-0.0185;
Lgrfrontx=1.92; Lgrfronty=0; Lgrfrontz=0.5;

cfg_vars = {'Mass','g','Mass_kg','Ixx','Iyy','Izz','Ixz','Inertia', ...
    'Ixx1','Iyy1','Izz1','Ixz1','InertiaEmpty','y_cg','z_cg','Xcg', ...
    'V_reference_mps','rho_reference_kgpm3','nv','nrho', ...
    'alfaf_deg','xf_m','zf_m','Tmax','S','c','b', ...
    'CL0','CL_alpha','CL_elev','CL_AlphaDot','CL_q','CD0','CD_alpha','CD_elev', ...
    'CY_beta','CY_rud','CY_ail','CY_r','CY_p', ...
    'Cl_beta','Cl_rud','Cl_ail','Cl_r','Cl_p', ...
    'Cm0','Cm_alpha','Cm_elev','Cm_AlphaDot','Cm_q', ...
    'Cn_beta','Cn_rud','Cn_ail','Cn_r','Cn_p', ...
    'Lgrfrontx','Lgrfronty','Lgrfrontz'};
for k = 1:numel(cfg_vars), assignin('base', cfg_vars{k}, eval(cfg_vars{k})); end
fprintf('[setup_fcs] Aircraft config loaded (%d vars).\n', numel(cfg_vars));

%% ---- 2. FCS gains / limits -------------------------------------------
run(fullfile(fileparts(mfilename('fullpath')),'fcs_params.m'));

%  INDI G-matrix schedule (control effectiveness vs alpha,Mach from aero tables)
run(fullfile(fileparts(mfilename('fullpath')),'setup_gschedule.m'));

%% ---- 3. Choose run scenario ------------------------------------------
%  'mission' : ground start (on runway, V=0) -> full autonomous mission
%              (ground roll -> ... -> landing).  Uses StopTime = 1000 s.
%  'trim'    : airborne start at cruise setpoints -> quick FCS trim/stability
%              check.  Uses StopTime = 60 s.
RUN_MODE = 'mission';     % <-- change to 'trim' for the airborne trim check

if strcmpi(RUN_MODE,'trim')
    h0=FCS_h_cmd; V0=FCS_V_cmd; theta0=2.0; Tend='60';
    set_param([mdl '/EqM/InertialData/ZI_m'],     'InitialCondition', num2str(-h0));
    set_param([mdl '/EqM/EulerAngles/THETA_rad'], 'InitialCondition', num2str(theta0*pi/180));
    set_param([mdl '/EqM/BodyVelocities/u_mps'],  'InitialCondition', num2str(V0));
    set_param([mdl '/EqM/BodyVelocities/v_mps'],  'InitialCondition', '0');
    set_param([mdl '/EqM/BodyVelocities/w_mps'],  'InitialCondition', '0');
    fprintf('[setup_fcs] MODE=trim  IC: h=%.0f m, V=%.0f m/s (airborne).\n', h0, V0);
else   % 'mission' — ground start
    Tend='1000';
    set_param([mdl '/EqM/InertialData/ZI_m'],     'InitialCondition', '-2.24');   % on gear
    set_param([mdl '/EqM/EulerAngles/THETA_rad'], 'InitialCondition', '2*pi/180'); % ground attitude
    set_param([mdl '/EqM/BodyVelocities/u_mps'],  'InitialCondition', '0');
    set_param([mdl '/EqM/BodyVelocities/v_mps'],  'InitialCondition', '0');
    set_param([mdl '/EqM/BodyVelocities/w_mps'],  'InitialCondition', '0');
    fprintf('[setup_fcs] MODE=mission  IC: on runway, V=0 (full autonomous mission).\n');
end

%% ---- 4. Solver & logging ---------------------------------------------
set_param(mdl, 'StopTime',   Tend);
set_param(mdl, 'SolverName', 'ode15s');
set_param(mdl, 'SolverType', 'Variable-step');
set_param(mdl, 'RelTol',     '1e-3');
set_param(mdl, 'AbsTol',     '1e-4');
set_param(mdl, 'MaxStep',    '0.05');
set_param(mdl, 'AlgebraicLoopMsg', 'none');
set_param(mdl, 'LoadExternalInput', 'off');
set_param(mdl, 'ReturnWorkspaceOutputs', 'off');

fprintf('\n============================================================\n');
fprintf('  FCS READY  (RUN_MODE = %s,  StopTime = %s s)\n', RUN_MODE, Tend);
fprintf('  Vr = %.0f m/s | descent command steps at t=400 s\n', FCS_V_rotate);
fprintf('  Aircraft: Mass=%.0f kg, T/W=%.2f\n', Mass, Tmax/(Mass*g));
fprintf('  --> Press the green RUN button in Simulink, or call sim(''%s'').\n', mdl);
fprintf('============================================================\n\n');
