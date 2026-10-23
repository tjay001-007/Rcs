%% setup_propulsion.m
%  Replaces the Propulsion subsystem in ACFT11_a_121.slx with a proper
%  2-D Mach × Altitude thrust table for a twin-engine aircraft.
%
%  ARCHITECTURE
%  ─────────────────────────────────────────────────────────────────────
%  Atmospheric block (existing):
%    port 1 → FromAtmos bus (contains rho_kgpm3)
%    port 4 → mach              ← added wire to Propulsion port 3
%
%  Propulsion subsystem (rebuilt internals):
%    in1: FromAtmos  (bus)  → BusSelector → rho_kgpm3
%    in2: Throttle   (0-1)
%    in3: Mach       (new)  ← from Atmospheric/4
%    MATLAB Fn: PropulsionFCN(Mach, rho_kgpm3, Throttle)
%      • altitude from rho via ISA inverse
%      • bilinear table lookup → T_max per engine
%      • T_total = Throttle × n_engines × T_max
%      • engine angle decomposition → [Fx;0;Fz;0;My;0]
%    out1: Engine_Forces_and Moments  [6×1 N / N·m]
%    out2: Thrust_N                   [scalar N]
%
%  HOW TO USE
%  1. Paste YOUR engine data into the TABLE DATA section below.
%  2. Run this script once — it rewires and injects the model.
%  3. No other file needs changing; run setup_takeoff_fcs.m as normal.
% -----------------------------------------------------------------------

%% =====================================================================
%  TABLE DATA  ← replace placeholder values with your engine data
%% =====================================================================

%  ── Engine geometry (existing values — change if needed) ─────────────
n_engines = 2;          % total engine count
alfaf_deg = -1.8;       % engine incidence angle [deg] (nose-down = negative)
xf_m      = 0.0;        % thrust line x-offset from CG [m] (body x, +fwd)
ye_m      = 0.0;        % lateral distance of each engine FROM centreline [m]
                         %   each engine is at ±ye_m  (right = +y, left = −y)
                         %   SYMMETRIC case: Mx, Mz cancel → net [Fx;0;Fz;0;My;0]
                         %   OEI case: single engine → Mz = ±ye_m × Fx (yaw)
zf_m      = -0.08;      % thrust line z-offset from CG [m] (body z-down, pos = down)

%  ── Mach breakpoints (strictly increasing, starting from 0) ──────────
Mach_bp = [0.00, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80];

%  ── Altitude breakpoints [m], strictly increasing ────────────────────
Alt_bp_m = [0, 2000, 4000, 6000, 8000, 10000, 12000];

%  ── Thrust table: ONE engine, full throttle (Throttle = 1) [N] ───────
%     ROWS    = Mach  (match Mach_bp order, one row per point)
%     COLUMNS = Altitude (match Alt_bp_m order, one col per point)
%     Size must be  numel(Mach_bp) × numel(Alt_bp_m)
%
%  >>>>  REPLACE THE VALUES BELOW WITH YOUR ACTUAL ENGINE DATA  <<<<
%                    0m      2000m   4000m   6000m   8000m  10000m  12000m
T_table_N = [
    70000   60000   51000   43000   36000   29500   23500;  % Mach 0.00
    72000   62000   53000   44500   37500   30500   24500;  % Mach 0.10
    72500   62500   53500   45000   38000   31000   25000;  % Mach 0.20
    71500   61500   52500   44000   37000   30200   24200;  % Mach 0.30
    70000   60000   51500   43000   36000   29500   23500;  % Mach 0.40
    67500   58000   49500   41500   35000   28500   22500;  % Mach 0.50
    63000   54000   46000   38500   32500   26500   21000;  % Mach 0.60
    57000   49000   42000   35000   29500   24000   19000;  % Mach 0.70
    50000   43000   37000   31000   26000   21000   17000;  % Mach 0.80
];
%  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

%% =====================================================================
%  END TABLE DATA
%% =====================================================================

%% ── Validate ─────────────────────────────────────────────────────────
nM = numel(Mach_bp);
nH = numel(Alt_bp_m);
assert(size(T_table_N,1)==nM, 'T_table rows (%d) != Mach_bp count (%d)', size(T_table_N,1), nM);
assert(size(T_table_N,2)==nH, 'T_table cols (%d) != Alt_bp count (%d)',  size(T_table_N,2), nH);
assert(all(diff(Mach_bp)>0),  'Mach_bp must be strictly increasing');
assert(all(diff(Alt_bp_m)>0), 'Alt_bp_m must be strictly increasing');

fprintf('\n=================================================================\n');
fprintf('  PROPULSION SETUP — 2-D Mach × Altitude Thrust Table\n');
fprintf('  Engines : %d × %.0f kN max  (SL, static)\n', n_engines, T_table_N(1,1)/1e3);
fprintf('  Table   : %d Mach points × %d altitude points\n', nM, nH);
fprintf('=================================================================\n');

% Print table preview
fprintf('\n  T_max per engine [kN] (your table):\n');
fprintf('  M\\Alt(m)  ');
for j = 1:nH, fprintf('%7.0f', Alt_bp_m(j)); end; fprintf('\n');
fprintf('  %s\n', repmat('-', 1, 10+nH*7));
for i = 1:nM
    fprintf('  M=%4.2f    ', Mach_bp(i));
    for j = 1:nH, fprintf('%7.1f', T_table_N(i,j)/1e3); end
    fprintf('\n');
end

% Quick sanity checks at representative operating points
T_SL_static = T_table_N(1,1);    % static thrust per engine, SL
T_TO_M03    = interp2(Alt_bp_m, Mach_bp, T_table_N, 0, 0.30, 'linear');
T_CLB_M07   = interp2(Alt_bp_m, Mach_bp, T_table_N, 8000, 0.70, 'linear');

mass_ref = evalin('base','Mass'); if isempty(mass_ref), mass_ref = 28500; end
W_ref = mass_ref * 9.80665;

fprintf('\n  SANITY CHECKS:\n');
fprintf('    SL static   : %5.1f kN/eng  → total %5.1f kN\n', T_SL_static/1e3, n_engines*T_SL_static/1e3);
fprintf('    TO  (SL, M0.3): %5.1f kN/eng → T/W = %.3f (%.0f kg aircraft)\n', ...
    T_TO_M03/1e3, n_engines*T_TO_M03/W_ref, mass_ref);
fprintf('    Cruise (8km, M0.7): %5.1f kN/eng\n', T_CLB_M07/1e3);

%% ── Build PropulsionFCN code string ──────────────────────────────────
Mach_str = vec2str(Mach_bp);
Alt_str  = vec2str(Alt_bp_m);
T_str    = mat2str_2d(T_table_N);

%  PropulsionFCN outputs a SINGLE 7-element column vector y:
%    y(1:6) = Engine_Forces_and_Moments  [Fx;0;Fz;0;My;0]
%    y(7)   = Thrust_N  (scalar, for logging)
%  Two Selector blocks inside the subsystem split y back into the
%  two outports expected by the rest of the model.
%  This avoids needing SimulationCommand/update to refresh output port count.
prop_code = sprintf([...
'function y = PropulsionFCN(Mach, rho_kgpm3, Throttle)\n'...
'%%#codegen\n'...
'%% 2-engine thrust model: Mach x Altitude lookup table.\n'...
'%% Auto-generated by setup_propulsion.m — edit that file, not this.\n'...
'%%   Engines: %d x %.0f kN SL static  |  incidence = %.1f deg\n'...
'%%   Output: y = [Fx;0;Fz;0;My;0;Thrust_N]  (7x1)\n'...
'%%           y(1:6) = Engine_Forces_and_Moments,  y(7) = Thrust_N\n'...
'%%\n'...
'%% --- Engine constants -----------------------------------------------\n'...
'n_eng  = %d;\n'...
'af_rad = %.6f;   %% engine incidence [rad] (alfaf_deg=%.2f)\n'...
'xf_m   = %.4f;   %% thrust line x-offset from CG [m]\n'...
'ye_m   = %.4f;   %% lateral distance of EACH engine from centreline [m]\n'...
'                 %%   engines at +ye_m (right) and -ye_m (left)\n'...
'                 %%   symmetric: Mx,Mz cancel; OEI: Mz = +-ye_m*Fx\n'...
'zf_m   = %.4f;   %% thrust line z-offset from CG [m]\n'...
'%% --- ISA inverse: rho -> altitude [m] ------------------------------\n'...
'rho0=1.225; T0=288.15; g0=9.80665; R=287.058; L=0.0065;\n'...
'rho_11=0.3639; T_11=216.65; h_11=11000.0;\n'...
'rho_c = max(rho_kgpm3, 1e-4);\n'...
'if rho_c >= rho_11\n'...
'    T_K   = T0*(rho_c/rho0)^(1.0/4.2561);\n'...
'    alt_m = (T0-T_K)/L;\n'...
'else\n'...
'    T_K   = T_11;\n'...
'    alt_m = h_11 + log(rho_11/rho_c)*R*T_11/g0;\n'...
'end\n'...
'%% --- Mach-Altitude thrust table: ONE engine, Throttle = 1 ----------\n'...
'Mach_bp = %s;\n'...
'Alt_bp  = %s;\n'...
'T_tab   = %s;\n'...
'%% --- Bilinear interpolation (clamped at table boundaries) ----------\n'...
'T_max_1eng = bilerp(Mach_bp, Alt_bp, T_tab, Mach, alt_m);\n'...
'%% --- Total thrust (both engines, throttle scaled) ------------------\n'...
'Thrust_N = max(0.0, Throttle * double(n_eng) * T_max_1eng);\n'...
'%% --- Engine force/moment decomposition (body frame) ----------------\n'...
'%% Each engine at [xf_m; ±ye_m; zf_m] relative to CG\n'...
'%% Symmetric pair: Mx,Mz cancel; My = 2*(zf*Fx_1 - xf*Fz_1)\n'...
'T_1  = Thrust_N * 0.5;\n'...
'Fx_1 = T_1 * cos(af_rad);\n'...
'Fz_1 = T_1 * sin(af_rad);\n'...
'Fx   = 2.0 * Fx_1;\n'...
'Fz   = 2.0 * Fz_1;\n'...
'My   = 2.0 * (zf_m*Fx_1 - xf_m*Fz_1);\n'...
'%% Pack output: [forces(6); Thrust_N] — Selector blocks split this\n'...
'y = [Fx; 0.0; Fz; 0.0; My; 0.0; Thrust_N];\n'...
'end\n'...
'\n'...
'%% ----- Clamped bilinear interpolation (codegen-safe) ---------------\n'...
'function Tq = bilerp(Mv, Hv, T, M, H)\n'...
'%%BILERP  Bilinear lookup in 2-D table, clamped at boundaries.\n'...
'%%  Mv [1 x nM], Hv [1 x nH], T [nM x nH], M and H scalars.\n'...
'nM = numel(Mv); nH_= numel(Hv);\n'...
'Mc = max(Mv(1), min(Mv(nM),  M));\n'...
'Hc = max(Hv(1), min(Hv(nH_), H));\n'...
'im = nM - 1;\n'...
'for ii = 1:nM-1\n'...
'    if Mc <= Mv(ii+1); im = ii; break; end\n'...
'end\n'...
'ih = nH_ - 1;\n'...
'for ii = 1:nH_-1\n'...
'    if Hc <= Hv(ii+1); ih = ii; break; end\n'...
'end\n'...
'tm = (Mc - Mv(im)) / (Mv(im+1) - Mv(im));\n'...
'th = (Hc - Hv(ih)) / (Hv(ih+1) - Hv(ih));\n'...
'Tq = (1-tm)*(1-th)*T(im,  ih  ) + ...\n'...
'       tm  *(1-th)*T(im+1,ih  ) + ...\n'...
'     (1-tm)*  th  *T(im,  ih+1) + ...\n'...
'       tm  *  th  *T(im+1,ih+1);\n'...
'end\n'], ...
    n_engines, T_table_N(1,1)/1e3, alfaf_deg, ...
    n_engines, alfaf_deg*pi/180, alfaf_deg, xf_m, ye_m, zf_m, ...
    Mach_str, Alt_str, T_str);

%% ── Load model ───────────────────────────────────────────────────────
mdl = 'ACFT11_a_121';
if ~bdIsLoaded(mdl), load_system([mdl '.slx']); fprintf('  Model loaded.\n');
else, fprintf('  Model already open.\n'); end

prop_path = [mdl '/Propulsion'];

%% ── STEP 1: Clear all lines inside Propulsion ────────────────────────
lines = find_system(prop_path,'FindAll','on','SearchDepth',1,'Type','line');
for k = 1:numel(lines)
    try; delete_line(lines(k)); catch; end
end
fprintf('  Existing propulsion wiring cleared.\n');

%% ── STEP 2: Delete old computation blocks (keep ports + BusSelector) ─
keep_types = {'Inport','Outport','BusSelector'};
blks = find_system(prop_path,'SearchDepth',1,'Type','Block');
for k = 1:numel(blks)
    if strcmp(blks{k}, prop_path), continue; end
    bt = get_param(blks{k},'BlockType');
    if ~ismember(bt, keep_types)
        try; delete_block(blks{k}); catch e
            fprintf('    skip %s: %s\n', blks{k}, e.message);
        end
    end
end
fprintf('  Old computation blocks removed.\n');

%% ── STEP 3: Add new Mach Inport (port 3) inside Propulsion ──────────
inport_names = get_inport_names(prop_path);
if ~ismember('Mach', inport_names)
    add_block('simulink/Ports & Subsystems/In1', [prop_path '/Mach'], ...
        'Port','3', 'Position',[30 280 60 300]);
    fprintf('  Mach inport added (port 3).\n');
else
    fprintf('  Mach inport already exists.\n');
end

%% ── STEP 4: Configure BusSelector to output rho_kgpm3 only ──────────
bs_blks = find_system(prop_path,'SearchDepth',1,'Type','Block','BlockType','BusSelector');
if isempty(bs_blks)
    error('BusSelector not found in Propulsion subsystem.');
end
bs_name = get_param(bs_blks{1},'Name');
set_param(bs_blks{1}, 'OutputSignals', 'rho_kgpm3');
fprintf('  BusSelector configured: outputs rho_kgpm3.\n');

%% ── STEP 5: Add PropulsionFCN MATLAB Function block ──────────────────
%  Single output: y [7x1] = [Fx;0;Fz;0;My;0;Thrust_N]
%  Selector blocks (added below) split y into the two outports.
%  This avoids any need for SimulationCommand/update to expose port count.
fcn_path = [prop_path '/PropulsionFCN'];
add_block('simulink/User-Defined Functions/MATLAB Function', fcn_path, ...
    'Position',[220 110 400 230]);
fprintf('  PropulsionFCN block added.\n');

%% ── STEP 6: Inject code immediately (EMChart created at add_block time)
%  No SimulationCommand/update needed — sfroot can find the chart right away.
fprintf('  Finding EMChart and injecting code...\n');
rt      = sfroot;
machine = rt.find('-isa','Stateflow.Machine','Name',mdl);
if isempty(machine)
    error('[setup_propulsion] Stateflow machine for %s not found.', mdl);
end
em_charts = machine.find('-isa','Stateflow.EMChart');
chart_fcn = [];
for k = 1:numel(em_charts)
    if strcmp(em_charts(k).Path, fcn_path)
        chart_fcn = em_charts(k);
        break;
    end
end
if isempty(chart_fcn)
    error('[setup_propulsion] PropulsionFCN EMChart not found. Check add_block succeeded.');
end
chart_fcn.Script = prop_code;
fprintf('  PropulsionFCN code injected (single 7-element output).\n');

%% ── STEP 7: Add Selector blocks to split y → [forces(6), thrust(1)] ─
sel_f_path = [prop_path '/Sel_Forces'];   % elements 1:6
sel_t_path = [prop_path '/Sel_Thrust'];   % element 7
add_block('simulink/Signal Routing/Selector', sel_f_path, ...
    'Position',[460 105 510 195], ...
    'InputPortWidth','7', ...
    'Indices','[1 2 3 4 5 6]', ...
    'IndexMode','One-based');
add_block('simulink/Signal Routing/Selector', sel_t_path, ...
    'Position',[460 230 510 260], ...
    'InputPortWidth','7', ...
    'Indices','[7]', ...
    'IndexMode','One-based');
fprintf('  Selector blocks added (Sel_Forces: idx 1-6, Sel_Thrust: idx 7).\n');

%% ── STEP 8: Wire internals ───────────────────────────────────────────
%   FromAtmos/1  → BusSelector/1  (bus in)
%   BusSel/1     → PropulsionFCN/2  (rho_kgpm3 — input port auto-created)
%   Mach/1       → PropulsionFCN/1  (Mach)
%   Throttle/1   → PropulsionFCN/3  (Throttle)
%   PropulsionFCN/1 → Sel_Forces/1  (y → elements 1:6)
%   PropulsionFCN/1 → Sel_Thrust/1  (y → element 7)   [branched line]
%   Sel_Forces/1 → outport 1   (Engine_Forces_and Moments)
%   Sel_Thrust/1 → outport 2   (Thrust_N)
add_line(prop_path, 'FromAtmos/1',    [bs_name '/1'],    'autorouting','on');
add_line(prop_path, 'Mach/1',         'PropulsionFCN/1', 'autorouting','on');
add_line(prop_path, [bs_name '/1'],   'PropulsionFCN/2', 'autorouting','on');
add_line(prop_path, 'Throttle/1',     'PropulsionFCN/3', 'autorouting','on');
add_line(prop_path, 'PropulsionFCN/1','Sel_Forces/1',    'autorouting','on');
add_line(prop_path, 'PropulsionFCN/1','Sel_Thrust/1',    'autorouting','on');
% Find outport block names dynamically (handles special chars in name)
op_blks = find_system(prop_path,'SearchDepth',1,'Type','Block','BlockType','Outport');
op_names = cellfun(@(b) get_param(b,'Name'), op_blks, 'UniformOutput', false);
op_ports = cellfun(@(b) str2double(get_param(b,'Port')), op_blks);
[~,ord]  = sort(op_ports);
op_names = op_names(ord);
add_line(prop_path, 'Sel_Forces/1', [op_names{1} '/1'], 'autorouting','on');
add_line(prop_path, 'Sel_Thrust/1', [op_names{2} '/1'], 'autorouting','on');
fprintf('  Internal wiring complete.\n');
fprintf('    outport 1 (Engine_Forces): "%s"\n', op_names{1});
fprintf('    outport 2 (Thrust_N)     : "%s"\n', op_names{2});

%% ── STEP 9: Add Mach wire at ROOT level (Atmospheric/4 → Propulsion/3)
root_lines = find_system(mdl,'FindAll','on','SearchDepth',1,'Type','line');
mach_already = false;
for k = 1:numel(root_lines)
    try
        src     = get_param(root_lines(k),'SrcBlockHandle');
        spt     = get_param(root_lines(k),'SrcPortHandle');
        blk_nm  = get_param(src,'Name');
        port_no = get_param(spt,'PortNumber');
        if contains(blk_nm,'Atmospheric') && port_no == 4
            dst_nm = get_param(get_param(root_lines(k),'DstBlockHandle'),'Name');
            if contains(dst_nm,'Propulsion'), mach_already = true; end
        end
    catch; end
end
if ~mach_already
    add_line(mdl, 'Atmospheric/4', 'Propulsion/3', 'autorouting','on');
    fprintf('  Root: Atmospheric/4 (mach) → Propulsion/3 connected.\n');
else
    fprintf('  Root: Atmospheric→Propulsion mach wire already exists.\n');
end

%% ── STEP 10: Save model ──────────────────────────────────────────────
save_system(mdl);
fprintf('  Model saved.\n');

%% ── Summary ──────────────────────────────────────────────────────────
fprintf('\n=================================================================\n');
fprintf('  PROPULSION UPDATE COMPLETE\n');
fprintf('  Model   : %s\n', mdl);
fprintf('  Formula : T = Throttle × %d × table(Mach, h_ISA(rho))\n', n_engines);
fprintf('  Inputs  : in1=FromAtmos  in2=Throttle  in3=Mach (new)\n');
fprintf('  T_max SL static  : %.0f kN total\n', n_engines*T_table_N(1,1)/1e3);
fprintf('  Engine incidence : %.1f deg  (xf=%.3f m  zf=%.3f m)\n', alfaf_deg, xf_m, zf_m);
fprintf('  Engine y-offset  : ±%.3f m from centreline  (symmetric → Mx=Mz=0)\n', ye_m);
fprintf('    OEI note: single engine produces  Mz = ±%.1f × Fx  Nm/N\n', ye_m);
fprintf('\n  To update with your real table:\n');
fprintf('    1. Edit TABLE DATA block at top of setup_propulsion.m\n');
fprintf('    2. Re-run setup_propulsion.m\n');
fprintf('=================================================================\n\n');

%% =====================================================================
%  LOCAL HELPER FUNCTIONS
%% =====================================================================

function s = vec2str(v)
%VEC2STR  Format a row vector as a MATLAB literal '[a, b, c]'.
    s = '[';
    for i = 1:numel(v)
        s = [s, sprintf('%.6g', v(i))]; %#ok<AGROW>
        if i < numel(v), s = [s, ', ']; end %#ok<AGROW>
    end
    s = [s, ']'];
end

function s = mat2str_2d(M)
%MAT2STR_2D  Format a matrix as a MATLAB literal '[r1c1, r1c2; r2c1, ...]'.
    [nr, nc] = size(M);
    s = '[';
    for i = 1:nr
        for j = 1:nc
            s = [s, sprintf('%.0f', M(i,j))]; %#ok<AGROW>
            if j < nc, s = [s, ', ']; end %#ok<AGROW>
        end
        if i < nr, s = [s, '; ']; %#ok<AGROW>
        else,      s = [s, ']']; end %#ok<AGROW>
    end
end

function names = get_inport_names(subsys_path)
%GET_INPORT_NAMES  Return cell array of Inport block names in a subsystem.
    blks = find_system(subsys_path,'SearchDepth',1,'Type','Block','BlockType','Inport');
    names = cellfun(@(b) get_param(b,'Name'), blks, 'UniformOutput', false);
end
