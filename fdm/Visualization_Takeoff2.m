%% ========================================================================
%  PROJECT:   ACFT11 TAKEOFF FLIGHT DYNAMICS & CONTROL
%  MODULE:    CINEMATIC TAKEOFF VISUALIZATION  (v2.0 — universal STL)
%  ========================================================================
%  DESCRIPTION:
%     Cinematic 3-D replay of the full takeoff maneuver.
%     Drop in ANY aircraft STL — the script auto-scales, auto-centres,
%     and auto-derives engine / gear positions from the mesh bounding box.
%
%  PHASE TRAIL COLOURS:
%     Phase 0 — Ground Roll   → lime green
%     Phase 1 — Rotation      → amber
%     Phase 2 — Climb Hold    → cyan
%
%  USAGE:
%     1) run('setup_takeoff_fcs.m')
%     2) Press Run in Simulink
%     3) (optional) analyze_takeoff
%     4) Visualization_Takeoff   ← this script
% ========================================================================

%% ========================================================================
%  ███  USER CONFIGURATION  ███  — only edit this block
%% ========================================================================

% ── STL file path ────────────────────────────────────────────────────────
STL_Path = 'C:\Users\tejve\Downloads\rollout\T38.stl';

% ── Aircraft visual size in world-space [m] ───────────────────────────────
%    Set this to the real fuselage length of your aircraft.
%    The mesh is scaled so its longest axis (or your chosen forward axis)
%    spans exactly this many metres in the animation.
Target_Length_m = 14.5;          % T-38 Talon ≈ 14.5 m

% ── STL axis alignment ────────────────────────────────────────────────────
%    Answer: which axis in the RAW STL file points toward the NOSE?
%    Options: '+x'  '-x'  '+y'  '-y'  '+z'  '-z'
%    Use 'auto' to let the script pick the longest bounding-box axis.
STL_Forward_Axis = 'auto';       % change if the aircraft renders backward

%    Which STL axis points UPWARD on the aircraft?
STL_Up_Axis      = '+z';         % change if the aircraft renders upside-down

% ── Engine plumes ─────────────────────────────────────────────────────────
n_engines_vis    = 2;            % 1 = centerline  2 = twin nacelles
Engine_Y_Sep_m   = 0.0;         % half-span to each engine [m]
                                  % (0 = centreline; set to real value for twin)

% ── Mesh quality ─────────────────────────────────────────────────────────
Target_Faces     = 5000;         % max triangle count after decimation

% ── Playback speed ───────────────────────────────────────────────────────
ANIM_PAUSE = 0.06;               % seconds per frame (0=max speed, 0.1=real-time)

%% ========================================================================
%  END USER CONFIGURATION
%% ========================================================================

clearvars -except TW_time TW_X_m TW_TAS_mps TW_Theta_deg TW_q_deg ...
                  TW_Alpha_deg TW_Gamma_deg TW_Thrust_N TW_posNED ...
                  TW_Phase TW_qcmd TW_elev_cmd TW_Ntotal out ...
                  STL_Path Target_Length_m STL_Forward_Axis STL_Up_Axis ...
                  n_engines_vis Engine_Y_Sep_m Target_Faces ANIM_PAUSE;
clc; close all;

%% ========================================================================
%  0.  DATA INGESTION
%% ========================================================================
fprintf('>> [VIS] ACFT11 Takeoff Visualizer v2.0\n');
fprintf('>> [VIS] -----------------------------------\n');

if exist('out','var') && isa(out,'Simulink.SimulationOutput') && ~exist('TW_time','var')
    fprintf('>> [VIS] Extracting from Simulink SimulationOutput...\n');
    tw_list = {'TW_time','TW_X_m','TW_TAS_mps','TW_Theta_deg','TW_q_deg', ...
               'TW_Alpha_deg','TW_Gamma_deg','TW_Thrust_N', ...
               'TW_posNED','TW_Phase','TW_qcmd','TW_elev_cmd'};
    for vi = 1:numel(tw_list)
        try; eval([tw_list{vi} ' = out.' tw_list{vi} ';']); catch; end
    end
end

if ~exist('TW_time','var') || ~exist('TW_X_m','var')
    error(['>> FATAL: Takeoff data not in workspace.\n' ...
           '  1) run(''setup_takeoff_fcs.m'')  2) Run Simulink  3) Re-run this script.']);
end

%% ========================================================================
%  1.  SIGNAL PREPARATION
%% ========================================================================
t      = TW_time(:);
x      = TW_X_m(:);
TAS    = TW_TAS_mps(:);
pitch  = TW_Theta_deg(:) * pi/180;
q_act  = TW_q_deg(:);
alpha  = TW_Alpha_deg(:);
gamma  = TW_Gamma_deg(:);
thrust = TW_Thrust_N(:) / 1000;
mach   = TAS / 340.3;

n_fc     = size(TW_posNED, 1);
t_fc     = (0:n_fc-1)' * 0.02;
alt_fc   = -TW_posNED(:,3);
phase_fc = TW_Phase(:);
qcmd_fc  = TW_qcmd(:);
ecmd_fc  = TW_elev_cmd(:);

alt   = max(0, interp1(t_fc, alt_fc,   t, 'linear', 'extrap'));
phase = interp1(t_fc, phase_fc, t, 'nearest','extrap');
q_cmd = interp1(t_fc, qcmd_fc,  t, 'nearest','extrap');
e_cmd = interp1(t_fc, ecmd_fc,  t, 'linear', 'extrap');

L = length(t);
fprintf('>> [VIS] %d frames  (dt=0.1 s,  %.1f s total)\n', L, t(end));

%% ========================================================================
%  2.  UNIVERSAL STL LOADER & AUTO-SCALER
%% ========================================================================
fprintf('>> [VIS] Loading STL: %s\n', STL_Path);
stl_ok = false;
try
    raw   = stlread(STL_Path);
    V_raw = raw.Points;
    F_raw = raw.ConnectivityList;
    fprintf('>> [VIS] Raw mesh: %d vertices  %d faces\n', size(V_raw,1), size(F_raw,1));
    stl_ok = true;
catch ME
    warning('>> [VIS] Could not read STL (%s).\n>> Using procedural placeholder.', ME.message);
end

if ~stl_ok
    % ── Procedural placeholder: fuselage + swept wings ───────────────────
    [V_raw, F_raw] = make_placeholder_aircraft();
    fprintf('>> [VIS] Placeholder mesh: %d vertices  %d faces\n', size(V_raw,1), size(F_raw,1));
end

% ── Bounding box ─────────────────────────────────────────────────────────
bb_min = min(V_raw,[],1);
bb_max = max(V_raw,[],1);
bb_sz  = bb_max - bb_min;
bb_ctr = (bb_min + bb_max) / 2;

fprintf('>> [VIS] STL bounding box:\n');
fprintf('         X: %.3g → %.3g   (span %.3g)\n', bb_min(1),bb_max(1),bb_sz(1));
fprintf('         Y: %.3g → %.3g   (span %.3g)\n', bb_min(2),bb_max(2),bb_sz(2));
fprintf('         Z: %.3g → %.3g   (span %.3g)\n', bb_min(3),bb_max(3),bb_sz(3));

% ── Parse axis specs → (dim_index, sign) ─────────────────────────────────
[fwd_dim, fwd_sgn] = parse_axis_spec(STL_Forward_Axis, bb_sz);
[up_dim,  up_sgn ] = parse_axis_spec(STL_Up_Axis,  bb_sz);

if fwd_dim == up_dim
    warning('Forward and Up axes are the same — resetting Up to next largest axis.');
    [~, ord] = sort(bb_sz, 'descend');
    for ci = 1:3
        if ord(ci) ~= fwd_dim, up_dim = ord(ci); up_sgn = 1; break; end
    end
end

% Right-hand side axis (cross product: fwd × up in index space → right)
right_dim = setdiff(1:3, [fwd_dim, up_dim]);
% Determine right sign so the triad is right-handed
e_fwd = zeros(1,3); e_fwd(fwd_dim)   = fwd_sgn;
e_up  = zeros(1,3); e_up(up_dim)     = up_sgn;
e_rgt = cross(e_fwd, e_up);          % should be unit ±axis
right_sgn = e_rgt(right_dim);        % +1 or -1

fprintf('>> [VIS] Axis mapping:  fwd=axis%d(×%+.0f)  up=axis%d(×%+.0f)  right=axis%d(×%+.0f)\n', ...
    fwd_dim, fwd_sgn, up_dim, up_sgn, right_dim, right_sgn);

% ── Fuselage length in STL units along forward axis ──────────────────────
fus_len_stl = bb_sz(fwd_dim);
scale_f     = Target_Length_m / fus_len_stl;
fprintf('>> [VIS] Fuselage: %.4g STL units  →  scale = %.6g  →  %.2f m\n', ...
    fus_len_stl, scale_f, Target_Length_m);

% ── Build 3×3 rotation: STL frame → animation frame ─────────────────────
%    Animation frame: fwd=+X, right=+Y, up=+Z
%    Row i of R_stl2vis = which STL column maps to animation axis i
R_stl2vis = zeros(3,3);
R_stl2vis(1, fwd_dim)   = fwd_sgn;    % anim X  ← STL fwd_dim
R_stl2vis(3, up_dim)    = up_sgn;     % anim Z  ← STL up_dim
R_stl2vis(2, right_dim) = right_sgn;  % anim Y  ← STL right_dim

% ── Apply: centre → scale → rotate ──────────────────────────────────────
V_cen = (V_raw - bb_ctr) * scale_f;  % centred in STL frame, scaled
V_vis = (R_stl2vis * V_cen')';        % rotated into animation frame

% ── Mesh decimation ──────────────────────────────────────────────────────
if size(F_raw,1) > Target_Faces
    ratio = Target_Faces / size(F_raw,1);
    tmp   = patch('Faces',F_raw,'Vertices',V_vis,'Visible','off');
    reducepatch(tmp, ratio);
    F_vis = get(tmp,'Faces');
    V_vis = get(tmp,'Vertices');
    delete(tmp);
    fprintf('>> [VIS] Decimated to %d faces.\n', size(F_vis,1));
else
    F_vis = F_raw;
end

% ── Derive scene geometry from scaled mesh ────────────────────────────────
v_nose_x  = max(V_vis(:,1));    % nose tip  (+X)
v_tail_x  = min(V_vis(:,1));    % tail tip  (−X)
v_top_z   = max(V_vis(:,3));    % aircraft top
v_bot_z   = min(V_vis(:,3));    % aircraft bottom (belly / gear attach)

half_len   = (v_nose_x - v_tail_x) / 2;
half_ht    = (v_top_z  - v_bot_z)  / 2;

% Gear height: distance from belly of mesh to ground contact
%  = how much to lift the aircraft so it "sits" on the runway
Gear_Height = abs(v_bot_z) + 0.05;   % belly → runway  (small margin)

% Engine flame geometry (aft end of fuselage)
AB_X_Offset = v_tail_x;                        % nozzle plane in local X
AB_Length   = max(1.5, half_len * 0.45);       % 45% of half-length
AB_Radius   = max(0.25, half_ht  * 0.25);      % 25% of half-height
AB_Sep_Y    = Engine_Y_Sep_m;                   % user-set lateral offset

fprintf('>> [VIS] Mesh vis-frame extents:  nose=%.2f  tail=%.2f  top=%.2f  bot=%.2f\n', ...
    v_nose_x, v_tail_x, v_top_z, v_bot_z);
fprintf('>> [VIS] Gear_Height=%.2f m   AB_X=%.2f  AB_L=%.2f  AB_R=%.2f  AB_Y=±%.2f\n', ...
    Gear_Height, AB_X_Offset, AB_Length, AB_Radius, AB_Sep_Y);

%% ========================================================================
%  3.  SCENE & RENDERER SETUP
%% ========================================================================
f  = figure('Color','k','Name','ACFT11 Takeoff Cinematic Replay');
set(f,'WindowState','maximized');
ax = axes('Parent',f,'Color','k','GridColor',[0.3 0.3 0.3],'GridAlpha',0.35);
hold(ax,'on');  axis(ax,'equal');  grid(ax,'on');
xlabel(ax,'Longitudinal (m)','Color','w');
ylabel(ax,'Lateral (m)','Color','w');
zlabel(ax,'Altitude (m)','Color','w');
ax.XColor = 'w';  ax.YColor = 'w';  ax.ZColor = 'w';
camproj(ax,'perspective');
view(ax, 40, 12);

% ── Runway environment ────────────────────────────────────────────────────
Rwy_Len = max(3500, x(end) * 0.55);
Rwy_W   = 23;
patch(ax, [0, Rwy_Len, Rwy_Len, 0], ...
          [-Rwy_W, -Rwy_W, Rwy_W, Rwy_W], [0 0 0 0], ...
          [0.18 0.18 0.18], 'EdgeColor','none','FaceAlpha',0.95);
for dx = 0:100:Rwy_Len
    patch(ax,[dx,dx+40,dx+40,dx],[-0.8,-0.8,0.8,0.8],[0.01 0.01 0.01 0.01], ...
              [0.92 0.92 0.72],'EdgeColor','none','FaceAlpha',0.55);
end
patch(ax,[0,6,6,0],[-Rwy_W,-Rwy_W,-Rwy_W+4,-Rwy_W+4],[.01 .01 .01 .01],[1 1 1],'EdgeColor','none');
patch(ax,[0,6,6,0],[ Rwy_W-4, Rwy_W-4, Rwy_W, Rwy_W],[.01 .01 .01 .01],[1 1 1],'EdgeColor','none');
patch(ax,[-500,max(x)+2000,max(x)+2000,-500],[-400,-400,400,400], ...
          [-0.2 -0.2 -0.2 -0.2],[0.04 0.12 0.04],'EdgeColor','none','FaceAlpha',0.7);

% ── Phase-coloured trail objects ──────────────────────────────────────────
Trail_GR  = plot3(ax,x(1),0,alt(1)+Gear_Height,'-','Color',[0.20 1.00 0.20],'LineWidth',1.8);
Trail_ROT = plot3(ax,x(1),0,alt(1)+Gear_Height,'-','Color',[1.00 0.75 0.00],'LineWidth',1.8);
Trail_CLM = plot3(ax,x(1),0,alt(1)+Gear_Height,'-','Color',[0.00 0.90 1.00],'LineWidth',1.8);

% ── Aircraft mesh ─────────────────────────────────────────────────────────
CombinedObject = hgtransform('Parent', ax);

patch('Parent', CombinedObject, ...
      'Faces', F_vis, 'Vertices', V_vis, ...
      'FaceColor', [0.82 0.82 0.88], ...
      'EdgeColor', 'none', ...
      'FaceLighting', 'gouraud', ...
      'AmbientStrength',  0.45, ...
      'DiffuseStrength',  0.70, ...
      'SpecularStrength', 0.45);

% ── Engine plumes ─────────────────────────────────────────────────────────
%  Cone: opens rearward (tail → aft).  Built in local X: from AB_X_Offset
%  forward by AB_Length into the fuselage (so it looks like exhaust coming out).
[cf_r, cf_y, cf_z] = cylinder([AB_Radius 0.01], 10);
% cf_z goes 0→1; map to X: X = AB_X_Offset + (0→-AB_Length) [rearward]
cf_x = AB_X_Offset - cf_z * AB_Length;

Y_offsets = AB_Sep_Y * (linspace(-1,1, n_engines_vis));
Flames = gobjects(1, n_engines_vis);
for ei = 1:n_engines_vis
    Flames(ei) = surface('Parent', CombinedObject, ...
        'XData', cf_x, 'YData', cf_y + Y_offsets(ei), 'ZData', cf_r, ...
        'FaceColor', [1.0 0.50 0.05], 'EdgeColor', 'none', ...
        'FaceAlpha', 0.85, 'AmbientStrength', 1.0, 'Visible', 'on');
end

% ── Lighting ─────────────────────────────────────────────────────────────
light(ax,'Position',[500 -800 2000],'Style','local','Color',[1.00 0.95 0.90]);
light(ax,'Position',[-200 200 400],'Style','local','Color',[0.30 0.30 0.60]);
lighting(ax,'gouraud');

fprintf('>> [VIS] Scene built.  %d engine plume(s).\n', n_engines_vis);

%% ========================================================================
%  4.  HUD — STATUS BANNER + FLIGHT DATA PANEL
%% ========================================================================
h_Status = uicontrol('Parent',f,'Style','text','Units','normalized', ...
    'Position',[0.28 0.88 0.44 0.05], ...
    'BackgroundColor','k','ForegroundColor',[0.2 1 0.2], ...
    'FontSize',15,'FontWeight','bold','String','PHASE: GROUND ROLL');

hud_bg = uipanel('Parent',f,'Position',[0.75 0.20 0.23 0.75], ...
    'BackgroundColor',[0.08 0.08 0.08], ...
    'Title','FLIGHT DATA','TitlePosition','centertop', ...
    'FontSize',11,'ForegroundColor',[1 1 1],'FontWeight','bold');

add_lbl = @(y,s) uicontrol('Parent',hud_bg,'Style','text','Units','normalized', ...
    'Position',[0.04 y 0.48 0.055],'BackgroundColor',[0.08 0.08 0.08], ...
    'ForegroundColor',[1 1 1],'HorizontalAlignment','left', ...
    'FontSize',10,'FontWeight','bold','String',s);
add_val = @(y) uicontrol('Parent',hud_bg,'Style','text','Units','normalized', ...
    'Position',[0.50 y 0.46 0.055],'BackgroundColor',[0.08 0.08 0.08], ...
    'ForegroundColor',[1 1 1],'HorizontalAlignment','right', ...
    'FontSize',10,'FontWeight','bold','String','---');

add_lbl(0.92,'TAS');       val_TAS   = add_val(0.92);
add_lbl(0.84,'MACH');      val_Mach  = add_val(0.84);
add_lbl(0.76,'PITCH');     val_Pitch = add_val(0.76);
add_lbl(0.68,'ALPHA');     val_Alph  = add_val(0.68);
add_lbl(0.60,'ALTITUDE');  val_Alt   = add_val(0.60);
add_lbl(0.52,'GAMMA');     val_Gam   = add_val(0.52);
add_lbl(0.44,'Q ACT');     val_Qact  = add_val(0.44);
add_lbl(0.36,'Q CMD');     val_Qcmd  = add_val(0.36);
add_lbl(0.28,'ELEV CMD');  val_Elev  = add_val(0.28);
add_lbl(0.20,'THRUST');    val_Thr   = add_val(0.20);
add_lbl(0.12,'DIST GND');  val_Dist  = add_val(0.12);
add_lbl(0.04,'SIM TIME');  val_Time  = add_val(0.04);

fprintf('>> [VIS] HUD ready.\n');
fprintf('>> [VIS] Starting animation (%d frames)...\n', L);
fprintf('>> [VIS] Close the figure window to stop.\n\n');

%% ========================================================================
%  5.  ANIMATION LOOP
%% ========================================================================
Win_X_Back  = max(40, Target_Length_m * 5);
Win_X_Fwd   = max(150, Target_Length_m * 14);
Win_Y       = 120;
Win_Z_Pad   = max(40, Target_Length_m * 4);
Trail_Gap   = Target_Length_m * 0.8;

idx_rot_start = find(phase >= 1, 1, 'first');
idx_clm_start = find(phase >= 2, 1, 'first');
if isempty(idx_rot_start), idx_rot_start = L+1; end
if isempty(idx_clm_start), idx_clm_start = L+1; end

% No Scl or InitialRotation needed — baked into V_vis already
k = 1;
while k <= L
    if ~isvalid(f), break; end

    cx = x(k);  ca = alt(k);  cp = pitch(k);  cph = phase(k);

    % Phase label + colour
    switch cph
        case 0
            PLabel = 'PHASE: GROUND ROLL';
            PColor = [0.20 1.00 0.20];
        case 1
            PLabel = 'PHASE: ROTATION';
            PColor = [1.00 0.82 0.10];
        otherwise
            PLabel = sprintf('PHASE: CLIMB  ALT %.0f m', ca);
            PColor = [0.10 0.90 1.00];
    end
    if isvalid(h_Status)
        set(h_Status,'String',PLabel,'ForegroundColor',PColor);
    end

    % Aircraft transform: translate to position + pitch rotation
    %   All mesh alignment is pre-baked → no InitialRotation or Scl needed.
    T_world = makehgtform('translate', [cx, 0, ca + Gear_Height]);
    R_pitch = makehgtform('yrotate', -cp);   % nose-up pitch = +cp
    set(CombinedObject,'Matrix', T_world * R_pitch);

    % Phase trails
    lim_x = cx - Trail_Gap;
    i_gr  = min(k, idx_rot_start - 1);
    if i_gr >= 1
        m = x(1:i_gr) < lim_x;
        if any(m)
            set(Trail_GR,'XData',x(1:i_gr),'YData',zeros(1,i_gr),'ZData',alt(1:i_gr)+Gear_Height);
        end
    end
    if k >= idx_rot_start
        i_rt = min(k, idx_clm_start-1);
        xv = x(idx_rot_start:i_rt); av = alt(idx_rot_start:i_rt);
        if any(xv < lim_x)
            set(Trail_ROT,'XData',xv,'YData',zeros(1,numel(xv)),'ZData',av+Gear_Height);
        end
    end
    if k >= idx_clm_start
        xv = x(idx_clm_start:k); av = alt(idx_clm_start:k);
        if any(xv < lim_x)
            set(Trail_CLM,'XData',xv,'YData',zeros(1,numel(xv)),'ZData',av+Gear_Height);
        end
    end

    % Afterburner flicker
    flk = 0.55 + 0.40*rand();
    for ei = 1:n_engines_vis
        set(Flames(ei),'FaceAlpha', flk);
    end

    % HUD
    set(val_TAS,  'String', sprintf('%.1f m/s',   TAS(k)));
    set(val_Mach, 'String', sprintf('%.3f',        mach(k)));
    set(val_Pitch,'String', sprintf('%.1f deg',    TW_Theta_deg(k)));
    set(val_Alph, 'String', sprintf('%.1f deg',    alpha(k)));
    set(val_Alt,  'String', sprintf('%.1f m',      ca));
    set(val_Gam,  'String', sprintf('%.1f deg',    gamma(k)));
    set(val_Qact, 'String', sprintf('%.2f d/s',    q_act(k)));
    set(val_Qcmd, 'String', sprintf('%.2f d/s',    q_cmd(k)));
    set(val_Elev, 'String', sprintf('%.1f deg',    e_cmd(k)));
    set(val_Thr,  'String', sprintf('%.1f kN',     thrust(k)));
    set(val_Dist, 'String', sprintf('%.0f m',      cx));
    set(val_Time, 'String', sprintf('%.1f s',      t(k)));

    % Camera
    xlim(ax,[cx - Win_X_Back, cx + Win_X_Fwd]);
    ylim(ax,[-Win_Y, Win_Y]);
    if ca < Win_Z_Pad
        zlim(ax,[-2, Win_Z_Pad * 2]);
    else
        zlim(ax,[ca - Win_Z_Pad, ca + Win_Z_Pad * 1.5]);
    end

    drawnow;
    pause(ANIM_PAUSE);
    k = k + 1;
end

if isvalid(f) && isvalid(h_Status)
    set(h_Status,'String', sprintf('COMPLETE  %.0f m  |  %.1f s', max(alt), t(end)), ...
        'ForegroundColor',[0.2 1 0.2]);
end
fprintf('>> [VIS] Replay complete.\n');

%% ========================================================================
%  LOCAL HELPER FUNCTIONS
%% ========================================================================

function [dim, sgn] = parse_axis_spec(spec, bb_sz)
%PARSE_AXIS_SPEC  Convert '+x'/'-y'/... to (dim_index, sign).
%  'auto' → dimension with largest bounding-box span.
    if strcmpi(spec,'auto')
        [~, dim] = max(bb_sz);
        sgn = 1;
        fprintf('>> [VIS] Auto-detected forward axis: axis%d (span=%.3g)\n', dim, bb_sz(dim));
        return;
    end
    map = struct('x',1,'y',2,'z',3);
    spec = lower(strtrim(spec));
    if spec(1) == '-', sgn = -1; ax_ch = spec(2);
    else,              sgn = +1; ax_ch = spec(end); end
    dim = map.(ax_ch);
end

function [V, F] = make_placeholder_aircraft()
%MAKE_PLACEHOLDER_AIRCRAFT  Generate a simple triangulated aircraft shape.
%  Fuselage along X, wings along Y, up = +Z.  Useful when no STL is available.
    pts = [];
    % Fuselage: stretched ellipsoid approximation via cylinder of varying radius
    n_fus = 30;
    t_fus = linspace(0,1,n_fus);
    % Radius profile: pointed nose, wide centre, tapered tail
    r_fus = 0.5 * sin(pi * t_fus) .* (0.4 + 0.6*(1-t_fus));
    r_fus(1) = 0.01;  r_fus(end) = 0.01;
    for i = 1:n_fus
        th = linspace(0,2*pi,12)';
        ring = [repmat(t_fus(i), 12, 1), ...
                r_fus(i)*cos(th), ...
                r_fus(i)*sin(th) * 0.7];   % slightly flattened vertically
        pts = [pts; ring]; %#ok<AGROW>
    end
    % Wing slab: thin flat panel
    wing = [ 0.45  0.0  0.00;  0.45  1.8  0.02;  0.60  1.8  0.01;  0.60  0.0  0.0;
             0.45  0.0 -0.01;  0.45  1.8 -0.02;  0.60  1.8 -0.01;  0.60  0.0  0.0];
    % Mirror for left wing
    wing_L = wing;  wing_L(:,2) = -wing_L(:,2);
    pts = [pts; wing; wing_L];
    % Tail fin (vertical)
    tail = [0.85  0.0 0.00; 0.85  0.0 0.30; 1.00  0.0  0.10; 1.00  0.0  0.00];
    pts = [pts; tail];
    % Build convex hull over all points
    try
        F = convhull(pts(:,1), pts(:,2), pts(:,3), 'Simplify',true);
    catch
        F = convhull(pts);
    end
    V = pts;
    % Scale to unit fuselage (X: 0→1)
    V(:,1) = V(:,1) - 0.5;   % centre at X=0
end
