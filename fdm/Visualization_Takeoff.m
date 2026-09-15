%% ========================================================================
%  PROJECT:       ACFT11 TAKEOFF FLIGHT DYNAMICS & CONTROL
%  MODULE:        TAKEOFF CINEMATIC VISUALIZATION
%  VERSION:       1.0
%  ========================================================================
%  DESCRIPTION:
%     Cinematic 3D replay of the full takeoff maneuver:
%       Phase 0 — GROUND ROLL   (trail: lime green)
%       Phase 1 — ROTATION      (trail: amber)
%       Phase 2 — CLIMB HOLD    (trail: cyan)
%
%  DATA SOURCES (ToWorkspace blocks, 0.1 s or 0.02 s):
%     TW_time, TW_X_m, TW_TAS_mps, TW_Theta_deg, TW_q_deg,
%     TW_Alpha_deg, TW_Gamma_deg, TW_Thrust_N,
%     TW_posNED (0.02 s, Nx3), TW_Phase, TW_qcmd, TW_elev_cmd
%
%  USAGE:
%     1) run('setup_takeoff_fcs.m')
%     2) Press Run in Simulink  (stops at 1000 m)
%     3) analyze_takeoff            (optional — post-analysis figures)
%     4) Visualization_Takeoff      (this script — animation)
%
%  REFERENCE:  Visualization_part_2.m  (T-38 Descent / Landing)
%  STL ASSET:  C:\Users\tejve\Downloads\rollout\T38.stl
% ========================================================================
clearvars -except TW_time TW_X_m TW_TAS_mps TW_Theta_deg TW_q_deg ...
                  TW_Alpha_deg TW_Gamma_deg TW_Thrust_N TW_posNED ...
                  TW_Phase TW_qcmd TW_elev_cmd TW_Ntotal out;
clc; close all;

%% ========================================================================
%  0.  DATA INGESTION  —  handle 'out' bundle OR individual TW vars
%% ========================================================================
fprintf('>> [VIS] Data Processor (Takeoff Mode) initializing...\n');

% Auto-extract from Simulink ''out'' object if individual vars are absent.
if exist('out','var') && isa(out,'Simulink.SimulationOutput') && ~exist('TW_time','var')
    fprintf('>> [VIS] Extracting from Simulink SimulationOutput...\n');
    tw_list = {'TW_time','TW_X_m','TW_TAS_mps','TW_Theta_deg','TW_q_deg', ...
               'TW_Alpha_deg','TW_Gamma_deg','TW_Thrust_N', ...
               'TW_posNED','TW_Phase','TW_qcmd','TW_elev_cmd'};
    for vi = 1:numel(tw_list)
        try; eval([tw_list{vi} ' = out.' tw_list{vi} ';']); catch; end
    end
end

% Integrity check
if ~exist('TW_time','var') || ~exist('TW_X_m','var')
    error(['>> FATAL: Takeoff data not in workspace.\n' ...
           '  1) run(''setup_takeoff_fcs.m'')\n' ...
           '  2) Press Run in Simulink\n' ...
           '  3) Re-run this script.']);
end

%% ========================================================================
%  1.  SIGNAL PREPARATION
%% ========================================================================

% --- 0.1 s grid signals ---
t      = TW_time(:);
x      = TW_X_m(:);               % longitudinal position (m)
TAS    = TW_TAS_mps(:);           % true airspeed (m/s)
pitch  = TW_Theta_deg(:) * pi/180;% pitch angle (rad)
q_act  = TW_q_deg(:);             % actual pitch rate (deg/s)
alpha  = TW_Alpha_deg(:);         % angle of attack (deg)
gamma  = TW_Gamma_deg(:);         % flight path angle (deg)
thrust = TW_Thrust_N(:) / 1000;   % thrust (kN)

% Mach number (ISA sea-level a = 340.3 m/s)
mach   = TAS / 340.3;

% --- Altitude from posNED (0.02 s) → interpolate to 0.1 s grid ---
n_fc = size(TW_posNED, 1);
t_fc     = (0:n_fc-1)' * 0.02;           % 0.02 s time vector
alt_fc   = -TW_posNED(:,3);             % altitude (m): down→up
phase_fc = TW_Phase(:);                 % phase  0/1/2
qcmd_fc  = TW_qcmd(:);                 % q cmd  (deg/s)
ecmd_fc  = TW_elev_cmd(:);             % elevator cmd (deg)

alt    = interp1(t_fc, alt_fc,   t, 'linear', 'extrap');
phase  = interp1(t_fc, phase_fc, t, 'nearest','extrap');
q_cmd  = interp1(t_fc, qcmd_fc,  t, 'nearest','extrap');
e_cmd  = interp1(t_fc, ecmd_fc,  t, 'linear', 'extrap');

% Clamp altitude to >= 0 (small IC offset from ground model)
alt = max(0, alt);

L = length(t);
fprintf('>> [VIS] %d frames loaded  (%.1f s simulation).\n', L, t(end));

%% ========================================================================
%  2.  STL ASSET LOADER
%% ========================================================================
STL_Path         = 'C:\Users\tejve\Downloads\rollout\T38.stl';
Target_Faces     = 4000;

try
    fprintf('>> [VIS] Loading asset: %s\n', STL_Path);
    raw_model = stlread(STL_Path);
    fprintf('>> [VIS] Mesh loaded (%d faces).\n', size(raw_model.ConnectivityList,1));
catch
    warning('>> [VIS] STL not found — using placeholder fuselage geometry.');
    [bx,by,bz] = cylinder([0.5 1.2 1.5 1.5 1.2 0.8 0.4 0.1], 12);
    bz = bz * 8 - 1;
    pts  = [bx(:), by(:), bz(:)];
    tris = convhull(pts(:,1), pts(:,2), pts(:,3));
    raw_model = triangulation(tris, pts);
end

%% ========================================================================
%  3.  SCENE & RENDERER SETUP
%% ========================================================================
f = figure('Color','k','Name','ACFT11 Takeoff Cinematic Replay');
set(f,'WindowState','maximized');
ax = axes('Parent',f,'Color','k','GridColor',[0.3 0.3 0.3],'GridAlpha',0.4);
hold(ax,'on');  axis(ax,'equal');  grid(ax,'on');
xlabel(ax,'Longitudinal (m)','Color','w');
ylabel(ax,'Lateral (m)','Color','w');
zlabel(ax,'Altitude (m)','Color','w');
ax.XColor = 'w';  ax.YColor = 'w';  ax.ZColor = 'w';
camproj(ax,'perspective');
view(ax, 40, 12);   % azimuth 40°, elevation 12° — slight overhead-right view

% --- Runway Environment ---
Rwy_Len = max(3500, x(end) * 0.55);    % extend a little past liftoff point
Rwy_W   = 22;                            % half-width
patch(ax, [0, Rwy_Len, Rwy_Len, 0], ...
          [-Rwy_W, -Rwy_W, Rwy_W, Rwy_W], ...
          [0 0 0 0], [0.18 0.18 0.18], ...
          'EdgeColor','none','FaceAlpha',0.95);

% Runway centreline dashes
for dash_x = 0:100:Rwy_Len
    patch(ax,[dash_x, dash_x+40, dash_x+40, dash_x], ...
              [-0.8, -0.8, 0.8, 0.8], [0.01 0.01 0.01 0.01], ...
              [0.9 0.9 0.7],'EdgeColor','none','FaceAlpha',0.6);
end

% Threshold markers
patch(ax,[0, 6, 6, 0], [-Rwy_W, -Rwy_W, -Rwy_W+4, -Rwy_W+4], ...
          [0.01 0.01 0.01 0.01],[1 1 1],'EdgeColor','none');
patch(ax,[0, 6, 6, 0], [ Rwy_W-4,  Rwy_W-4,  Rwy_W,  Rwy_W], ...
          [0.01 0.01 0.01 0.01],[1 1 1],'EdgeColor','none');

% Ground plane (dark green, extends beyond runway)
patch(ax, [-500, max(x)+2000, max(x)+2000, -500], ...
          [-400, -400, 400, 400], ...
          [-0.2 -0.2 -0.2 -0.2], [0.04 0.12 0.04], ...
          'EdgeColor','none','FaceAlpha',0.7);

% --- Phase-coloured trail objects (one per phase) ---
Trail_GR  = plot3(ax, x(1), 0, alt(1), '-', 'Color',[0.20 1.00 0.20], 'LineWidth',1.8); % Ground Roll — lime
Trail_ROT = plot3(ax, x(1), 0, alt(1), '-', 'Color',[1.00 0.75 0.00], 'LineWidth',1.8); % Rotation   — amber
Trail_CLM = plot3(ax, x(1), 0, alt(1), '-', 'Color',[0.00 0.90 1.00], 'LineWidth',1.8); % Climb      — cyan

% --- Aircraft mesh + afterburner ---
CombinedObject = hgtransform('Parent', ax);

temp_p = patch(ax,'Faces',raw_model.ConnectivityList,'Vertices',raw_model.Points,'Visible','off');
if size(raw_model.ConnectivityList,1) > Target_Faces
    reducepatch(temp_p, Target_Faces / size(raw_model.ConnectivityList,1));
end
patch('Parent',CombinedObject, ...
      'Faces',temp_p.Faces,'Vertices',temp_p.Vertices, ...
      'FaceColor',[0.85 0.85 0.90], ...
      'EdgeColor','none', ...
      'FaceLighting','gouraud', ...
      'AmbientStrength',0.5, ...
      'SpecularStrength',0.4);
delete(temp_p);

% Afterburner plumes (twin engines, full thrust throughout takeoff)
AB_X_Offset = 180.0;
AB_Sep_Y    = 5.0;
AB_Length   = 60.0;
AB_Radius   = 6.5;
[cf_x,cf_y,cf_z] = cylinder([AB_Radius 0.0], 8);
cf_z    = cf_z * AB_Length;
flame_X = cf_z + AB_X_Offset;
flame_Y = cf_y;
flame_Z = cf_x;

Flame_L = surface('Parent',CombinedObject, ...
    'XData',flame_X,'YData',flame_Y - AB_Sep_Y,'ZData',flame_Z, ...
    'FaceColor',[1 0.55 0.05],'EdgeColor','none','FaceAlpha',0.85, ...
    'AmbientStrength',1.0,'Visible','on');
Flame_R = surface('Parent',CombinedObject, ...
    'XData',flame_X,'YData',flame_Y + AB_Sep_Y,'ZData',flame_Z, ...
    'FaceColor',[1 0.55 0.05],'EdgeColor','none','FaceAlpha',0.85, ...
    'AmbientStrength',1.0,'Visible','on');

% Scene lighting
light(ax,'Position',[500 -800 2000],'Style','local','Color',[1 0.95 0.9]);
light(ax,'Position',[-200 200 400],'Style','local','Color',[0.3 0.3 0.6]);
lighting(ax,'gouraud');

%% ========================================================================
%  4.  HUD — STATUS BANNER + PRIMARY FLIGHT DISPLAY
%% ========================================================================
ModelScale      = 0.1;
InitialRotation = makehgtform('xrotate',0,'zrotate',pi);
Gear_Height     = 2.5;

% --- Phase status banner ---
h_Status = uicontrol('Parent',f,'Style','text','Units','normalized', ...
    'Position',[0.28 0.88 0.44 0.05], ...
    'BackgroundColor','k','ForegroundColor',[0.2 1 0.2], ...
    'FontSize',15,'FontWeight','bold','String','PHASE: GROUND ROLL');

% --- Flight Computer panel ---
hud_bg = uipanel('Parent',f,'Position',[0.75 0.20 0.23 0.75], ...
    'BackgroundColor',[0.08 0.08 0.08], ...
    'Title','FLIGHT DATA','TitlePosition','centertop', ...
    'FontSize',11,'ForegroundColor',[1 1 1],'FontWeight','bold');

add_lbl = @(y,s) uicontrol('Parent',hud_bg,'Style','text','Units','normalized', ...
    'Position',[0.04 y 0.48 0.055],'BackgroundColor',[0.08 0.08 0.08], ...
    'ForegroundColor',[1 1 1],'HorizontalAlignment','left', ...
    'FontSize',10,'FontWeight','bold','String',s);

add_val = @(y)   uicontrol('Parent',hud_bg,'Style','text','Units','normalized', ...
    'Position',[0.50 y 0.46 0.055],'BackgroundColor',[0.08 0.08 0.08], ...
    'ForegroundColor',[1 1 1],'HorizontalAlignment','right', ...
    'FontSize',10,'FontWeight','bold','String','---');

% Row layout (y normalised inside panel, top→bottom)
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

fprintf('>> [VIS] Rendering engine started.\n');
fprintf('>> [VIS] Close the figure to stop.\n\n');

%% ========================================================================
%  5.  MAIN ANIMATION LOOP
%% ========================================================================
Win_X_Back  = 80;     % m behind aircraft in x
Win_X_Fwd   = 200;    % m ahead of aircraft in x
Win_Y       = 120;    % total lateral half-width
Win_Z_Pad   = 60;     % vertical padding above aircraft
Trail_Gap   = 12;     % m — gap kept between aircraft nose and trail end

% ---- Playback speed controls -------------------------------------------
%  ANIM_PAUSE (seconds per frame, data dt = 0.1 s):
%    0.00 = flat out (very fast)   0.05 = ~2x real-time
%    0.10 = real-time              0.20 = 2x slow-motion
ANIM_PAUSE  = 0.08;   % <-- TUNE THIS to taste
Step_GR     = 1;      % show every frame — ground roll
Step_ROT    = 1;      % show every frame — rotation  (most dramatic phase)
Step_CLM    = 1;      % show every frame — climb

% Indices where each phase segment starts/ends (for trail colouring)
idx_rot_start = find(phase >= 1, 1, 'first');
idx_clm_start = find(phase >= 2, 1, 'first');
if isempty(idx_rot_start), idx_rot_start = L+1; end
if isempty(idx_clm_start), idx_clm_start = L+1; end

k = 1;

while k <= L
    if ~isvalid(f), break; end

    curr_x     = x(k);
    curr_alt   = alt(k);
    curr_pitch = pitch(k);
    curr_phase = phase(k);

    % --- Adaptive step + phase status ---
    if curr_phase == 0
        Step   = Step_GR;
        PLabel = 'PHASE: GROUND ROLL';
        PColor = [0.20 1.00 0.20];
    elseif curr_phase == 1
        Step   = Step_ROT;
        PLabel = 'PHASE: ROTATION';
        PColor = [1.00 0.82 0.10];
    else
        Step   = Step_CLM;
        PLabel = sprintf('PHASE: CLIMB HOLD   ALT %.0f m', curr_alt);
        PColor = [0.10 0.90 1.00];
    end
    if isvalid(h_Status)
        set(h_Status,'String',PLabel,'ForegroundColor',PColor);
    end

    % --- Aircraft transform: translate + pitch rotation ---
    Trans  = makehgtform('translate',[curr_x, 0, curr_alt + Gear_Height]);
    SimRot = makehgtform('yrotate', -curr_pitch);
    Scl    = makehgtform('scale', ModelScale);
    set(CombinedObject,'Matrix', Trans * SimRot * InitialRotation * Scl);

    % --- Phase-coloured trail (gap-based, prevent wrap artefacts) ---
    %  Each trail object shows only its own phase segment up to k.
    Trail_Limit_X = curr_x - Trail_Gap;

    % Ground-roll segment  (phase 0  →  before rotation)
    i_end_gr = min(k, idx_rot_start - 1);
    if i_end_gr >= 1
        xv = x(1:i_end_gr);
        av = alt(1:i_end_gr);
        valid = xv < Trail_Limit_X;
        if any(valid)
            set(Trail_GR,'XData',xv(valid),'YData',zeros(1,sum(valid)),'ZData',av(valid)+Gear_Height);
        end
    end

    % Rotation segment  (phase 1)
    if k >= idx_rot_start && idx_rot_start <= L
        i_end_rot = min(k, idx_clm_start - 1);
        xv = x(idx_rot_start:i_end_rot);
        av = alt(idx_rot_start:i_end_rot);
        valid = xv < Trail_Limit_X;
        if any(valid)
            set(Trail_ROT,'XData',xv(valid),'YData',zeros(1,sum(valid)),'ZData',av(valid)+Gear_Height);
        end
    end

    % Climb segment  (phase 2)
    if k >= idx_clm_start && idx_clm_start <= L
        xv = x(idx_clm_start:k);
        av = alt(idx_clm_start:k);
        valid = xv < Trail_Limit_X;
        if any(valid)
            set(Trail_CLM,'XData',xv(valid),'YData',zeros(1,sum(valid)),'ZData',av(valid)+Gear_Height);
        end
    end

    % --- Afterburner flicker (full thrust throughout takeoff) ---
    Flicker = 0.60 + 0.35*rand();
    set(Flame_L,'Visible','on','FaceAlpha',Flicker);
    set(Flame_R,'Visible','on','FaceAlpha',Flicker);

    % --- HUD telemetry update ---
    set(val_TAS,  'String', sprintf('%.1f m/s', TAS(k)));
    set(val_Mach, 'String', sprintf('%.3f',     mach(k)));
    set(val_Pitch,'String', sprintf('%.1f deg', TW_Theta_deg(k)));
    set(val_Alph, 'String', sprintf('%.1f deg', alpha(k)));
    set(val_Alt,  'String', sprintf('%.1f m',   curr_alt));
    set(val_Gam,  'String', sprintf('%.1f deg', gamma(k)));
    set(val_Qact, 'String', sprintf('%.2f d/s', q_act(k)));
    set(val_Qcmd, 'String', sprintf('%.2f d/s', q_cmd(k)));
    set(val_Elev, 'String', sprintf('%.1f deg', e_cmd(k)));
    set(val_Thr,  'String', sprintf('%.1f kN',  thrust(k)));
    set(val_Dist, 'String', sprintf('%.0f m',   curr_x));
    set(val_Time, 'String', sprintf('%.1f s',   t(k)));

    % --- Dynamic camera tracking ---
    %  X: window slides with aircraft
    xlim(ax, [curr_x - Win_X_Back, curr_x + Win_X_Fwd]);
    ylim(ax, [-Win_Y, Win_Y]);
    %  Z: stay near ground during ground roll; rise during climb
    if curr_alt < Win_Z_Pad
        zlim(ax, [-5, Win_Z_Pad * 2]);
    else
        zlim(ax, [curr_alt - Win_Z_Pad, curr_alt + Win_Z_Pad * 1.5]);
    end

    drawnow;
    pause(ANIM_PAUSE);
    k = k + Step;
end

% --- Mission complete ---
if isvalid(f) && isvalid(h_Status)
    set(h_Status,'String', ...
        sprintf('MISSION COMPLETE   %.0f m  |  %.1f s', max(alt), t(end)), ...
        'ForegroundColor',[0.2 1 0.2]);
end
fprintf('>> [VIS] Replay complete.\n');
