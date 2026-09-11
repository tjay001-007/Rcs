%% analyze_takeoff.m
%  Takeoff performance analysis — uses ONLY ToWorkspace variables.
%  Works regardless of Simulink workspace-output settings or MATLAB version.
%
%  Required variables (all saved by ToWorkspace blocks at 0.1 s):
%    TW_time      TW_TAS_mps   TW_Theta_deg  TW_q_deg
%    TW_Alpha_deg TW_X_m       TW_Thrust_N   TW_Gamma_deg
%    TW_Ntotal    TW_posNED    TW_Phase       TW_qcmd   TW_elev_cmd
%
%  Usage:
%    1) run('setup_takeoff_fcs.m')
%    2) Press Run in Simulink  (stops automatically at 1000 m)
%    3) analyze_takeoff
% -------------------------------------------------------------------------

%% =========================================================================
%  0.  CHECK DATA EXISTS  — handle 'out' bundle OR individual TW vars
%% =========================================================================
% Newer Simulink (ReturnWorkspaceOutputs=on) packs everything into 'out'.
% Extract to individual variables so the rest of the script is uniform.
if exist('out','var') && isa(out,'Simulink.SimulationOutput') && ~exist('TW_time','var')
    fprintf('[analyze] Extracting signals from Simulink ''out'' object...\n');
    tw_list = {'TW_time','TW_TAS_mps','TW_Theta_deg','TW_q_deg', ...
               'TW_Alpha_deg','TW_X_m','TW_Thrust_N','TW_Gamma_deg', ...
               'TW_Ntotal','TW_posNED','TW_Phase','TW_qcmd','TW_elev_cmd'};
    for vi = 1:numel(tw_list)
        try; eval([tw_list{vi} ' = out.' tw_list{vi} ';']); catch; end
    end
end

if ~exist('TW_time','var')
    fprintf('\nWorkspace contents:\n'); whos
    error(['No ToWorkspace data found.\n' ...
           '1) run(''setup_takeoff_fcs.m'')\n' ...
           '2) Press Run in Simulink\n' ...
           '3) Run this script again']);
end

%% =========================================================================
%  1.  LOAD ALL SIGNALS  (everything from ToWorkspace, 0.1 s grid)
%% =========================================================================
ts        = TW_time(:);
TAS_mps   = TW_TAS_mps(:);
Theta_deg = TW_Theta_deg(:);
q_deg     = TW_q_deg(:);
Alpha_deg = TW_Alpha_deg(:);
X_m       = TW_X_m(:);
Thrust_N  = TW_Thrust_N(:);
Gamma_deg = TW_Gamma_deg(:);

% Gear and position (may be at a different rate — interpolate onto ts)
if exist('TW_Ntotal','var') && ~isempty(TW_Ntotal)
    t_N    = linspace(0, ts(end), numel(TW_Ntotal))';
    Ntot   = interp1(t_N, TW_Ntotal(:), ts, 'linear','extrap');
    t_pos  = linspace(0, ts(end), size(TW_posNED,1))';
    alt_m  = interp1(t_pos, -TW_posNED(:,3), ts, 'linear','extrap');
else
    Ntot  = zeros(size(ts));
    alt_m = zeros(size(ts));
    warning('[analyze] TW_Ntotal / TW_posNED missing — gear and altitude unavailable.');
end

% FCS signals (0.02 s from discrete controller — interpolate onto ts)
if exist('TW_Phase','var') && ~isempty(TW_Phase)
    t_fcs    = linspace(0, ts(end), numel(TW_Phase))';
    Phase    = interp1(t_fcs, TW_Phase(:),    ts, 'linear','extrap');
    qcmd     = interp1(t_fcs, TW_qcmd(:),     ts, 'linear','extrap');
    elev_cmd = interp1(t_fcs, TW_elev_cmd(:), ts, 'linear','extrap');
else
    Phase    = zeros(size(ts));
    qcmd     = zeros(size(ts));
    elev_cmd = zeros(size(ts));
    warning('[analyze] TW_Phase / TW_qcmd / TW_elev_cmd missing.');
end

%% =========================================================================
%  2.  AIRCRAFT CONSTANTS
%% =========================================================================
Mass = 24650;  g_n = 9.80665;  W = Mass*g_n;  Tmax = 140000;

%% =========================================================================
%  3.  EVENT DETECTION
%% =========================================================================
% Throttle-up
i_thr = find(Thrust_N > 0.05*Tmax, 1);

% V_R: TAS >= 110 m/s while still on ground (Phase 0)
i_VR = find(TAS_mps >= 110 & Phase < 1, 1);
if isempty(i_VR),  i_VR = find(TAS_mps >= 110, 1);  end

% FCS phase transitions
i_rot = find(Phase >= 0.9, 1);    % Phase 0 -> 1 (rotation)
i_clm = find(Phase >= 1.9, 1);    % Phase 1 -> 2 (climb hold)

% Liftoff: gear < 500 N and alt > 0.3 m
i_lo  = find(Ntot < 500 & alt_m > 0.3, 1);

% 1000 m altitude
i_1km = find(alt_m >= 1000, 1);

%% =========================================================================
%  4.  CONSOLE PERFORMANCE REPORT
%% =========================================================================
fprintf('\n');
fprintf('=============================================================\n');
fprintf('           TAKEOFF PERFORMANCE REPORT\n');
fprintf('=============================================================\n');
fprintf('  Aircraft : Mass=%g kg | Tmax=%g N | T/W=%.2f\n', Mass, Tmax, Tmax/W);
fprintf('  Sim end  : t=%.1f s | max alt=%.0f m\n', ts(end), max(alt_m));

fprintf('\n--- GROUND ROLL ---\n');
if ~isempty(i_thr)
    fprintf('  Throttle-up  : t = %.1f s\n', ts(i_thr));
end
if ~isempty(i_VR)
    fprintf('  V_R (110 m/s): t = %.1f s | dist = %.0f m\n', ts(i_VR), X_m(i_VR));
    fprintf('  TAS at V_R   : %.1f m/s\n', TAS_mps(i_VR));
    if ~isempty(i_thr) && i_VR > i_thr
        dt_gr   = ts(i_VR) - ts(i_thr);
        avg_acc = TAS_mps(i_VR) / dt_gr;
        fprintf('  Avg accel    : %.2f m/s^2 (%.3f g) | roll time = %.1f s\n', ...
            avg_acc, avg_acc/g_n, dt_gr);
    end
end

fprintf('\n--- ROTATION ---\n');
if ~isempty(i_rot)
    fprintf('  Rotation start: t=%.1f s | TAS=%.1f m/s | theta=%.1f deg\n', ...
        ts(i_rot), TAS_mps(i_rot), Theta_deg(i_rot));
end
if ~isempty(i_lo)
    fprintf('  Liftoff       : t=%.1f s | dist=%.0f m\n', ts(i_lo), X_m(i_lo));
    fprintf('                  TAS=%.1f m/s | theta=%.1f deg | alpha=%.1f deg | elev=%.1f deg\n', ...
        TAS_mps(i_lo), Theta_deg(i_lo), Alpha_deg(i_lo), elev_cmd(i_lo));
    if ~isempty(i_rot) && i_lo > i_rot
        dt_rot = ts(i_lo) - ts(i_rot);
        dth    = Theta_deg(i_lo) - Theta_deg(i_rot);
        fprintf('  Rot duration  : %.1f s | delta-theta=%.1f deg | mean q=%.2f deg/s\n', ...
            dt_rot, dth, dth/max(dt_rot,0.1));
    end
end

fprintf('\n--- CLIMB ---\n');
checkpoints = [50 100 200 300 500 1000];
t_prev = ts(max(i_lo,1));  a_prev = 0;
for ca = checkpoints
    ic = find(alt_m >= ca, 1);
    if ~isempty(ic)
        dt_s = max(ts(ic)-t_prev, 0.01);
        vs   = (alt_m(ic)-a_prev) / dt_s;
        fprintf('  %4.0f m: t=%6.1f s | TAS=%5.1f m/s | theta=%5.1f deg | gamma=%5.1f deg | VS=%5.0f m/s\n', ...
            ca, ts(ic), TAS_mps(ic), Theta_deg(ic), Gamma_deg(ic), vs);
        t_prev = ts(ic);  a_prev = alt_m(ic);
    end
end

if ~isempty(i_1km)
    fprintf('\n  *** 1000 m at t=%.1f s | dist=%.0f m | TAS=%.1f m/s ***\n', ...
        ts(i_1km), X_m(i_1km), TAS_mps(i_1km));
else
    fprintf('\n  Max alt = %.0f m (sim ended before 1000 m)\n', max(alt_m));
end
fprintf('=============================================================\n\n');

%% =========================================================================
%  5.  FIGURE 1 — GROUND ROLL
%% =========================================================================
n_gr = max([i_lo, i_VR, 2]);
if isempty(n_gr) || n_gr > numel(ts), n_gr = numel(ts); end
tgr = ts(1:n_gr);

fig1 = figure('Name','Ground Roll Analysis','Color','w', ...
    'Position',[20 40 1300 820],'NumberTitle','off');

subplot(3,3,1);
plot(tgr, TAS_mps(1:n_gr),'b','LineWidth',1.8); hold on
yline(110,'k:','V_R'); grid on
if ~isempty(i_VR)                          , xline(ts(i_VR),'r--','V_R','LabelVerticalAlignment','bottom','FontSize',8); end
if ~isempty(i_lo) && i_lo<=n_gr           , xline(ts(i_lo),'g--','Liftoff','LabelVerticalAlignment','bottom','FontSize',8); end
xlabel('t (s)'); ylabel('TAS (m/s)'); title('Airspeed');

subplot(3,3,2);
plot(tgr, Thrust_N(1:n_gr)/1e3,'r','LineWidth',1.8); hold on
yline(Tmax/1e3,'k:','T_{max}'); grid on
if ~isempty(i_VR), xline(ts(i_VR),'r--'); end
xlabel('t (s)'); ylabel('Thrust (kN)'); title('Thrust');

subplot(3,3,3);
acc_v = gradient(TAS_mps(1:n_gr), tgr);
plot(TAS_mps(1:n_gr), acc_v,'m','LineWidth',1.5); hold on
xline(110,'r--','V_R'); grid on
xlabel('TAS (m/s)'); ylabel('a (m/s^2)'); title('Accel vs Speed');

subplot(3,3,4);
plot(X_m(1:n_gr), TAS_mps(1:n_gr),'k','LineWidth',1.5); hold on
yline(110,'r--','V_R'); grid on
xlabel('Dist (m)'); ylabel('TAS (m/s)'); title('Speed vs Distance');

subplot(3,3,5);
plot(tgr, Ntot(1:n_gr)/1e3,'Color',[0.8 0.4 0],'LineWidth',1.5); hold on
yline(W/1e3,'b:','W'); yline(0,'k:'); grid on
if ~isempty(i_VR)              , xline(ts(i_VR),'r--'); end
if ~isempty(i_lo) && i_lo<=n_gr, xline(ts(i_lo),'g--'); end
xlabel('t (s)'); ylabel('N_{gear} (kN)'); title('Gear Load');

subplot(3,3,6);
plot(tgr, Theta_deg(1:n_gr),'r','LineWidth',1.5); hold on
yline(2,'b:','IC 2°'); grid on
if ~isempty(i_VR)              , xline(ts(i_VR),'r--'); end
if ~isempty(i_lo) && i_lo<=n_gr, xline(ts(i_lo),'g--'); end
xlabel('t (s)'); ylabel('\theta (deg)'); title('Pitch');

subplot(3,3,7);
plot(tgr, Alpha_deg(1:n_gr),'Color',[0 0.6 0],'LineWidth',1.5); hold on; grid on
if ~isempty(i_VR), xline(ts(i_VR),'r--'); end
xlabel('t (s)'); ylabel('\alpha (deg)'); title('AoA');

subplot(3,3,8);
plot(tgr, elev_cmd(1:n_gr),'Color',[0.5 0 0.8],'LineWidth',1.5); hold on
yline(0,'k:'); yline(-25,'r:'); yline(25,'r:'); grid on
if ~isempty(i_VR)              , xline(ts(i_VR),'r--'); end
if ~isempty(i_lo) && i_lo<=n_gr, xline(ts(i_lo),'g--'); end
xlabel('t (s)'); ylabel('\delta_e (deg)'); title('Elevator (INDI)');

subplot(3,3,9);
stairs(tgr, Phase(1:n_gr),'b','LineWidth',2);
yticks([0 1 2]); yticklabels({'GndRoll','Rotation','Climb'});
ylim([-0.2 2.2]); grid on; xlabel('t (s)'); title('FCS Phase');

sgtitle('Ground Roll Analysis','FontSize',13,'FontWeight','bold');

%% =========================================================================
%  6.  FIGURE 2 — ROTATION & LIFTOFF
%% =========================================================================
if isempty(i_rot), i_rot = max(i_VR,1); end
dt0   = ts(2)-ts(1);
i_rs  = max(1, i_rot - round(3/dt0));
i_re  = min(numel(ts), max(i_lo  + round(6/dt0), i_rot + round(12/dt0)));
idx_r = i_rs:i_re;
tr    = ts(idx_r);

fig2 = figure('Name','Rotation & Liftoff','Color','w', ...
    'Position',[40 40 1280 760],'NumberTitle','off');

subplot(2,3,1);
plot(tr, Theta_deg(idx_r),'r','LineWidth',1.8); hold on
yline(12,'k:','\theta_{cmd}=12°'); grid on
if ~isempty(i_rot) && i_rot>=i_rs, xline(ts(i_rot),'b--','Rot start','FontSize',8,'LabelVerticalAlignment','bottom'); end
if ~isempty(i_lo)  && i_lo >=i_rs, xline(ts(i_lo), 'g--','Liftoff',  'FontSize',8,'LabelVerticalAlignment','bottom'); end
xlabel('t (s)'); ylabel('\theta (deg)'); title('Pitch Angle');

subplot(2,3,2);
plot(tr, q_deg(idx_r),   'm',  'LineWidth',1.8); hold on
plot(tr, qcmd(idx_r),    'k--','LineWidth',1.2);
legend('q actual','q_{cmd}','Location','best'); grid on
if ~isempty(i_rot) && i_rot>=i_rs, xline(ts(i_rot),'b--'); end
if ~isempty(i_lo)  && i_lo >=i_rs, xline(ts(i_lo), 'g--'); end
xlabel('t (s)'); ylabel('q (deg/s)'); title('Pitch Rate — INDI Tracking');

subplot(2,3,3);
plot(tr, elev_cmd(idx_r),'Color',[0.5 0 0.8],'LineWidth',1.8); hold on
yline(0,'k:'); yline(-25,'r:'); yline(25,'r:'); grid on
if ~isempty(i_rot) && i_rot>=i_rs, xline(ts(i_rot),'b--'); end
if ~isempty(i_lo)  && i_lo >=i_rs, xline(ts(i_lo), 'g--'); end
xlabel('t (s)'); ylabel('\delta_e (deg)'); title('Elevator Cmd (INDI)');

subplot(2,3,4);
plot(tr, Ntot(idx_r)/1e3,'Color',[0.8 0.4 0],'LineWidth',1.8); hold on
yline(0,'k:'); yline(W/1e3,'b:','W'); grid on
if ~isempty(i_rot) && i_rot>=i_rs, xline(ts(i_rot),'b--'); end
if ~isempty(i_lo)  && i_lo >=i_rs, xline(ts(i_lo), 'g--'); end
xlabel('t (s)'); ylabel('N_{gear} (kN)'); title('Gear Normal Force');

subplot(2,3,5);
plot(tr, Alpha_deg(idx_r),'Color',[0 0.6 0],'LineWidth',1.8); hold on; grid on
if ~isempty(i_rot) && i_rot>=i_rs, xline(ts(i_rot),'b--'); end
if ~isempty(i_lo)  && i_lo >=i_rs, xline(ts(i_lo), 'g--'); end
xlabel('t (s)'); ylabel('\alpha (deg)'); title('AoA');

subplot(2,3,6);
plot(tr, alt_m(idx_r),'b','LineWidth',1.8); hold on; grid on
if ~isempty(i_lo)  && i_lo >=i_rs, xline(ts(i_lo), 'g--','Liftoff','FontSize',8); end
if ~isempty(i_clm) && i_clm>=i_rs, xline(ts(i_clm),'r--','Climb',  'FontSize',8); end
xlabel('t (s)'); ylabel('Alt (m)'); title('Altitude Detail');

sgtitle('Rotation & Liftoff Analysis','FontSize',13,'FontWeight','bold');

%% =========================================================================
%  7.  FIGURE 3 — CLIMB TO 1000 m
%% =========================================================================
i_cls = max(i_lo, 1);
idx_c = i_cls:numel(ts);
tc    = ts(idx_c);

fig3 = figure('Name','Climb to 1000 m','Color','w', ...
    'Position',[60 40 1300 820],'NumberTitle','off');

subplot(2,4,1);
plot(tc, alt_m(idx_c),'b','LineWidth',1.8); hold on
yline(1000,'k:','1000 m'); grid on
if ~isempty(i_1km), xline(ts(i_1km),'r--','1000 m','FontSize',8,'LabelVerticalAlignment','bottom'); end
xlabel('t (s)'); ylabel('Alt (m)'); title('Altitude vs Time');

subplot(2,4,2);
plot(X_m(idx_c)/1e3, alt_m(idx_c),'k','LineWidth',1.8); hold on
yline(1000,'k:');
if ~isempty(i_1km), plot(X_m(i_1km)/1e3, 1000,'ro','MarkerSize',10,'MarkerFaceColor','r'); end
grid on; xlabel('Dist (km)'); ylabel('Alt (m)'); title('Flight Profile');

subplot(2,4,3);
plot(TAS_mps(idx_c), alt_m(idx_c),'m','LineWidth',1.8); hold on
yline(1000,'k:'); grid on
xlabel('TAS (m/s)'); ylabel('Alt (m)'); title('Speed vs Altitude');

subplot(2,4,4);
VS = gradient(alt_m(idx_c), tc);
plot(tc, VS,'Color',[0 0.6 0],'LineWidth',1.5); hold on
yline(0,'k:'); grid on
if ~isempty(i_1km), xline(ts(i_1km),'r--'); end
xlabel('t (s)'); ylabel('VS (m/s)'); title('Vertical Speed');

subplot(2,4,5);
plot(tc, Theta_deg(idx_c),'r','LineWidth',1.5); hold on
yline(12,'k:','\theta_{cmd}'); grid on
if ~isempty(i_1km), xline(ts(i_1km),'r--'); end
xlabel('t (s)'); ylabel('\theta (deg)'); title('Pitch');

subplot(2,4,6);
plot(tc, Alpha_deg(idx_c),'Color',[0 0.6 0],'LineWidth',1.5); hold on; grid on
if ~isempty(i_1km), xline(ts(i_1km),'r--'); end
xlabel('t (s)'); ylabel('\alpha (deg)'); title('AoA');

subplot(2,4,7);
plot(tc, elev_cmd(idx_c),'Color',[0.5 0 0.8],'LineWidth',1.5); hold on
yline(0,'k:'); yline(-25,'r:'); yline(25,'r:'); grid on
if ~isempty(i_1km), xline(ts(i_1km),'r--'); end
xlabel('t (s)'); ylabel('\delta_e (deg)'); title('Elevator (INDI)');

subplot(2,4,8);
plot(tc, q_deg(idx_c),'m','LineWidth',1.5); hold on
plot(tc, qcmd(idx_c), 'k--','LineWidth',1.0);
legend('q','q_{cmd}','Location','best'); grid on
if ~isempty(i_1km), xline(ts(i_1km),'r--'); end
xlabel('t (s)'); ylabel('q (deg/s)'); title('Pitch Rate Tracking');

sgtitle('Climb to 1000 m','FontSize',13,'FontWeight','bold');

%% =========================================================================
%  8.  SAVE FIGURES
%% =========================================================================
try
    outdir = 'C:\Users\tejve\Downloads\acft';
    saveas(fig1, fullfile(outdir,'fig_ground_roll.png'));
    saveas(fig2, fullfile(outdir,'fig_rotation.png'));
    saveas(fig3, fullfile(outdir,'fig_climb.png'));
    fprintf('Figures saved to: %s\n', outdir);
catch e
    fprintf('Figure save skipped: %s\n', e.message);
end
