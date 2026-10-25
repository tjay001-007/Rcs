%% analyze_bfl.m
%  Balanced Field Length (BFL) — comprehensive post-simulation analysis.
%  Run after setup_bfl_analysis.m + Simulink Run.
%
%  Produces 4 figures:
%    Fig 1  Full simulation timeline (3×3, 9 panels)
%    Fig 2  GO scenario  — OEI climb to 100 m (2×3, 6 panels)
%    Fig 3  STOP scenario — braking performance (2×3, 6 panels)
%    Fig 4  BFL summary  — distance breakdown + balance assessment
%
%  Simulation auto-stops at:
%    GO  : alt >= 100 m  (well above 35 ft obstacle)
%    STOP: TAS <= 1 m/s  (aircraft fully stopped)
% -----------------------------------------------------------------------

%% =========================================================================
%  0.  CONSTANTS & LOAD DATA
%% =========================================================================
Mass = 28500;  g_n = 9.80665;  W = Mass*g_n;  Tmax = 140000;
ALT_35FT = 10.668;    % 35 ft [m]
ALT_STOP = 100.0;     % GO sim stop altitude [m]

% Try to pull from 'out' object if workspace scalars not yet set
if exist('out','var') && isa(out,'Simulink.SimulationOutput') && ~exist('TW_time','var')
    tw_list = {'TW_time','TW_TAS_mps','TW_Theta_deg','TW_q_deg', ...
               'TW_Alpha_deg','TW_X_m','TW_Thrust_N','TW_Gamma_deg', ...
               'TW_Ntotal','TW_posNED','TW_Phase','TW_qcmd', ...
               'TW_elev_cmd','TW_Throttle'};
    for vi = 1:numel(tw_list)
        try; eval([tw_list{vi} ' = out.' tw_list{vi} ';']); catch; end
    end
end

if ~exist('TW_time','var')
    error('[analyze_bfl] No data. Run setup_bfl_analysis.m -> Simulink Run -> this script.');
end

%% =========================================================================
%  1.  SIGNAL EXTRACTION & RE-SAMPLING TO COMMON TIME BASE
%% =========================================================================
ts        = TW_time(:);
TAS_mps   = TW_TAS_mps(:);
Theta_deg = TW_Theta_deg(:);
q_deg     = TW_q_deg(:);
Alpha_deg = TW_Alpha_deg(:);
X_m       = TW_X_m(:);
N         = numel(ts);

% Altitude (NED z -> positive-up)
if exist('TW_posNED','var') && ~isempty(TW_posNED)
    t_pos = linspace(0, ts(end), size(TW_posNED,1))';
    alt_m = interp1(t_pos, -TW_posNED(:,3), ts, 'linear','extrap');
else
    alt_m = zeros(N,1);
    warning('[analyze_bfl] TW_posNED missing — altitude set to 0.');
end

% Gear load
if exist('TW_Ntotal','var') && ~isempty(TW_Ntotal)
    t_N  = linspace(0, ts(end), numel(TW_Ntotal))';
    Ntot = interp1(t_N, TW_Ntotal(:), ts, 'linear','extrap');
else
    Ntot = zeros(N,1);
end

% Thrust
if exist('TW_Thrust_N','var') && ~isempty(TW_Thrust_N)
    t_T    = linspace(0, ts(end), numel(TW_Thrust_N))';
    Thrust = interp1(t_T, TW_Thrust_N(:), ts, 'linear','extrap');
else
    Thrust = Tmax * ones(N,1);
end

% Flight path angle Gamma
if exist('TW_Gamma_deg','var') && ~isempty(TW_Gamma_deg)
    t_G   = linspace(0, ts(end), numel(TW_Gamma_deg))';
    Gamma = interp1(t_G, TW_Gamma_deg(:), ts, 'linear','extrap');
else
    Gamma = zeros(N,1);
end

% FCS signals (0.02 s sample)
if exist('TW_Phase','var') && ~isempty(TW_Phase)
    t_fcs    = linspace(0, ts(end), numel(TW_Phase))';
    Phase    = interp1(t_fcs, TW_Phase(:),     ts, 'nearest','extrap');
    qcmd     = interp1(t_fcs, TW_qcmd(:),      ts, 'nearest','extrap');
    elev_cmd = interp1(t_fcs, TW_elev_cmd(:),  ts, 'nearest','extrap');
else
    Phase = zeros(N,1);  qcmd = zeros(N,1);  elev_cmd = zeros(N,1);
end

% Throttle
if exist('TW_Throttle','var') && ~isempty(TW_Throttle)
    t_thr = linspace(0, ts(end), numel(TW_Throttle))';
    Thr   = interp1(t_thr, TW_Throttle(:), ts, 'linear','extrap');
else
    Thr = ones(N,1);
end

%% =========================================================================
%  2.  DERIVED SIGNALS
%% =========================================================================
mu_vec = 0.03 * ones(N,1);
mu_vec(Phase >= 9.5) = 0.3;

decel = -gradient(TAS_mps, ts);           % m/s^2  (positive = decelerating)
VS    =  gradient(alt_m,   ts);           % m/s    vertical speed

% Climb gradient (only meaningful in climb phases 3)
V_horiz = TAS_mps .* max(cosd(Gamma), 0.01);
climb_grad_pct = (VS ./ V_horiz) * 100;
climb_grad_pct(Phase < 2.9 | Phase > 5) = NaN;

% Net braking force
brake_force = Mass .* decel - Thrust;
brake_force(Phase < 9.5) = NaN;

%% =========================================================================
%  3.  EVENT DETECTION
%% =========================================================================
is_go   = any(Phase >= 1 & Phase <= 4);
is_stop = any(Phase >= 9.5);

i_V1   = find(Phase >= 0.9,                          1);
i_VR   = find(Phase >= 1.9,                          1);
i_lo   = find(Ntot < 500 & alt_m > 0.3,              1);
i_35ft = find(alt_m >= ALT_35FT,                     1);
i_100m = find(alt_m >= ALT_STOP,                     1);
i_stp  = find(TAS_mps < 1.5 & Phase >= 9.5,         1);

V1_val = safe_get(TAS_mps, i_V1, NaN);

%% =========================================================================
%  4.  CONSOLE REPORT
%% =========================================================================
if is_go, scen = 'GO (continue, OEI climb)';
else,     scen = 'STOP (braking to full stop)'; end

fprintf('\n');
fprintf('================================================================\n');
fprintf('  BFL ANALYSIS  —  %s\n', scen);
fprintf('  T_sim=%.1f s  |  T_total=%.0f kN  |  T_OEI=%.0f kN\n', ...
    ts(end), Tmax/1e3, Tmax/2e3);
fprintf('================================================================\n');

fprintf('\n  ENGINE FAILURE POINT (V1)\n');
if ~isempty(i_V1)
    fprintf('    V1  = %.1f m/s (%.0f kt)  t=%.1f s  X=%.0f m\n', ...
        V1_val, V1_val/0.51444, ts(i_V1), X_m(i_V1));
end

if is_go
    fprintf('\n  GO SCENARIO\n');
    if ~isempty(i_VR),  fprintf('    VR     : %.1f m/s  t=%.1f s  X=%.0f m\n', ...
            TAS_mps(i_VR), ts(i_VR), X_m(i_VR)); end
    if ~isempty(i_lo),  fprintf('    Liftoff: t=%.1f s  X=%.0f m  TAS=%.1f m/s  th=%.1f deg\n', ...
            ts(i_lo), X_m(i_lo), TAS_mps(i_lo), Theta_deg(i_lo)); end
    if ~isempty(i_35ft),fprintf('    35 ft  : t=%.1f s  X=%.0f m  TAS=%.1f m/s  gam=%.1f deg\n', ...
            ts(i_35ft), X_m(i_35ft), TAS_mps(i_35ft), Gamma(i_35ft)); end
    if ~isempty(i_100m),fprintf('    100 m  : t=%.1f s  X=%.0f m  TAS=%.1f m/s\n', ...
            ts(i_100m), X_m(i_100m), TAS_mps(i_100m)); end

    fprintf('\n  DISTANCE BREAKDOWN (GO)\n');
    if ~isempty(i_V1)
        fprintf('    0 to V1         (2-eng roll) : %6.0f m\n', X_m(i_V1)); end
    if ~isempty(i_V1) && ~isempty(i_VR) && i_VR > i_V1
        fprintf('    V1 to VR        (OEI roll)   : %6.0f m\n', X_m(i_VR)-X_m(i_V1)); end
    if ~isempty(i_VR) && ~isempty(i_lo) && i_lo > i_VR
        fprintf('    VR to Liftoff   (rotation)   : %6.0f m\n', X_m(i_lo)-X_m(i_VR)); end
    if ~isempty(i_lo) && ~isempty(i_35ft) && i_35ft > i_lo
        fprintf('    LO to 35ft      (airborne)   : %6.0f m\n', X_m(i_35ft)-X_m(i_lo)); end
    if ~isempty(i_35ft)
        D_go = X_m(i_35ft);
        fprintf('    -----------------------------------------\n');
        fprintf('    ACCEL-GO DIST (to 35 ft)    : %6.0f m  (%.0f ft)\n', D_go, D_go/0.3048);
        assignin('base','D_go', D_go);
    end
end

if is_stop
    fprintf('\n  STOP SCENARIO\n');
    if ~isempty(i_V1) && ~isempty(i_stp)
        dx_b = X_m(i_stp)-X_m(i_V1);
        dt_b = ts(i_stp)-ts(i_V1);
        a_avg= TAS_mps(i_V1)/max(dt_b,0.01);
        fprintf('    Braking dist : %.0f m  (%.0f ft)\n', dx_b, dx_b/0.3048);
        fprintf('    Stop time    : %.1f s  |  Avg decel : %.2f m/s2 (%.3f g)\n', ...
            dt_b, a_avg, a_avg/g_n);
    end
    fprintf('\n  DISTANCE BREAKDOWN (STOP)\n');
    if ~isempty(i_V1)
        fprintf('    0 to V1         (2-eng roll) : %6.0f m\n', X_m(i_V1)); end
    if ~isempty(i_V1) && ~isempty(i_stp) && i_stp > i_V1
        fprintf('    V1 to Stop      (braking)    : %6.0f m\n', X_m(i_stp)-X_m(i_V1)); end
    if ~isempty(i_stp)
        D_stop = X_m(i_stp);
        fprintf('    -----------------------------------------\n');
        fprintf('    ACCEL-STOP DIST             : %6.0f m  (%.0f ft)\n', D_stop, D_stop/0.3048);
        assignin('base','D_stop', D_stop);
    end
end

if exist('D_go','var') && exist('D_stop','var')
    diff_m = D_go - D_stop;
    fprintf('\n  BFL BALANCE CHECK\n');
    fprintf('    D_go=%.0f m  |  D_stop=%.0f m  |  Diff=%+.0f m\n', D_go, D_stop, diff_m);
    if abs(diff_m) < 25
        BFL = (D_go+D_stop)/2;
        fprintf('    BALANCED!  BFL = %.0f m  (%.0f ft)\n', BFL, BFL/0.3048);
    elseif diff_m > 0
        fprintf('    D_go > D_stop by %.0f m -- lower V1 to balance\n', diff_m);
    else
        fprintf('    D_stop > D_go by %.0f m -- raise V1 to balance\n', -diff_m);
    end
end
fprintf('================================================================\n\n');

%% =========================================================================
%  COLOUR PALETTE
%% =========================================================================
C.blue = [0.00 0.45 0.74];
C.red  = [0.84 0.09 0.09];
C.grn  = [0.13 0.54 0.13];
C.pur  = [0.49 0.18 0.56];
C.org  = [0.93 0.53 0.18];
C.dk   = [0.15 0.15 0.15];

%% =========================================================================
%  FIG 1 — Full Simulation Timeline (3x3)
%% =========================================================================
fig1 = figure('Name','BFL Timeline','Color','w', ...
    'Position',[20 40 1380 860],'NumberTitle','off');

subplot(3,3,1);
plot(ts,TAS_mps,'Color',C.blue,'LineWidth',1.8); hold on;
if ~isempty(i_V1), yline(V1_val,'r:','V1','FontSize',7,'LabelHorizontalAlignment','right'); end
yline(110,'k:','VR','FontSize',7,'LabelHorizontalAlignment','right');
add_event_lines(ts,i_V1,i_VR,i_lo,i_35ft,i_stp);
grid on; xlabel('t (s)'); ylabel('TAS (m/s)'); title('True Airspeed');

subplot(3,3,2);
stairs(ts,Phase,'Color',C.dk,'LineWidth',2); hold on;
add_event_lines(ts,i_V1,i_VR,i_lo,i_35ft,i_stp);
grid on; xlabel('t (s)');
yticks([0 1 2 3 10]); yticklabels({'GndRoll','OEI Roll','ROT','CLIMB','BRAKE'});
ylim([-0.5 11]); title('FCS Phase');

subplot(3,3,3);
plot(ts,Thr*100,'Color',C.org,'LineWidth',1.8); hold on;
yline(100,'k:','2-eng','FontSize',7); yline(50,'b:','OEI','FontSize',7);
yline(5,'r:','Idle','FontSize',7);
add_event_lines(ts,i_V1,i_VR,i_lo,i_35ft,i_stp);
grid on; xlabel('t (s)'); ylabel('%'); title('Throttle'); ylim([0 110]);

subplot(3,3,4);
plot(ts,alt_m,'Color',C.blue,'LineWidth',1.8); hold on;
yline(ALT_35FT,'r--','35 ft','FontSize',7);
yline(ALT_STOP,'b--','100 m','FontSize',7);
add_event_lines(ts,i_V1,i_VR,i_lo,i_35ft,i_stp);
grid on; xlabel('t (s)'); ylabel('Alt (m)'); title('Altitude');

subplot(3,3,5);
plot(ts,Ntot/1e3,'Color',C.org,'LineWidth',1.5); hold on;
yline(W/1e3,'b:','W','FontSize',7); yline(0,'k:');
add_event_lines(ts,i_V1,i_VR,i_lo,i_35ft,i_stp);
grid on; xlabel('t (s)'); ylabel('N_{gear} (kN)'); title('Gear Normal Force');

subplot(3,3,6);
plot(ts,mu_vec,'Color',C.red,'LineWidth',2); hold on;
yline(0.03,'k:','\mu_{roll}','FontSize',7,'LabelHorizontalAlignment','right');
yline(0.30,'r:','\mu_{brk}', 'FontSize',7,'LabelHorizontalAlignment','right');
add_event_lines(ts,i_V1,i_VR,i_lo,i_35ft,i_stp);
grid on; xlabel('t (s)'); ylabel('\mu'); title('Ground Friction'); ylim([0 0.35]);

subplot(3,3,7);
plot(ts,elev_cmd,'Color',C.pur,'LineWidth',1.5); hold on;
yline(0,'k:');
add_event_lines(ts,i_V1,i_VR,i_lo,i_35ft,i_stp);
grid on; xlabel('t (s)'); ylabel('\delta_e (deg)'); title('Elevator Command (INDI)');

subplot(3,3,8);
plot(ts,q_deg,'Color',C.pur,'LineWidth',1.8); hold on;
plot(ts,qcmd,'k--','LineWidth',1.2);
add_event_lines(ts,i_V1,i_VR,i_lo,i_35ft,i_stp);
grid on; legend('q_{act}','q_{cmd}','Location','best','FontSize',7);
xlabel('t (s)'); ylabel('q (deg/s)'); title('Pitch Rate INDI Tracking');

subplot(3,3,9);
plot(ts,Theta_deg,'Color',C.red,'LineWidth',1.5); hold on;
plot(ts,Alpha_deg,'Color',C.grn,'LineWidth',1.5);
yline(12,'k:','\theta_{cmd}=12 deg','FontSize',7);
add_event_lines(ts,i_V1,i_VR,i_lo,i_35ft,i_stp);
grid on; legend('\theta','\alpha','Location','best','FontSize',7);
xlabel('t (s)'); ylabel('(deg)'); title('Pitch Angle & AoA');

sgtitle(sprintf('BFL — %s  |  V1=%.1f m/s', scen, V1_val), ...
    'FontSize',12,'FontWeight','bold');

%% =========================================================================
%  FIG 2 — GO: OEI Climb Performance
%% =========================================================================
if is_go && ~isempty(i_lo)
    i_air = i_lo:N;
    t_air = ts(i_air);
    fig2 = figure('Name','BFL OEI Climb','Color','w', ...
        'Position',[40 40 1300 720],'NumberTitle','off');

    subplot(2,3,1);
    plot(X_m(i_air)/1e3, alt_m(i_air),'Color',C.blue,'LineWidth',2); hold on;
    yline(ALT_35FT,'r--','35 ft','FontSize',8);
    yline(ALT_STOP,'b--','100 m','FontSize',8);
    if ~isempty(i_35ft)&&i_35ft>=i_lo
        plot(X_m(i_35ft)/1e3,alt_m(i_35ft),'rv','MarkerSize',10,'MarkerFaceColor','r'); end
    if ~isempty(i_100m)&&i_100m>=i_lo
        plot(X_m(i_100m)/1e3,alt_m(i_100m),'b^','MarkerSize',10,'MarkerFaceColor','b'); end
    grid on; xlabel('Distance (km)'); ylabel('Alt (m)');
    title('OEI Flight Profile (Alt vs X)');

    subplot(2,3,2);
    plot(TAS_mps(i_air),alt_m(i_air),'Color',C.org,'LineWidth',2); hold on;
    yline(ALT_35FT,'r--'); yline(ALT_STOP,'b--');
    grid on; xlabel('TAS (m/s)'); ylabel('Alt (m)');
    title('Speed vs Altitude');

    subplot(2,3,3);
    plot(t_air,Gamma(i_air),'Color',C.grn,'LineWidth',1.8); hold on;
    yline(0,'k:');
    if ~isempty(i_35ft)&&i_35ft>=i_lo
        xline(ts(i_35ft),'r--','35ft','FontSize',8,'LabelVerticalAlignment','bottom'); end
    grid on; xlabel('t (s)'); ylabel('\gamma (deg)');
    title('Flight Path Angle');

    subplot(2,3,4);
    plot(t_air,VS(i_air),'Color',C.blue,'LineWidth',1.5); hold on;
    yline(0,'k:');
    if ~isempty(i_35ft)&&i_35ft>=i_lo, xline(ts(i_35ft),'r--'); end
    grid on; xlabel('t (s)'); ylabel('VS (m/s)'); title('Vertical Speed');

    subplot(2,3,5);
    cg = climb_grad_pct(i_air);
    plot(t_air,cg,'Color',C.red,'LineWidth',1.5); hold on;
    yline(2.4,'k--','FAR25 min 2.4%','FontSize',7,'LabelVerticalAlignment','bottom');
    yline(0,'k:');
    if ~isempty(i_35ft)&&i_35ft>=i_lo, xline(ts(i_35ft),'r--'); end
    maxcg = max(cg,[],'omitnan'); if isnan(maxcg)||isempty(maxcg), maxcg=15; end
    ylim([-2 max(15,maxcg+2)]);
    grid on; xlabel('t (s)'); ylabel('Gradient (%)');
    title('OEI Climb Gradient (FAR25 min = 2.4%)');

    subplot(2,3,6);
    plot(t_air,Thrust(i_air)/1e3,'Color',C.pur,'LineWidth',1.5); hold on;
    yline(Tmax/2e3,'k--','OEI Tmax','FontSize',7);
    yline(Tmax/1e3,'b:','2-eng Tmax','FontSize',7);
    grid on; xlabel('t (s)'); ylabel('Thrust (kN)'); title('Engine Thrust (OEI)');

    sgtitle('OEI Climb to 100 m — GO Scenario','FontSize',12,'FontWeight','bold');
end

%% =========================================================================
%  FIG 3 — STOP: Braking Performance
%% =========================================================================
if is_stop && ~isempty(i_V1)
    i_brk = i_V1:N;
    t_brk = ts(i_brk);
    fig3 = figure('Name','BFL Braking','Color','w', ...
        'Position',[60 40 1300 720],'NumberTitle','off');

    subplot(2,3,1);
    dx_brk = X_m(i_brk) - X_m(i_V1);
    plot(dx_brk, TAS_mps(i_brk),'Color',C.blue,'LineWidth',2); hold on;
    if ~isempty(i_stp)&&i_stp>=i_V1
        plot(X_m(i_stp)-X_m(i_V1),TAS_mps(i_stp),'ko','MarkerSize',12,'MarkerFaceColor','k'); end
    grid on; xlabel('Distance from V1 (m)'); ylabel('TAS (m/s)');
    title('Speed vs Distance (Braking)');

    subplot(2,3,2);
    plot(t_brk,decel(i_brk),'Color',C.red,'LineWidth',1.8); hold on;
    yline(0.3*g_n,'k--','\mu=0.3*g','FontSize',7,'LabelVerticalAlignment','bottom');
    yline(0,'k:');
    if ~isempty(i_stp)&&i_stp>=i_V1
        xline(ts(i_stp),'k--','Stop','FontSize',8,'LabelVerticalAlignment','bottom'); end
    grid on; xlabel('t (s)'); ylabel('Decel (m/s2)'); title('Braking Deceleration');

    subplot(2,3,3);
    bf = brake_force(i_brk);
    plot(t_brk,bf/1e3,'Color',C.pur,'LineWidth',1.5); hold on;
    yline(0,'k:');
    if ~isempty(i_stp)&&i_stp>=i_V1, xline(ts(i_stp),'k--'); end
    grid on; xlabel('t (s)'); ylabel('Force (kN)');
    title('Net Braking Force (M*a - Thrust)');

    subplot(2,3,4);
    plot(t_brk,Ntot(i_brk)/1e3,'Color',C.org,'LineWidth',1.5); hold on;
    yline(W/1e3,'b:','W','FontSize',7); yline(0,'k:');
    grid on; xlabel('t (s)'); ylabel('N_{gear} (kN)'); title('Gear Load During Braking');

    subplot(2,3,5);
    plot(ts,TAS_mps,'Color',C.blue,'LineWidth',1.8); hold on;
    xline(ts(i_V1),'r--','V1','FontSize',8,'LabelVerticalAlignment','bottom');
    i_end = N; if ~isempty(i_stp), i_end = i_stp; end
    if ~isempty(i_stp)
        xline(ts(i_stp),'k--','Stop','FontSize',8,'LabelVerticalAlignment','bottom'); end
    fill([ts(i_V1);ts(i_end);ts(i_end);ts(i_V1)], ...
         [0;0;TAS_mps(i_V1);TAS_mps(i_V1)], ...
         [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4);
    grid on; xlabel('t (s)'); ylabel('TAS (m/s)');
    title('Full Speed History + Brake Zone');

    subplot(2,3,6);
    if ~isempty(i_stp) && i_stp > i_V1
        idx_b = i_V1:i_stp;
        dx_b  = diff(X_m(idx_b));
        bf_b  = brake_force(idx_b(1:end-1));
        bf_b(isnan(bf_b)) = 0;
        KE_diss = cumsum(abs(bf_b .* dx_b)) / 1e6;
        plot((X_m(idx_b(1:end-1))-X_m(i_V1))/1e3, KE_diss,'Color',C.red,'LineWidth',1.8);
        grid on; xlabel('Dist from V1 (km)'); ylabel('Energy (MJ)');
        title('Cumulative Braking Energy');
    else
        text(0.5,0.5,'Stopped event not detected', ...
            'HorizontalAlignment','center','Units','normalized','FontSize',11);
        title('Braking Energy');
    end

    sgtitle(sprintf('STOP Scenario — Max Braking  |  V1=%.1f m/s',V1_val), ...
        'FontSize',12,'FontWeight','bold');
end

%% =========================================================================
%  FIG 4 — BFL Summary: Distance Breakdown + Balance
%% =========================================================================
fig4 = figure('Name','BFL Summary','Color','w', ...
    'Position',[80 40 1100 520],'NumberTitle','off');

subplot(1,3,1:2); hold on;
seg_go  = []; seg_stp = [];
bar_colors = {[0.20 0.62 0.17],[0.94 0.50 0.10],[0.80 0.20 0.20],[0.20 0.40 0.80]};

if is_go
    if ~isempty(i_V1), seg_go(end+1) = X_m(i_V1); end
    if ~isempty(i_V1)&&~isempty(i_VR)&&i_VR>i_V1, seg_go(end+1)=X_m(i_VR)-X_m(i_V1); end
    if ~isempty(i_VR)&&~isempty(i_lo)&&i_lo>i_VR,  seg_go(end+1)=X_m(i_lo)-X_m(i_VR); end
    if ~isempty(i_lo)&&~isempty(i_35ft)&&i_35ft>i_lo, seg_go(end+1)=X_m(i_35ft)-X_m(i_lo); end
end
if is_stop
    if ~isempty(i_V1), seg_stp(end+1) = X_m(i_V1); end
    if ~isempty(i_V1)&&~isempty(i_stp)&&i_stp>i_V1, seg_stp(end+1)=X_m(i_stp)-X_m(i_V1); end
end

% Draw GO bar (y=1.0)
x0 = 0;
for k = 1:numel(seg_go)
    c = bar_colors{min(k,numel(bar_colors))};
    rectangle('Position',[x0, 0.725, seg_go(k), 0.35],'FaceColor',c,'EdgeColor','w','LineWidth',1.5);
    if seg_go(k) > 20
        text(x0+seg_go(k)/2,0.9,sprintf('%.0fm',seg_go(k)),...
            'HorizontalAlignment','center','FontSize',8,'FontWeight','bold','Color','w');
    end
    x0 = x0 + seg_go(k);
end
% Draw STOP bar (y=1.5)
x0 = 0;
for k = 1:numel(seg_stp)
    c = bar_colors{min(k,numel(bar_colors))};
    rectangle('Position',[x0, 1.275, seg_stp(k), 0.35],'FaceColor',c,'EdgeColor','w','LineWidth',1.5);
    if seg_stp(k) > 20
        text(x0+seg_stp(k)/2,1.45,sprintf('%.0fm',seg_stp(k)),...
            'HorizontalAlignment','center','FontSize',8,'FontWeight','bold','Color','w');
    end
    x0 = x0 + seg_stp(k);
end

yticks([0.9 1.45]); yticklabels({'GO','STOP'}); ylim([0.55 1.75]);
maxX = max([sum(seg_go) sum(seg_stp) 100]);
xlim([0 maxX*1.08]);
grid on; xlabel('Runway Distance (m)'); title('BFL Distance Breakdown');

if ~isempty(seg_go)
    xline(sum(seg_go),'b--',sprintf('D_{go}=%.0f m',sum(seg_go)), ...
        'FontSize',9,'LabelVerticalAlignment','bottom');
end
if ~isempty(seg_stp)
    xline(sum(seg_stp),'r--',sprintf('D_{stop}=%.0f m',sum(seg_stp)), ...
        'FontSize',9,'LabelVerticalAlignment','bottom');
end

% Key numbers panel
subplot(1,3,3); axis off;
info = {};
info{end+1} = sprintf('V1 = %.1f m/s  (%.0f kt)', V1_val, V1_val/0.51444);
info{end+1} = '---------------------------';
if is_go&&~isempty(i_35ft)
    info{end+1} = sprintf('D_go   = %.0f m',   X_m(i_35ft));
    info{end+1} = sprintf('       = %.0f ft',  X_m(i_35ft)/0.3048);
    info{end+1} = ' ';
end
if is_stop&&~isempty(i_stp)
    info{end+1} = sprintf('D_stop = %.0f m',   X_m(i_stp));
    info{end+1} = sprintf('       = %.0f ft',  X_m(i_stp)/0.3048);
    info{end+1} = ' ';
end
if exist('D_go','var')&&exist('D_stop','var')
    diff_m2 = D_go - D_stop;
    info{end+1} = sprintf('Diff = %+.0f m', diff_m2);
    if abs(diff_m2) < 25
        bfl_val = (D_go+D_stop)/2;
        info{end+1} = ' ';
        info{end+1} = sprintf('BFL = %.0f m', bfl_val);
        info{end+1} = sprintf('    = %.0f ft', bfl_val/0.3048);
        info{end+1} = '** BALANCED **';
    elseif diff_m2 > 0
        info{end+1} = '>> Lower V1 to balance';
    else
        info{end+1} = '>> Raise V1 to balance';
    end
end
for k = 1:numel(info)
    is_hi = contains(info{k},'BALANCED') || startsWith(info{k},'>>') || startsWith(info{k},'BFL');
    clr = [0.1 0.1 0.1]; fw = 'normal';
    if is_hi, clr = [0.7 0 0]; fw = 'bold'; end
    text(0.05, 1.0-(k-1)*0.07, info{k}, 'Units','normalized',...
        'FontSize',10,'FontWeight',fw,'Color',clr,'VerticalAlignment','top');
end
title('Key Numbers');
sgtitle(sprintf('BFL Summary  |  V1=%.1f m/s  |  %s', V1_val, scen), ...
    'FontSize',12,'FontWeight','bold');

%% =========================================================================
%  SAVE FIGURES
%% =========================================================================
outdir = 'C:\Users\tejve\Downloads\acft';
stag = 'go'; if is_stop, stag = 'stop'; end
try; saveas(fig1, fullfile(outdir, sprintf('bfl_timeline_%s.png', stag)));    catch; end
try; if is_go&&exist('fig2','var'), saveas(fig2, fullfile(outdir,'bfl_oei_climb.png')); end; catch; end
try; if is_stop&&exist('fig3','var'), saveas(fig3, fullfile(outdir,'bfl_braking.png')); end; catch; end
try; saveas(fig4, fullfile(outdir,'bfl_summary.png'));                         catch; end
fprintf('Figures saved to %s\n', outdir);

%% =========================================================================
%  LOCAL HELPER FUNCTIONS  —  must be LAST in the file
%% =========================================================================

function add_event_lines(ts, i_V1, i_VR, i_lo, i_35ft, i_stp)
%ADD_EVENT_LINES  Draw coloured vertical markers for key BFL events.
    if ~isempty(i_V1),   xline(ts(i_V1), 'r--','V1',  'FontSize',7,'LabelVerticalAlignment','bottom'); end
    if ~isempty(i_VR),   xline(ts(i_VR), 'm--','VR',  'FontSize',7,'LabelVerticalAlignment','bottom'); end
    if ~isempty(i_lo),   xline(ts(i_lo), 'g--','LO',  'FontSize',7,'LabelVerticalAlignment','bottom'); end
    if ~isempty(i_35ft), xline(ts(i_35ft),'b--','35ft','FontSize',7,'LabelVerticalAlignment','bottom'); end
    if ~isempty(i_stp),  xline(ts(i_stp),'k--','Stop','FontSize',7,'LabelVerticalAlignment','bottom'); end
end

function v = safe_get(vec, idx, default)
%SAFE_GET  Return vec(idx) or default when idx is empty.
    if isempty(idx), v = default; else, v = vec(idx); end
end
