%% run_bfl_sweep.m
%  Automated Balanced Field Length (BFL) sweep.
%  Sweeps V1 from V1_min to V1_max, runs GO + STOP sim at each V1,
%  plots D_go and D_stop vs V1, and reports the balanced V1 and BFL.
%
%  Prerequisites:
%    run('setup_bfl_analysis.m')   (to load workspace + configure model once)
%    Then run this script.         (DOES call sim() automatically)
%
%  Expected runtime: ~2–5 min for 10 V1 points (both sims each).
% -----------------------------------------------------------------------

mdl = 'ACFT11_a_121';
if ~bdIsLoaded(mdl)
    error('Model not loaded. Run setup_bfl_analysis.m first.');
end

%% ========================  SWEEP PARAMETERS  ============================
V1_min   = 78;     % m/s  lower bound (must be >= 60, < VR=110)
V1_max   = 105;    % m/s  upper bound
V1_N     = 10;     % number of V1 points to evaluate
SIM_TIME = '150';   % simulation stop time (s)  — enough for braking to stop
%% ========================================================================

V1_vec    = linspace(V1_min, V1_max, V1_N);
D_go_vec  = NaN(1, V1_N);
D_stop_vec= NaN(1, V1_N);
ALT_SCREEN = 10.668;   % 35 ft (m)

%% Save original model settings
orig_StopTime = get_param(mdl, 'StopTime');
orig_RWO      = get_param(mdl, 'ReturnWorkspaceOutputs');
set_param(mdl, 'ReturnWorkspaceOutputs', 'off');
set_param(mdl, 'StopTime', SIM_TIME);
try
    set_param([mdl '/Alt_1000m_trigger'], 'const', '9999');
catch; end

fprintf('\n========================================================\n');
fprintf('  BFL SWEEP   V1 = %.1f … %.1f m/s  (%d points each)\n', ...
    V1_min, V1_max, V1_N);
fprintf('  Each point: 2 sims (GO + STOP)\n');
fprintf('========================================================\n');
fprintf('%-8s | %-12s | %-12s | %-10s\n','V1 (m/s)','D_go (m)','D_stop (m)','Diff (m)');
fprintf('%s\n', repmat('-',1,50));

rt = sfroot;
charts = rt.find('-isa','Stateflow.EMChart');

for vi = 1:V1_N
    V1 = V1_vec(vi);

    %% --- GO simulation ---
    inject_bfl_code(mdl, charts, V1, 1);
    try
        sim(mdl);
        ts   = evalin('base','TW_time');
        X_m  = evalin('base','TW_X_m');
        alt_m_raw = evalin('base','TW_posNED'); alt_m = -alt_m_raw(:,3);
        t_pos = linspace(0, ts(end), numel(alt_m))';
        alt_i = interp1(t_pos, alt_m, ts(:), 'linear','extrap');
        i35 = find(alt_i >= ALT_SCREEN, 1);
        if ~isempty(i35)
            D_go_vec(vi) = X_m(i35);
        end
    catch ME
        fprintf('  [GO sim error V1=%.1f] %s\n', V1, ME.message);
    end

    %% --- STOP simulation ---
    inject_bfl_code(mdl, charts, V1, 0);
    try
        sim(mdl);
        ts_s  = evalin('base','TW_time');
        X_s   = evalin('base','TW_X_m');
        TAS_s = evalin('base','TW_TAS_mps');
        Ph_s  = evalin('base','TW_Phase');
        t_ph  = linspace(0, ts_s(end), numel(Ph_s))';
        Ph_i  = interp1(t_ph, Ph_s, ts_s(:), 'nearest','extrap');
        i_stp = find(TAS_s < 1.5 & Ph_i >= 9.5, 1);
        if ~isempty(i_stp)
            D_stop_vec(vi) = X_s(i_stp);
        end
    catch ME
        fprintf('  [STOP sim error V1=%.1f] %s\n', V1, ME.message);
    end

    diff_m = D_go_vec(vi) - D_stop_vec(vi);
    fprintf('%-8.1f | %-12.0f | %-12.0f | %+.0f\n', ...
        V1, D_go_vec(vi), D_stop_vec(vi), diff_m);
end

%% Restore model settings
set_param(mdl, 'StopTime', orig_StopTime);
set_param(mdl, 'ReturnWorkspaceOutputs', orig_RWO);
try, set_param([mdl '/Alt_1000m_trigger'], 'const', '1000'); catch; end

%% =========================================================================
%  Find balanced V1 by interpolation (D_go - D_stop changes sign)
%% =========================================================================
valid = ~isnan(D_go_vec) & ~isnan(D_stop_vec);
if sum(valid) >= 2
    diff_vec = D_go_vec(valid) - D_stop_vec(valid);
    V1_valid = V1_vec(valid);

    % Find zero crossing
    sign_chg = find(diff(sign(diff_vec)) ~= 0, 1);
    if ~isempty(sign_chg)
        % Linear interpolation for exact crossing
        V1_a = V1_valid(sign_chg);   D_a = diff_vec(sign_chg);
        V1_b = V1_valid(sign_chg+1); D_b = diff_vec(sign_chg+1);
        V1_bal = V1_a - D_a*(V1_b - V1_a)/(D_b - D_a);

        % BFL at balanced V1 (interpolate distances)
        D_go_bal  = interp1(V1_valid, D_go_vec(valid),  V1_bal, 'linear','extrap');
        D_stop_bal= interp1(V1_valid, D_stop_vec(valid),V1_bal, 'linear','extrap');
        BFL       = (D_go_bal + D_stop_bal) / 2;

        fprintf('\n========================================================\n');
        fprintf('  BALANCED FIELD LENGTH RESULT\n');
        fprintf('  V1_balanced = %.1f m/s  (%.1f kt)\n', V1_bal, V1_bal/0.51444);
        fprintf('  D_go        = %.0f m  (%.0f ft)\n', D_go_bal, D_go_bal/0.3048);
        fprintf('  D_stop      = %.0f m  (%.0f ft)\n', D_stop_bal, D_stop_bal/0.3048);
        fprintf('  BFL         = %.0f m  (%.0f ft)\n', BFL, BFL/0.3048);
        fprintf('========================================================\n\n');

        assignin('base','V1_balanced', V1_bal);
        assignin('base','BFL_m', BFL);
    else
        fprintf('\n  No zero crossing found in sweep range.\n');
        fprintf('  D_go - D_stop range: [%.0f, %.0f] m\n', min(diff_vec), max(diff_vec));
        if all(diff_vec > 0)
            fprintf('  --> D_go always > D_stop: reduce V1_min below %.1f m/s\n', V1_min);
        else
            fprintf('  --> D_stop always > D_go: increase V1_max above %.1f m/s\n', V1_max);
        end
    end
else
    fprintf('\n  Insufficient valid data points for BFL calculation.\n');
end

%% =========================================================================
%  PLOT SWEEP RESULTS
%% =========================================================================
if sum(valid) >= 2
    V1_plot = V1_vec(valid);
    figure('Name','BFL Sweep','Color','w','Position',[100 100 900 500],'NumberTitle','off');

    subplot(1,2,1);
    plot(V1_plot, D_go_vec(valid)/1e3, 'b-o','LineWidth',2,'MarkerFaceColor','b'); hold on
    plot(V1_plot, D_stop_vec(valid)/1e3,'r-s','LineWidth',2,'MarkerFaceColor','r');
    if exist('V1_bal','var')
        xline(V1_bal,'k--','V1_{bal}','FontSize',10,'LabelVerticalAlignment','bottom');
        plot(V1_bal, BFL/1e3,'k*','MarkerSize',16,'LineWidth',2);
    end
    grid on; xlabel('V1 (m/s)'); ylabel('Distance (km)');
    title('BFL Sweep — Accel-Go vs Accel-Stop');
    legend('D_{go} (GO scenario)','D_{stop} (STOP scenario)','V1_{bal}','BFL','Location','best');

    subplot(1,2,2);
    plot(V1_plot, (D_go_vec(valid)-D_stop_vec(valid)),'k-o','LineWidth',2,'MarkerFaceColor','k');
    hold on; yline(0,'r--','Balance','LineWidth',1.5);
    if exist('V1_bal','var'), xline(V1_bal,'k--'); end
    grid on; xlabel('V1 (m/s)'); ylabel('D_{go} - D_{stop} (m)');
    title('Distance Difference (zero = BFL)');

    if exist('V1_bal','var') && exist('BFL','var')
        sgtitle(sprintf('BFL = %.0f m (%.0f ft)  at  V1 = %.1f m/s', ...
            BFL, BFL/0.3048, V1_bal), 'FontSize',13,'FontWeight','bold');
    end

    try
        saveas(gcf, 'C:\Users\tejve\Downloads\acft\bfl_sweep.png');
    catch; end
end

%% =========================================================================
%  LOCAL FUNCTION: inject BFL FCS
%% =========================================================================
function inject_bfl_code(mdl, charts, V1, go_flag)
    code = sprintf([...
'function [Elevator_deg_cmd, Throttle_cmd, Phase, q_cmd_deg_s, mu_cmd] = Takeoff_FCS(V_mps, BodyRates_radps, EulerAngles_rad, N_total_N, alt_m)\n'...
'%%#codegen\n'...
'persistent delta_e_prev phase_state q_prev qdot_filt thr_prev V_filt\n'...
'if isempty(delta_e_prev)\n'...
'    delta_e_prev=0.0; phase_state=int32(0); q_prev=0.0; qdot_filt=0.0;\n'...
'    thr_prev=1.0; V_filt=0.0;\n'...
'end\n'...
'V1_mps=%.4f; BFL_GO=%d;\n'...
'Cm_elev=-0.0084; S=85.25; c=15.55; Iyy=288344.0; rho=1.225;\n'...
'K_q=5.0; K_theta=1.0; N_filt=15.0; dt=0.02;\n'...
'V_R=110.0; q_rot_cmd=3.0; theta_clmb=12.0; theta_rot_end=12.0;\n'...
'N_gear_lo=1000.0; q_max=8.0; de_max=25.0; tau_V=1.0;\n'...
'q_rps=BodyRates_radps(2); theta_rad=EulerAngles_rad(2);\n'...
'theta_deg=theta_rad*(180/pi); q_deg=q_rps*(180/pi);\n'...
'alpha_V=dt/(tau_V+dt);\n'...
'if V_filt<1.0, V_filt=V_mps; end\n'...
'V_filt=V_filt+alpha_V*(V_mps-V_filt);\n'...
'alpha_f=N_filt*dt/(1+N_filt*dt);\n'...
'qdot_raw=(q_deg-q_prev)/dt;\n'...
'qdot_filt=(1-alpha_f)*qdot_filt+alpha_f*qdot_raw;\n'...
'if phase_state==int32(0)\n'...
'    if V_mps>=V1_mps\n'...
'        if BFL_GO==1, phase_state=int32(1);\n'...
'        else,         phase_state=int32(10); end\n'...
'    end\n'...
'elseif phase_state==int32(1)\n'...
'    if V_mps>=V_R, phase_state=int32(2); end\n'...
'elseif phase_state==int32(2)\n'...
'    if N_total_N<N_gear_lo||theta_deg>=theta_rot_end, phase_state=int32(3); end\n'...
'end\n'...
'if phase_state<=int32(1), q_cmd=0.0;\n'...
'elseif phase_state==int32(2), q_cmd=q_rot_cmd;\n'...
'elseif phase_state==int32(3)\n'...
'    q_cmd=K_theta*(theta_clmb-theta_deg); q_cmd=max(-q_max,min(q_max,q_cmd));\n'...
'else, q_cmd=0.0; end\n'...
'if phase_state==int32(0)||phase_state==int32(10)\n'...
'    delta_e_cmd=0.0;\n'...
'else\n'...
'    qbar=max(0.5*rho*V_mps^2,0.5*rho*30.0^2);\n'...
'    G_de=Cm_elev*qbar*S*c/Iyy;\n'...
'    nu_q=K_q*(q_cmd-q_deg);\n'...
'    if abs(G_de)>1e-6, Delta_de=(nu_q-qdot_filt)/G_de; else, Delta_de=0.0; end\n'...
'    delta_e_cmd=delta_e_prev+Delta_de;\n'...
'    delta_e_cmd=max(-de_max,min(de_max,delta_e_cmd));\n'...
'end\n'...
'if phase_state==int32(0),  Throttle_cmd=1.0;\n'...
'elseif phase_state==int32(10), Throttle_cmd=0.05;\n'...
'else,  Throttle_cmd=0.5; end\n'...
'thr_prev=Throttle_cmd;\n'...
'if phase_state==int32(10), mu_cmd=0.3; else, mu_cmd=0.03; end\n'...
'delta_e_prev=delta_e_cmd; q_prev=q_deg;\n'...
'Elevator_deg_cmd=delta_e_cmd;\n'...
'Phase=double(phase_state); q_cmd_deg_s=q_cmd;\n'...
'end\n'], V1, go_flag);

    for k = 1:numel(charts)
        if contains(charts(k).Path, [mdl '/Takeoff_FCS'])
            charts(k).Script = code;
            return;
        end
    end
    error('Takeoff_FCS not found.');
end
