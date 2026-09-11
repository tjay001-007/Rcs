%% optimize_indi_gains.m
%  Two-stage optimizer for INDI gains (K_q, N_filt) that drive jerky
%  elevator activity.
%
%  Stage 1 — 4×3 coarse grid search
%  Stage 2 — fminsearch refinement from grid winner
%  Stage 3 — injects best gains into Takeoff_FCS and saves model
%
%  Cost  = W_TRACK*RMS_q_err  +  W_SMOOTH*RMS_de_rate  +  W_OVER*peak_err
%  Default weights favour smoothness (W_SMOOTH=1.5) over tracking (W_TRACK=1.0)
%  to reduce elevator chatter.
%
%  Prerequisites:
%    run('setup_takeoff_fcs.m')    <- loads workspace + configures model
%    Then run this script.         <- finds better gains and saves them
%
%  Expected runtime: 20–60 min depending on CPU speed.
% -----------------------------------------------------------------------

mdl = 'ACFT11_a_121';
if ~bdIsLoaded(mdl)
    error('[opt] Model not loaded.  Run setup_takeoff_fcs.m first.');
end

% ---- Cost weights -------------------------------------------------------
% Increase W_SMOOTH to trade tracking bandwidth for smoother elevator.
W_TRACK  = 1.0;   % q tracking RMS (deg/s)
W_SMOOTH = 1.5;   % elevator rate RMS (deg/s per step) — primary anti-jerk
W_OVER   = 0.3;   % peak q overshoot during rotation (deg/s)

% ---- Optimisation horizon -----------------------------------------------
% 45 s covers full ground roll + rotation + initial climb without needing
% the 1000 m stop trigger.
OPT_STOP = '45';

% K_theta held fixed — it mainly sets trim attitude, not chatter frequency.
K_THETA  = 1.0;

% Preserve original model settings so we can restore them afterwards.
orig_StopTime = get_param(mdl, 'StopTime');
orig_RWO      = get_param(mdl, 'ReturnWorkspaceOutputs');

% ToWorkspace blocks write directly to base workspace during programmatic
% sim() — this is what evalin('base',...) relies on.
set_param(mdl, 'ReturnWorkspaceOutputs', 'off');

fprintf('\n========================================================\n');
fprintf('  INDI Gain Optimizer\n');
fprintf('  Cost = %.1f*RMS_q_err  +  %.1f*RMS_de_rate  +  %.1f*Peak_over\n', ...
    W_TRACK, W_SMOOTH, W_OVER);
fprintf('========================================================\n\n');

% =========================================================================
%  Stage 1 — Coarse grid search
% =========================================================================
fprintf('Stage 1: Grid search  (4 K_q × 3 N_filt = 12 simulations)\n');
fprintf('%-7s %-8s | %-9s %-9s %-9s | %-8s\n', ...
    'K_q','N_filt','Track','Smooth','Over','Cost');
fprintf('%s\n', repmat('-',1,60));

K_q_grid    = [2.0  3.5  5.0  7.0];
N_filt_grid = [5.0  10.0 20.0];

best_cost = inf;
best_Kq   = 5.0;
best_Nf   = 15.0;

for gi = 1:numel(K_q_grid)
    for gj = 1:numel(N_filt_grid)
        Kq = K_q_grid(gi);
        Nf = N_filt_grid(gj);
        [c, ct, cs, co] = eval_gains(mdl, Kq, K_THETA, Nf, OPT_STOP, ...
                                     W_TRACK, W_SMOOTH, W_OVER);
        marker = '';
        if c < best_cost
            best_cost = c;  best_Kq = Kq;  best_Nf = Nf;
            marker = '  <--';
        end
        fprintf('%-7.1f %-8.1f | %-9.3f %-9.3f %-9.3f | %-8.4f%s\n', ...
            Kq, Nf, ct, cs, co, c, marker);
    end
end

fprintf('\n  Grid best:  K_q = %.1f,  N_filt = %.1f,  cost = %.4f\n\n', ...
    best_Kq, best_Nf, best_cost);

% =========================================================================
%  Stage 2 — fminsearch refinement
% =========================================================================
fprintf('Stage 2: fminsearch from [K_q=%.1f, N_filt=%.1f]...\n', ...
    best_Kq, best_Nf);

cost_fn = @(x) eval_gains(mdl, abs(x(1)), K_THETA, abs(x(2)), ...
    OPT_STOP, W_TRACK, W_SMOOTH, W_OVER);

fms_opts = optimset('Display','iter','TolX',0.05,'TolFun',1e-3,'MaxIter',20);
[xopt, fopt] = fminsearch(cost_fn, [best_Kq, best_Nf], fms_opts);

Kq_opt = abs(xopt(1));
Nf_opt = abs(xopt(2));

% Clip to sensible physical bounds
Kq_opt = max(0.5, min(15.0, Kq_opt));
Nf_opt = max(2.0, min(50.0, Nf_opt));

fprintf('\n  fminsearch result:  K_q = %.3f,  N_filt = %.3f,  cost = %.4f\n\n', ...
    Kq_opt, Nf_opt, fopt);

% =========================================================================
%  Stage 3 — Restore model, inject optimised gains, save
% =========================================================================
set_param(mdl, 'StopTime',               orig_StopTime);
set_param(mdl, 'ReturnWorkspaceOutputs', orig_RWO);

fprintf('Stage 3: Injecting optimised gains and saving model...\n');
inject_gains(mdl, Kq_opt, K_THETA, Nf_opt);
save_system(mdl);
fprintf('  Model saved.\n');

% Print comparison
fprintf('\n========================================================\n');
fprintf('  OPTIMISED INDI GAINS\n');
fprintf('  K_q     = %.3f s^-1   (was  5.000)\n', Kq_opt);
fprintf('  K_theta = %.3f s^-1   (unchanged)\n',   K_THETA);
fprintf('  N_filt  = %.3f rad/s  (was 15.000)\n',  Nf_opt);
if best_cost > 1e5
    pct = 0;
else
    pct = 100*(best_cost - fopt) / max(best_cost, 1e-9);
end
fprintf('  Cost improvement from grid best: %.1f%%\n', pct);
fprintf('========================================================\n');
fprintf('\n  --> Run setup_takeoff_fcs.m then press Run in Simulink\n');
fprintf('      Then run analyze_takeoff to compare results.\n\n');

% =========================================================================
%  LOCAL FUNCTIONS
% =========================================================================

% -------------------------------------------------------------------------
function [cost, ct, cs, co] = eval_gains(mdl, K_q, K_theta, N_filt, ...
                                          t_stop, w1, w2, w3)
%EVAL_GAINS  Simulate model with given gains; compute weighted cost.

    % 1. Inject gains into MATLAB Function block
    inject_gains(mdl, K_q, K_theta, N_filt);

    % 2. Set optimisation stop time
    set_param(mdl, 'StopTime', t_stop);

    % 3. Run simulation
    try
        sim(mdl);
    catch ME
        fprintf('  [sim error at K_q=%.2f N_filt=%.2f] %s\n', ...
            K_q, N_filt, ME.message);
        cost=1e6; ct=1e6; cs=1e6; co=1e6;
        return;
    end

    % 4. Read results from base workspace
    try
        t_q   = evalin('base','TW_time');      % 0.1 s clock
        q_act = evalin('base','TW_q_deg');     % actual pitch rate (deg/s), 0.1s
        e_cmd = evalin('base','TW_elev_cmd');  % elevator cmd (deg), 0.02 s
        ph    = evalin('base','TW_Phase');     % phase (0/1/2),        0.02 s
        qcmd  = evalin('base','TW_qcmd');      % q command (deg/s),    0.02 s
    catch ME2
        fprintf('  [data error] %s\n', ME2.message);
        cost=1e6; ct=1e6; cs=1e6; co=1e6;
        return;
    end

    % 5. Build time vectors
    n_q  = numel(t_q);
    n_fc = numel(e_cmd);
    if n_q < 5 || n_fc < 5
        cost=1e6; ct=1e6; cs=1e6; co=1e6;
        return;
    end
    % FCS signals logged at 0.02 s from t=0
    t_fc = (0:n_fc-1)' * 0.02;

    % 6. Interpolate FCS signals onto the 0.1 s query grid
    phase_i = interp1(t_fc, ph,   t_q, 'nearest','extrap');
    qcmd_i  = interp1(t_fc, qcmd, t_q, 'nearest','extrap');

    % 7. Active control window: phases 1 and 2 only
    active = phase_i >= 1;
    if sum(active) < 5
        % Aircraft never reached rotation — badly tuned; heavy penalty
        cost=1e4; ct=1e4; cs=1e4; co=1e4;
        return;
    end

    % 8a. Tracking RMS  (q_actual vs q_cmd, active phases)
    q_err = q_act(active) - qcmd_i(active);
    ct    = sqrt(mean(q_err.^2));

    % 8b. Elevator activity — rate at native 0.02 s resolution
    %     High-pass derivative is the main jerk metric.
    de_dt = diff(e_cmd) / 0.02;   % deg/s
    cs    = sqrt(mean(de_dt.^2));

    % 8c. Peak overshoot during rotation phase (phase = 1)
    rot = phase_i == 1;
    if sum(rot) > 2
        co = max(abs(q_act(rot) - qcmd_i(rot)));
    else
        co = 0;
    end

    cost = w1*ct + w2*cs + w3*co;
end

% -------------------------------------------------------------------------
function inject_gains(mdl, K_q, K_theta, N_filt)
%INJECT_GAINS  Overwrite Takeoff_FCS source via Stateflow API.

    new_code = make_fcs_code(K_q, K_theta, N_filt);

    rt     = sfroot;
    charts = rt.find('-isa','Stateflow.EMChart');
    for k  = 1:numel(charts)
        if contains(charts(k).Path, [mdl '/Takeoff_FCS'])
            charts(k).Script = new_code;
            return;
        end
    end
    error('[inject_gains] Takeoff_FCS block not found in model %s', mdl);
end

% -------------------------------------------------------------------------
function code = make_fcs_code(K_q, K_theta, N_filt)
%MAKE_FCS_CODE  Generate INDI + PI-autothrottle source with parametric gains.
%  Pitch gains (K_q, K_theta, N_filt) are swept by the optimiser.
%  Speed controller gains are fixed constants (not optimised).

    code = sprintf([ ...
'function [Elevator_deg_cmd, Throttle_cmd, Phase, q_cmd_deg_s] = Takeoff_FCS(V_mps, BodyRates_radps, EulerAngles_rad, N_total_N, alt_m)\n' ...
'%%#codegen\n' ...
'persistent delta_e_prev phase_state q_prev qdot_filt thr_int thr_prev V_filt spd_captured\n' ...
'if isempty(delta_e_prev)\n' ...
'    delta_e_prev=0.0; phase_state=int32(0); q_prev=0.0; qdot_filt=0.0;\n' ...
'    thr_int=1.0; thr_prev=1.0; V_filt=0.0; spd_captured=int32(0);\n' ...
'end\n' ...
'%% --- Aircraft constants ---\n' ...
'Cm_elev=-0.0084; S=85.25; c=15.55; Iyy=288344.0; rho=1.225;\n' ...
'%% --- Optimised pitch controller gains ---\n' ...
'K_q     = %.6f;   %% s^-1  inner pitch-rate gain\n' ...
'K_theta = %.6f;   %% s^-1  outer attitude gain\n' ...
'N_filt  = %.6f;   %% rad/s derivative filter bandwidth\n' ...
'dt      = 0.02;   %% s     controller sample time\n' ...
'%% --- Phase thresholds ---\n' ...
'V_R=110.0; q_rot_cmd=3.0; theta_clmb=12.0; theta_rot_end=12.0;\n' ...
'N_gear_lo=1000.0; q_max=8.0; de_max=25.0;\n' ...
'%% --- Speed hold parameters (fixed) ---\n' ...
'V_target=200.0; Kp_V=0.005; Ki_V=0.002;\n' ...
'tau_V=1.0; thr_slew=0.10;\n' ...
'%% --- State extraction ---\n' ...
'q_rps=BodyRates_radps(2); theta_rad=EulerAngles_rad(2);\n' ...
'theta_deg=theta_rad*(180/pi); q_deg=q_rps*(180/pi);\n' ...
'%% --- TAS low-pass filter ---\n' ...
'alpha_V=dt/(tau_V+dt);\n' ...
'if V_filt<1.0, V_filt=V_mps; end\n' ...
'V_filt=V_filt+alpha_V*(V_mps-V_filt);\n' ...
'%% --- Derivative filter (washout, Euler-forward) ---\n' ...
'alpha_f=N_filt*dt/(1+N_filt*dt);\n' ...
'qdot_raw=(q_deg-q_prev)/dt;\n' ...
'qdot_filt=(1-alpha_f)*qdot_filt+alpha_f*qdot_raw;\n' ...
'%% --- Phase state machine ---\n' ...
'if phase_state==int32(0)\n' ...
'    if V_mps>=V_R, phase_state=int32(1); end\n' ...
'elseif phase_state==int32(1)\n' ...
'    if N_total_N<N_gear_lo||theta_deg>=theta_rot_end, phase_state=int32(2); end\n' ...
'end\n' ...
'%% --- q command ---\n' ...
'if phase_state==int32(0), q_cmd=0.0;\n' ...
'elseif phase_state==int32(1), q_cmd=q_rot_cmd;\n' ...
'else, q_cmd=K_theta*(theta_clmb-theta_deg); q_cmd=max(-q_max,min(q_max,q_cmd)); end\n' ...
'%% --- INDI elevator inner loop ---\n' ...
'if phase_state==int32(0)\n' ...
'    delta_e_cmd=0.0;\n' ...
'else\n' ...
'    qbar=max(0.5*rho*V_mps^2,0.5*rho*30.0^2);\n' ...
'    G_de=Cm_elev*qbar*S*c/Iyy;\n' ...
'    nu_q=K_q*(q_cmd-q_deg);\n' ...
'    if abs(G_de)>1e-6, Delta_de=(nu_q-qdot_filt)/G_de; else, Delta_de=0.0; end\n' ...
'    delta_e_cmd=delta_e_prev+Delta_de;\n' ...
'    delta_e_cmd=max(-de_max,min(de_max,delta_e_cmd));\n' ...
'end\n' ...
'%% --- Speed controller: full throttle -> capture -> hold ---\n' ...
'if phase_state==int32(2)\n' ...
'    if spd_captured==int32(0) && V_filt>=V_target\n' ...
'        spd_captured=int32(1);\n' ...
'        thr_int=thr_prev;\n' ...
'    end\n' ...
'    if spd_captured==int32(1)\n' ...
'        e_V=V_target-V_filt;\n' ...
'        thr_int=thr_int+Ki_V*e_V*dt;\n' ...
'        thr_int=max(0.0,min(1.2,thr_int));\n' ...
'        Throttle_raw=max(0.0,min(1.0,Kp_V*e_V+thr_int));\n' ...
'    else\n' ...
'        Throttle_raw=1.0;\n' ...
'    end\n' ...
'else\n' ...
'    Throttle_raw=1.0;\n' ...
'    spd_captured=int32(0);\n' ...
'    thr_int=1.0;\n' ...
'end\n' ...
'%% --- Throttle rate limiter ---\n' ...
'delta_max=thr_slew*dt;\n' ...
'Throttle_cmd=max(thr_prev-delta_max,min(thr_prev+delta_max,Throttle_raw));\n' ...
'thr_prev=Throttle_cmd;\n' ...
'%% --- State update ---\n' ...
'delta_e_prev=delta_e_cmd; q_prev=q_deg;\n' ...
'Elevator_deg_cmd=delta_e_cmd;\n' ...
'Phase=double(phase_state); q_cmd_deg_s=q_cmd;\n' ...
'end\n'], K_q, K_theta, N_filt);
end
