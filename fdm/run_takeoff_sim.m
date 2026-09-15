%% run_takeoff_sim.m  — Open-loop nonlinear takeoff: ground roll → rotation → climb
%  Aircraft: Mass=24650 kg, Tmax=140 kN, T/W=0.58, V_R=110 m/s
%  Gear: quasi-rigid (k×10, ζ=5, d_static≈8mm — effectively rigid)
%  Controllers: OFF (open-loop pilot schedule via ExternalInput)

mdl = 'ACFT11_a_121';

%% ---- 0. Load workspace variables first --------------------------------
run(fullfile(fileparts(which('run_takeoff_sim.m')),'inertiatensor62.m'));
Mass_kg = Mass;   % alias — model references both 'Mass' and 'Mass_kg'
assignin('base','Mass_kg', Mass_kg);
fprintf('Workspace loaded: Mass=%.0f kg, g=%.5f m/s2\n', Mass, g);

%% ---- 1. Load model -----------------------------------------------------
if ~bdIsLoaded(mdl), load_system([mdl '.slx']); end

%% ---- 2. Controllers OFF -----------------------------------------------
for sw = {'Manual Switch','Manual Switch1','Manual Switch2','Manual Switch3'}
    try; set_param([mdl '/' sw{1}],'CurrentSetting','0'); catch; end
end

%% ---- 3. Initial conditions (quasi-rigid gear, theta=2°, from rest) ---
%  Static equilibrium: k_NLG=4e6, k_MLG=1.3e7, theta=2deg, mass=24650 kg
%  ZI_m = -2.32815 m  (CG altitude = 2.32815 m above ground)
set_param([mdl '/EqM/InertialData/ZI_m'],    'InitialCondition','-2.32815');
set_param([mdl '/EqM/EulerAngles/THETA_rad'],'InitialCondition','2*pi/180');
set_param([mdl '/EqM/BodyVelocities/u_mps'], 'InitialCondition','0');

%% ---- 4. Pilot schedule (ExternalInput) --------------------------------
%  Root inport order: 1=Elevator_deg  2=Aileron_deg  3=Rudder_deg  4=Throttle
dt    = 0.05;
t_end = 130;        % s — heavy aircraft, needs ~100 s to 1000 m

% Key times
t_idle = 5.0;       % throttle-up at 5 s (let gear settle from rest)
t_VR   = 25.5;      % V_R≈110 m/s at ~20 s ground roll
t_rot  = t_VR + 2;  % 2-s elevator ramp complete at 27.5 s

t_vec = (0:dt:t_end)';
n = numel(t_vec);

% Throttle: 0 until t_idle, then 1.0
thr = zeros(n,1);
thr(t_vec >= t_idle) = 1.0;

% Elevator (deg): 0 during ground roll; ramp 0→20° over 2 s at V_R
elev = zeros(n,1);
for k = 1:n
    tt = t_vec(k);
    if tt >= t_VR && tt <= t_rot
        elev(k) = 20 * (tt - t_VR) / (t_rot - t_VR);
    elseif tt > t_rot
        elev(k) = 20;
    end
end

% Build matrix [t, Elev, Ail, Rud, Thr]
takeoff_u_ext = [t_vec, elev, zeros(n,1), zeros(n,1), thr];
assignin('base','takeoff_u_ext', takeoff_u_ext);

fprintf('Pilot schedule: idle 0-%.1fs, throttle-up, V_R at t~%.1fs, elevator 0→20deg\n',...
    t_idle, t_VR);

%% ---- 5. Solver settings ------------------------------------------------
set_param(mdl,'StopTime',         num2str(t_end));
set_param(mdl,'SolverName',       'ode45');
set_param(mdl,'SolverType',       'Variable-step');
set_param(mdl,'RelTol',           '1e-4');
set_param(mdl,'AbsTol',           '1e-5');
set_param(mdl,'MaxStep',          '0.05');   % smaller for stiffer gear
set_param(mdl,'AlgebraicLoopMsg','none');
set_param(mdl,'LoadExternalInput','on');
set_param(mdl,'ExternalInput',    'takeoff_u_ext');

save_system(mdl);
fprintf('Running %.0f-s simulation...\n', t_end);

%% ---- 6. Run ------------------------------------------------------------
tic;
simOut = sim(mdl,'ReturnWorkspaceOutputs','on');
cpu = toc;
fprintf('Done in %.1f s CPU.\n\n', cpu);

%% ---- 7. Extract results ------------------------------------------------
ts    = simOut.tout;

% From model outports (yout):
%  col1=KCAS  col5=Alpha_deg  col7=u_mps  col14=Theta_deg
%  col16=X_m  col18=PresAlt_ft  col25=Thrust_N
yout  = simOut.yout;
CAS_ms  = yout(:, 1) * 0.5144;      % knots → m/s
Alpha   = yout(:, 5);                % deg
u_fwd   = yout(:, 7);                % m/s body
theta   = yout(:,14);                % deg
xgnd    = yout(:,16);                % m
alt_ft  = yout(:,18);
alt     = alt_ft * 0.3048;           % m

% From ToWorkspace blocks
Ntot    = simOut.TW_Ntotal;          % N
posNED  = simOut.TW_posNED;          % [X,Y,Z] m NED

% Re-derive altitude from NED position (more accurate than PresAlt for low alt)
if ~isempty(posNED)
    alt = -squeeze(posNED(:,3));     % NED-z sign flip → altitude (m above ground)
end

%% ---- 8. Events ---------------------------------------------------------
% Liftoff: gear unloads, alt>0, CAS>80 m/s  (only check after t=t_idle+5 s)
i_gated = find(ts > t_idle + 5);
i_lo  = find(Ntot(i_gated) < 500 & alt(i_gated) > 0.5 & CAS_ms(i_gated) > 80, 1);
if ~isempty(i_lo), i_lo = i_gated(i_lo); end
i_1km = find(alt >= 1000, 1);

fprintf('=== Takeoff Performance ===\n');
if ~isempty(i_lo)
    fprintf('  Lift-off:   t=%5.1f s  CAS=%5.1f m/s (%4.0f kt)  dist=%5.0f m  θ=%5.1f°\n',...
        ts(i_lo), CAS_ms(i_lo), CAS_ms(i_lo)/0.5144, xgnd(i_lo), theta(i_lo));
else
    fprintf('  No lift-off detected within %.0f s\n', t_end);
end
if ~isempty(i_1km)
    fprintf('  1000 m alt: t=%5.1f s  CAS=%5.1f m/s (%4.0f kt)\n',...
        ts(i_1km), CAS_ms(i_1km), CAS_ms(i_1km)/0.5144);
else
    fprintf('  Max alt = %.1f m at t=%.1f s (did not reach 1000 m)\n',...
        max(alt), ts(find(alt==max(alt),1)));
end

%% ---- 9. Plots ----------------------------------------------------------
figure('Name','Takeoff Simulation','Color','w','Position',[40 40 1300 860],'NumberTitle','off');

subplot(3,2,1)
plot(ts, alt,'b','LineWidth',1.5); hold on
yline(1000,'k:','1000 m'); grid on
if ~isempty(i_lo),  xline(ts(i_lo),'r--','Liftoff','LabelVerticalAlignment','bottom'); end
if ~isempty(i_1km), xline(ts(i_1km),'g--','1000 m','LabelVerticalAlignment','bottom'); end
xlabel('Time (s)'); ylabel('Altitude (m)'); title('Altitude');
ylim([-5  max(max(alt)+50, 200)]);

subplot(3,2,2)
plot(ts, CAS_ms,'m','LineWidth',1.5); hold on
yline(110,'k--','V_R=110 m/s'); grid on
if ~isempty(i_lo), xline(ts(i_lo),'r--'); end
xlabel('Time (s)'); ylabel('CAS (m/s)'); title('Calibrated Airspeed');

subplot(3,2,3)
plot(xgnd, alt,'k','LineWidth',1.5); hold on
if ~isempty(i_lo)
    plot(xgnd(i_lo), alt(i_lo),'ro','MarkerSize',10,'MarkerFaceColor','r');
end
grid on; xlabel('Ground Distance (m)'); ylabel('Altitude (m)'); title('Flight Profile');

subplot(3,2,4)
plot(ts, theta,'r','LineWidth',1.5); hold on
yline(2,'b:','Static 2°'); yline(20,'k:','Cmd 20°'); grid on
if ~isempty(i_lo), xline(ts(i_lo),'r--'); end
xlabel('Time (s)'); ylabel('\theta (°)'); title('Pitch Attitude');

subplot(3,2,5)
plot(ts, Ntot/1000,'Color',[0.8 0.4 0],'LineWidth',1.5); hold on
yline(0,'k:'); yline(24650*9.81/1000,'b:','Weight'); grid on
if ~isempty(i_lo), xline(ts(i_lo),'r--'); end
xlabel('Time (s)'); ylabel('N_{gear} (kN)'); title('Total Gear Normal Force');

subplot(3,2,6)
plot(ts, u_fwd,'Color',[0 0.6 0],'LineWidth',1.5); hold on
yline(110,'k--','V_R'); grid on
if ~isempty(i_lo), xline(ts(i_lo),'r--'); end
xlabel('Time (s)'); ylabel('u_{body} (m/s)'); title('Forward Body Velocity');

sgtitle(sprintf('Nonlinear Takeoff — Mass=%.0f kg, T_{max}=140 kN, V_R=110 m/s', 24650),...
    'FontSize',13,'FontWeight','bold');
