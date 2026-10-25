function sc = viz_satscenario(src, EXAG, playSpeed)
%VIZ_SATSCENARIO  Play back ACFT11 flight on a live Cesium globe using
%                 Aerospace Toolbox satelliteScenario + satelliteScenarioViewer.
%
%   viz_satscenario                        uses recef_log from base workspace
%   viz_satscenario(recef_log)             pass workspace timeseries directly
%   viz_satscenario('recef_log.mat')       loads R [Nx3] ECEF from .mat
%   viz_satscenario(R)                     R is [Nx3] ECEF [m]
%   viz_satscenario(R, EXAG)               altitude vertical-exaggeration (default 1)
%   viz_satscenario(R, EXAG, playSpeed)    playback speed multiplier (default 10)
%   sc = viz_satscenario(...)              returns the scenario object
%
%   Requires: Aerospace Toolbox (R2024a+) for satelliteScenario/platform.

%% ---- defaults ----------------------------------------------------------
if nargin < 1 || isempty(src),       src = evalin('base','recef_log'); end
if nargin < 2 || isempty(EXAG),      EXAG = 1;    end
if nargin < 3 || isempty(playSpeed), playSpeed = 10; end

%% ---- resolve ECEF trajectory R [Nx3] and time vector t_s (seconds) ----
if ischar(src) || isstring(src)
    S = load(char(src)); fn = fieldnames(S); R = S.(fn{1});
    t_s = (0:size(R,1)-1).';
elseif isa(src,'timeseries')
    R   = squeeze(src.Data);
    if size(R,1) == 3, R = R.'; end
    t_s = src.Time(:);
elseif isstruct(src) && isfield(src,'Values')
    % Simulink.SimulationData.Signal
    R   = squeeze(src.Values.Data);
    if size(R,1) == 3, R = R.'; end
    t_s = src.Values.Time(:);
else
    R   = double(src);
    t_s = (0:size(R,1)-1).';
end
if size(R,2) ~= 3, R = R.'; end
keep = all(isfinite(R),2);
R    = R(keep,:);
t_s  = t_s(keep);

%% ---- resample to max 2 Hz (satelliteScenario needs reasonable dt) ------
TARGET_HZ = 1;          % 1 sample/s → smooth playback, fast build
raw_dt    = median(diff(t_s));
if raw_dt < 1/TARGET_HZ
    step = max(1, round(1/(raw_dt*TARGET_HZ)));
    idx  = 1:step:size(R,1);
    R    = R(idx,:);
    t_s  = t_s(idx);
    fprintf('viz_satscenario: resampled from %d to %d pts (%.0f Hz → %g Hz)\n', ...
        numel(keep), numel(idx), 1/raw_dt, 1/median(diff(t_s)));
end
N  = size(R,1);
dt = median(diff(t_s));
if ~isfinite(dt) || dt <= 0, dt = 1; end
% Normalise time to start at 0
t_s = t_s - t_s(1);

%% ---- ECEF -> geodetic LLA [deg deg m] ----------------------------------
LLA     = ecef2lla(R);
lat     = LLA(:,1);  lon = LLA(:,2);  alt = LLA(:,3);
alt_vis = max(alt,0) * EXAG;

%% ---- datum info --------------------------------------------------------
[lat0r, lon0r, ~, ~] = acft_datum();
lat0 = lat0r*180/pi;   lon0 = lon0r*180/pi;

%% ---- build scenario ----------------------------------------------------
t0   = datetime(2024,1,1,0,0,0,'TimeZone','UTC');
tEnd = t0 + seconds(t_s(end) + dt);
scenario = satelliteScenario(t0, tEnd, dt);

%% ---- add aircraft platform ---------------------------------------------
pos_ts = timeseries([lat, lon, alt_vis], t_s, 'Name','LLA_ECEF');
ac = platform(scenario, pos_ts, 'Name','ACFT11');
ac.ShowLabel = true;

%% ---- runway datum ------------------------------------------------------
gs = groundStation(scenario, lat0, lon0, ...
    'Name','Andoya Runway', 'Altitude', 2.24);
gs.ShowLabel = true;

%% ---- open viewer -------------------------------------------------------
v = satelliteScenarioViewer(scenario, ...
    'Name','ACFT11 – Mission Replay', ...
    'Basemap','satellite');
try, path(ac,'LineColor',[1 0.3 0],'LineWidth',2); catch, end

%% ---- report ------------------------------------------------------------
fprintf('viz_satscenario: %d pts, dt=%.2f s, sim=%.0f s, alt %.0f..%.0f m\n', ...
    N, dt, t_s(end), min(alt), max(alt));
fprintf('lat %.4f..%.4f  lon %.4f..%.4f\n', min(lat),max(lat),min(lon),max(lon));
fprintf('Viewer open — press Play in the viewer toolbar (or call play(sc,%g) in command window)\n', playSpeed);

if nargout > 0, sc = scenario; end
end
