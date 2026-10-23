function sat_live_update(dt_refresh)
%SAT_LIVE_UPDATE  Near-live Cesium globe view while the sim is running.
%
%   sat_live_update          refresh every 15 s (default)
%   sat_live_update(30)      refresh every 30 s
%
%   Call this ONCE from the MATLAB command window AFTER the sim has been
%   started (but while it is still running).  It reads TW_recef (the
%   logged ECEF timeseries) from the base workspace every dt_refresh
%   seconds, rebuilds a satelliteScenario with the data so far, and plays
%   it back at 50x speed so you can watch the progress in the Cesium viewer.
%
%   Press Ctrl-C in MATLAB to stop the loop.
%
%   NOTE: satelliteScenario requires the trajectory ahead of time, so this
%   gives a "near-live" replay of what has already been simulated rather
%   than a true 1:1 live view.  For a true step-by-step live view while
%   the sim runs use traj3d_geo_live (called automatically by the model).

if nargin < 1 || isempty(dt_refresh), dt_refresh = 15; end

fprintf('sat_live_update: polling every %.0f s. Press Ctrl-C to stop.\n', dt_refresh);

while true
    try
        ts = evalin('base','TW_recef');
    catch
        fprintf('  waiting for TW_recef in base workspace...\n');
        pause(dt_refresh);
        continue;
    end

    R = squeeze(ts.Data);
    if size(R,1) == 3, R = R.'; end
    t = ts.Time;
    R = R(all(isfinite(R),2),:);

    if size(R,1) < 5
        fprintf('  not enough data yet (%d pts)\n', size(R,1));
        pause(dt_refresh);
        continue;
    end

    LLA = ecef2lla(R);
    lat = LLA(:,1); lon = LLA(:,2); alt = max(LLA(:,3),0);
    N   = size(R,1);
    dt  = median(diff(t));
    if ~isfinite(dt) || dt <= 0, dt = 1; end

    [lat0r, lon0r, ~, ~] = acft_datum();
    lat0 = lat0r*180/pi; lon0 = lon0r*180/pi;

    t0   = datetime(2024,1,1,0,0,0,'TimeZone','UTC');
    tEnd = t0 + seconds(N*dt);

    scenario = satelliteScenario(t0, tEnd, dt);

    pos_ts = timeseries([lat, lon, alt], (0:N-1).'*dt, 'Name','LLA');
    ac = platform(scenario, pos_ts, 'Name','ACFT11');
    ac.ShowLabel = true;

    gs = groundStation(scenario, lat0, lon0, ...
        'Name','Andoya', 'Altitude', 2.24);
    gs.ShowLabel = true;

    fprintf('  rebuilding scenario: %d pts, sim t=%.0f s, alt %.0f..%.0f m\n', ...
        N, t(end), min(alt), max(alt));

    v = satelliteScenarioViewer(scenario, ...
        'Name',sprintf('ACFT11 live (t=%.0f s)', t(end)), ...
        'Basemap','satellite');
    show(ac,'Path','on');
    play(scenario,'PlaybackSpeedMultiplier',50);

    pause(dt_refresh);
end
end
