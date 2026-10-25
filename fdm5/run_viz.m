% run_viz.m  — post-sim visualization (trajectory + ATTITUDE)
%
%   Run after a sim (needs PosLog + QuatLog signal logging, saved in the model;
%   or a previously saved attitude_data.mat).

%% ========================= SET YOUR AIRCRAFT MODEL / CAMERA =============
%AIRCRAFT_MODEL = 'AirTransport.glb';   % <-- model file: shipped 'AirTransport.glb'
                                       %     or YOUR OWN:  'C:\Users\tejve\...\MyAircraft.stl'
AIRCRAFT_MODEL = 'T38.stl';                                      %     (.glb / .gltf / .stl all accepted)
AIRCRAFT_SCALE = 1;                    % <-- model size multiplier
ATT_CONJUGATE  = true;                % <-- true if the model looks mirrored/reversed
CAM_MODE       = 'wing';          % <-- 'chase' | 'wing' | 'orbit' | 'top' | 'fpv' | 'cinematic'
CHASE_BACK     = 200;                  % <-- camera distance scale BEHIND the aircraft [m]
CHASE_UP       = 200;                  % <-- camera height scale ABOVE the aircraft [m]
                                       %     (shots scale off these; CHASE_BACK = 0 = free camera)
PLAY_SPEED     = 30;                   % <-- playback speed x real-time (lower = smoother)
%% ========================================================================

src = 'attitude_data.mat';
if exist('so','var')
    P = so.logsout.getElement('PosLog').Values;  Q = so.logsout.getElement('QuatLog').Values;
    t = P.Time(:);  R = squeeze(P.Data); if size(R,1)==3, R=R.'; end
    Qb2e = squeeze(Q.Data); if size(Qb2e,1)==4, Qb2e=Qb2e.'; end
    Qb2e = interp1(Q.Time, Qb2e, t);  Qb2e = Qb2e./vecnorm(Qb2e,2,2);
    save(src,'t','R','Qb2e');
end

attitude_ribbon(src, 55);                                   % E — whole-mission overview

% D — oriented aircraft mesh on the Cesium globe (viewer follow off; the HUD
%     playback drives the globe clock + the cinematic camera)
[sc, v] = viz_globe_attitude(src, AIRCRAFT_SCALE, ATT_CONJUGATE, AIRCRAFT_MODEL, false);

% Camera-only chase/cinematic (HUD off), smooth & synced to the globe
flight_hud(src, PLAY_SPEED, [], v, CHASE_BACK, CHASE_UP, CAM_MODE, false);

%   Trajectory-only (legacy):  sc = viz_satscenario(recef_log);  export_kml(recef_log)
