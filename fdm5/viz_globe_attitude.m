function [sc, v] = viz_globe_attitude(src, modelScale, attConj, modelFile, follow, playSpeed)
%VIZ_GLOBE_ATTITUDE  Mission replay on the Cesium globe with an ORIENTED aircraft
%   mesh — the airframe's true attitude (from the logged body->ECEF quaternion)
%   is shown on the satelliteScenario globe, not just the ground track.
%
%   viz_globe_attitude                         uses attitude_data.mat (t,R,Qb2e)
%   viz_globe_attitude(so)                     a SimulationOutput (PosLog/QuatLog)
%   viz_globe_attitude('attitude_data.mat')    load t,R(ECEF Nx3),Qb2e(Nx4 [w x y z])
%   viz_globe_attitude(src, modelScale)        3-D model size multiplier (default 3000)
%   viz_globe_attitude(src, modelScale, attConj)  flip attitude convention if the
%                                              mesh looks mirrored/reversed (default false)
%   viz_globe_attitude(src, modelScale, attConj, playSpeed)  playback speed (default 30)
%
%   Requires Aerospace Toolbox (satelliteScenario platform with Attitude +
%   Visual3DModel, R2024a+). Uses the shipped AirTransport.glb mesh.

if nargin<1||isempty(src),        src='attitude_data.mat'; end
if nargin<2||isempty(modelScale), modelScale=1; end
if nargin<3||isempty(attConj),    attConj=false; end
if nargin<4||isempty(modelFile),  modelFile='T38.stl'; end  % any glTF .glb/.gltf
if nargin<5||isempty(follow),     follow=true; end                   % chase-camera follow (default ON)
if nargin<6||isempty(playSpeed),  playSpeed=30; end

% ---- resolve t, R(ECEF Nx3), Q(body->ECEF Nx4 [w x y z]) ----------------
if ischar(src)||isstring(src)
    S=load(char(src)); t=S.t(:); R=S.R; Q=S.Qb2e;
elseif isa(src,'Simulink.SimulationOutput')
    P=src.logsout.getElement('PosLog').Values; Qv=src.logsout.getElement('QuatLog').Values;
    t=P.Time(:); R=squeeze(P.Data); if size(R,1)==3,R=R.'; end
    Q=squeeze(Qv.Data); if size(Q,1)==4,Q=Q.'; end
    Q=interp1(Qv.Time,Q,t); Q=Q./vecnorm(Q,2,2);
else, error('Unsupported source.'); end
ok=all(isfinite(R),2)&all(isfinite(Q),2); t=t(ok);R=R(ok,:);Q=Q(ok,:);
t=t-t(1);

% ---- resample to ~2 Hz for smooth, fast playback ------------------------
dt0=median(diff(t)); HZ=10;   % model keyframe rate on the globe (smoother motion/attitude)
if dt0<1/HZ
    step=max(1,round(1/(dt0*HZ))); k=1:step:numel(t);
    t=t(k); R=R(k,:); Q=Q(k,:);
end
dt=median(diff(t)); if ~isfinite(dt)||dt<=0, dt=0.5; end

% ---- geodetic position + body->NED attitude quaternion ------------------
LLA=ecef2lla(R); lat=LLA(:,1); lon=LLA(:,2); alt=max(LLA(:,3),0);
qbn=zeros(size(Q,1),4);
for i=1:size(Q,1)
    w=Q(i,1);x=Q(i,2);y=Q(i,3);z=Q(i,4);
    C_b2e=[w^2+x^2-y^2-z^2,2*(x*y-w*z),2*(x*z+w*y);
           2*(x*y+w*z),w^2-x^2+y^2-z^2,2*(y*z-w*x);
           2*(x*z-w*y),2*(y*z+w*x),w^2-x^2-y^2+z^2];
    la=lat(i)*pi/180; lo=lon(i)*pi/180; sl=sin(la);cl=cos(la);so_=sin(lo);co=cos(lo);
    C_e2ned=[-sl*co,-sl*so_,cl; -so_,co,0; -cl*co,-cl*so_,-sl];
    %C=C_e2ned*C_b2e;                       % body -> NED

    C = C_e2ned*C_b2e*[0 -1 0; 1 0 0; 0 0 1];
    qbn(i,:)=i_dcm2quat(C);
end
if attConj, qbn(:,2:4)=-qbn(:,2:4); end    % conjugate if convention is reversed

% ---- scenario + oriented aircraft ---------------------------------------
t0=datetime(2024,1,1,0,0,0,'TimeZone','UTC');
sc=satelliteScenario(t0, t0+seconds(t(end)+dt), dt);
pos=timeseries([lat lon alt], t);
att=timeseries(qbn, t);
ac=platform(sc, pos, 'Attitude', att, 'Name','ACFT11');
ac.Visual3DModel=modelFile;            % custom glTF: pass your own .glb/.gltf path
ac.Visual3DModelScale=modelScale;
ac.ShowLabel=true;

[lat0r,lon0r]=acft_datum();
gs=groundStation(sc, lat0r*180/pi, lon0r*180/pi, 'Name','Andoya Runway','Altitude',2.24);
gs.ShowLabel=true;

v=satelliteScenarioViewer(sc,'Name','ACFT11 - Attitude Replay','Basemap','satellite');
try, path(ac,'LineColor',[1 0.3 0],'LineWidth',2); catch, end
if follow                               % chase camera: lock onto + follow the aircraft
    try
        camtarget(v, ac);               % camera tracks the platform through the whole flight
        camheight(v, 250);             % chase distance above/behind (tune: camheight(v,..))
        campitch(v, -25);               % look down at the aircraft (tune: campitch(v,..))
    catch err
        fprintf('camera-follow note: %s\n', err.message);
    end
end
% Live camera tuning on the returned viewer v:  camheight(v,1000) | campitch(v,-15) |
%   camheading(v,180) | camtarget(v) to release the follow.

fprintf('viz_globe_attitude: %d pts, dt=%.2f s, sim=%.0f s, alt %.0f..%.0f m\n', numel(t),dt,t(end),min(alt),max(alt));
fprintf('Press Play in the viewer (or play(sc)). Model=AirTransport.glb scale=%g.\n', modelScale);
fprintf('If the aircraft looks mirrored/reversed, re-run with attConj=true.\n');
end

% ---- scalar-first quaternion from DCM (v_ref = C*v_body) ----------------
function q=i_dcm2quat(C)
tr=C(1,1)+C(2,2)+C(3,3);
if tr>0
    S=sqrt(tr+1)*2; w=0.25*S; x=(C(3,2)-C(2,3))/S; y=(C(1,3)-C(3,1))/S; z=(C(2,1)-C(1,2))/S;
elseif (C(1,1)>C(2,2))&&(C(1,1)>C(3,3))
    S=sqrt(1+C(1,1)-C(2,2)-C(3,3))*2; w=(C(3,2)-C(2,3))/S; x=0.25*S; y=(C(1,2)+C(2,1))/S; z=(C(1,3)+C(3,1))/S;
elseif C(2,2)>C(3,3)
    S=sqrt(1+C(2,2)-C(1,1)-C(3,3))*2; w=(C(1,3)-C(3,1))/S; x=(C(1,2)+C(2,1))/S; y=0.25*S; z=(C(2,3)+C(3,2))/S;
else
    S=sqrt(1+C(3,3)-C(1,1)-C(2,2))*2; w=(C(2,1)-C(1,2))/S; x=(C(1,3)+C(3,1))/S; y=(C(2,3)+C(3,2))/S; z=0.25*S;
end
q=[w x y z]; q=q/norm(q);
end

