function attitude_ribbon(src, nGlyph, scaleKm, altExag)
%ATTITUDE_RIBBON  Whole-mission attitude overview: aircraft glyphs oriented by
%                 the logged body->ECEF quaternion, sampled along the 3-D path.
%
%   attitude_ribbon                      uses attitude_data.mat in the cwd
%   attitude_ribbon('attitude_data.mat') load t,R(ECEF Nx3),Qb2e(Nx4 [w x y z])
%   attitude_ribbon(so)                  a SimulationOutput (reads PosLog/QuatLog)
%   attitude_ribbon(src, nGlyph, scaleKm, altExag)
%       nGlyph  : number of aircraft glyphs along the path (default 50)
%       scaleKm : glyph size in km (default auto ~ path span/25)
%       altExag : vertical exaggeration of altitude (default 1)
%
%   Frame: local ENU tangent plane at the runway datum (acft_datum). Glyphs are
%   coloured by altitude. One static figure — rotate it to inspect the attitude
%   through ground roll, zoom, coast, glide, etc.

if nargin<1 || isempty(src), src='attitude_data.mat'; end
if nargin<2 || isempty(nGlyph), nGlyph=50; end
if nargin<4 || isempty(altExag), altExag=1; end

% ---- load data ----------------------------------------------------------
if ischar(src)||isstring(src)
    S=load(char(src)); t=S.t(:); R=S.R; Q=S.Qb2e;
elseif isa(src,'Simulink.SimulationOutput')
    P=src.logsout.getElement('PosLog').Values; Qv=src.logsout.getElement('QuatLog').Values;
    t=P.Time(:); R=squeeze(P.Data); if size(R,1)==3,R=R.'; end
    Q=squeeze(Qv.Data); if size(Q,1)==4,Q=Q.'; end
    Q=interp1(Qv.Time,Q,t); Q=Q./vecnorm(Q,2,2);
else
    error('Unsupported source.');
end
ok=all(isfinite(R),2)&all(isfinite(Q),2); t=t(ok);R=R(ok,:);Q=Q(ok,:);

% ---- ECEF -> local ENU at datum -----------------------------------------
[lat0,lon0,h0]=acft_datum(); sl=sin(lat0);cl=cos(lat0);so_=sin(lon0);co=cos(lon0);
Rea=6378137.0; f=1/298.257223563; e2=f*(2-f);
N0=Rea/sqrt(1-e2*sl^2);
R0=[(N0+h0)*cl*co; (N0+h0)*cl*so_; (N0*(1-e2)+h0)*sl];
C_e2enu=[-so_, co, 0; -sl*co, -sl*so_, cl; cl*co, cl*so_, sl];   % ENU rows
P_enu=(C_e2enu*(R.'-R0)).';                                       % [E N U] m
P_enu(:,3)=P_enu(:,3)*altExag;

% ---- glyph sample indices (even spacing in arc length) ------------------
d=[0;cumsum(vecnorm(diff(P_enu),2,2))];
[du,iu]=unique(d,'stable');                       % drop stationary (ground) duplicates
gi=interp1(du,iu,linspace(du(1),du(end),nGlyph),'nearest');
gi=unique(max(1,min(numel(t),gi(isfinite(gi)))));

% ---- glyph geometry (body frame: x fwd, y right, z down) ----------------
span=max(range(P_enu(:,1)),range(P_enu(:,2)));
if nargin<3||isempty(scaleKm), L=max(span/25,1500); else, L=scaleKm*1000; end
% simple delta-wing + fin aircraft
V=[ 1.0 0 0;        %1 nose
   -0.6 -0.9 0;     %2 wing L (left = -y)
   -0.6  0.9 0;     %3 wing R
   -0.9 0 0;        %4 tail
   -0.8 0 -0.5]*L;  %5 fin top (up = -z)
F={[1 2 3],[1 4 2],[1 4 3],[4 5 1]};   % wing + body + fin faces

% ---- figure -------------------------------------------------------------
f=figure('Color','w','Name','Attitude ribbon','NumberTitle','off','Position',[60 60 1100 720]);
ax=axes(f); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(ax,45,20);
% trajectory coloured by altitude
patch(ax,'XData',[P_enu(:,1);NaN]/1000,'YData',[P_enu(:,2);NaN]/1000,'ZData',[P_enu(:,3);NaN]/1000, ...
    'FaceVertexCData',[P_enu(:,3);NaN]/1000,'EdgeColor','interp','FaceColor','none','LineWidth',1.5);
colormap(ax,turbo); cb=colorbar(ax); cb.Label.String='altitude (km)';
alt_km=P_enu(:,3)/1000; clim(ax,[min(alt_km) max(alt_km)+eps]);
% glyphs
for k=gi(:).'
    qn=Q(k,:); w=qn(1);x=qn(2);y=qn(3);z=qn(4);
    C_b2e=[w^2+x^2-y^2-z^2, 2*(x*y-w*z), 2*(x*z+w*y);
           2*(x*y+w*z), w^2-x^2+y^2-z^2, 2*(y*z-w*x);
           2*(x*z-w*y), 2*(y*z+w*x), w^2-x^2-y^2+z^2];
    C_b2enu=C_e2enu*C_b2e;                 % body -> ENU
    Venu=(C_b2enu*V.').' + P_enu(k,:);     % rotate + translate [m]
    cdat=repmat(P_enu(k,3)/1000,5,1);
    for ff=F
        patch(ax,'Vertices',Venu/1000,'Faces',ff{1}, ...
            'FaceVertexCData',cdat,'FaceColor','flat','EdgeColor','k','FaceAlpha',0.9,'LineWidth',0.3);
    end
end
plot3(ax,0,0,0,'kp','MarkerSize',14,'MarkerFaceColor','y');   % datum
xlabel(ax,'East (km)'); ylabel(ax,'North (km)'); zlabel(ax,'Up (km)');
title(ax,sprintf('Aircraft attitude along mission (%d glyphs, glyph %.1f km)',numel(gi),L/1000));
hold(ax,'off');
fprintf('attitude_ribbon: %d glyphs over %.0f s, alt %.1f..%.1f km, span %.0f km\n', ...
    numel(gi), t(end)-t(1), min(alt_km), max(alt_km), span/1000);
end
