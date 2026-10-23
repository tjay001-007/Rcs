function viz_globe(src, EXAG)
%VIZ_GLOBE  Flight trajectory on a curved WGS-84 Earth globe (geodetic lat/lon).
%
%   viz_globe                      uses recef_log from the base workspace
%   viz_globe('viz_data.mat')      loads R [N x 3] ECEF (m) from a .mat
%   viz_globe(R)                   R is an [N x 3] ECEF trajectory (m)
%   viz_globe(..., EXAG)           altitude vertical-exaggeration (default 15)
%
%   Left panel : textured Earth globe (base-MATLAB topo) showing real
%                curvature, with the flight path colored by TRUE altitude
%                (altitude is vertically exaggerated for visibility).
%   Right panel: lat/lon ground track over a topographic map.
%   The yellow star marks the Andoya runway datum (acft_datum.m).
%
%   Requires: Aerospace Toolbox (ecef2lla/lla2ecef). Saves flight_globe.png.

if nargin<1 || isempty(src), src = evalin('base','recef_log'); end
if nargin<2 || isempty(EXAG), EXAG = 15; end

% ---- resolve the ECEF trajectory R [N x 3] ---------------------------
if ischar(src) || isstring(src)
    S = load(char(src)); R = S.R;
elseif isa(src,'timeseries')
    R = squeeze(src.Data); if size(R,1)==3, R = R.'; end
else
    R = src;
end
R = double(R); if size(R,2)~=3, R = R.'; end
R = R(all(isfinite(R),2),:);

lla = ecef2lla(R); lat = lla(:,1); lon = lla(:,2); alt = max(lla(:,3),0);
[lat0r,lon0r,~,~] = acft_datum(); lat0 = lat0r*180/pi; lon0 = lon0r*180/pi;
Re = 6378137;

% ---- topo as TRUE-COLOR so it doesn't fight the altitude colormap ----
S2 = load('topo.mat');  topo = S2.topo;  cmapT = S2.topomap1;
[nlat,nlon] = size(topo);
ci  = round((topo-min(topo(:)))/(max(topo(:))-min(topo(:)))*(size(cmapT,1)-1))+1;
RGB = ind2rgb(ci, cmapT);                         % nlat x nlon x 3 truecolor
latg = linspace(-89.5,89.5,nlat);
long = linspace(0.5,359.5,nlon);

fig = figure('Name','ECEF Flight Globe','Color','k','Position',[60 60 1300 680]);

%% ===== Panel 1: curved-Earth globe =====
ax1 = subplot(1,2,1,'Parent',fig); hold(ax1,'on');
[LON,LAT] = meshgrid(long,latg);
Xg = Re*cosd(LAT).*cosd(LON); Yg = Re*cosd(LAT).*sind(LON); Zg = Re*sind(LAT);
surf(ax1,Xg,Yg,Zg,RGB,'EdgeColor','none','FaceColor','texturemap');

Rt = lla2ecef([lat, lon, EXAG*alt]);              % exaggerated-altitude path
Rh = lla2ecef([lat0, lon0, 0]);
patch(ax1,'XData',Rt(:,1),'YData',Rt(:,2),'ZData',Rt(:,3), ...
      'FaceVertexCData',alt,'EdgeColor','interp','FaceColor','none','LineWidth',3);
plot3(ax1,Rh(1),Rh(2),Rh(3),'pentagram','MarkerSize',16, ...
      'MarkerFaceColor','y','MarkerEdgeColor','k');
colormap(ax1,turbo); cb = colorbar(ax1,'Color','w');
cb.Label.String = 'true altitude (m)'; cb.Label.Color = 'w';

axis(ax1,'equal'); axis(ax1,'off');
allP = [Rt; Rh.'];
c = mean(allP,1); span = max(range(allP,1));
xlim(ax1,c(1)+[-1 1]*span*0.7); ylim(ax1,c(2)+[-1 1]*span*0.7); zlim(ax1,c(3)+[-1 1]*span*0.7);
campos(ax1, c + (c-[0 0 0])/norm(c)*span*1.8 + [0 0 span*0.6]);
camtarget(ax1,c); camva(ax1,35);
camlight(ax1,'headlight'); lighting(ax1,'gouraud'); material(ax1,'dull');
title(ax1,sprintf('Rotating-ECEF trajectory on WGS-84 globe  (alt \\times%g)',EXAG),'Color','w');

%% ===== Panel 2: lat/lon ground track =====
ax2 = subplot(1,2,2,'Parent',fig);
image(ax2,long,latg,RGB); set(ax2,'YDir','normal'); hold(ax2,'on');
lonp = mod(lon,360);
plot(ax2,lonp,lat,'-','Color',[1 0.2 0.2],'LineWidth',1.8);
plot(ax2,lonp(1),lat(1),'go','MarkerSize',8,'MarkerFaceColor','g');
plot(ax2,lonp(end),lat(end),'rs','MarkerSize',8,'MarkerFaceColor','r');
plot(ax2,mod(lon0,360),lat0,'pentagram','MarkerSize',14,'MarkerFaceColor','y','MarkerEdgeColor','k');
dlon = max(3, (max(lonp)-min(lonp))*0.6); dlat = max(2,(max(lat)-min(lat))*0.6);
xlim(ax2,[min(lonp)-dlon max(lonp)+dlon]); ylim(ax2,[min(lat)-dlat max(lat)+dlat]);
xlabel(ax2,'longitude (\circE)'); ylabel(ax2,'latitude (\circN)');
title(ax2,sprintf('Ground track  (start %.3f\\circN %.3f\\circE)',lat(1),lon(1)),'Color','k');
grid(ax2,'on'); axis(ax2,'xy');

drawnow;
print(fig,'flight_globe.png','-dpng','-r120');
fprintf('viz_globe: %d pts, alt %.0f..%.0f m, lat %.3f..%.3f, lon %.3f..%.3f\n', ...
    size(R,1), min(alt), max(alt), min(lat), max(lat), min(lon), max(lon));
fprintf('saved flight_globe.png\n');
end
