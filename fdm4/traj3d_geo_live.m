function traj3d_geo_live(r_ecef, phase)
%TRAJ3D_GEO_LIVE  Live geodetic trajectory on a textured WGS-84 globe.
%   Called from the Simulink Traj3D block each step:
%       traj3d_geo_live(r_ecef, phase)
%
%   Left panel  : textured Earth globe (topo.mat), path coloured by altitude.
%   Right panel : 2-D lat/lon ground-track map.
%   One persistent figure (tag 'GeoLiveFig'). Camera stays aimed at Earth
%   centre and follows the flight path -- never clips the globe.

persistent fig ax1 ax2 hTraj hPos hGnd hGpos LAT LON ALT n lat0 lon0 EXAG

TAG  = 'GeoLiveFig';
Re   = 6378137;

% ---- one-time init ----
if isempty(n)
    n = 0; EXAG = 12;
    [a,b] = acft_datum(); lat0 = a*180/pi; lon0 = b*180/pi;
    LAT = zeros(1,200000,'single');
    LON = zeros(1,200000,'single');
    ALT = zeros(1,200000,'single');
end

lla = ecef2lla(double(r_ecef(:)).');   % [lat lon alt]
la  = lla(1); lo = lla(2); al = max(lla(3), 0);

% ---- one-shot reset when a new run begins (was airborne → back at datum) ----
if n > 0 && ALT(n) > 300 && al < 30 && abs(la-lat0) < 0.05 && abs(lo-lon0) < 0.05
    n = 0;
end

% ---- ensure exactly ONE figure (reuse by tag) ----
if isempty(fig) || ~isvalid(fig)
    old = findobj(0,'Type','figure','Tag',TAG);
    if ~isempty(old), close(old); end

    fig = figure('Name','Live Geodetic Trajectory','NumberTitle','off', ...
                 'Tag',TAG,'Color','k','Position',[80 80 1300 660]);

    % ----- build topo texture -----
    S = load('topo.mat');
    topo   = S.topo;
    cmapT  = S.topomap1;
    [nla, nlo] = size(topo);
    ci  = round((double(topo) - min(double(topo(:)))) / ...
                (max(double(topo(:))) - min(double(topo(:)))) * ...
                (size(cmapT,1)-1)) + 1;
    RGB = ind2rgb(ci, cmapT);        % nla x nlo x 3, double [0..1]
    latg = linspace(-89.5, 89.5, nla);
    long = linspace(  0.5, 359.5, nlo);
    [LOgrid, LAgrid] = meshgrid(long, latg);

    % ===== LEFT: 3-D globe =====
    ax1 = subplot(1,2,1,'Parent',fig);
    hold(ax1,'on');

    % Globe surface — truecolor texturemap, NO per-vertex colourmap scaling
    surf(ax1, ...
         Re*cosd(LAgrid).*cosd(LOgrid), ...
         Re*cosd(LAgrid).*sind(LOgrid), ...
         Re*sind(LAgrid), ...
         RGB, ...
         'EdgeColor','none', ...
         'FaceColor','texturemap', ...
         'FaceLighting','gouraud', ...
         'AmbientStrength',0.6, ...
         'DiffuseStrength',0.6, ...
         'SpecularStrength',0.05);

    % Datum marker
    Rh = lla2ecef([lat0 lon0 0]);
    plot3(ax1, Rh(1), Rh(2), Rh(3), 'p', 'MarkerSize',14, ...
          'MarkerFaceColor','y', 'MarkerEdgeColor','k');

    % Trajectory patch (altitude-coloured line)
    hTraj = patch(ax1, 'XData',NaN, 'YData',NaN, 'ZData',NaN, ...
                  'FaceVertexCData', 0, ...
                  'EdgeColor','interp', 'FaceColor','none', 'LineWidth',2.5);

    % Current-position marker
    hPos = plot3(ax1, Rh(1), Rh(2), Rh(3), 'o', ...
                 'MarkerSize',9, 'MarkerFaceColor','r', 'MarkerEdgeColor','w', 'LineWidth',1.5);

    % Colourmap for trajectory only (globe uses truecolor, unaffected)
    colormap(ax1, turbo);
    clim(ax1, [0 30000]);
    cb = colorbar(ax1, 'Color','w');
    cb.Label.String = 'altitude (m)';
    cb.Label.Color  = 'w';

    % Axis / camera
    axis(ax1, 'equal'); axis(ax1, 'off'); axis(ax1, 'vis3d');

    % Camera positioned above the datum, aimed at Earth centre
    cam_dir = Rh / norm(Rh);               % unit radial toward datum
    campos(ax1,  (cam_dir * Re * 2.6).');  % 2.6 Re from centre
    camtarget(ax1, [0 0 0]);               % look at Earth centre
    camva(ax1, 28);                        % covers ~55° arc = full disc + margin

    % Single soft sun-like light (fixed in scene)
    light(ax1, 'Position', [1 0.5 1]*1e8, 'Style','infinite', 'Color','w');

    title(ax1, 'Live ECEF trajectory – WGS-84 globe', 'Color','w', 'FontSize',10);

    % ===== RIGHT: 2-D ground track =====
    ax2 = subplot(1,2,2,'Parent',fig);
    image(ax2, long, latg, RGB);
    set(ax2, 'YDir','normal');
    hold(ax2,'on');

    hGnd  = plot(ax2, NaN, NaN, '-', 'Color',[1 0.3 0.3], 'LineWidth',1.8);
    hGpos = plot(ax2, mod(lon0,360), lat0, 'o', 'MarkerSize',7, ...
                 'MarkerFaceColor','r', 'MarkerEdgeColor','w');
    plot(ax2, mod(lon0,360), lat0, 'p', 'MarkerSize',13, ...
         'MarkerFaceColor','y', 'MarkerEdgeColor','k');

    set(ax2, 'XColor','w', 'YColor','w');
    grid(ax2,'on');
    xlabel(ax2,'longitude (°E)','Color','w');
    ylabel(ax2,'latitude (°N)','Color','w');
    title(ax2,'Ground track','Color','w','FontSize',10);
    xlim(ax2,[lon0-25  lon0+25]);
    ylim(ax2,[lat0-15  lat0+15]);

    drawnow;
end

% ---- accumulate point ----
n = n + 1;
if n > numel(LAT)
    LAT(2*numel(LAT)) = 0;
    LON(2*numel(LON)) = 0;
    ALT(2*numel(ALT)) = 0;
end
LAT(n) = la; LON(n) = lo; ALT(n) = al;

% ---- throttled live update (every 30 steps, or at end) ----
if mod(n,30) == 0 || phase >= 15
    idx = 1:n;

    % Exaggerated-altitude ECEF path (for 3-D globe visibility)
    Rt = lla2ecef([double(LAT(idx)).', double(LON(idx)).', EXAG*double(ALT(idx)).']);

    % Update trajectory line
    set(hTraj, 'XData',Rt(:,1), 'YData',Rt(:,2), 'ZData',Rt(:,3), ...
               'FaceVertexCData', double(ALT(idx)).');

    % Update position marker
    set(hPos, 'XData',Rt(end,1), 'YData',Rt(end,2), 'ZData',Rt(end,3));

    % Update globe title
    if isvalid(ax1)
        title(ax1, sprintf('%.3f°N  %.3f°E  alt %.0f m  (phase %g)', la, lo, al, phase), ...
              'Color','w','FontSize',9);

        % Move camera so it stays "above" the midpoint of the path
        if n > 10
            mid_lla  = [mean(double(LAT(idx))), mean(double(LON(idx))), 0];
            Rmid     = lla2ecef(mid_lla);
            new_dir  = Rmid / norm(Rmid);
            new_cam  = new_dir * 6378137 * 2.6;
            % Smooth: blend current camera toward new position
            cur_cam  = campos(ax1);
            campos(ax1, cur_cam*0.97 + new_cam*0.03);
            camtarget(ax1, [0 0 0]);
        end
    end

    % Update 2-D ground track
    lonp = mod(double(LON(idx)), 360);
    set(hGnd,  'XData', lonp,     'YData', double(LAT(idx)));
    set(hGpos, 'XData', lonp(end),'YData', double(LAT(idx(end))));

    if isvalid(ax2)
        lo_ctr = lonp(end); la_ctr = double(LAT(idx(end)));
        dlat = max(5, (max(double(LAT(idx)))-min(double(LAT(idx))))*0.7);
        dlon = max(5, (max(lonp)-min(lonp))*0.7);
        xlim(ax2, [lo_ctr-dlon  lo_ctr+dlon]);
        ylim(ax2, [la_ctr-dlat  la_ctr+dlat]);
    end

    drawnow limitrate;
end
end
