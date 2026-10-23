function kml_path = export_kml(src, filename, EXAG)
%EXPORT_KML  Export ACFT11 trajectory to KML — open in Google Earth.
%
%   export_kml                            uses recef_log from base workspace
%   export_kml('recef_log.mat')           loads from .mat
%   export_kml(R)                         R is [Nx3] ECEF [m]
%   export_kml(R, 'flight.kml')           custom filename (default: acft11_flight.kml)
%   export_kml(R, [], EXAG)               altitude exaggeration (default 1 = true alt)
%   kml_path = export_kml(...)            returns full path to .kml file
%
%   After running, open the .kml file in:
%     * Google Earth desktop  — File > Open, or double-click
%     * Google Earth web      — earth.google.com  (drag & drop the file)
%   No MATLAB toolbox required.

if nargin < 1 || isempty(src),      src = evalin('base','recef_log'); end
if nargin < 2 || isempty(filename), filename = 'acft11_flight.kml'; end
if nargin < 3 || isempty(EXAG),     EXAG = 1; end

%% ---- resolve ECEF trajectory ------------------------------------------
if ischar(src) || isstring(src)
    S = load(char(src)); R = S.R;
elseif isa(src,'timeseries')
    R = squeeze(src.Data); if size(R,1)==3, R = R.'; end
elseif isstruct(src) && isfield(src,'Data')
    R = squeeze(src.Values.Data); if size(R,1)==3, R = R.'; end
else
    R = double(src);
end
if size(R,2) ~= 3, R = R.'; end
R = R(all(isfinite(R),2),:);

%% ---- ECEF -> geodetic --------------------------------------------------
LLA = ecef2lla(R);
lat = LLA(:,1); lon = LLA(:,2); alt = max(LLA(:,3),0);
alt_vis = alt * EXAG;

%% ---- datum ------------------------------------------------------------
[lat0r, lon0r, h0, ~] = acft_datum();
lat0 = lat0r*180/pi;   lon0 = lon0r*180/pi;

%% ---- altitude range for color scale -----------------------------------
alt_max = max(alt_vis);
N = numel(lat);

%% ---- subsample for file size (max 8000 points) ------------------------
if N > 8000
    idx = round(linspace(1,N,8000));
else
    idx = 1:N;
end
lat_s = lat(idx); lon_s = lon(idx); alt_s = alt_vis(idx); alt_true = alt(idx);

%% ---- build KML color string (altitude → blue→yellow→red) -------------
function hex = alt2color(a, amax)
    frac = min(max(a/max(amax,1), 0), 1);
    r = min(1, 2*frac);
    b = min(1, 2*(1-frac));
    g = 1 - abs(2*frac-1);
    % KML color: aabbggrr  (alpha=ff)
    hex = sprintf('ff%02x%02x%02x', round(b*255), round(g*255), round(r*255));
end

%% ---- assemble coordinate strings for KML segments --------------------
% Split into ~200 pt segments so Google Earth can color by altitude
seg_len = 200;
n_segs  = ceil(numel(lat_s)/seg_len);

seg_xml = '';
for k = 1:n_segs
    i0 = (k-1)*seg_len + 1;
    i1 = min(k*seg_len, numel(lat_s));
    seg_lat = lat_s(i0:i1);
    seg_lon = lon_s(i0:i1);
    seg_alt = alt_s(i0:i1);
    a_mid   = mean(alt_true(i0:i1));
    col     = alt2color(a_mid, alt_max);
    % Build coordinate list
    coord_str = '';
    for j = 1:numel(seg_lat)
        coord_str = [coord_str sprintf('%.6f,%.6f,%.1f\n', seg_lon(j), seg_lat(j), seg_alt(j))]; %#ok<AGROW>
    end
    seg_xml = [seg_xml sprintf([ ...
        '<Placemark>\n' ...
        '  <Style><LineStyle><color>%s</color><width>3</width></LineStyle></Style>\n' ...
        '  <LineString><altitudeMode>absolute</altitudeMode><coordinates>\n' ...
        '%s' ...
        '  </coordinates></LineString>\n' ...
        '</Placemark>\n'], col, coord_str)]; %#ok<AGROW>
end

%% ---- build full KML document ------------------------------------------
kml_doc = ['<?xml version="1.0" encoding="UTF-8"?>\n' ...
'<kml xmlns="http://www.opengis.net/kml/2.2">\n' ...
'<Document>\n' ...
'  <name>ACFT11 – Andoya Mission</name>\n' ...
'  <description>Rotating-ECEF 6-DOF flight, ' sprintf('%d',N) ' points</description>\n' ...
'\n' ...
'  <!-- Runway datum marker -->\n' ...
'  <Placemark>\n' ...
'    <name>Andoya Runway</name>\n' ...
'    <Style><IconStyle><color>ff00ffff</color><scale>1.5</scale>' ...
'<Icon><href>http://maps.google.com/mapfiles/kml/shapes/airports.png</href></Icon>' ...
'</IconStyle></Style>\n' ...
'    <Point><altitudeMode>relativeToGround</altitudeMode>' ...
'<coordinates>' sprintf('%.6f,%.6f,%.1f', lon0, lat0, h0) '</coordinates></Point>\n' ...
'  </Placemark>\n' ...
'\n' ...
'  <!-- Start point -->\n' ...
'  <Placemark>\n' ...
'    <name>Take-off</name>\n' ...
'    <Style><IconStyle><color>ff00ff00</color><scale>1.2</scale></IconStyle></Style>\n' ...
'    <Point><altitudeMode>absolute</altitudeMode>' ...
'<coordinates>' sprintf('%.6f,%.6f,%.1f', lon_s(1), lat_s(1), alt_s(1)) '</coordinates></Point>\n' ...
'  </Placemark>\n' ...
'\n' ...
'  <!-- End point -->\n' ...
'  <Placemark>\n' ...
'    <name>Landing</name>\n' ...
'    <Style><IconStyle><color>ff0000ff</color><scale>1.2</scale></IconStyle></Style>\n' ...
'    <Point><altitudeMode>absolute</altitudeMode>' ...
'<coordinates>' sprintf('%.6f,%.6f,%.1f', lon_s(end), lat_s(end), alt_s(end)) '</coordinates></Point>\n' ...
'  </Placemark>\n' ...
'\n' ...
'  <!-- Flight path segments colored by altitude -->\n' ...
'  <Folder><name>Flight path (colored by altitude)</name>\n' ...
seg_xml ...
'  </Folder>\n' ...
'</Document>\n' ...
'</kml>\n'];

%% ---- write file -------------------------------------------------------
kml_content = sprintf(kml_doc);
fid = fopen(filename,'w');
if fid < 0
    error('export_kml: cannot open "%s" for writing', filename);
end
fwrite(fid, kml_content, 'char');
fclose(fid);

kml_path = fullfile(pwd, filename);
fprintf('export_kml: wrote %s\n', kml_path);
fprintf('  %d track points, alt %.0f..%.0f m, lat %.3f..%.3f deg\n', ...
    numel(lat_s), min(alt_true), max(alt_true), min(lat_s), max(lat_s));
fprintf('\nTo view:\n');
fprintf('  Google Earth desktop : double-click  %s\n', filename);
fprintf('  Google Earth web     : drag & drop   %s  onto earth.google.com\n', filename);

if nargout < 1, clear kml_path; end
end
