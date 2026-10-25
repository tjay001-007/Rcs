function [lat0, lon0, h0, rwy_hdg, land_hdg, rwy_len] = acft_datum()
%#codegen
% ACFT_DATUM  Single source of truth for the launch/runway geodetic datum.
%   Change THIS to relocate the whole simulation (EqM ECEF core, RTB
%   guidance, initial-condition builders all read it).
%
%   lat0, lon0 : geodetic latitude / longitude of the runway datum  [rad]
%   h0         : geodetic altitude of main-gear ground contact      [m]
%   rwy_hdg    : runway heading (direction of takeoff)              [deg]
%   land_hdg   : landing/approach heading = reciprocal of rwy_hdg   [deg]
%                The mission flies out and glides straight back, so it
%                returns from the far end and lands on the OPPOSITE end
%                (same strip, opposite direction). RTB_Guidance uses this.
%   rwy_len    : runway length (datum/RWY32 thr to RWY14 thr)        [m]
%                The datum is the RWY32 threshold (departure, SE end).
%                The LANDING threshold (RWY14, NW end) is rwy_len up the
%                runway; RTB_Guidance aims the glide slope there so the
%                aircraft touches down ON the runway, not past the far end.
%
%   NOTE: after changing this, also update the "Flat Earth to LLA" block's
%   LL0 mask to [lat0_deg lon0_deg] (kept in sync by setup_ecef.m's check).

% --- Andoya Airport (Andenes, Norway; ICAO ENAN / IATA ANX) -----------
%   Datum = RWY 32 threshold (SE end), WGS-84, from AIP-consistent data.
%   Takeoff on runway 32 (toward the NW). The heading below is the true
%   great-circle bearing from the RWY32 threshold to the RWY14 threshold
%   (69.307042 N, 16.119844 E), i.e. 329.75 deg true; computed geometry
%   agrees with the published "true alignment 150/330".
lat0_deg = 69.287933;    % RWY 32 threshold latitude  (69 deg 17' 16.6" N)
lon0_deg = 16.151344;    % RWY 32 threshold longitude (16 deg 09' 04.8" E)
h0       = 2.24;         % gear-contact altitude above the datum [m]
rwy_hdg  = 329.754;      % takeoff heading [deg true], runway 32

land_hdg = mod(rwy_hdg + 180, 360);   % landing heading [deg true] = 149.754, runway 14
rwy_len  = 2459.3;                    % runway length [m] (RWY32 thr -> RWY14 thr, ENAN 14/32)

lat0 = lat0_deg*pi/180;
lon0 = lon0_deg*pi/180;
end
