%% setup_ecef.m  --  ECEF 6-DOF initial conditions & datum constants
%  Run this BEFORE simulating ACFT11_a_121 once the EqM block has been
%  converted to the rotating-ECEF formulation.  It defines the geodetic
%  datum (runway threshold), the WGS-84 constants, and the 13-element
%  initial state vector ecef_x0 used by the ECEF_Core integrators.
%
%  State layout (13x1):
%     1:3   dr      ECEF position offset from datum origin r0   [m]
%     4:6   v_e     velocity w.r.t Earth, expressed in ECEF     [m/s]
%     7:10  q_b2e   unit quaternion body->ECEF, scalar-first    [-]
%     11:13 w_b     body rate w.r.t inertial, in body           [rad/s]
%
%  The datum is defined in acft_datum.m (Andoya ENAN, RWY 14 threshold).
%  Initial attitude: phi=0, theta=2deg, psi=rwy_hdg (nose along the runway
%  centerline), with the aircraft at rest on the runway (alt = 2.24 m).
% -----------------------------------------------------------------------

%% --- WGS-84 / Earth constants -----------------------------------------
ECEF_we  = 7.292115e-5;        % Earth rotation rate                [rad/s]
ECEF_mu  = 3.986004418e14;     % Earth gravitational parameter      [m^3/s^2]
ECEF_J2  = 1.082626683e-3;     % second zonal harmonic              [-]
ECEF_Rea = 6378137.0;          % WGS-84 equatorial radius           [m]
ECEF_f   = 1/298.257223563;    % WGS-84 flattening                  [-]
ECEF_e2  = ECEF_f*(2-ECEF_f);  % first eccentricity squared         [-]

%% --- Geodetic datum (runway threshold) --------------------------------
[lat0, lon0, h0, rwy_hdg] = acft_datum();   % geodetic datum (see acft_datum.m)

%% --- Datum origin in ECEF ---------------------------------------------
N0 = ECEF_Rea/sqrt(1 - ECEF_e2*sin(lat0)^2);
ECEF_r0 = [ (N0+h0)*cos(lat0)*cos(lon0);
            (N0+h0)*cos(lat0)*sin(lon0);
            (N0*(1-ECEF_e2)+h0)*sin(lat0) ];

%% --- Initial attitude --------------------------------------------------
phi0 = 0; theta0 = 2*pi/180; psi0 = rwy_hdg*pi/180;   % nose aligned with runway

% Body->NED DCM (3-2-1 Euler), i.e. v_ned = C_b2ned * v_body
cph=cos(phi0); sph=sin(phi0);
cth=cos(theta0); sth=sin(theta0);
cps=cos(psi0); sps=sin(psi0);
C_ned2b = [ cth*cps,              cth*sps,             -sth;
            sph*sth*cps-cph*sps,  sph*sth*sps+cph*cps,  sph*cth;
            cph*sth*cps+sph*sps,  cph*sth*sps-sph*cps,  cph*cth ];
C_b2ned = C_ned2b.';

% ECEF->NED DCM at the datum
slat=sin(lat0); clat=cos(lat0); slon=sin(lon0); clon=cos(lon0);
C_e2ned = [ -slat*clon, -slat*slon,  clat;
            -slon,        clon,       0;
            -clat*clon, -clat*slon, -slat ];
C_ned2e = C_e2ned.';

% Body->ECEF DCM and quaternion (scalar-first)
C_b2e0 = C_ned2e * C_b2ned;
ECEF_q0 = dcm2quat_sf(C_b2e0);

%% --- Initial rates & velocity -----------------------------------------
ECEF_v0 = [0;0;0];                       % at rest w.r.t Earth
w_ie_b0 = C_b2e0.'*[0;0;ECEF_we];        % Earth rate seen in body
ECEF_w0 = w_ie_b0;                        % rigid-with-Earth at t=0

%% --- Assembled initial state ------------------------------------------
ecef_x0 = [ [0;0;0]; ECEF_v0; ECEF_q0; ECEF_w0 ];

fprintf('[setup_ecef] datum r0 = [%.1f %.1f %.1f] m, |r0|=%.1f km\n', ...
        ECEF_r0(1),ECEF_r0(2),ECEF_r0(3), norm(ECEF_r0)/1000);
fprintf('[setup_ecef] q0 (b->e) = [%.6f %.6f %.6f %.6f]\n', ECEF_q0);
fprintf('[setup_ecef] |q0| = %.12f (should be 1)\n', norm(ECEF_q0));

%% --- local helper ------------------------------------------------------
function q = dcm2quat_sf(C)
% Convert a DCM (C = body->ref, v_ref = C*v_body) to a scalar-first
% unit quaternion q=[w x y z] with the convention v_ref = R(q)*v_body.
    tr = C(1,1)+C(2,2)+C(3,3);
    if tr > 0
        S = sqrt(tr+1)*2;
        w = 0.25*S;
        x = (C(3,2)-C(2,3))/S;
        y = (C(1,3)-C(3,1))/S;
        z = (C(2,1)-C(1,2))/S;
    elseif (C(1,1)>C(2,2)) && (C(1,1)>C(3,3))
        S = sqrt(1+C(1,1)-C(2,2)-C(3,3))*2;
        w = (C(3,2)-C(2,3))/S;
        x = 0.25*S;
        y = (C(1,2)+C(2,1))/S;
        z = (C(1,3)+C(3,1))/S;
    elseif C(2,2)>C(3,3)
        S = sqrt(1+C(2,2)-C(1,1)-C(3,3))*2;
        w = (C(1,3)-C(3,1))/S;
        x = (C(1,2)+C(2,1))/S;
        y = 0.25*S;
        z = (C(2,3)+C(3,2))/S;
    else
        S = sqrt(1+C(3,3)-C(1,1)-C(2,2))*2;
        w = (C(2,1)-C(1,2))/S;
        x = (C(1,3)+C(3,1))/S;
        y = (C(2,3)+C(3,2))/S;
        z = 0.25*S;
    end
    q = [w;x;y;z];
    q = q/norm(q);
end
