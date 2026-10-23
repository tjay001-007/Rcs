function x0 = build_ecef_x0(h_init, u_body, theta0)
% BUILD_ECEF_X0  13-state ECEF initial condition for ACFT11_a_121.
%   x0 = build_ecef_x0(h_init, u_body, theta0)
%     h_init : geodetic altitude at start                [m]
%     u_body : body-forward speed rel Earth (u; v=w=0)   [m/s]
%     theta0 : pitch angle (phi=0, psi=rwy_hdg from datum) [rad]
%
%   Returns the 13-state vector consumed by ECEF_Core:
%     [dr(3); v_e(3); q_b2e(4); w_b(3)]
%   referenced to the SAME fixed datum origin (h=2.24 m) that
%   ecef_eom_fcn / ecef_out_fcn hard-code, so dr is the offset from it.

we=7.292115e-5; Rea=6378137.0; f=1/298.257223563; e2=f*(2-f);
[lat0,lon0,h_dat,rwy_hdg]=acft_datum();
phi0=0; psi0=rwy_hdg*pi/180;   % nose aligned with runway

% --- body->NED and NED->ECEF DCMs at the datum ---
cph=cos(phi0);sph=sin(phi0);cth=cos(theta0);sth=sin(theta0);cps=cos(psi0);sps=sin(psi0);
C_ned2b=[ cth*cps, cth*sps, -sth;
          sph*sth*cps-cph*sps, sph*sth*sps+cph*cps, sph*cth;
          cph*sth*cps+sph*sps, cph*sth*sps-sph*cps, cph*cth];
C_b2ned=C_ned2b.';
slat=sin(lat0);clat=cos(lat0);slon=sin(lon0);clon=cos(lon0);
C_e2ned=[-slat*clon,-slat*slon,clat; -slon,clon,0; -clat*clon,-clat*slon,-slat];
C_b2e=C_e2ned.'*C_b2ned;

% --- ECEF positions (datum origin vs initial altitude) ---
N0=Rea/sqrt(1-e2*sin(lat0)^2);
ecef=@(h)[(N0+h)*cos(lat0)*cos(lon0); (N0+h)*cos(lat0)*sin(lon0); (N0*(1-e2)+h)*sin(lat0)];
dr0 = ecef(h_init) - ecef(h_dat);

% --- velocity, quaternion, body rate ---
v_e0 = C_b2e*[u_body;0;0];
q0   = i_dcm2quat(C_b2e);
w0   = C_b2e.'*[0;0;we];          % rigid-with-Earth at t=0

x0 = [dr0; v_e0; q0; w0];
end

function q = i_dcm2quat(C)
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
q=[w;x;y;z]; q=q/norm(q);
end
