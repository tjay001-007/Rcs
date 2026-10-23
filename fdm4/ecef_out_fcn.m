function [r_ecef, v_ecef, q_b2e, DCM_b2e, wbody, ...
          VB_mps, Euler_rad, posNED, GroundSpeed, Gamma_deg, Track_deg, Vz, LLA, ...
          LBE, VI_ned] ...
          = ecef_out_fcn(state)
%#codegen
% ECEF_OUT_FCN  Derive all downstream signals from the ECEF state vector.
%
%   Raw ECEF outputs (the rotating-ECEF interface):
%     r_ecef   ECEF position [m]        v_ecef  ECEF velocity [m/s]
%     q_b2e    body->ECEF quaternion     DCM_b2e body->ECEF DCM (3x3)
%     wbody    body rates [rad/s]
%   Local/body outputs (so existing consumers keep working):
%     VB_mps   body velocity rel Earth [m/s]
%     Euler_rad [phi;theta;psi] rel local NED [rad]
%     posNED   NED position rel datum [m]
%     GroundSpeed [m/s]  Gamma_deg [deg]  Track_deg [deg]  Vz (climb +) [m/s]
%     LLA      [lat;lon;alt] (deg,deg,m)

we=7.292115e-5; Rea=6378137.0; f=1/298.257223563; e2=f*(2-f); b_e=Rea*(1-f); %#ok<NASGU>
[lat0,lon0,h0]=acft_datum();
N0=Rea/sqrt(1-e2*sin(lat0)^2);
r0=[(N0+h0)*cos(lat0)*cos(lon0);(N0+h0)*cos(lat0)*sin(lon0);(N0*(1-e2)+h0)*sin(lat0)];

dr=state(1:3); v_e=state(4:6); q=state(7:10); w_b=state(11:13);
qn=q/sqrt(q(1)^2+q(2)^2+q(3)^2+q(4)^2+eps);
qw=qn(1);qx=qn(2);qy=qn(3);qz=qn(4);
C_b2e=[qw^2+qx^2-qy^2-qz^2,2*(qx*qy-qw*qz),2*(qx*qz+qw*qy);
       2*(qx*qy+qw*qz),qw^2-qx^2+qy^2-qz^2,2*(qy*qz-qw*qx);
       2*(qx*qz-qw*qy),2*(qy*qz+qw*qx),qw^2-qx^2-qy^2+qz^2];
r_e=r0+dr;

% ---- raw ECEF ----
r_ecef=r_e; v_ecef=v_e; q_b2e=qn; DCM_b2e=C_b2e; wbody=w_b;

% ---- geodetic lat/lon/alt (Bowring) ----
x=r_e(1);y=r_e(2);z=r_e(3);
lon=atan2(y,x);
p=sqrt(x^2+y^2);
ep2=(Rea^2-b_e^2)/b_e^2;
th=atan2(z*Rea,p*b_e);
lat=atan2(z+ep2*b_e*sin(th)^3, p-e2*Rea*cos(th)^3);
Nlat=Rea/sqrt(1-e2*sin(lat)^2);
alt=p/cos(lat)-Nlat;
LLA=[lat*180/pi; lon*180/pi; alt];

% ---- NED frames ----
slat=sin(lat);clat=cos(lat);slon=sin(lon);clon=cos(lon);
C_e2ned=[-slat*clon,-slat*slon,clat; -slon,clon,0; -clat*clon,-clat*slon,-slat];
slat0=sin(lat0);clat0=cos(lat0);slon0=sin(lon0);clon0=cos(lon0);
C_e2ned0=[-slat0*clon0,-slat0*slon0,clat0; -slon0,clon0,0; -clat0*clon0,-clat0*slon0,-slat0];

% ---- position NED: horizontal datum-tangent, vertical = GEODETIC altitude ----
%  The vertical uses true geodetic altitude (not the datum tangent-plane),
%  so h = -posNED(3) stays correct hundreds of km from the runway -- the
%  tangent-plane altitude drops by the Earth-curvature term d^2/(2R) far out
%  and would otherwise break altitude hold and the mission phase logic.
r0_sl  = [ N0*cos(lat0)*cos(lon0); N0*cos(lat0)*sin(lon0); (N0*(1-e2))*sin(lat0) ];
pNE    = C_e2ned0*(r_e - r0_sl);
posNED = [pNE(1); pNE(2); -alt];

% ---- body velocity rel Earth ----
VB_mps = C_b2e.'*v_e;

% ---- Euler angles rel local NED ----
C_b2ned = C_e2ned*C_b2e;
sth = -max(-1,min(1,C_b2ned(3,1)));
theta = asin(sth);
phi = atan2(C_b2ned(3,2),C_b2ned(3,3));
psi = atan2(C_b2ned(2,1),C_b2ned(1,1));
Euler_rad=[phi;theta;psi];

% ---- trajectory from local NED velocity ----
vned=C_e2ned*v_e;
GroundSpeed=sqrt(vned(1)^2+vned(2)^2);
Gamma_deg=atan2(-vned(3),GroundSpeed)*180/pi;
Track_deg=atan2(vned(2),vned(1))*180/pi;
Vz=-vned(3);

% ---- extra bus signals ----
LBE    = C_b2ned;   % body->NED DCM (matches old EqM "LBE")
VI_ned = vned;      % inertial (NED) velocity vector
end
