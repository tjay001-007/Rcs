function dstate = ecef_eom_fcn(state, Fb, Mb, mass, Inertia)
%#codegen
% ECEF_EOM_FCN  Rotating-Earth 6-DOF state derivative (WGS-84 + J2 gravity).
%
%   state = [dr(3); v_e(3); q_b2e(4); w_b(3)]   (13x1)
%     dr    : ECEF position offset from datum origin r0        [m]
%     v_e   : velocity w.r.t Earth, expressed in ECEF          [m/s]
%     q_b2e : unit quaternion body->ECEF, scalar-first [w x y z]
%     w_b   : body angular rate w.r.t inertial, in body        [rad/s]
%   Fb,Mb  : non-gravity body forces [N] and moments [N*m]
%   mass   : [kg]    Inertia : 3x3 [kg*m^2]
%
%   Datum matches the model's "Flat Earth to LLA" block
%   (lat0 = 36.973889 deg, lon0 = -25.171111 deg, h0 = 2.24 m).

% --- Earth / WGS-84 constants -----------------------------------------
we  = 7.292115e-5;        % Earth rotation rate   [rad/s]
mu  = 3.986004418e14;     % GM                    [m^3/s^2]
J2  = 1.082626683e-3;     % 2nd zonal harmonic
Rea = 6378137.0;          % equatorial radius     [m]
f   = 1/298.257223563;    e2 = f*(2-f);

% --- datum origin in ECEF ---------------------------------------------
[lat0, lon0, h0] = acft_datum();
N0 = Rea/sqrt(1-e2*sin(lat0)^2);
r0 = [ (N0+h0)*cos(lat0)*cos(lon0);
       (N0+h0)*cos(lat0)*sin(lon0);
       (N0*(1-e2)+h0)*sin(lat0) ];

% --- unpack & normalize quaternion ------------------------------------
dr=state(1:3); v_e=state(4:6); q=state(7:10); w_b=state(11:13);
qn=q/sqrt(q(1)^2+q(2)^2+q(3)^2+q(4)^2+eps);
qw=qn(1); qx=qn(2); qy=qn(3); qz=qn(4);

C_b2e=[qw^2+qx^2-qy^2-qz^2, 2*(qx*qy-qw*qz),     2*(qx*qz+qw*qy);
       2*(qx*qy+qw*qz),     qw^2-qx^2+qy^2-qz^2, 2*(qy*qz-qw*qx);
       2*(qx*qz-qw*qy),     2*(qy*qz+qw*qx),     qw^2-qx^2-qy^2+qz^2];

% --- gravitational acceleration (point-mass + J2) ---------------------
r_e=r0+dr; r=sqrt(r_e(1)^2+r_e(2)^2+r_e(3)^2);
x=r_e(1); y=r_e(2); z=r_e(3);
g_e = -mu/r^3*[x;y;z] + ...
      (-1.5*J2*mu*Rea^2/r^5)*[ x*(1-5*z^2/r^2); y*(1-5*z^2/r^2); z*(3-5*z^2/r^2) ];

% --- translational dynamics (ECEF) ------------------------------------
we_e=[0;0;we];
F_e=C_b2e*(Fb/mass);
v_e_dot = F_e + g_e - 2*cross(we_e,v_e) - cross(we_e,cross(we_e,r_e));
dr_dot  = v_e;

% --- rotational dynamics ----------------------------------------------
Iw=Inertia*w_b;
w_b_dot=Inertia\(Mb-cross(w_b,Iw));

% --- attitude kinematics (body rate relative to ECEF) -----------------
w_ie_b=C_b2e.'*we_e;
w_eb_b=w_b-w_ie_b;
a=w_eb_b(1); bb=w_eb_b(2); c=w_eb_b(3);
qdot=0.5*[ -(qx*a+qy*bb+qz*c);
            qw*a + qy*c - qz*bb;
            qw*bb + qz*a - qx*c;
            qw*c + qx*bb - qy*a ];
qdot = qdot + 1.0*(1-(qw^2+qx^2+qy^2+qz^2))*qn;   % unit-norm feedback

dstate=[dr_dot; v_e_dot; qdot; w_b_dot];
end
