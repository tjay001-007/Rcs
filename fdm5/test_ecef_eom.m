%% test_ecef_eom.m -- numerical sanity check of the ECEF EOM derivative
run('setup_ecef.m');

Inertia = diag([3e5 5e5 7e5]);     % arbitrary plausible tensor
mass    = 24650;

% ---- Case 1: free fall (no external forces) --------------------------
x = ecef_x0;
dx = ECEF_EOM_local(x, [0;0;0], [0;0;0], mass, Inertia, ...
                    ECEF_r0, ECEF_we, ECEF_mu, ECEF_J2, ECEF_Rea);
a_free = dx(4:6);                   % ECEF accel of a free body
fprintf('Free-fall |accel| at surface = %.4f m/s^2 (expect ~9.79-9.83)\n', norm(a_free));

% direction should point roughly toward Earth centre (-r0 hat)
rhat = ECEF_r0/norm(ECEF_r0);
fprintf('  accel.rhat = %.4f (expect ~ -9.8, i.e. downward)\n', dot(a_free,rhat));

% ---- Case 2: static equilibrium -------------------------------------
% Gear reaction must supply +g (specific force) in body axes to hold still.
% Required body specific force f_b = -C_b2e' * (g_e - centripetal)
qn=x(7:10); qn=qn/norm(qn); qw=qn(1);qx=qn(2);qy=qn(3);qz=qn(4);
C_b2e = [ qw^2+qx^2-qy^2-qz^2, 2*(qx*qy-qw*qz),     2*(qx*qz+qw*qy);
          2*(qx*qy+qw*qz),     qw^2-qx^2+qy^2-qz^2, 2*(qy*qz-qw*qx);
          2*(qx*qz-qw*qy),     2*(qy*qz+qw*qx),     qw^2-qx^2-qy^2+qz^2 ];
we_e=[0;0;ECEF_we];
r_e = ECEF_r0;
r=norm(r_e); xx=r_e(1);yy=r_e(2);zz=r_e(3);
g_e = -ECEF_mu/r^3*[xx;yy;zz] + ...
      (-1.5*ECEF_J2*ECEF_mu*ECEF_Rea^2/r^5)*[xx*(1-5*zz^2/r^2);yy*(1-5*zz^2/r^2);zz*(3-5*zz^2/r^2)];
cen = cross(we_e,cross(we_e,r_e));
Fb_hold = -mass*(C_b2e.'*(g_e - cen));
dx2 = ECEF_EOM_local(x, Fb_hold, [0;0;0], mass, Inertia, ...
                     ECEF_r0, ECEF_we, ECEF_mu, ECEF_J2, ECEF_Rea);
fprintf('Static hold: |v_e_dot| = %.3e m/s^2 (expect ~0)\n', norm(dx2(4:6)));
fprintf('Static hold: required specific force |f_b| = %.4f m/s^2 (effective g)\n', norm(Fb_hold)/mass);
fprintf('  f_b body = [%.3f %.3f %.3f] (expect dominant -Z up-reaction)\n', Fb_hold/mass);

% ---- Case 3: quaternion stays unit under pure spin -------------------
x3 = ecef_x0; x3(11:13) = [0.1;0;0];   % 0.1 rad/s roll
dx3 = ECEF_EOM_local(x3, [0;0;0],[0;0;0], mass, Inertia, ...
                     ECEF_r0, ECEF_we, ECEF_mu, ECEF_J2, ECEF_Rea);
qdot = dx3(7:10); q=x3(7:10);
fprintf('Spin: d/dt|q|^2 = 2*q.qdot = %.3e (expect ~0)\n', 2*dot(q,qdot));

%% ===================== EOM under test (local copy) ====================
function dstate = ECEF_EOM_local(state, Fb, Mb, mass, Inertia, r0, we, mu, J2, Rea)
dr=state(1:3); v_e=state(4:6); q=state(7:10); w_b=state(11:13);
qn=q/sqrt(q(1)^2+q(2)^2+q(3)^2+q(4)^2+eps);
qw=qn(1);qx=qn(2);qy=qn(3);qz=qn(4);
C_b2e=[qw^2+qx^2-qy^2-qz^2, 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy);
       2*(qx*qy+qw*qz), qw^2-qx^2+qy^2-qz^2, 2*(qy*qz-qw*qx);
       2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), qw^2-qx^2-qy^2+qz^2];
r_e=r0+dr; r=sqrt(r_e(1)^2+r_e(2)^2+r_e(3)^2);
x=r_e(1);y=r_e(2);z=r_e(3);
g_e=-mu/r^3*[x;y;z]+(-1.5*J2*mu*Rea^2/r^5)*[x*(1-5*z^2/r^2);y*(1-5*z^2/r^2);z*(3-5*z^2/r^2)];
we_e=[0;0;we];
F_e=C_b2e*(Fb/mass);
v_e_dot=F_e+g_e-2*cross(we_e,v_e)-cross(we_e,cross(we_e,r_e));
dr_dot=v_e;
Iw=Inertia*w_b; w_b_dot=Inertia\(Mb-cross(w_b,Iw));
w_ie_b=C_b2e.'*we_e; w_eb_b=w_b-w_ie_b;
a=w_eb_b(1);b=w_eb_b(2);c=w_eb_b(3);
qdot=0.5*[-(qx*a+qy*b+qz*c); qw*a+qy*c-qz*b; qw*b+qz*a-qx*c; qw*c+qx*b-qy*a];
qdot=qdot+1.0*(1-(qw^2+qx^2+qy^2+qz^2))*qn;
dstate=[dr_dot;v_e_dot;qdot;w_b_dot];
end
