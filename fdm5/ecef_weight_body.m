function [Fw_body, W_mag] = ecef_weight_body(r_ecef, DCM_b2e, mass)
%#codegen
% ECEF_WEIGHT_BODY  Gravitational body-axis force & effective-weight scalar.
%
%   Uses the SAME point-mass + J2 gravity model as ecef_eom_fcn, so the
%   logged ExtForces_N and LoadFactors are consistent with the integrated
%   ECEF dynamics (no longer a flat-earth constant-g approximation).
%
%   r_ecef  : ECEF position [m]                       (3x1)
%   DCM_b2e : body->ECEF DCM, v_ecef = DCM_b2e*v_body (3x3)
%   mass    : [kg]
%
%   Fw_body : gravitational force in body axes [N]    (3x1)
%   W_mag   : |gravity force| = mass*|g_e| [N]        (scalar)
%             used as the load-factor reference so n is specific-force/g
%             against the same gravity the dynamics apply.
%
%   NOTE: this is the pure gravitation term (point-mass + J2), matching the
%   g_e term in ecef_eom_fcn.  Earth-rotation (centrifugal/Coriolis) effects
%   are handled separately in the EOM and are intentionally NOT folded in
%   here, so Fw_body is exactly the gravitational force used by the dynamics.

% --- WGS-84 gravity constants (identical to ecef_eom_fcn) --------------
mu  = 3.986004418e14;     % GM                    [m^3/s^2]
J2  = 1.082626683e-3;     % 2nd zonal harmonic    [-]
Rea = 6378137.0;          % equatorial radius     [m]

% --- gravitational acceleration (point-mass + J2) in ECEF --------------
x = r_ecef(1); y = r_ecef(2); z = r_ecef(3);
r = sqrt(x^2 + y^2 + z^2);
g_e = -mu/r^3*[x;y;z] + ...
      (-1.5*J2*mu*Rea^2/r^5)*[ x*(1-5*z^2/r^2); y*(1-5*z^2/r^2); z*(3-5*z^2/r^2) ];

% --- body-axis weight force & magnitude --------------------------------
Fw_body = mass * (DCM_b2e.' * g_e);
W_mag   = mass * sqrt(g_e(1)^2 + g_e(2)^2 + g_e(3)^2);
end
