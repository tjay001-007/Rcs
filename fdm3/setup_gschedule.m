%% setup_gschedule.m
%  Build the INDI control-effectiveness (G-matrix) SCHEDULE from the aircraft's
%  own aero tables, so the inner-loop inversion gain matches the plant across
%  the whole (alpha, Mach) envelope instead of a single anchor point.
%
%  The aero model (Aerovegamidfid) already carries the per-radian control
%  effectiveness as 2-D lookups:
%     Cm_elev (blk 4:302)  = d(Cm)/d(elevator)  (alpha_deg, Mach)
%     Cl_AilT (blk 4:310)  = d(Cl)/d(aileron)   (alpha_deg, Mach)
%     Cn_rudder(blk 4:362) = d(Cn)/d(rudder)    (alpha_deg, Mach)
%  This script extracts those tables (auto-syncing with the aero) and folds in
%  the geometry/inertia so the INDI lookups directly output the G-coefficient
%     Gx(alpha,Mach) = deriv(alpha,Mach) * (S * ref / I)
%  and the INDI only has to multiply by live qbar:  G = Gx * qbar.
%
%  Produces base-workspace variables consumed by the scheduled Lookup blocks
%  inside INDI_Inner:
%     GS_alpha_cm/_cl/_cn   alpha breakpoints  [deg]
%     GS_mach_cm/_cl/_cn    Mach  breakpoints  [-]
%     GS_Gq_tab/_Gp_tab/_Gr_tab   G-coefficient tables (deriv * geom/I)
%
%  Run AFTER fcs_params.m (needs FCS_S, FCS_c, FCS_b, FCS_Ixx/Iyy/Izz).
% -----------------------------------------------------------------------
mdl = 'ACFT11_a_121';
if ~bdIsLoaded(mdl), load_system(mdl); end

% geometry / inertia (pull from base; fall back to nominal if absent)
def = struct('FCS_S',85.25,'FCS_c',15.55,'FCS_b',9.05, ...
             'FCS_Ixx',74848,'FCS_Iyy',288344,'FCS_Izz',390125);
fn = fieldnames(def);
for i=1:numel(fn)
    if evalin('base', sprintf('exist(''%s'',''var'')', fn{i}))
        geom.(fn{i}) = evalin('base', fn{i});
    else
        geom.(fn{i}) = def.(fn{i});
    end
end
S=geom.FCS_S; c=geom.FCS_c; b=geom.FCS_b; Ixx=geom.FCS_Ixx; Iyy=geom.FCS_Iyy; Izz=geom.FCS_Izz;

% --- extract the three effectiveness tables straight from the aero blocks ----
rd = @(sid,prm) eval(get_param(Simulink.ID.getFullName([mdl ':' sid]),prm));
Cm_de = rd('4:302','Table');  a_cm = rd('4:302','BreakpointsForDimension1');  m_cm = rd('4:302','BreakpointsForDimension2');
Cl_da = rd('4:310','Table');  a_cl = rd('4:310','BreakpointsForDimension1');  m_cl = rd('4:310','BreakpointsForDimension2');
Cn_dr = rd('4:362','Table');  a_cn = rd('4:362','BreakpointsForDimension1');  m_cn = rd('4:362','BreakpointsForDimension2');

% --- fold geometry/inertia -> G-coefficient tables (angular-accel per rad per Pa)
GS_Gq_tab = Cm_de * (S*c/Iyy);     % pitch
GS_Gp_tab = Cl_da * (S*b/Ixx);     % roll
GS_Gr_tab = Cn_dr * (S*b/Izz);     % yaw

% --- publish to base workspace for the INDI lookups -------------------------
vars = {'GS_alpha_cm',a_cm(:)','GS_mach_cm',m_cm(:)','GS_Gq_tab',GS_Gq_tab, ...
        'GS_alpha_cl',a_cl(:)','GS_mach_cl',m_cl(:)','GS_Gp_tab',GS_Gp_tab, ...
        'GS_alpha_cn',a_cn(:)','GS_mach_cn',m_cn(:)','GS_Gr_tab',GS_Gr_tab};
for i=1:2:numel(vars), assignin('base',vars{i},vars{i+1}); end

fprintf('[gschedule] INDI G-matrix scheduled from aero tables:\n');
fprintf('   pitch Gq-coef range [%.3g .. %.3g]  (anchor %.3g)\n', min(GS_Gq_tab(:)),max(GS_Gq_tab(:)), -0.144*S*c/Iyy);
fprintf('   roll  Gp-coef range [%.3g .. %.3g]  (anchor %.3g)\n', min(GS_Gp_tab(:)),max(GS_Gp_tab(:)), -0.126*S*b/Ixx);
fprintf('   yaw   Gr-coef range [%.3g .. %.3g]  (anchor %.3g)\n', min(GS_Gr_tab(:)),max(GS_Gr_tab(:)), -0.059*S*b/Izz);
