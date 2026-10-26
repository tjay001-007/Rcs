function make_fcs_detailed()
%MAKE_FCS_DETAILED  Render the full ACFT11 FCS block schematic as a PNG,
%   matching the detailed ASCII layout (sequencer -> guidance -> 4 outer laws
%   -> envelope -> INDI inner pipeline -> actuators/RCS -> 6-DOF plant), with
%   the state/sensor feedback bus up the right side.

outdir = fileparts(mfilename('fullpath'));
fpath  = fullfile(outdir,'FCS_schematic_detailed.png');

P.navy  =[ 11  42  74]/255;  P.teal =[ 28 114 147]/255;  P.sea  =[  0 148 136]/255;
P.amber =[198 128  18]/255;  P.green=[ 46 125  50]/255;  P.purple=[120 96 170]/255;
P.slate =[ 84 110 122]/255;  P.red  =[176  42  42]/255;  P.ink  =[ 32  42  54]/255;
P.grey  =[100 112 122]/255;  P.gold =[180 120  20]/255;

fig = figure('Color','w','Units','pixels','Position',[40 20 1550 2020],'Visible','off');
ax  = axes('Parent',fig,'Position',[0 0 1 1]); hold(ax,'on');
set(ax,'XLim',[0 100],'YLim',[0 130]); axis(ax,'off');

% ---------- title ----------
text(ax,50,128,'ACFT11\_a\_121 Flight Control System — Block Schematic', ...
    'HorizontalAlignment','center','FontWeight','bold','FontSize',16,'Color',P.navy,'Interpreter','tex');
text(ax,50,125.3,'Cascaded (nested-loop) architecture — forward path flows DOWN, state feedback flows UP the right side', ...
    'HorizontalAlignment','center','FontSize',9.5,'FontAngle','italic','Color',P.grey);

% ================= SEQUENCER =================
seqX=6; seqW=72; seqY=117; seqH=6.0;
box(ax,seqX,seqY,seqW,seqH,'MISSION SEQUENCER  (Stateflow FSM)', ...
 {'17 phases: GROUNDROLL .. ROTATE .. CLIMB .. ACCEL .. ZOOM .. COAST .. GLIDE .. RTB .. APPROACH .. FLARE .. ROLLOUT', ...
  'outputs:   guidance MODE (GM\_*)    |    law\_select (FCS\_LAW\_*)    |    prop\_mode (PM\_*)'}, ...
 tint(P.navy,.86),P.navy,P.navy,11,8);

% ================= GUIDANCE =================
guX=6; guW=72; guY=106.5; guH=6.0;
box(ax,guX,guY,guW,guH,'GUIDANCE / TRAJECTORY', ...
 {'climb/cruise set-points | ACCEL pre-pitch | ZOOM gamma | COAST track | GlideCompute | L1 track | ILS loc + glide slope', ...
  'outputs:   gamma\_cmd  .  V\_cmd  .  h\_cmd  .  psi\_cmd'}, ...
 tint(P.teal,.86),P.teal,P.teal,11,8);

arrow(ax,42,seqY,42,guY+guH,P.ink,'-',1.7);
text(ax,43,(seqY+guY+guH)/2,'MODE, law\_select','HorizontalAlignment','left','FontSize',8,'Color',P.ink,'Interpreter','tex');

% ================= OUTER LAWS (4) =================
oY=92; oH=10.5; oW=16;
oX=[6 24.667 43.333 62];  oC=oX+oW/2;
box(ax,oX(1),oY,oW,oH,'TECS', {'energy -> throttle','+ pitch cmd','Kti/Ktp/Kei/Kep'}, tint(P.sea,.86),P.sea,P.sea,10,7.6);
box(ax,oX(2),oY,oW,oH,'ACAH', {'attitude -> rate cmd','Ktheta=2  Kphi=3','Kbeta=2'}, tint(P.sea,.86),P.sea,P.sea,10,7.6);
box(ax,oX(3),oY,oW,oH,'gamma-law', {'flight-path -> q\_cmd','Kgamma=1.5','(zoom/coast/glide)'}, tint(P.sea,.86),P.sea,P.sea,10,7.6);
box(ax,oX(4),oY,oW,oH,'L1 LATERAL', {'track -> bank -> p\_cmd','L1\_tc=18 s','bank <= 35 deg'}, tint(P.sea,.86),P.sea,P.sea,10,7.6);

arrow(ax,42,guY,42,oY+oH,P.ink,'-',1.7);
text(ax,43,(guY+oY+oH)/2,'gamma / V / h / psi cmd','HorizontalAlignment','left','FontSize',8,'Color',P.ink,'Interpreter','tex');

% ================= ENVELOPE =================
enX=24; enW=36; enY=82; enH=6.0; enC=enX+enW/2;
% merge the 4 outer laws down into the envelope
mY=89.2;
for k=2:4, line(ax,[oC(k) oC(k)],[oY mY],'Color',P.ink,'LineWidth',1.1); end
line(ax,[oC(1) oC(1)],[oY mY],'Color',P.ink,'LineWidth',1.1);
line(ax,[oC(1) oC(4)],[mY mY],'Color',P.ink,'LineWidth',1.1);
arrow(ax,enC,mY,enC,enY+enH,P.ink,'-',1.5);
text(ax,enC+1,(mY+enY+enH)/2,'(merged body-rate cmds)','HorizontalAlignment','left','FontSize',7.6,'Color',P.ink);
box(ax,enX,enY,enW,enH,'ENVELOPE PROTECTION', ...
 {'alpha limiter 12/16 deg  .  Nz limiter +3/-1 g', 'bank limiter 35/60 deg  .  rate & theta clamps'}, ...
 tint(P.amber,.84),P.amber,P.amber,10.5,8);

% ================= INDI =================
inX=14; inW=72; inY=58; inH=19.5; inC=inX+inW/2;
rectangle(ax,'Position',[inX inY inW inH],'Curvature',0.04,'FaceColor',tint(P.green,.90),'EdgeColor',P.green,'LineWidth',1.6);
text(ax,inC,inY+inH-2.0,'INDI INNER LOOP   (INDI\_Inner,  100 Hz)','HorizontalAlignment','center','FontWeight','bold','FontSize',11,'Color',P.green,'Interpreter','tex');
text(ax,inC,inY+inH-4.4,'d\_{cmd} = d\_0 + G^{-1} \cdot ( nu - omega\_dot\_{meas} )        nu = [ Kp(p\_c-p) ; Kq(q\_c-q) ; Kr(r\_c-r) ],   Kp=6  Kq=5  Kr=4', ...
    'HorizontalAlignment','center','FontSize',8.4,'Color',P.ink,'Interpreter','tex');
% 4 sub-boxes
sbY=61.0; sbH=6.6; sbW=14; sbX=[18 33 48 63]; sbC=sbX+sbW/2;
sub(ax,sbX(1),sbY,sbW,sbH,{'rate error','\times K(p,q,r)'},P.green);
sub(ax,sbX(2),sbY,sbW,sbH,{'G^{-1} scheduled','(alpha, Mach)','from aero tabs'},P.green);
sub(ax,sbX(3),sbY,sbW,sbH,{'q-bar fade','1000 .. 6000 Pa','(anti-windup)'},P.green);
sub(ax,sbX(4),sbY,sbW,sbH,{'per-step rate','clamps de/da/dr','(rate \cdot dt)'},P.green);
for k=1:3, arrow(ax,sbX(k)+sbW,sbY+sbH/2,sbX(k+1),sbY+sbH/2,P.green,'-',1.3); end
text(ax,inC,inY+1.4,'omega\_dot\_{meas}  <--  washout / derivative filter  (Nfilt = 20 rad/s)', ...
    'HorizontalAlignment','center','FontSize',8,'FontAngle','italic','Color',P.green,'Interpreter','tex');

arrow(ax,enC,enY,enC,inY+inH,P.ink,'-',1.6);
text(ax,enC+1,(enY+inY+inH)/2,'protected p/q/r cmd','HorizontalAlignment','left','FontSize',8,'Color',P.ink);

% ================= ACTUATORS + RCS =================
acY=46; acH=9.5;
acX=16; acW=30; acC=acX+acW/2;
rcX=52; rcW=30; rcC=rcX+rcW/2;
box(ax,acX,acY,acW,acH,'ACTUATORS', ...
 {'elevator / aileron / rudder','pos lim 25 / 20 / 25 deg','rate lim 80 / 100 / 80 deg/s'}, ...
 tint(P.slate,.86),P.slate,P.slate,10.5,8);
box(ax,rcX,rcY_(acY),rcW,acH,'RCS  (reaction control)', ...
 {'thin-air attitude law','+3 deg AoA,  wn = 0.35','budget 4000 N\cdotm / axis','q-bar faded OUT > 3 kPa'}, ...
 tint(P.purple,.86),P.purple,P.purple,10.5,8);

arrow(ax,acC,inY,acC,acY+acH,P.ink,'-',1.6);
text(ax,acC+1,(inY+acY+acH)/2,'d_e , d_a , d_r','HorizontalAlignment','left','FontSize',8,'Color',P.ink,'Interpreter','tex');

% ================= PLANT (dashed container) =================
plX=5; plW=80; plY=3; plH=41.5;
rectangle(ax,'Position',[plX plY plW plH],'Curvature',0.01,'FaceColor',[0.975 0.975 0.98],'EdgeColor',P.ink,'LineStyle',':','LineWidth',1.4);
text(ax,plX+1.5,plY+plH-1.6,'PLANT — 6-DOF ECEF  (physics)','HorizontalAlignment','left','FontWeight','bold','FontSize',10.5,'Color',P.ink);

% Propulsion / Aero / Gear
pgY=32; pgH=9.0;
prX=9; prW=22;  aeX=34; aeW=22;  geX=59; geW=21;
box(ax,prX,pgY,prW,pgH,'PROPULSION', ...
 {'Prop. Manager + SpoolLag (tau=2 s)','aerospike 250 kN fixed, 1 s decay'}, ...
 tint(P.gold,.86),P.gold,P.gold,9.5,7.8);
box(ax,aeX,pgY,aeW,pgH,'AERODYNAMICS  (Aerovegamidfid)', ...
 {'CL CD CY / Cl Cm Cn tables','\times q-bar \cdot S (, b, c)'}, ...
 tint(P.teal,.86),P.teal,P.teal,9.5,7.8);
box(ax,geX,pgY,geW,pgH,'LANDING GEAR', ...
 {'normal + friction','(while weight-on-wheels)'}, ...
 tint(P.slate,.86),P.slate,P.slate,9.5,7.8);

% ExtForcesMoments + GravityBody
efY=21; efH=7.0;
efX=9;  efW=34;  grX=47; grW=33;
box(ax,efX,efY,efW,efH,'ExtForcesMoments', ...
 {'sum of NON-gravity forces & moments','-> Fb , Mb'}, ...
 tint(P.ink,.90),P.ink,P.ink,9.5,7.8);
box(ax,grX,efY,grW,efH,'GravityBody  (analytic g\_e)', ...
 {'WGS-84 + J2 gravity','-> logged weight / load factor'}, ...
 tint(P.red,.88),P.red,P.red,9.5,7.8);

% ECEF_Core EOM
eoX=9; eoW=71; eoY=6; eoH=11;
box(ax,eoX,eoY,eoW,eoH,'ECEF\_Core  —  EQUATIONS OF MOTION  (13-state)', ...
 {'translational:  v\_dot = C_{b2e} Fb/m + g\_e(J2) - 2 we\timesv - we\times(we\timesr)', ...
  'rotational:  omega\_dot = I^{-1}( Mb - omega\timesI omega )    attitude: quaternion propagation (+ unit-norm fb)', ...
  'outputs via ecef\_out\_fcn:  LLA, NED, Euler, gamma, track, climb rate'}, ...
 tint(P.purple,.88),P.purple,P.purple,10,8);

% plant internal arrows
arrow(ax,prX+prW/2,pgY,prX+prW/2+3,efY+efH,P.ink,'-',1.2);
arrow(ax,aeX+aeW/2,pgY,aeX+aeW/2-3,efY+efH,P.ink,'-',1.2);
arrow(ax,geX+geW/2,pgY,efX+efW-3,efY+efH,P.ink,'-',1.2);
arrow(ax,efX+efW/2,efY,efX+efW/2,eoY+eoH,P.ink,'-',1.4);
text(ax,efX+efW/2+1,(efY+eoY+eoH)/2,'Fb , Mb','HorizontalAlignment','left','FontSize',8,'Color',P.ink);
arrow(ax,grX+grW/2,efY,eoX+eoW-6,eoY+eoH,P.red,'-',1.1);

% actuators -> aero ; (into plant)
arrow(ax,acC,acY,aeX+aeW/2,pgY+pgH,P.ink,'-',1.4);
text(ax,acC+1,(acY+pgY+pgH)/2,'surface deflection','HorizontalAlignment','left','FontSize',7.8,'Color',P.ink);

% ================= LEFT RAIL: prop_mode + throttle -> propulsion =================
railX=2.4;
drawPath(ax,[seqX seqX-1.0 railX railX prX],[seqY+1 seqY+1 seqY+1 pgY+pgH*0.6 pgY+pgH*0.6],P.gold,'-',1.3);
line(ax,[oC(1) railX],[oY+2 oY+2],'Color',P.gold,'LineWidth',1.1);      % TECS throttle joins rail
line(ax,[railX railX],[oY+2 pgY+pgH*0.6],'Color',P.gold,'LineWidth',1.1);
text(ax,railX+0.4,seqY-1.5,'prop\_mode','Rotation',90,'HorizontalAlignment','left','FontSize',7.6,'Color',P.gold,'Interpreter','tex');
text(ax,railX+0.4,oY-10,'throttle','Rotation',90,'HorizontalAlignment','left','FontSize',7.6,'Color',P.gold,'Interpreter','tex');

% ================= RCS torque -> ExtForcesMoments (routed around the gear box) =
rcLegX=81.5; rcTravY=30.3; efTopC=efX+efW/2;
drawPath(ax,[rcC rcC rcLegX rcLegX efTopC efTopC], ...
            [acY acY-1.5 acY-1.5 rcTravY rcTravY efY+efH], P.purple,'-',1.2);
text(ax,rcLegX+0.5,(acY-1.5+rcTravY)/2,'RCS torque','Rotation',90, ...
     'HorizontalAlignment','center','VerticalAlignment','middle','FontSize',7.6,'Color',P.purple);

% ================= FEEDBACK BUS (right, dashed, UP) =================
busX=90;
yEOM=eoY+eoH/2; ySEQ=seqY+seqH/2;
arrow(ax,eoX+eoW,yEOM,busX,yEOM,P.red,'-',1.5);          % EOM out to bus
arrow(ax,busX,yEOM,busX,ySEQ,P.red,'--',1.8);            % up the bus
arrow(ax,busX,ySEQ,seqX+seqW,ySEQ,P.red,'-',1.5);        % into sequencer
taps=[guY+guH/2, oY+oH/2, enY+enH/2, inY+inH*0.55, rcY_(acY)+acH/2];
tapR=[guX+guW,   oX(4)+oW, enX+enW,  inX+inW,      rcX+rcW];
for k=1:numel(taps)
    arrow(ax,busX,taps(k),tapR(k),taps(k),P.red,'--',1.1);
end
text(ax,busX+1.0,(yEOM+ySEQ)/2, ...
 sprintf('STATE / SENSOR FEEDBACK\nV, h, Mach, alpha, beta, gamma,\nphi/theta/psi, p/q/r, q-bar, WoW'), ...
 'Rotation',90,'HorizontalAlignment','center','VerticalAlignment','top','FontWeight','bold','FontSize',8,'Color',P.red);

% ---------- footer legend ----------
text(ax,5,1.2,'Solid = forward command path      - - -  = state/sensor feedback      gold = propulsion cmds      q-bar schedules surface vs. RCS authority', ...
    'HorizontalAlignment','left','FontSize',8,'FontAngle','italic','Color',P.grey);

exportgraphics(fig,fpath,'Resolution',200);
close(fig);
fprintf('Wrote %s\n',fpath);
end

% ===================================================================
function y=rcY_(acY), y=acY; end

function box(ax,x,y,w,h,ttl,lines,face,edge,tcol,tfs,bfs)
rectangle(ax,'Position',[x y w h],'Curvature',0.10,'FaceColor',face,'EdgeColor',edge,'LineWidth',1.5);
if isempty(lines)
    text(ax,x+w/2,y+h/2,ttl,'HorizontalAlignment','center','VerticalAlignment','middle', ...
        'FontWeight','bold','FontSize',tfs,'Color',tcol,'Interpreter','tex');
    return;
end
if ischar(lines), lines={lines}; end
nL=numel(lines);
text(ax,x+w/2,y+h-1.15,ttl,'HorizontalAlignment','center','VerticalAlignment','middle', ...
    'FontWeight','bold','FontSize',tfs,'Color',tcol,'Interpreter','tex');
body=strjoin(lines,sprintf('\n'));
text(ax,x+w/2,y+(h-2.4)*0.45,body,'HorizontalAlignment','center','VerticalAlignment','middle', ...
    'FontSize',bfs,'Color',[0.24 0.3 0.37],'Interpreter','tex');
end

function sub(ax,x,y,w,h,lines,edge)
rectangle(ax,'Position',[x y w h],'Curvature',0.12,'FaceColor','w','EdgeColor',edge,'LineWidth',1.1);
body=strjoin(lines,sprintf('\n'));
text(ax,x+w/2,y+h/2,body,'HorizontalAlignment','center','VerticalAlignment','middle', ...
    'FontSize',7.6,'Color',[0.2 0.28 0.34],'Interpreter','tex');
end

function arrow(ax,x1,y1,x2,y2,color,ls,lw)
line(ax,[x1 x2],[y1 y2],'Color',color,'LineStyle',ls,'LineWidth',lw);
d=[x2-x1,y2-y1]; Ln=hypot(d(1),d(2)); if Ln<1e-9, return; end
u=d/Ln; nrm=[-u(2) u(1)]; hs=1.4; tip=[x2 y2];
b1=tip-hs*u+0.6*hs*nrm; b2=tip-hs*u-0.6*hs*nrm;
patch(ax,'XData',[tip(1) b1(1) b2(1)],'YData',[tip(2) b1(2) b2(2)],'FaceColor',color,'EdgeColor',color);
end

function drawPath(ax,X,Y,color,ls,lw)
line(ax,X,Y,'Color',color,'LineStyle',ls,'LineWidth',lw);
x1=X(end-1); y1=Y(end-1); x2=X(end); y2=Y(end);
d=[x2-x1,y2-y1]; Ln=hypot(d(1),d(2)); if Ln<1e-9, return; end
u=d/Ln; nrm=[-u(2) u(1)]; hs=1.4; tip=[x2 y2];
b1=tip-hs*u+0.6*hs*nrm; b2=tip-hs*u-0.6*hs*nrm;
patch(ax,'XData',[tip(1) b1(1) b2(1)],'YData',[tip(2) b1(2) b2(2)],'FaceColor',color,'EdgeColor',color);
end

function c=tint(c0,f), c=c0+(1-c0)*f; end
