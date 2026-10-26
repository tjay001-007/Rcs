function make_fcs_diagrams()
%MAKE_FCS_DIAGRAMS  Render professional PNG schematics of the ACFT11 FCS.
%   Produces two images in the project folder:
%     FCS_controller_structure.png  - cascaded controller block diagram
%     FCS_decision_flow.png         - mission-phase decision flowchart
%   No model dependency; pure drawing + exportgraphics.

outdir = fileparts(mfilename('fullpath'));

% ---- palette (RGB 0..1) ----
P.navy    = [ 11  42  74]/255;
P.teal    = [ 28 114 147]/255;
P.sea     = [  0 148 136]/255;
P.amber   = [214 140  20]/255;
P.purple  = [120  96 170]/255;
P.red     = [176  42  42]/255;
P.slate   = [ 84 110 122]/255;
P.green   = [ 46 125  50]/255;
P.ink     = [ 32  42  54]/255;
P.grey    = [100 112 122]/255;

make_structure(fullfile(outdir,'FCS_controller_structure.png'), P);
make_flow(fullfile(outdir,'FCS_decision_flow.png'), P);
fprintf('Done. Wrote:\n  %s\n  %s\n', ...
    fullfile(outdir,'FCS_controller_structure.png'), ...
    fullfile(outdir,'FCS_decision_flow.png'));
end

% =====================================================================
function make_structure(fpath, P)
fig = figure('Color','w','Units','pixels','Position',[60 60 1500 1050],'Visible','off');
ax  = axes('Parent',fig,'Position',[0 0 1 1]); hold(ax,'on');
set(ax,'XLim',[0 100],'YLim',[0 100]); axis(ax,'off');

% title
text(ax,50,97.5,'ACFT11 Flight Control System — Controller Structure', ...
    'HorizontalAlignment','center','FontWeight','bold','FontSize',17,'Color',P.navy);
text(ax,50,94.2,'Cascaded (nested-loop) architecture:  references flow DOWN,  state feedback flows UP', ...
    'HorizontalAlignment','center','FontSize',10.5,'FontAngle','italic','Color',P.grey);

% layer boxes (top -> down)
L = { ...
 'MISSION SEQUENCER',       'Stateflow FSM: 17 mission phases -> MODE, law\_select, prop\_mode', P.navy;
 'GUIDANCE / TRAJECTORY',   'Targets:  gamma\_cmd  .  V\_cmd  .  h\_cmd  .  psi\_cmd', P.teal;
 'OUTER CONTROL LAWS',      'TECS (energy)  .  ACAH (attitude)  .  gamma-law  .  L1 lateral', P.sea;
 'ENVELOPE PROTECTION',     'alpha / Nz / bank limiters  .  rate & theta clamps', P.amber;
 'INDI INNER LOOP  (100 Hz)','d = d\_0 + G^{-1}(nu - omega\_dot);  G scheduled x q-bar; rate clamps', P.green;
 'ACTUATORS  +  RCS',       'elevator / aileron / rudder (pos+rate lim)   |   RCS thrusters (vacuum)', P.slate;
 'PLANT — 6-DOF ECEF',      'Aero + Propulsion + Gear -> forces/moments -> Equations of Motion', P.purple };

n   = size(L,1);
bx  = 8;  bw = 62;        % box x and width
yTop= 90; yBot= 7; bh = 7.6;
gap = (yTop - yBot - n*bh)/(n-1);
yc  = zeros(n,1);
for i=1:n
    y = yTop - bh - (i-1)*(bh+gap);
    yc(i) = y + bh/2;
    drawBox(ax, bx, y, bw, bh, L{i,1}, L{i,2}, tint(L{i,3},0.86), L{i,3}, L{i,3});
end

% forward arrows + labels between layers
flab = {'MODE, law\_select','gamma / V / h / psi cmd','p / q / r rate cmd', ...
        'protected p/q/r cmd','d_e , d_a , d_r','surface + thruster forces'};
xc = bx + bw/2;
for i=1:n-1
    yto = yc(i)-bh/2;   yfr = yc(i+1)+bh/2;
    drawArrow(ax, xc, yto, xc, yfr, P.ink, '-', 1.8);
    text(ax, xc+1.2, (yto+yfr)/2, flab{i}, 'HorizontalAlignment','left','FontSize',8,'Color',P.ink);
end

% feedback bus (right side, dashed, pointing UP into sequencer)
xb = 82;
yPlant = yc(n); ySeq = yc(1);
% stub from plant out to the bus
drawArrow(ax, bx+bw, yPlant, xb, yPlant, P.red, '-', 1.6);
% vertical bus upward
drawArrow(ax, xb, yPlant, xb, ySeq, P.red, '--', 1.8);
% tap into sequencer
drawArrow(ax, xb, ySeq, bx+bw, ySeq, P.red, '-', 1.6);
% small tap arrows into each middle layer
for i=2:n-1
    line(ax,[xb xb],[yc(i) yc(i)],'Color',P.red);  %#ok<*NASGU>
    drawArrow(ax, xb, yc(i), bx+bw, yc(i), P.red, '--', 1.1);
end
text(ax, xb+1.0, (yPlant+ySeq)/2, ...
    sprintf('STATE / SENSOR FEEDBACK\nV, h, Mach, alpha, beta, gamma,\nphi/theta/psi, p/q/r, q-bar, WoW'), ...
    'HorizontalAlignment','left','Rotation',90,'FontSize',8.5,'FontWeight','bold','Color',P.red,'VerticalAlignment','bottom');

% legend
text(ax, bx, 3.2, 'Solid = forward command path      - - -  = state / sensor feedback      q-bar (dynamic pressure) schedules surface vs. RCS authority', ...
    'HorizontalAlignment','left','FontSize',8.5,'FontAngle','italic','Color',P.grey);

exportgraphics(fig, fpath, 'Resolution', 200);
close(fig);
end

% =====================================================================
function make_flow(fpath, P)
fig = figure('Color','w','Units','pixels','Position',[60 60 1600 1040],'Visible','off');
ax  = axes('Parent',fig,'Position',[0 0 1 1]); hold(ax,'on');
set(ax,'XLim',[0 100],'YLim',[0 100]); axis(ax,'off');

text(ax,50,97.5,'ACFT11 Mission Control — Decision Flow (phase sequencer)', ...
    'HorizontalAlignment','center','FontWeight','bold','FontSize',17,'Color',P.navy);
text(ax,50,94.2,'Each phase advances automatically when its entry condition is met (shown on the arrows)', ...
    'HorizontalAlignment','center','FontSize',10.5,'FontAngle','italic','Color',P.grey);

ph = { 'GROUND ROLL','ROTATION','LIFT-OFF','CLIMB','CRUISE','ACCELERATE', ...
       'ZOOM CLIMB','SEPARATION','COAST','GLIDE','ENGINE RELIGHT','RTB CRUISE', ...
       'DESCENT','APPROACH','FLARE','TOUCHDOWN / ROLLOUT','STOPPED' };
% entry condition INTO phase i+1
cond = { 'V >= Vr','wheels light','h > h\_safe','h = h\_cruise','boost armed', ...
         'M >= M\_trans','apogee / burnout','settle time','q-bar >= q-bar\_glide', ...
         'in relight box','spool complete','R <= R\_capture','GS intercept', ...
         'h <= h\_flare','on ground','V <= V\_stop' };
% colour by mission segment
seg = [1 1 1 2 2 2 3 3 3 4 4 4 5 5 5 6 6];   % 1 ground/climb 2 cruise/accel 3 boost/coast 4 glide/relight 5 descent/appr 6 land
cols= {P.slate,P.teal,P.purple,P.sea,P.amber,P.green};

N=numel(ph); ncol=4;
x0=5; colw=22; bw=20; bh=7.2;
xcol = x0 + (0:ncol-1)*colw;
yrow0=86; rowh=15;    % vertical pitch between rows

cx=zeros(N,1); cy=zeros(N,1);
for i=1:N
    r=floor((i-1)/ncol);
    k=mod(i-1,ncol);
    if mod(r,2)==0, c=k; else, c=ncol-1-k; end
    x=xcol(c+1); y=yrow0 - r*rowh;
    cx(i)=x+bw/2; cy(i)=y+bh/2;
    col=cols{seg(i)};
    drawBox(ax, x, y, bw, bh, ph{i}, '', tint(col,0.84), col, col);
    % phase index badge
    text(ax, x+1.4, y+bh-1.3, sprintf('%d',i), 'HorizontalAlignment','center','FontSize',8, ...
        'FontWeight','bold','Color','w','BackgroundColor',col,'Margin',1);
end

% connective arrows + conditions
for i=1:N-1
    sameRow = floor((i-1)/ncol)==floor(i/ncol);
    if sameRow
        if cx(i+1)>cx(i)   % left-to-right
            drawArrow(ax, cx(i)+bw/2, cy(i), cx(i+1)-bw/2, cy(i+1), P.ink,'-',1.5);
        else               % right-to-left
            drawArrow(ax, cx(i)-bw/2, cy(i), cx(i+1)+bw/2, cy(i+1), P.ink,'-',1.5);
        end
        text(ax,(cx(i)+cx(i+1))/2, cy(i)-bh/2-1.4, cond{i},'HorizontalAlignment','center','VerticalAlignment','middle','FontSize',7.6,'FontAngle','italic','Color',P.red);
    else               % drop to next row (same column)
        drawArrow(ax, cx(i), cy(i)-bh/2, cx(i+1), cy(i+1)+bh/2, P.ink,'-',1.5);
        text(ax, cx(i)+0.6, (cy(i)+cy(i+1))/2, cond{i},'HorizontalAlignment','left','FontSize',7.6,'FontAngle','italic','Color',P.red);
    end
end

% segment legend
segname={'Takeoff & climb','Cruise & accelerate','Boost & coast','Glide & relight','Descent & approach','Landing'};
lx=5; ly=3.0;
for s=1:6
    rectangle(ax,'Position',[lx ly-0.9 2 1.8],'Curvature',0.4,'FaceColor',tint(cols{s},0.6),'EdgeColor',cols{s},'LineWidth',1);
    text(ax,lx+2.6,ly,segname{s},'HorizontalAlignment','left','FontSize',8.2,'Color',P.ink);
    lx=lx+16;
end

exportgraphics(fig, fpath, 'Resolution', 200);
close(fig);
end

% =====================================================================
function drawBox(ax,x,y,w,h,ttl,desc,face,edge,tcol)
rectangle(ax,'Position',[x y w h],'Curvature',0.16,'FaceColor',face,'EdgeColor',edge,'LineWidth',1.5);
if isempty(desc)
    text(ax,x+w/2,y+h/2,ttl,'HorizontalAlignment','center','VerticalAlignment','middle','FontWeight','bold', ...
        'FontSize',11,'Color',tcol,'Interpreter','tex');
else
    text(ax,x+w/2,y+h*0.64,ttl,'HorizontalAlignment','center','VerticalAlignment','middle','FontWeight','bold', ...
        'FontSize',11.5,'Color',tcol,'Interpreter','tex');
    text(ax,x+w/2,y+h*0.28,desc,'HorizontalAlignment','center','VerticalAlignment','middle', ...
        'FontSize',8.6,'Color',[0.24 0.3 0.37],'Interpreter','tex');
end
end

function drawArrow(ax,x1,y1,x2,y2,color,ls,lw)
line(ax,[x1 x2],[y1 y2],'Color',color,'LineStyle',ls,'LineWidth',lw);
d=[x2-x1, y2-y1]; Ln=hypot(d(1),d(2));
if Ln<1e-9, return; end
u=d/Ln; nrm=[-u(2) u(1)];
hs=1.5;                      % head length (data units)
tip=[x2 y2];
b1=tip-hs*u+0.62*hs*nrm;
b2=tip-hs*u-0.62*hs*nrm;
patch(ax,'XData',[tip(1) b1(1) b2(1)],'YData',[tip(2) b1(2) b2(2)], ...
    'FaceColor',color,'EdgeColor',color);
end

function c=tint(c0,f)
% blend colour c0 toward white by fraction f (0=original,1=white)
c = c0 + (1-c0)*f;
end
