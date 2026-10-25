function flight_hud(src, playSpeed, staticT, viewer, chaseBack, chaseUp, camMode, showHUD)
%FLIGHT_HUD  Cinematic camera (and optional side HUD) synced to the mission.
%   Drives the Cesium globe clock per-frame at a fixed offset so the camera keeps
%   a CONSTANT geometry at any altitude. Data is downsampled and every camera
%   frame is precomputed, so playback is smooth.
%
%   flight_hud(src, speed, [], viewer, back, up, camMode)        HUD + camera
%   flight_hud(src, speed, [], viewer, back, up, camMode, false) camera only (NO HUD)
%
%   camMode shots:
%     'chase'  trailing            'wing'  off the wing        'orbit'  circling
%     'top'    overhead            'fpv'   onboard forward     'reveal' wide pull-back
%     'low'    low trailing angle  'cinematic' AUTO-CYCLE all of them along the flight
%   back/up (metres) scale the shots. Lower `speed` (e.g. 10) = smoother.

if nargin<1||isempty(src), src='attitude_data.mat'; end
if nargin<2||isempty(playSpeed), playSpeed=20; end
if nargin<3, staticT=[]; end
if nargin<4, viewer=[]; end
if nargin<5, chaseBack=[]; end
if nargin<6||isempty(chaseUp), chaseUp=2000; end
if nargin<7||isempty(camMode), camMode='chase'; end
if nargin<8||isempty(showHUD), showHUD=true; end

% ---- load + downsample --------------------------------------------------
if isa(src,'Simulink.SimulationOutput')
    P=src.logsout.getElement('PosLog').Values; Qv=src.logsout.getElement('QuatLog').Values;
    t=P.Time(:); R=squeeze(P.Data); if size(R,1)==3,R=R.'; end
    Q=squeeze(Qv.Data); if size(Q,1)==4,Q=Q.'; end
    Q=interp1(Qv.Time,Q,t); Q=Q./vecnorm(Q,2,2);
else
    S=load(char(src)); t=S.t(:); R=S.R; Q=S.Qb2e;
end
ok=all(isfinite(R),2)&all(isfinite(Q),2); t=t(ok);R=R(ok,:);Q=Q(ok,:);
dt0=median(diff(t)); step=max(1,round((1/30)/max(dt0,1e-3)));   % ~30 Hz sampling
k=1:step:numel(t); if numel(k)>30000, k=round(linspace(1,numel(t),30000)); end
t=t(k); R=R(k,:); Q=Q(k,:); N=numel(t);

% ---- telemetry ---------------------------------------------------------
LLA=ecef2lla(R); lat=LLA(:,1); lon=LLA(:,2); alt=LLA(:,3);
Vecef=[gradient(R(:,1),t), gradient(R(:,2),t), gradient(R(:,3),t)];
spd=vecnorm(Vecef,2,2);
gam=atan2d(gradient(alt,t), max(spd,1e-3));
Tisa=288.15-0.0065*min(alt,11000); Tisa(alt>11000)=216.65;
Mach=spd./sqrt(1.4*287.058*Tisa);
pit=zeros(N,1); rol=zeros(N,1); yaw=zeros(N,1);
for i=1:N
    w=Q(i,1);x=Q(i,2);y=Q(i,3);z=Q(i,4);
    C_b2e=[w^2+x^2-y^2-z^2,2*(x*y-w*z),2*(x*z+w*y);
           2*(x*y+w*z),w^2-x^2+y^2-z^2,2*(y*z-w*x);
           2*(x*z-w*y),2*(y*z+w*x),w^2-x^2-y^2+z^2];
    la=lat(i)*pi/180; lo=lon(i)*pi/180; sl=sin(la);cl=cos(la);so_=sin(lo);co=cos(lo);
    C_e2ned=[-sl*co,-sl*so_,cl; -so_,co,0; -cl*co,-cl*so_,-sl];
    C=C_e2ned*C_b2e;
    pit(i)=asind(max(-1,min(1,-C(3,1)))); rol(i)=atan2d(C(3,2),C(3,3));
    h=atan2d(C(2,1),C(1,1)); if h<0,h=h+360; end; yaw(i)=h;
end

% ---- PRECOMPUTE every camera frame -------------------------------------
doCam = ~isempty(viewer) && ~isempty(chaseBack) && chaseBack>0;
seq={'chase','wing','orbit','reveal','top','orbit','fpv','low','chase','wing'};
if doCam
    vhat=Vecef./max(vecnorm(Vecef,2,2),1e-6);
    uphat=R./vecnorm(R,2,2); sidehat=cross(uphat,vhat,2);
    sidehat=sidehat./max(vecnorm(sidehat,2,2),1e-6);
    camLLAall=zeros(N,3); chdAll=zeros(N,1); cptAll=zeros(N,1); shotAll=strings(N,1);
    b=chaseBack; u=chaseUp;
    for i=1:N
        if strcmpi(camMode,'cinematic'), m=seq{mod(floor(t(i)/75),numel(seq))+1}; else, m=camMode; end
        af=1+min(alt(i)/20000,2);                 % grander framing up high
        Ri=R(i,:); vh=vhat(i,:); uh=uphat(i,:); sh=sidehat(i,:); look=true;
        switch lower(m)
            case 'wing',   cam=Ri+2.0*b*sh+u*uh-0.5*b*vh;
            case 'top',    cam=Ri+max(4*u,3*b)*uh;
            case 'orbit',  an=20*t(i); cam=Ri+af*2.5*b*(cosd(an)*(-vh)+sind(an)*sh)+af*u*uh;
            case 'reveal', cam=Ri-af*4*b*vh+af*2.5*u*uh;
            case 'low',    cam=Ri-1.3*b*vh+0.12*u*uh;
            case 'fpv',    cam=Ri+0.25*b*vh+0.12*u*uh; look=false;
            otherwise,     cam=Ri-b*vh+u*uh;         % chase
        end
        if look
            d=Ri-cam; laC=atan2(cam(3),hypot(cam(1),cam(2))); loC=atan2(cam(2),cam(1));
            sl=sin(laC);cl=cos(laC);so_=sin(loC);co=cos(loC);
            dE=[-so_,co,0; -sl*co,-sl*so_,cl; cl*co,cl*so_,sl]*d.';
            chdAll(i)=atan2d(dE(1),dE(2)); cptAll(i)=atan2d(dE(3),hypot(dE(1),dE(2)));
        else
            chdAll(i)=yaw(i); cptAll(i)=gam(i)-3;
        end
        camLLAall(i,:)=ecef2lla(cam); shotAll(i)=upper(m);
    end
    t0abs=viewer.CurrentTime;
end

% ---- optional HUD panel -------------------------------------------------
fig=[]; hVal=[]; hShot=[]; hT=[];
if showHUD
    fl=listfonts; pick=@(opts) opts{find(cellfun(@(f)any(strcmp(fl,f)),opts),1)};
    Flab=pick({'Bahnschrift','Segoe UI','Tahoma','Arial'});
    Fval=pick({'Segoe UI Semibold','Bahnschrift','Consolas','Arial'});
    BG=[0.055 0.065 0.080]; ACC=[0.98 0.62 0.09]; VAL=[1 0.80 0.28];
    LBL=[0.49 0.57 0.66]; UNIT=[0.40 0.47 0.55]; HDR=[0.90 0.94 0.99]; SEP=[0.13 0.15 0.19];
    su=get(0,'ScreenSize'); W=240; H=500;
    if ~isempty(viewer)&&isprop(viewer,'Position'), vp=viewer.Position; pos=[vp(1)+vp(3)-W-12, vp(2)+vp(4)-H-12, W, H];
    else, pos=[su(3)-W-28,160,W,H]; end
    fig=figure('Name','Flight HUD','NumberTitle','off','Color',BG,'Resize','off', ...
        'MenuBar','none','ToolBar','none','DockControls','off','Position',pos);
    ax=axes(fig,'Position',[0 0 1 1],'Color','none','XColor','none','YColor','none','XLim',[0 1],'YLim',[0 1]); hold(ax,'on');
    patch(ax,[0 .018 .018 0],[0 0 1 1],ACC,'EdgeColor','none');
    text(ax,.11,.955,'ACFT11','Color',HDR,'FontName',Fval,'FontSize',17,'FontWeight','bold');
    hShot=text(ax,.11,.915,'FLIGHT DATA','Color',ACC,'FontName',Flab,'FontSize',10.5);
    plot(ax,[.09 .93],[.90 .90],'-','Color',SEP,'LineWidth',1);
    rows={'ALT','m';'TAS','m/s';'MACH','';'PITCH',char(176);'ROLL',char(176);'HDG',char(176);'FPA',char(176)};
    yv=linspace(.82,.11,size(rows,1)); hVal=gobjects(size(rows,1),1);
    for kk=1:size(rows,1)
        text(ax,.11,yv(kk),rows{kk,1},'Color',LBL,'FontName',Flab,'FontSize',12.5);
        hVal(kk)=text(ax,.78,yv(kk),'--','Color',VAL,'FontName',Fval,'FontSize',20,'FontWeight','bold','HorizontalAlignment','right');
        if ~isempty(rows{kk,2}), text(ax,.80,yv(kk)-.006,rows{kk,2},'Color',UNIT,'FontName',Flab,'FontSize',10.5); end
        if kk<size(rows,1), plot(ax,[.11 .90],[1 1]*mean(yv(kk:kk+1)),'-','Color',SEP,'LineWidth',0.5); end
    end
    hT=text(ax,.5,.045,'T  0 s','Color',UNIT,'FontName',Flab,'FontSize',11,'HorizontalAlignment','center');
end

    function apply(i)
        if showHUD && isvalid(fig)
            set(hVal(1),'String',sprintf('%.0f',alt(i)));  set(hVal(2),'String',sprintf('%.0f',spd(i)));
            set(hVal(3),'String',sprintf('%.2f',Mach(i))); set(hVal(4),'String',sprintf('%+.1f',pit(i)));
            set(hVal(5),'String',sprintf('%+.1f',rol(i))); set(hVal(6),'String',sprintf('%.0f',yaw(i)));
            set(hVal(7),'String',sprintf('%+.1f',gam(i))); set(hT,'String',sprintf('T  %.0f s',t(i)));
            set(hShot,'String',shotAll(i));
        end
        if doCam && isvalid(viewer)
            try
                viewer.CurrentTime=t0abs+seconds(t(i)-t(1));
                campos(viewer,camLLAall(i,1),camLLAall(i,2),camLLAall(i,3));
                camheading(viewer,chdAll(i)); campitch(viewer,cptAll(i));
            catch, end
        end
    end
    function a=alive()
        a=true; if showHUD && ~isvalid(fig), a=false; end
        if ~showHUD && ~isempty(viewer) && ~isvalid(viewer), a=false; end
    end

% ---- static frame or steady-cadence playback ----------------------------
if ~isempty(staticT), [~,i]=min(abs(t-staticT)); apply(i); drawnow; return; end
fps=30; nF=max(2,round((t(end)-t(1))/max(playSpeed,1e-3)*fps));   % 30 fps playback
tgrid=linspace(t(1),t(end),nF); idx=interp1(t,1:N,tgrid,'nearest');
for f=1:nF
    if ~alive(), break; end
    apply(idx(f)); drawnow limitrate; pause(1/fps);
end
if alive(), apply(N); end
end
