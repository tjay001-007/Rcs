function A = analyze_control_authority(src, doPlot)
% ANALYZE_CONTROL_AUTHORITY  Control-surface authority / effectiveness report.
%
%   A = analyze_control_authority(simOut)     % pass the sim output
%   A = analyze_control_authority('out.mat')  % or a saved logsout .mat
%   A = analyze_control_authority()           % uses 'so' or 'logsout' from base
%   A = analyze_control_authority(src,false)  % tables only, no figures
%
% Analyses, per surface (elevator de, aileron da, rudder dr) and per mission
% phase:
%   * DEFLECTION authority  - peak |defl|, % of position limit, % time saturated
%   * RATE authority        - peak/RMS deflection rate, % of rate limit, and a
%                             chatter (limit-cycle) frequency estimate
%   * DYNAMIC-PRESSURE sensitivity - how deflection scales with qbar (control
%                             power ~ effectiveness*qbar), binned usage, and the
%                             qbar at which each surface first saturates
%   * RCS moments vs the per-axis budget
%
% Requires logged signals (signal-logging names): Elevlog, Ailog, Rudlog (deg),
% RCSlog ([Fx Fy Fz Mx My Mz]), PhaseLog, and Qbarlog (Pa) for the qbar study.
% Limits/budget are read from the base workspace (FCS_de_max/_rate, FCS_da_*,
% FCS_dr_*, FCS_rcs_budget) with sensible defaults.
%
% Rates use a uniform 0.1 s "sustained" grid for the authority %, and the native
% samples for the chatter-frequency estimate (so solver-step blips are reported
% as chatter, not mistaken for real sustained slew).

if nargin < 2, doPlot = true; end
if nargin < 1 || isempty(src), src = 'base'; end
ls = i_getlogsout(src);

g = @(n,d) i_basevar(n,d);
lim = struct('de',g('FCS_de_max',25), 'da',g('FCS_da_max',20), 'dr',g('FCS_dr_max',25));
rl  = struct('de',g('FCS_de_rate',80),'da',g('FCS_da_rate',100),'dr',g('FCS_dr_rate',80));
Mbud = g('FCS_rcs_budget',4000);

[tde,de] = i_sig(ls,'Elevlog');
[tda,da] = i_sig(ls,'Ailog');
[tdr,dr] = i_sig(ls,'Rudlog');
[tph,ph] = i_sig(ls,'PhaseLog');
[tqb,qb] = i_sig(ls,'Qbarlog');
[trc,rc] = i_sig(ls,'RCSlog');
if ~isempty(trc) && size(rc,2)~=6 && size(rc,1)==6, rc = rc.'; end
if ~isempty(trc) && size(rc,1)~=numel(trc) && size(rc,2)==numel(trc), rc = rc.'; end
if isempty(tde), error('No Elevlog signal found - enable FCS surface logging first.'); end

% uniform 0.02 s grid for peaks/%, 0.1 s subgrid for sustained rate
tf  = (tde(1):0.02:tde(end)).';
rsi = @(tt,x) interp1(tt,x,tf,'linear','extrap');
DE=rsi(tde,de); DA=rsi(tda,da); DR=rsi(tdr,dr);
PH = interp1(tph,ph,tf,'previous','extrap');
if ~isempty(tqb), QB = interp1(tqb,qb,tf,'linear','extrap'); else, QB = nan(size(tf)); end
if ~isempty(trc), MY=abs(interp1(trc,rc(:,5),tf)); MX=abs(interp1(trc,rc(:,4),tf)); MZ=abs(interp1(trc,rc(:,6),tf));
else, MX=zeros(size(tf)); MY=MX; MZ=MX; end

A = struct();
A.surfaces = i_surfTable({'de',DE,lim.de,rl.de,tde,de; 'da',DA,lim.da,rl.da,tda,da; 'dr',DR,lim.dr,rl.dr,tdr,dr});
A.rcs = struct('Mx',max(MX),'My',max(MY),'Mz',max(MZ),'budget',Mbud,'My_pct',100*max(MY)/Mbud);

% ---- mission-wide surface summary (deflection + RATE + chatter) ----------
fprintf('\n=== MISSION-WIDE SURFACE AUTHORITY ===\n');
fprintf('%-4s %8s %6s %10s %7s %9s %11s %10s\n','sfc','max|d|','%pos','max rate','%rate','RMS rate','%t>=90%%pos','chatter');
for i=1:numel(A.surfaces)
    s=A.surfaces(i);
    fprintf('%-4s %7.1f %6.0f %10.0f %7.0f %9.1f %11.1f %8.0f Hz\n', ...
        s.name,s.maxDefl,s.pctPos,s.maxRate,s.pctRate,s.rmsRate,s.pctSat,s.chatterHz);
end
fprintf('RCS: |My|max=%.0f Nm (%.0f%% of %.0f); |Mx|=%.0f |Mz|=%.0f\n',max(MY),A.rcs.My_pct,Mbud,max(MX),max(MZ));

% ---- per-phase deflection AND rate (% of limits) -------------------------
pn = i_phaseNames(); up = unique(PH(~isnan(PH))).';
P = struct('phase',{},'de_pct',{},'da_pct',{},'dr_pct',{},'de_rate_pct',{},'da_rate_pct',{},'dr_rate_pct',{},'My_pct',{},'qbar_lo',{},'qbar_hi',{});
fprintf('\n=== PER-PHASE: peak deflection %% / peak RATE %% of limit ===\n');
fprintf('%-9s %11s %11s %11s %9s %11s\n','phase','de d/rate','da d/rate','dr d/rate','My %%','qbar[Pa]');
for k=up
    m=PH==k; if ~any(m), continue; end
    dep=100*max(abs(DE(m)))/lim.de; dap=100*max(abs(DA(m)))/lim.da; drp=100*max(abs(DR(m)))/lim.dr;
    rr=@(x,L) 100*max(abs([0;diff(x(1:5:end))]/0.1))/L; mm=m(1:5:end); % rate on 0.1s grid within phase
    der=i_phaseRate(DE,PH,k,rl.de); dar=i_phaseRate(DA,PH,k,rl.da); drr=i_phaseRate(DR,PH,k,rl.dr);
    qlo=min(QB(m)); qhi=max(QB(m));
    fprintf('%-9s %3.0f/%3.0f%%   %3.0f/%3.0f%%   %3.0f/%3.0f%%  %5.0f   %5.0f-%-5.0f\n', ...
        i_name(pn,k), dep,der, dap,dar, drp,drr, 100*max(MY(m))/Mbud, qlo,qhi);
    P(end+1)=struct('phase',i_name(pn,k),'de_pct',dep,'da_pct',dap,'dr_pct',drp, ...
        'de_rate_pct',der,'da_rate_pct',dar,'dr_rate_pct',drr,'My_pct',100*max(MY(m))/Mbud,'qbar_lo',qlo,'qbar_hi',qhi); %#ok<AGROW>
end
A.perPhase = P;

% ---- dynamic-pressure sensitivity ----------------------------------------
if ~isempty(tqb)
    fprintf('\n=== DYNAMIC-PRESSURE SENSITIVITY (mean |defl| by qbar bin) ===\n');
    edges = [0 1000 2000 3000 5000 8000 inf];
    fprintf('%-14s %8s %8s %8s %8s\n','qbar bin [Pa]','N','de','da','dr');
    A.qbarBins = struct('lo',{},'hi',{},'de',{},'da',{},'dr',{});
    for b=1:numel(edges)-1
        m = QB>=edges(b) & QB<edges(b+1);
        if ~any(m), continue; end
        fprintf('%6.0f-%-7.0f %8d %8.2f %8.2f %8.2f\n', edges(b),min(edges(b+1),99999), nnz(m), mean(abs(DE(m))),mean(abs(DA(m))),mean(abs(DR(m))));
        A.qbarBins(end+1)=struct('lo',edges(b),'hi',edges(b+1),'de',mean(abs(DE(m))),'da',mean(abs(DA(m))),'dr',mean(abs(DR(m)))); %#ok<AGROW>
    end
    % qbar at first saturation of each surface
    sat=@(x,L) i_firstSatQbar(x,L,QB);
    fprintf('qbar at onset of saturation:  de=%s  da=%s  dr=%s\n', sat(DE,lim.de),sat(DA,lim.da),sat(DR,lim.dr));
end

if doPlot
    i_plotHist(tf,DE,DA,DR,MY,QB,lim,Mbud,PH);
    if ~isempty(tqb), i_plotQbar(DE,DA,DR,QB,lim); end
end
end

% ========================= helpers =======================================
function T = i_surfTable(C)
T=struct('name',{},'maxDefl',{},'pctPos',{},'maxRate',{},'pctRate',{},'rmsRate',{},'pctSat',{},'chatterHz',{});
for i=1:size(C,1)
    x=C{i,2}; L=C{i,3}; RL=C{i,4}; tn=C{i,5}; xn=C{i,6};
    xc=x(1:5:end); r=[0;diff(xc)]/0.1;           % 0.1s sustained rate
    % chatter: rate of *meaningful* slope reversals / 2 (Hz). A reversal counts
    % only when the local step exceeds a small amplitude gate, so negligible
    % numerical dither is not reported as flutter.
    dn=diff(xn); amp=0.05;                           % deg per step amplitude gate
    big=abs(dn)>amp; sgn=sign(dn);
    rev=sum( (abs(diff(sgn))>0) & big(1:end-1) & big(2:end) );
    dur=max(tn(end)-tn(1),eps); chat = rev/2/dur;
    T(end+1)=struct('name',C{i,1},'maxDefl',max(abs(x)),'pctPos',100*max(abs(x))/L, ...
        'maxRate',max(abs(r)),'pctRate',100*max(abs(r))/RL,'rmsRate',rms(r), ...
        'pctSat',100*mean(abs(x)>0.9*L),'chatterHz',chat); %#ok<AGROW>
end
end

function p = i_phaseRate(X,PH,k,L)
m=PH==k; xi=X(m); if numel(xi)<3, p=0; return; end
xc=xi(1:5:end); if numel(xc)<3, xc=xi; dt=0.02; else, dt=0.1; end
p=100*max(abs(diff(xc)/dt))/L;
end

function q = i_firstSatQbar(x,L,QB)
idx=find(abs(x)>=0.98*L,1);
if isempty(idx)||isnan(QB(idx)), q='(never)'; else, q=sprintf('%.0f Pa',QB(idx)); end
end

function ls = i_getlogsout(src)
if isa(src,'Simulink.SimulationOutput'), ls=src.logsout; return; end
if isa(src,'Simulink.SimulationData.Dataset'), ls=src; return; end
if ischar(src)||isstring(src)
    if strcmpi(src,'base')
        if evalin('base','exist(''so'',''var'')'), ls=evalin('base','so').logsout;
        elseif evalin('base','exist(''logsout'',''var'')'), ls=evalin('base','logsout');
        else, error('No ''so'' or ''logsout'' in base workspace.'); end
    else
        D=load(char(src));
        if isfield(D,'logsout'), ls=D.logsout; elseif isfield(D,'so'), ls=D.so.logsout; else, error('MAT has no logsout/so.'); end
    end
else, error('Unsupported source.'); end
end

function [t,x]=i_sig(ls,name)
t=[]; x=[];
try, e=ls.getElement(name); v=e.Values; t=v.Time; x=squeeze(v.Data); if size(x,1)==1, x=x.'; end, catch, end
end

function v=i_basevar(n,d)
try, if evalin('base',['exist(''' n ''',''var'')']), v=evalin('base',n); else, v=d; end, catch, v=d; end
end

function nm=i_phaseNames()
nm=containers.Map('KeyType','double','ValueType','char');
k={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
v={'GND','ROT','LOF','CLB','CRZ','ACC','ZOOM','SEP','GLIDE','RELIGHT','CRZ_RTB','DESC','APPR','FLARE','TD','ROLL','STOP','COAST'};
for i=1:numel(k), nm(k{i})=v{i}; end
end
function s=i_name(mp,k), if isKey(mp,k), s=mp(k); else, s=sprintf('ph%d',k); end, end

function i_plotHist(t,DE,DA,DR,MY,QB,lim,Mbud,PH)
figure('Color','w','Name','Authority - time histories','NumberTitle','off');
bnd=t([true;diff(PH)~=0]);
pan={'\delta_e (deg)',DE,lim.de,'b'; '\delta_a (deg)',DA,lim.da,[0 .6 0]; ...
     '\delta_r (deg)',DR,lim.dr,[.85 .3 .1]; 'RCS M_y (Nm)',MY,Mbud,[.5 0 .6]; 'qbar (Pa)',QB,[],[.2 .2 .2]};
n=5-double(all(isnan(QB)));
for i=1:n
    subplot(n,1,i); plot(t,pan{i,2},'Color',pan{i,4}); hold on; grid on;
    if ~isempty(pan{i,3}), yline(pan{i,3},'r--'); if min(pan{i,2})<0, yline(-pan{i,3},'r--'); end, end
    for b=bnd.', xline(b,'Color',[.86 .86 .86]); end
    ylabel(pan{i,1}); xlim([t(1) t(end)]); if i<n, set(gca,'XTickLabel',[]); end
end
xlabel('time (s)'); try, sgtitle('Control authority vs limits (dashed)','FontWeight','bold'); catch, end
end

function i_plotQbar(DE,DA,DR,QB,lim)
figure('Color','w','Name','Dynamic-pressure sensitivity','NumberTitle','off');
S={'\delta_e',DE,lim.de,'b'; '\delta_a',DA,lim.da,[0 .6 0]; '\delta_r',DR,lim.dr,[.85 .3 .1]};
for i=1:3
    subplot(1,3,i); plot(QB,abs(S{i,2}),'.','Color',S{i,4},'MarkerSize',3); hold on; grid on;
    yline(S{i,3},'r--'); xlabel('qbar (Pa)'); ylabel(['|' S{i,1} '| (deg)']); title(S{i,1});
end
try, sgtitle('|deflection| vs dynamic pressure (authority tightens at low qbar)','FontWeight','bold'); catch, end
end
