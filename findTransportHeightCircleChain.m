function [bestZ, out] = findTransportHeightCircleChain(envS, rebarMeta, varargin)
% findTransportHeightCircleChain
%   Circle-chain planner with adaptive sampling across height slices.
%
% Usage (full sweep):
%   [bestZ,out] = findTransportHeightCircleChain(envS,rebarMeta);
%
% Usage (single height, no plots):
%   [~,outZ] = findTransportHeightCircleChain(envS,rebarMeta, ...
%                 'z',1.35,'computeOnly',true);
%
% Optional name-value args:
%   'z'            : scalar height (evaluate only this plane). Default: sweep.
%   'computeOnly'  : logical, suppress plots (default: false).
%   'bounds'       : [xlim; ylim], e.g., [-3 3; -3.5 2].
%   'res'          : grid resolution in meters (default: 0.05).
%   'goalXY'       : [x y] override for goal center (default from rebarMeta).
%
% Returns:
%   bestZ : winning height (NaN if none)
%   out   :
%       out.best.chainXY  : Mx2 centers of the chosen circle-chain
%       out.best.radii    : Mx1 radii of the chosen circle-chain
%       out.best.nodes    : struct array of nodes (winning z)
%       out.best.edges    : Ex3 edges [i j length] (winning z)
%       out.best.z        : winning z
%       out.best.len      : polyline length of chain centers
%       out.best.cost     : final A* cost
%       out.meta.bounds   : [xlim; ylim]
%       out.meta.res      : grid resolution
%       out.meta.time     : total compute time (s)

% --------- Parse options ----------
p = inputParser;
addParameter(p, 'z', [], @(v) isempty(v) || (isscalar(v) && isfinite(v)));
addParameter(p, 'computeOnly', false, @(v)islogical(v) || ismember(v,[0 1]));
addParameter(p, 'bounds', [], @(v) isempty(v) || (isnumeric(v) && isequal(size(v),[2 2])));
addParameter(p, 'res', 0.05, @(v)isnumeric(v) && isscalar(v) && v>0);
addParameter(p, 'goalXY', [], @(v) isempty(v) || (isnumeric(v) && numel(v)==2));
parse(p, varargin{:});
opt = p.Results;

% --- Allow zero-arg convenience: auto-load env & metadata if missing -----
if nargin < 2 || isempty(envS) || isempty(rebarMeta)
    try
        [~, ~, envRaw, ~, ~, placeRebarCenterPosition, rebarMeta2] = ...
            loadRebarTransportScenario;
        % normalize env to struct-wrapped format
        envS = wrapEnvToStruct(envRaw);
        % propagate goal XY into rebarMeta if not present
        if ~isfield(rebarMeta2,'placeCenterXY') || isempty(rebarMeta2.placeCenterXY)
            rebarMeta2.placeCenterXY = placeRebarCenterPosition(1:2).';
        end
        rebarMeta = rebarMeta2;
        fprintf('[Auto-load] envS/rebarMeta were missing; loaded from loadRebarTransportScenario.\n');
    catch ME
        error(['No inputs provided and auto-load failed. Provide (envS, rebarMeta) or ensure ', ...
               'loadRebarTransportScenario.m is on your MATLAB path.\nReason: %s'], ME.message);
    end
end

% --------- Use the inputs as given ----------
env = envS;  % can be {struct(Name,CollisionObj)} or plain cell of collision objects

% WORLD bounds & grid
if isempty(opt.bounds)
    xlimW = [-5.0, 5.0];
    ylimW = [-5, 5.0];
else
    xlimW = opt.bounds(1,:);
    ylimW = opt.bounds(2,:);
end
res = opt.res;

% Start/Goal XY from rebarMeta (and/or override)
start_xy = rebarMeta.T_start_W(1:2,4).';
if ~isempty(opt.goalXY)
    goal_xy = opt.goalXY(:).';
elseif isfield(rebarMeta,'placeCenterXY') && ~isempty(rebarMeta.placeCenterXY)
    goal_xy = rebarMeta.placeCenterXY(:).';
elseif isfield(rebarMeta,'placeRebarCenterPosition') && ~isempty(rebarMeta.placeRebarCenterPosition)
    goal_xy = rebarMeta.placeRebarCenterPosition(1:2).';
else
    error('Goal XY not found in rebarMeta and not provided via ''goalXY''.');
end
start_xy = start_xy(:).';  goal_xy = goal_xy(:).';

% Heights (single-Z or sweep)
if ~isempty(opt.z)
    z_list = opt.z;
else
    z_start = rebarMeta.T_start_W(3,4) + rebarMeta.DiameterM/2 + 0.01;
    z_end   = 2.00;
    dz      = 0.20;
    z_list  = z_start:dz:z_end; if isempty(z_list), z_list = z_start; end
end
computeOnly = opt.computeOnly;

% ----------------- Parameters -----------------
Lbar       = rebarMeta.LengthM;
rmin_soft  = 0.25 * Lbar;
rmax       = 0.6 * Lbar;
rmin_hard  = max(0.05, 2*res);

stepS  = 0.3;    % base grid step (for fallback)
margin = 0.00;   % segment safety margin (>=0)

% Weights for A*
W = struct('alpha',1.0,'beta',4,'gamma',0.0,'delta',0.0,'rpen',0,'rdrop',0.0);
maxNbr = [];     % neighbor fanout limit (empty = adaptive by grid hashing)

% Adaptive sampling of candidate circle centers
adapt = struct( ...
    'enabled',   true, ...
    'resS',      max(res, stepS), ... % candidate grid spacing
    'minSep',    0.35*stepS, ...
    'alpha',     0.7, ...
    'beta',      2.0, ...
    'maxPts',    800 ...
);

% Visuals (only used when ~computeOnly)
ORANGE       = [0.95 0.55 0.15];
ORANGE_EDGE  = [0.55 0.28 0.08];
ORANGE_ALPHA = 0.35;
TitleFS = 16;
LabelFS = 12;

% ----------------- 2D figure setup -----------------
if ~computeOnly
    figG = figure('Name','Circle-Chain Connectivity per height','Units','normalized','OuterPosition',[0.02 0.05 0.47 0.90]);
    tlG  = tiledlayout(figG,'flow','Padding','compact','TileSpacing','compact');

    figF = figure('Name','Best Circle-Chain per height','Units','normalized','OuterPosition',[0.51 0.05 0.47 0.90]);
    tlF  = tiledlayout(figF,'flow','Padding','compact','TileSpacing','compact');
end

bestOverall = struct('J',inf,'z',NaN,'path',[],'r',[],'len',NaN,'nodes',[],'edges',[]);
timeTic = tic;

% --- Track per-Z diagnostics (for "flat" detection) ---
z_checked  = [];
z_cost     = [];   % A* cost per Z (Inf if no path)
z_hasPath  = [];   % logical per Z

% ===================== MAIN HEIGHT LOOP =====================
for z = z_list
    % ---------- Compute slice, distance, and candidate graph ----------
    [occ,xs,ys] = sliceEnv2D(env, xlimW, ylimW, res, z);
    D = distFromOcc(occ, res);
    [si,sj] = worldToGridSnap(xs,ys,res,occ,start_xy(1),start_xy(2));
    [gi,gj] = worldToGridSnap(xs,ys,res,occ,goal_xy(1), goal_xy(2));
    s_snap  = [ xs(sj), ys(si) ];
    g_snap  = [ xs(gj), ys(gi) ];
    bounds  = [xlimW; ylimW];

    [nodes, edges] = buildCircleGraph(xs, ys, D, occ, bounds, ...
                                      rmin_hard, rmin_soft, rmax, ...
                                      stepS, margin, s_snap, g_snap, maxNbr, adapt);

    % ---------- Connectivity Map window ----------
    if ~computeOnly
        figure(figG);
        axG = nexttile(tlG); hold(axG,'on'); grid(axG,'on'); axis(axG,'equal');
        axis(axG,[xlimW ylimW]); set(axG,'YDir','normal');
        drawObstacles(axG, env, z, ORANGE, ORANGE_ALPHA, ORANGE_EDGE);
        if isempty(nodes) || isempty(edges)
            title(axG, sprintf('z = %.2f — no connectivity', z), 'FontSize', TitleFS, 'FontWeight','bold');
        else
            optsGraph = struct('drawAllCircles',true,'drawAllEdges',false,'maxCirclesToDraw',500, ...
                               'colCirc',[0.83 0.83 0.83],'colNode',[0.35 0.35 0.35]);
            drawCircleGraph(axG, nodes, edges, optsGraph);
            title(axG, sprintf('z = %.2f — circle-chain connectivity', z), 'FontSize', TitleFS, 'FontWeight','bold');
        end
        plot(axG, start_xy(1), start_xy(2), 'go', 'MarkerFaceColor','g', 'MarkerSize',7, 'LineWidth',1.2);
        plot(axG, goal_xy(1),  goal_xy(2),  'co', 'MarkerFaceColor','c', 'MarkerSize',6);
        xlabel(axG,'X (m)','FontSize',LabelFS); ylabel(axG,'Y (m)','FontSize',LabelFS);
    end

    % ---------- A* on circle graph ----------
    if isempty(nodes) || isempty(edges)
        if ~computeOnly
            figure(figF);
            axF = nexttile(tlF); hold(axF,'on'); grid(axF,'on'); axis(axF,'equal');
            axis(axF,[xlimW ylimW]); set(axF,'YDir','normal');
            drawObstacles(axF, env, z, ORANGE, ORANGE_ALPHA, ORANGE_EDGE);
            plot(axF, start_xy(1), start_xy(2), 'go', 'MarkerFaceColor','g', 'MarkerSize',7, 'LineWidth',1.2);
            plot(axF, goal_xy(1),  goal_xy(2),  'co', 'MarkerFaceColor','c', 'MarkerSize',6);
            title(axF, sprintf('z = %.2f — no feasible chain', z), 'FontSize',TitleFS,'FontWeight','bold');
            xlabel(axF,'X (m)','FontSize',LabelFS); ylabel(axF,'Y (m)','FontSize',LabelFS);
        end
        continue;
    end

    [nodePathIdx, J] = astarCircleChain(nodes, edges, W, rmin_soft);
    if isempty(nodePathIdx)
        if ~computeOnly
            figure(figF);
            axF = nexttile(tlF); hold(axF,'on'); grid(axF,'on'); axis(axF,'equal');
            axis(axF,[xlimW ylimW]); set(axF,'YDir','normal');
            drawObstacles(axF, env, z, ORANGE, ORANGE_ALPHA, ORANGE_EDGE);
            plot(axF, start_xy(1), start_xy(2), 'go', 'MarkerFaceColor','g', 'MarkerSize',7, 'LineWidth',1.2);
            plot(axF, goal_xy(1),  goal_xy(2),  'co', 'MarkerFaceColor','c', 'MarkerSize',6);
            title(axF, sprintf('z = %.2f — no feasible chain', z), 'FontSize',TitleFS,'FontWeight','bold');
            xlabel(axF,'X (m)','FontSize',LabelFS); ylabel(axF,'Y (m)','FontSize',LabelFS);
        end
        continue;
    end

    % --- Record per-Z results for flat-cost detection ---
    z_checked(end+1) = z; %#ok<AGROW>
    if isempty(nodePathIdx)
        z_cost(end+1)    = Inf; %#ok<AGROW>
        z_hasPath(end+1) = false; %#ok<AGROW>
    else
        z_cost(end+1)    = J; %#ok<AGROW>
        z_hasPath(end+1) = true; %#ok<AGROW>
    end

    % ---------- Stats ----------
    pathXY = vertcat(nodes(nodePathIdx).pos);
    r_vec  = vertcat(nodes(nodePathIdx).r);
    L      = sum( sqrt(sum(diff(pathXY,1,1).^2,2)) );

    % ---------- Best Chain window ----------
    if ~computeOnly
        figure(figF);
        axF = nexttile(tlF); hold(axF,'on'); grid(axF,'on'); axis(axF,'equal');
        axis(axF,[xlimW ylimW]); set(axF,'YDir','normal');
        drawObstacles(axF, env, z, ORANGE, ORANGE_ALPHA, ORANGE_EDGE);
        plot(axF, start_xy(1), start_xy(2), 'go', 'MarkerFaceColor','g', 'MarkerSize',7, 'LineWidth',1.2);
        plot(axF, goal_xy(1),  goal_xy(2),  'co', 'MarkerFaceColor','c', 'MarkerSize',6);

        k  = min(24, size(pathXY,1));
        ii = unique(round(linspace(1,size(pathXY,1),k)));
        drawCircles(axF, pathXY(ii,:), r_vec(ii), [0 0.45 0.74], '', 1.8); % blue
        plot(axF, pathXY(:,1), pathXY(:,2), '-', 'Color',[1 0 0], 'LineWidth',2);

        title(axF, {sprintf('z = %.2f — best circle-chain', z), ...
                    sprintf('Cost C = %.3f | len = %.2f | nodes=%d', ...
                    J, L, numel(nodePathIdx))}, 'FontSize',TitleFS,'FontWeight','bold');
        xlabel(axF,'X (m)','FontSize',LabelFS); ylabel(axF,'Y (m)','FontSize',LabelFS);
    end

    % ---------- Track global best ----------
    if J < bestOverall.J
        bestOverall = struct( ...
            'J',     J, ...
            'z',     z, ...
            'path',  pathXY, ...
            'r',     r_vec, ...
            'len',   L, ...
            'nodes', nodes, ...
            'edges', edges );
    end
end
% ===================== END LOOP =====================

timeTaken = toc(timeTic);

% --- Detect "height-invariant" (all feasible slices have ~same cost) ---
finiteCosts = z_cost(isfinite(z_cost));
flatTolAbs  = 1e-6;                 % absolute tolerance
flatTolRel  = 1e-2;                 % 1% relative tolerance
costSpread  = Inf;
if numel(finiteCosts) >= 2
    mn = min(finiteCosts); mx = max(finiteCosts);
    costSpread = mx - mn;
end
isHeightInvariant = numel(finiteCosts) >= 2 && ...
    costSpread <= max(flatTolAbs, flatTolRel * min(finiteCosts));

% stash diagnostics
diag = struct();
diag.z_list      = z_checked(:);
diag.costPerZ    = z_cost(:);
diag.hasPath     = z_hasPath(:);
diag.costSpread  = costSpread;
diag.flatTolAbs  = flatTolAbs;
diag.flatTolRel  = flatTolRel;

% ---------- Summary ----------
if isfinite(bestOverall.J)
    fprintf('\n>>> Best height: z=%.2f | Cost=%.3f | Mean r=%.2f | Len=%.2f | Time = %.3f s\n', ...
        bestOverall.z, bestOverall.J, mean(bestOverall.r), bestOverall.len, timeTaken);
else
    fprintf('\nNo feasible circle-chain path found. Time = %.3f s\n', timeTaken);
end

% ---------- Optional 3D views ----------
if ~computeOnly && isfinite(bestOverall.J)
    % Fig A: Full 3D environment only
    fig3A = figure('Name','3D Env — No Circles','Units','normalized', ...
                   'OuterPosition',[0.03 0.08 0.45 0.84], 'Renderer','opengl');
    axA = axes(fig3A); hold(axA,'on'); grid(axA,'on'); view(axA,45,25);
    axis(axA,'equal'); axis(axA,'vis3d');
    xlabel(axA,'X (m)'); ylabel(axA,'Y (m)'); zlabel(axA,'Z (m)');
    title(axA,'Full 3D Environment');
    drawEnv3D(axA, env, 0.35);
    fixFloorShading(axA, "nolight");
    default3D(axA);

    % Fig B: Full 3D environment + candidate slices + selected best chain
    fig3B = figure('Name','3D Env — Candidate Slices + Best Circle-Chain','Units','normalized', ...
                   'OuterPosition',[0.52 0.08 0.45 0.84], 'Renderer','opengl');
    axB = axes(fig3B); hold(axB,'on'); grid(axB,'on'); view(axB,45,25);
    axis(axB,'equal'); axis(axB,'vis3d');
    xlabel(axB,'X (m)'); ylabel(axB,'Y (m)'); zlabel(axB,'Z (m)');
    title(axB, sprintf('Best chain @ z = %.2f (Cost = %.3f, Len = %.2f)', ...
        bestOverall.z, bestOverall.J, bestOverall.len));

    drawEnv3D(axB, env, 0.35);
    fixFloorShading(axB, "nolight");
    drawHeightSlices3D(axB, [xlimW; ylimW], z_list, bestOverall.z);

    zBest = bestOverall.z;
    plot3(axB, start_xy(1), start_xy(2), zBest, 'go', 'MarkerFaceColor','g', 'MarkerSize',7, 'LineWidth',1.2);
    plot3(axB, goal_xy(1),  goal_xy(2),  zBest, 'co', 'MarkerFaceColor','c', 'MarkerSize',6);
    if ~isempty(bestOverall.path)
        drawCircleChain3D(axB, bestOverall.path, bestOverall.r, zBest, ...
            'circleColor',[0 0.45 0.74], 'tubeRadius',0.008, 'tubeAlpha',0.95);
        plot3(axB, bestOverall.path(:,1), bestOverall.path(:,2), ...
              zBest*ones(size(bestOverall.path,1),1), '-', ...
              'Color',[1 0 0], 'LineWidth',2.5);
    end
    default3D(axB);

    % Fig C: same as Fig B, but without the best circle-chain overlay
    fig3C = figure('Name','3D Env - Candidate Slices Only','Units','normalized', ...
                   'OuterPosition',[0.56 0.12 0.40 0.78], 'Renderer','opengl');
    axC = axes(fig3C); hold(axC,'on'); grid(axC,'on'); view(axC,45,25);
    axis(axC,'equal'); axis(axC,'vis3d');
    xlabel(axC,'X (m)'); ylabel(axC,'Y (m)'); zlabel(axC,'Z (m)');
    title(axC, sprintf('Candidate slices @ z = %.2f (no circle-chain)', bestOverall.z));

    drawEnv3D(axC, env, 0.35);
    fixFloorShading(axC, "nolight");
    drawHeightSlices3D(axC, [xlimW; ylimW], z_list, bestOverall.z);

    plot3(axC, start_xy(1), start_xy(2), zBest, 'go', 'MarkerFaceColor','g', 'MarkerSize',7, 'LineWidth',1.2);
    plot3(axC, goal_xy(1),  goal_xy(2),  zBest, 'co', 'MarkerFaceColor','c', 'MarkerSize',6);
    default3D(axC);
end

% ---------- Outputs ----------
bestZ = bestOverall.z;

out = struct();
if isfinite(bestOverall.J)
    out.best = struct( ...
        'chainXY', bestOverall.path, ...
        'radii',   bestOverall.r(:), ...
        'nodes',   bestOverall.nodes, ...
        'edges',   bestOverall.edges, ...
        'z',       bestOverall.z, ...
        'len',     bestOverall.len, ...
        'cost',    bestOverall.J );
else
    out.best = struct('chainXY',[], 'radii',[], 'nodes',[], 'edges',[], ...
                      'z',NaN, 'len',NaN, 'cost',Inf);
end

out.meta = struct('bounds',[xlimW; ylimW], 'res',res, 'time',timeTaken, ...
                  'z_list', diag.z_list, 'costPerZ', diag.costPerZ, ...
                  'hasPath', diag.hasPath);

out.flags = struct('heightInvariant', isHeightInvariant, ...
                   'flatReason', 'flat_cost_across_z', ...
                   'costSpread', diag.costSpread, ...
                   'tolAbs', flatTolAbs, 'tolRel', flatTolRel);

% ====================== END MAIN FUNCTION ================================
end

%% ====================== Helpers (env + geometry) ========================
function envStruct = wrapEnvToStruct(envIn)
% Normalize env into {struct('Name',..., 'CollisionObj',...)} cell
    if isempty(envIn), envStruct = {}; return; end
    if ~iscell(envIn), envIn = num2cell(envIn); end
    envStruct = cell(size(envIn));
    for i = 1:numel(envIn)
        e = envIn{i};
        if isstruct(e) && isfield(e,'CollisionObj')
            if ~isfield(e,'Name') || isempty(e.Name)
                e.Name = guessObjName(e.CollisionObj);
            end
            envStruct{i} = e;
        else
            envStruct{i} = struct('Name', guessObjName(e), 'CollisionObj', e);
        end
    end
end

function obj = getObj(entry)
    if isstruct(entry) && isfield(entry,'CollisionObj')
        obj = entry.CollisionObj;
    else
        obj = entry;
    end
end

function name = guessObjName(obj)
    if isa(obj,'collisionBox'),           name = "box";
    elseif isa(obj,'collisionCylinder'),  name = "cylinder";
    elseif isa(obj,'collisionSphere'),    name = "sphere";
    elseif isa(obj,'collisionCapsule'),   name = "capsule";
    elseif isa(obj,'collisionMesh'),      name = "mesh";
    else,                                 name = "object";
    end
end

function [occ,xs,ys] = sliceEnv2D(env, xlimW, ylimW, res, z)
xs = xlimW(1):res:xlimW(2);
ys = ylimW(1):res:ylimW(2);
[XX,YY] = meshgrid(xs,ys);
ZZ = z*ones(size(XX));
occ = false(size(XX));

for k=1:numel(env)
    obj = getObj(env{k});
    if isempty(obj), continue; end
    T = obj.Pose;
    P = [XX(:),YY(:),ZZ(:),ones(numel(XX),1)]';
    Pl = T \ P;  xl=Pl(1,:); yl=Pl(2,:); zl=Pl(3,:);

    inside = false(1,numel(xl));
    if     isa(obj,'collisionBox')
        inside = (abs(xl)<=obj.X/2) & (abs(yl)<=obj.Y/2) & (abs(zl)<=obj.Z/2);
    elseif isa(obj,'collisionCylinder')
        inside = (xl.^2+yl.^2<=obj.Radius^2) & (abs(zl)<=obj.Length/2);
    elseif isa(obj,'collisionSphere')
        inside = (xl.^2+yl.^2+zl.^2<=obj.Radius^2);
    elseif isa(obj,'collisionMesh')
        if ~isempty(obj.Vertices)
            vmin=min(obj.Vertices,[],1); vmax=max(obj.Vertices,[],1);
            inside=(xl>=vmin(1)&xl<=vmax(1)&yl>=vmin(2)&yl<=vmax(2)&zl>=vmin(3)&zl<=vmax(3));
        end
    end
    occ = occ | reshape(inside,size(XX));
end
end

function D = distFromOcc(occ, res)
if exist('bwdist','file')==2
    D = bwdist(occ) * res; D(occ) = 0;
else
    [iy,ix] = find(occ);
    if isempty(ix), D = inf(size(occ)); return; end
    xs = 1:size(occ,2); ys = 1:size(occ,1);
    [XX,YY]=meshgrid(xs,ys); D = zeros(size(occ));
    for r=1:numel(xs)
        for c=1:numel(ys)
            if occ(c,r), D(c,r)=0; else
                dx = XX(c,r)-ix; dy = YY(c,r)-iy;
                D(c,r) = sqrt(min(dx.^2+dy.^2))*res;
            end
        end
    end
end
end

function [gi,gj] = worldToGridSnap(xs,ys,res,occ,x,y)
gj = round((x - xs(1))/res)+1;
gi = round((y - ys(1))/res)+1;
gi = max(1,min(size(occ,1),gi));
gj = max(1,min(size(occ,2),gj));
if occ(gi,gj), [gi,gj] = snapToNearestFree(occ, gi, gj); end
end

function [oi,oj] = snapToNearestFree(occ, i, j)
[H,W] = size(occ);
if ~occ(i,j), oi=i; oj=j; return; end
vis=false(H,W); vis(i,j)=true; Q=[i j];
NBR=[-1 0;1 0;0 -1;0 1;-1 -1;-1 1;1 -1;1 1];
while ~isempty(Q)
    qi=Q(1,1); qj=Q(1,2); Q(1,:)=[]; %#ok<AGROW>
    for k=1:8
        ni=qi+NBR(k,1); nj=qj+NBR(k,2);
        if ni<1||ni>H||nj<1||nj>W||vis(ni,nj), continue; end
        if ~occ(ni,nj), oi=ni; oj=nj; return; end
        vis(ni,nj)=true; Q(end+1,:)=[ni nj]; %#ok<AGROW>
    end
end
oi=i; oj=j;
end

function [gi,gj] = worldToGrid(xs,ys,x,y)
gj = round( (x - xs(1)) / (xs(2)-xs(1)) ) + 1;
gi = round( (y - ys(1)) / (ys(2)-ys(1)) ) + 1;
end

function r = interpRstar(p, xs, ys, D, rmax)
[gi,gj,wi,wj] = bilinearWeights(xs,ys,p(1),p(2));
if any(isnan([gi gj])), r=-inf; return; end
i1=gi; i2=min(gi+1,size(D,1)); j1=gj; j2=min(gj+1,size(D,2));
v11=D(i1,j1); v12=D(i1,j2); v21=D(i2,j1); v22=D(i2,j2);
r=(1-wi)*(1-wj)*v11 + (1-wi)*wj*v12 + wi*(1-wj)*v21 + wi*wj*v22;
r=min(r,rmax);
end

function [gi,gj,wi,wj] = bilinearWeights(xs,ys,x,y)
if x<xs(1) || x>xs(end) || y<ys(1) || y>ys(end)
    gi=NaN; gj=NaN; wi=NaN; wj=NaN; return;
end
dx=xs(2)-xs(1); dy=ys(2)-ys(1);
gjf=(x-xs(1))/dx + 1; gif=(y-ys(1))/dy + 1;
gj=max(1,min(numel(xs)-1,floor(gjf)));
gi=max(1,min(numel(ys)-1,floor(gif)));
wj=gjf-gj; wi=gif-gi;
end

%% ---- buildCircleGraph with adaptive sampling ---------------------------
function [nodes, edges] = buildCircleGraph(xs, ys, D, occ, bounds, ...
                                           rmin_hard, rmin_soft, rmax, ...
                                           stepS, margin, s_snap, g_snap, maxNbr, adapt)
nodes = struct('pos', {}, 'r', {}, 'isSG', {});
edges = [];

% Candidate centers: adaptive or uniform
if adapt.enabled
    candCenters = sampleCentersAdaptive(xs,ys,D,occ,bounds,stepS,adapt);
else
    [Xg,Yg] = meshgrid(bounds(1,1):stepS:bounds(1,2), bounds(2,1):stepS:bounds(2,2));
    candCenters = [Xg(:), Yg(:)];
end

% Query function
getRstar = @(p) interpRstar(p, xs, ys, D, rmax);

% Keep centers with clearance
for i=1:size(candCenters,1)
    c = candCenters(i,:);
    [gi,gj] = worldToGrid(xs,ys,c(1),c(2));
    if gi<1 || gi>size(occ,1) || gj<1 || gj>size(occ,2), continue; end
    if occ(gi,gj), continue; end
    rStar = getRstar(c);
    if isfinite(rStar) && rStar >= rmin_hard
        nodes(end+1) = struct('pos',c,'r',min(rStar,rmax),'isSG',false); %#ok<AGROW>
    end
end

% Add endpoints (soft snap if needed)
nodes = addEndpointNode_soft(nodes, s_snap, getRstar, rmin_hard, rmax);
nodes = addEndpointNode_soft(nodes, g_snap, getRstar, rmin_hard, rmax);

if numel(nodes) < 2, edges=[]; return; end

% Build edges via grid hashing neighborhood
posM = vertcat(nodes.pos); rM = vertcat(nodes.r); N = size(posM,1);
cellSz = max(stepS, 0.5*mean([rmin_hard rmin_soft]));
[cellId, cellMap] = buildCellHash(posM, cellSz);

for i=1:N
    cand = candidateNeighbors(i,posM,rM,cellId,cellMap,maxNbr);
    ci=posM(i,:); ri=rM(i);
    for j=cand(:).'
        if j<=i, continue; end
        cj=posM(j,:); rj=rM(j);
        dij=hypot(ci(1)-cj(1),ci(2)-cj(2));
        if dij > (ri+rj-1e-6), continue; end
        if ~segmentSafe(xs,ys,D,occ,ci,cj,margin), continue; end
        edges(end+1,:)=[i j dij]; %#ok<AGROW>
    end
end
end

%% ---- Adaptive sampling helper ------------------------------------------
function centers = sampleCentersAdaptive(xs, ys, D, occ, bounds, stepS, adapt)
dx = adapt.resS;
[xg, yg] = meshgrid(bounds(1,1):dx:bounds(1,2), bounds(2,1):dx:bounds(2,2));
Pg = [xg(:), yg(:)];

% Free mask
gi = max(1,min(numel(ys), round((Pg(:,2)-ys(1))/(ys(2)-ys(1))+1)));
gj = max(1,min(numel(xs), round((Pg(:,1)-xs(1))/(xs(2)-xs(1))+1)));
maskIn = Pg(:,1)>=xs(1)&Pg(:,1)<=xs(end)&Pg(:,2)>=ys(1)&Pg(:,2)<=ys(end);
maskFree = maskIn & ~occ(sub2ind(size(occ),gi,gj));
Pg = Pg(maskFree,:); gi=gi(maskFree); gj=gj(maskFree);

if isempty(Pg), centers=Pg; return; end

Dg = D(sub2ind(size(D),gi,gj));
w  = 1 ./ max(1e-6,Dg).^adapt.beta;

[~,ord]=sort(w,'descend'); Pg=Pg(ord,:); Dg=Dg(ord);
localSep = max(adapt.minSep, adapt.alpha*Dg);

cellSz=adapt.minSep; mins=min(Pg,[],1);
cellId=@(P) floor((P-mins)./cellSz);
keyfn=@(c) c(:,1)*73856093 + c(:,2)*19349663;

centers=zeros(0,2); buckets=containers.Map('KeyType','int64','ValueType','any');

for k=1:size(Pg,1)
    q=Pg(k,:); s_q=localSep(k);
    cid=cellId(q); closePts=[];
    for dx=-1:1, for dy=-1:1
        nb=cid+[dx dy]; key=int64(keyfn(nb));
        if isKey(buckets,key), closePts=[closePts; buckets(key)]; end
    end, end
    ok=true;
    if ~isempty(closePts)
        d=sqrt(sum((closePts-q).^2,2));
        if any(d<s_q), ok=false; end
    end
    if ok
        centers(end+1,:)=q; %#ok<AGROW>
        key=int64(keyfn(cid));
        if isKey(buckets,key), buckets(key)=[buckets(key); q];
        else, buckets(key)=q; end
        if size(centers,1)>=adapt.maxPts, break; end
    end
end

if isempty(centers)
    [Xg,Yg]=meshgrid(bounds(1,1):stepS:bounds(1,2),bounds(2,1):stepS:bounds(2,2));
    cand=[Xg(:),Yg(:)];
    gi=max(1,min(numel(ys), round((cand(:,2)-ys(1))/(ys(2)-ys(1))+1)));
    gj=max(1,min(numel(xs), round((cand(:,1)-xs(1))/(xs(2)-xs(1))+1)));
    maskFree=~occ(sub2ind(size(occ),gi,gj));
    centers=cand(maskFree,:);
end
end

function nodes = addEndpointNode_soft(nodes,p,getRstar,rmin_hard,rmax)
rStar = getRstar(p);
if ~isfinite(rStar) || rStar < rmin_hard
    rho = linspace(0.0, 0.6*max(rmin_hard,0.2), 7);
    ang = linspace(0, 2*pi, 16);
    ok = false; best = -inf; pbest = p; rbest = NaN;
    for rr = rho
        for aa = ang
            q = p + rr*[cos(aa) sin(aa)];
            r = getRstar(q);
            if isfinite(r) && r >= rmin_hard && r > best
                best = r; pbest = q; rbest = r; ok = true;
            end
        end
    end
    if ok
        nodes(end+1) = struct('pos',pbest,'r',min(rbest,rmax),'isSG',true);
    else
        nodes(end+1) = struct('pos',p,'r',max(1e-3,min(rStar,rmax)),'isSG',true);
    end
else
    nodes(end+1) = struct('pos',p,'r',min(rStar,rmax),'isSG',true);
end
end

function [cellId, cellMap] = buildCellHash(P, cellSz)
mins = min(P,[],1);
cellId = floor((P - mins) ./ cellSz);
keys   = cellId(:,1)*73856093 + cellId(:,2)*19349663;
[cellMap.keys,~,ic] = unique(keys);
cellMap.bucket = cell(numel(cellMap.keys),1);
for i=1:numel(ic)
    cellMap.bucket{ic(i)}(end+1) = i; %#ok<AGROW>
end
end

function nbrs = candidateNeighbors(i, P, ~, cellId, cellMap, maxNbr)
cid = cellId(i,:);
S=[];
for dx=-1:1
for dy=-1:1
    c=cid+[dx dy];
    key=c(1)*73856093 + c(2)*19349663;
    kIdx=find(cellMap.keys==key,1);
    if ~isempty(kIdx), S=[S, cellMap.bucket{kIdx}]; end %#ok<AGROW>
end
end
S=unique(S); S(S==i)=[];
if ~isempty(maxNbr) && numel(S)>maxNbr
    dij = vecnorm((P(S,:) - P(i,:)).',2,1);
    [~,ord]=sort(dij,'ascend'); S=S(ord(1:maxNbr));
end
nbrs=S;
end

function tf = segmentSafe(xs,ys,D,occ,a,b,margin)
nSamp = max(10, ceil( 2*hypot(b(1)-a(1), b(2)-a(2)) / max(1e-6, xs(2)-xs(1)) ));
t = linspace(0,1,nSamp).';
P = a.*(1-t) + b.*t;
tf = true;
for k=1:nSamp
    p=P(k,:);
    [gi,gj] = worldToGrid(xs,ys,p(1),p(2));
    if gi<1||gi>size(occ,1)||gj<1||gj>size(occ,2), tf=false; return; end
    if occ(gi,gj), tf=false; return; end
    if margin>0
        d = interpRstar(p,xs,ys,D,inf);
        if ~isfinite(d) || d<margin, tf=false; return; end
    end
end
end

function drawObstacles(ax, env, z, faceCol, faceAlpha, edgeCol)
if nargin<4 || isempty(faceCol),   faceCol = [0.95 0.55 0.15]; end
if nargin<5 || isempty(faceAlpha), faceAlpha = 0.35; end
if nargin<6 || isempty(edgeCol),   edgeCol = [0.55 0.28 0.08]; end

polys = slicePolys(env, z);
for k=1:numel(polys)
    P = polys{k};
    patch('Parent',ax, ...
          'XData',P(:,1),'YData',P(:,2), ...
          'FaceColor',faceCol,'FaceAlpha',faceAlpha, ...
          'EdgeColor',edgeCol,'LineWidth',1.25, ...
          'Tag','geom','HitTest','off','PickableParts','none');
end
end

function polys = slicePolys(env, z)
polys = {};
for i=1:numel(env)
    obj = getObj(env{i});
    if ~isa(obj,'collisionBox'), continue; end
    [P,ok] = intersect_obb_z_plane(obj.Pose, obj.X, obj.Y, obj.Z, z);
    if ok, polys{end+1} = P; end %#ok<AGROW>
end
end

function [verts2d, intersects] = intersect_obb_z_plane(T_W_box, L, W, H, z_slice)
verts2d=[]; intersects=false;
[X,Y,Z] = ndgrid([-L/2 L/2], [-W/2 W/2], [-H/2 H/2]);
V_local = [X(:),Y(:),Z(:),ones(8,1)]';
V_world = (T_W_box * V_local)';
zmin=min(V_world(:,3)); zmax=max(V_world(:,3));
if z_slice<zmin-1e-9 || z_slice>zmax+1e-9, return; end
edges = [1 2;2 4;4 3;3 1; 5 6;6 8;8 7;7 5; 1 5;2 6;3 7;4 8];
P=[];
for e=1:size(edges,1)
    p1 = V_world(edges(e,1),1:3);
    p2 = V_world(edges(e,2),1:3);
    if (p1(3)-z_slice)*(p2(3)-z_slice)<0
        t=(z_slice-p1(3))/(p2(3)-p1(3));
        P=[P; p1 + t*(p2-p1)]; %#ok<AGROW>
    end
end
mask = abs(V_world(:,3)-z_slice)<1e-9;
P=[P; V_world(mask,1:3)];
if size(P,1)<3, return; end
Q = unique(round(P(:,1:2),6),'rows','stable');
if size(Q,1)<3, return; end
c = mean(Q,1);
ang = atan2(Q(:,2)-c(2), Q(:,1)-c(1));
[~,ord]=sort(ang);
verts2d=Q(ord,:); intersects=true;
end

%% ======================== Graph drawing / A* ============================
function drawCircleGraph(ax, nodes, edges, opts)
if nargin < 4, opts = struct(); end
defaults = struct( ...
    'edgeAlpha',0.25,'edgeLineW',0.75, ...
    'drawAllEdges',false,'maxEdgesToDraw',2000, ...
    'drawAllCircles',false,'maxCirclesToDraw',150, ...
    'centerMarkerMin',6,'centerMarkerMax',18, ...
    'colStart',[0.85 0.10 0.10], 'colGoal',[0.10 0.70 0.90], ...
    'colNode',[0.25 0.25 0.25], 'colEdge',[0.10 0.10 0.10], ...
    'colCirc',[0.65 0.65 0.65]);
fns = fieldnames(defaults);
for i=1:numel(fns), if ~isfield(opts,fns{i}), opts.(fns{i})=defaults.(fns{i}); end, end

C = vertcat(nodes.pos); R = vertcat(nodes.r); N = size(C,1);

% Edges
if ~isempty(edges)
    if opts.drawAllEdges, idxE = 1:size(edges,1);
    else, M = min(opts.maxEdgesToDraw, size(edges,1)); idxE = randperm(size(edges,1), M);
    end
    XY = nan(3*numel(idxE),2); p=1;
    for k=idxE
        i=edges(k,1); j=edges(k,2);
        XY(p,:)   = C(i,:); XY(p+1,:) = C(j,:); XY(p+2,:)=[NaN NaN]; p=p+3;
    end
    plot(ax, XY(:,1), XY(:,2), '-', 'Color', opts.colEdge, ...
        'LineWidth',opts.edgeLineW, 'HandleVisibility','off', ...
        'HitTest','off','PickableParts','none');
end

% Circles (subset, large-first)
if opts.drawAllCircles, idxCirc=1:N;
else, M=min(opts.maxCirclesToDraw,N); [~,ord]=sort(R,'descend'); idxCirc=sort(ord(1:M));
end
t = linspace(0,2*pi,64)';
for k=idxCirc(:).'
    r=R(k); if ~isfinite(r) || r<=0, continue; end
    xy = C(k,:) + r*[cos(t) sin(t)];
    plot(ax, xy(:,1), xy(:,2), '-', 'Color',opts.colCirc, 'LineWidth',1.0, ...
        'HandleVisibility','off','HitTest','off','PickableParts','none');
end

% Centers by radius
rmin=min(R); rmax=max(R); span=max(1e-6,rmax-rmin);
sz = opts.centerMarkerMin + (opts.centerMarkerMax-opts.centerMarkerMin)*(R-rmin)/span;

sg = find([nodes.isSG]); maskSG=false(N,1); maskStart=false(N,1); maskGoal=false(N,1);
if numel(sg)>=1, maskStart(sg(1))=true; maskSG(sg(1))=true; end
if numel(sg)>=2, maskGoal(sg(2))=true;  maskSG(sg(2))=true; end

maskNodes = ~maskSG;
if any(maskNodes)
    scatter(ax, C(maskNodes,1),C(maskNodes,2), sz(maskNodes), ...
        'MarkerFaceColor',opts.colNode,'MarkerEdgeColor','none','MarkerFaceAlpha',0.65, ...
        'HandleVisibility','off','HitTest','off','PickableParts','none');
end
if any(maskStart)
    scatter(ax, C(maskStart,1),C(maskStart,2), max(sz(maskStart),opts.centerMarkerMax), ...
        'MarkerFaceColor',opts.colStart,'MarkerEdgeColor','k','LineWidth',0.6, ...
        'DisplayName','Start circle','HitTest','off','PickableParts','none');
end
if any(maskGoal)
    scatter(ax, C(maskGoal,1),C(maskGoal,2), max(sz(maskGoal),opts.centerMarkerMax), ...
        'MarkerFaceColor',opts.colGoal,'MarkerEdgeColor','k','LineWidth',0.6, ...
        'DisplayName','Goal circle','HitTest','off','PickableParts','none');
end
end

function [idxPath, J] = astarCircleChain(nodes, edges, W, rmin_soft)
N = numel(nodes);
adj = cell(N,1); len = cell(N,1);
for e = 1:size(edges,1)
    i = edges(e,1); j = edges(e,2); L = edges(e,3);
    adj{i}(end+1) = j; len{i}(end+1) = L;
    adj{j}(end+1) = i; len{j}(end+1) = L;
end
sg = find([nodes.isSG]); if numel(sg) < 2, idxPath=[]; J=inf; return; end
sIdx = sg(1); gIdx = sg(2);

rAll = [nodes.r]; rgmax=max(rAll); rgmin=min(rAll); rSpan=max(1e-6, rgmax-rgmin);
hfun = @(i) W.alpha*norm(nodes(i).pos - nodes(gIdx).pos) + ...
            W.beta * max(0, (rgmax - nodes(i).r) / rSpan);

g = inf(1,N); f = inf(1,N); parent = zeros(1,N,'uint32'); prevAng = nan(1,N);
g(sIdx)=0; f(sIdx)=hfun(sIdx);
open=false(1,N); open(sIdx)=true; closed=false(1,N);

while any(open)
    openIdx = find(open);
    [~,k] = min(f(openIdx));
    cur = openIdx(k); open(cur)=false; closed(cur)=true;
    if cur==gIdx, break; end

    ci = nodes(cur).pos;
    for t=1:numel(adj{cur})
        nb = adj{cur}(t); if closed(nb), continue; end
        L = len{cur}(t);
        ri = nodes(cur).r; rj = nodes(nb).r;

        sizeBias    = W.beta * max(0, (rgmax - 0.5*(ri+rj)) / rSpan);
        belowSoft   = max(0, (rmin_soft - min(ri,rj)));
        softPenalty = W.rpen * (belowSoft / max(1e-6, rmin_soft));
        base = W.alpha*L + sizeBias + W.gamma + softPenalty;
        if W.rdrop>0, base = base + W.rdrop * max(0, (ri - rj)); end

        sCost=0;
        if ~isnan(prevAng(cur))
            v = nodes(nb).pos - ci; ang = atan2(v(2),v(1));
            dtheta = wrapToPiLocal(ang - prevAng(cur));
            sCost = W.delta * (1 - cos(dtheta));
        end

        candG = g(cur) + base + sCost;
        if candG < g(nb)
            g(nb)=candG; parent(nb)=cur; f(nb)=g(nb)+hfun(nb);
            open(nb)=true; v = nodes(nb).pos - ci; prevAng(nb)=atan2(v(2),v(1));
        end
    end
end

if ~isfinite(g(gIdx)), idxPath=[]; J=inf; return; end
idxPath = gIdx;
while idxPath(1)~=sIdx
    p = parent(idxPath(1)); if p==0, idxPath=[]; J=inf; return; end
    idxPath = [p idxPath]; %#ok<AGROW>
end
J = g(gIdx);
end

function ang = wrapToPiLocal(ang)
ang = mod(ang + pi, 2*pi) - pi;
end

%% ======================== NEW 3D helpers ================================
function default3D(ax)
fig = ancestor(ax,'figure');
set(fig,'Renderer','opengl');
axis(ax,'equal'); axis(ax,'vis3d');
grid(ax,'on'); box(ax,'on');
view(ax, 45, 25);
camproj(ax,'perspective');

delete(findall(ax,'Type','light'));
lighting(ax,'gouraud');
l1 = camlight(ax,'headlight');
set(l1,'Color',[1 1 1]);

p = findall(ax,'Type','patch');
set(p,'FaceLighting','gouraud', ...
      'SpecularStrength',0.25, ...
      'SpecularExponent',20, ...
      'SpecularColorReflectance',0.4);

ln = findall(ax,'Type','line');
set(ln,'LineWidth',1.5);
end

function drawEnv3D(ax, env, alphaVal)
if nargin<3 || isempty(alphaVal), alphaVal = 0.35; end
for k=1:numel(env)
    obj = getObj(env{k});
    if isempty(obj), continue; end
    try
        h = show(obj, 'Parent', ax);
        if isgraphics(h)
            try, set(h, 'FaceAlpha', alphaVal); catch, end
        end
    catch
        if isa(obj,'collisionBox')
            drawBoxFallback(ax, obj.Pose, obj.X, obj.Y, obj.Z, alphaVal);
        end
    end
end
end

function drawBoxFallback(ax, T, L, W, H, a)
[X,Y,Z] = ndgrid([-L/2 L/2], [-W/2 W/2], [-H/2 H/2]);
V = [X(:),Y(:),Z(:),ones(8,1)] * T';
V = V(:,1:3);
F = [1 3 4 2; 5 7 8 6; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
patch('Parent',ax,'Vertices',V,'Faces',F, ...
    'FaceColor',[0.6 0.6 0.6],'EdgeColor',[0.25 0.25 0.25], ...
    'FaceAlpha',a,'LineWidth',0.75);
end

function drawCircleChain3D(ax, centersXY, radii, zConst, varargin)
p = inputParser;
addParameter(p,'circleColor',[1 1 0]);
addParameter(p,'tubeRadius',0.008);
addParameter(p,'tubeAlpha',0.9);
parse(p,varargin{:});
cc = p.Results.circleColor;
tR = p.Results.tubeRadius;
a  = p.Results.tubeAlpha;

t = linspace(0,2*pi,96).';
for i=1:numel(radii)
    r = radii(i); if ~isfinite(r) || r<=0, continue; end
    c = centersXY(i,:);
    xy_in  = c + max(r - tR, 0)*[cos(t) sin(t)];
    xy_out = c + (r + tR)*[cos(t) sin(t)];
    v = [xy_out, zConst*ones(size(xy_out,1),1); ...
         flipud([xy_in,  zConst*ones(size(xy_in,1),1)])];
    f = [1:size(xy_out,1), (size(xy_out,1)+1):(size(xy_out,1)+size(xy_in,1))];
    patch('Parent',ax,'Vertices',v,'Faces',f, ...
          'FaceColor',cc,'EdgeColor','none','FaceAlpha',a, 'Tag','circleChain');
    xy = c + r*[cos(t) sin(t)];
    plot3(ax, xy(:,1), xy(:,2), zConst*ones(size(xy,1),1), '-', ...
          'Color',[cc*0.8], 'LineWidth',0.75);
end
end

function drawHeightSlices3D(ax, bounds, zVals, zSelected)
if isempty(zVals), return; end

xlimW = bounds(1,:);
ylimW = bounds(2,:);

X = [xlimW(1) xlimW(2) xlimW(2) xlimW(1)];
Y = [ylimW(1) ylimW(1) ylimW(2) ylimW(2)];
tol = 1e-9;

for k = 1:numel(zVals)
    z = zVals(k);
    isSel = abs(z - zSelected) < tol;

    if isSel
        faceCol = [1.00 0.62 0.15];
        edgeCol = [0.80 0.30 0.05];
        faceA   = 0.28;
        lineW   = 1.6;
    else
        faceCol = [0.75 0.75 0.75];
        edgeCol = 'none';
        faceA   = 0.08;
        lineW   = 0.5;
    end

    patch('Parent', ax, ...
          'XData', X, ...
          'YData', Y, ...
          'ZData', z * ones(1,4), ...
          'FaceColor', faceCol, ...
          'FaceAlpha', faceA, ...
          'EdgeColor', edgeCol, ...
          'LineWidth', lineW, ...
          'Tag', 'slicePlane', ...
          'HandleVisibility', 'off', ...
          'HitTest', 'off', ...
          'PickableParts', 'none');

    % Draw dashed perimeter for all planes
    if isSel
        dashColor = edgeCol;
        dashWidth = 2;
    else
        dashColor = [0.5 0.5 0.5];
        dashWidth = 1;
    end
    plot3(ax, [X(1) X(2)], [Y(1) Y(1)], [z z], '--', 'Color', dashColor, 'LineWidth', dashWidth);
    plot3(ax, [X(2) X(2)], [Y(1) Y(3)], [z z], '--', 'Color', dashColor, 'LineWidth', dashWidth);
    plot3(ax, [X(3) X(4)], [Y(3) Y(3)], [z z], '--', 'Color', dashColor, 'LineWidth', dashWidth);
    plot3(ax, [X(4) X(4)], [Y(4) Y(1)], [z z], '--', 'Color', dashColor, 'LineWidth', dashWidth);
end

text(ax, xlimW(2), ylimW(2), zSelected, ...
     sprintf('  z* = %.2f m', zSelected), ...
     'Color', [0.80 0.25 0.05], ...
     'FontWeight', 'bold', ...
     'FontSize', 16, ...
     'HorizontalAlignment', 'left', ...
     'VerticalAlignment', 'bottom');
end

function fixFloorShading(ax, mode)
if nargin<2, mode = "nolight"; end
P = findall(ax,'Type','patch');
if isempty(P), return; end

bestIdx = [];
bestScore = -inf;
for i = 1:numel(P)
    v = get(P(i),'Vertices'); f = get(P(i),'Faces');
    if isempty(v) || isempty(f), continue; end
    tri = v(f(1,1:3),:);
    n = cross(tri(2,:)-tri(1,:), tri(3,:)-tri(1,:));
    if norm(n)==0, continue; end
    n = n./norm(n);
    hz = abs(n(3));
    A = 0;
    for t=1:size(f,1)
        tt = v(f(t,1:3),:);
        A = A + 0.5*norm(cross(tt(2,:)-tt(1,:), tt(3,:)-tt(1,:)));
    end
    score = hz * A;
    if score > bestScore
        bestScore = score; bestIdx = i;
    end
end
if isempty(bestIdx), return; end
floorPatch = P(bestIdx);

switch lower(mode)
    case "nolight"
        set(floorPatch, 'FaceLighting','none', 'EdgeColor','none');
        if ~isnan(get(floorPatch,'FaceAlpha')), set(floorPatch,'FaceAlpha',0.35); end
    case "flat"
        set(floorPatch, 'FaceLighting','flat', 'SpecularStrength',0);
    otherwise
        % do nothing
end
end

function h = drawCircles(ax, centers, radii, color, tag, lineW)
if nargin < 5 || isempty(tag),   tag = '';    end
if nargin < 6 || isempty(lineW), lineW = 1.2; end
t = linspace(0,2*pi,64)';
color = reshape(color,1,[]);
h = gobjects(0);
for k = 1:numel(radii)
    r = radii(k); if ~isfinite(r) || r<=0, continue; end
    c = centers(k,:);
    xy = c + r*[cos(t) sin(t)];
    h(end+1,1) = plot(ax, xy(:,1), xy(:,2), '-', 'Color', color, ...
        'LineWidth', lineW, 'Tag', tag, 'HitTest','off','PickableParts','none'); %#ok<AGROW>
end
end
