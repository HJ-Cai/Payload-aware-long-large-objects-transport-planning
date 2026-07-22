%% UR20 Mobile Pick & Place with HS-Bi-RRT
%% ========================================================================
%  1. INITIALIZATION AND SETUP
%  ========================================================================
clear;
clc; 
close all;
rng(1, 'twister');                 % fixed seed for a reproducible single run
runID = 1;
% The envelope is always displayed. This switch controls only whether
% Plan 4 collision checking considers it. Keep false for published runs.
useRebarEnvelope = false;

% --- Load Robot and Environment ---
[mobileUR20, startConfig, envS, pickPose, placePose, ~, rebarMeta] = ...
    loadRebarTransportScenario;
fprintf('\n==========================================\n');
fprintf('   HS-Bi-RRT: single simulation run');
fprintf('\n==========================================\n');
fprintf('Using default start configuration from the helper function.\n');
% Create a 20m x 20m floor, 10cm thick
floorObj = collisionBox(12, 12, 0.1); 

% Position it so the top surface is exactly at Z = 0
% (Center is at Z = -0.05)
floorObj.Pose = trvec2tform([0, 0, -0.05]); 

% Add to envS (for visualization) and envCol (for collision checking)
envS = [envS, {struct('Name','Floor','CollisionObj',floorObj)}];
envCol = cellfun(@(s) s.CollisionObj, envS, 'uni', false);

fprintf('[Setup] Added physical floor to envCol. Top surface at Z=0.\n');
% --- Configure Mobile Base Joints ---
bx = getBody(mobileUR20, "base_x");  bx.Joint.PositionLimits = [-6 6];
by = getBody(mobileUR20, "base_y");  by.Joint.PositionLimits = [-6 6];
bz = getBody(mobileUR20, "chassis"); bz.Joint.PositionLimits = [-pi pi];
% --- Setup Visualization ---
figure("Name","Mobile UR20 Rebar Pick and Place Using RRT", ...
       "Units","normalized","OuterPosition",[0,0,1,1]);
show(mobileUR20, startConfig, "Visuals","off","Collisions","on");
title("Initial Mobile UR20 and Environment Setup");
hold on; grid on;
axis([-8 8 -8 6 -0.1 3]);
view(120, 25);
for i = 1:numel(envS), show(envS{i}.CollisionObj); end
camlight headlight; lighting gouraud; % Add lighting
% --- Initialize Solvers and Planners ---
ik = inverseKinematics('RigidBodyTree', mobileUR20);
weights = [0.1 0.1 0.1 1 1 1]; % Prioritize orientation
endEffector = 'tcp';
makePlanner = @(robot, env) configurePlanner(manipulatorRRT(robot, env)); % Used for Plan 1
allStates = [];
%% ========================================================================
%  2. DEFINE PICK POSES
%  ========================================================================
rebarIdx = find(cellfun(@(s) strcmpi(s.Name,'rebar'), envS), 1);
assert(~isempty(rebarIdx), 'Could not find rebar object named "rebar" in envS.');
rebarObj = envS{rebarIdx}.CollisionObj;
T_rebar_W = rebarObj.Pose;
UR20Meta = evalin('base','UR20Meta');
approachPose = pickPose;
graspPose    = UR20Meta.T_tcp_touch_world;
retreatPose  = approachPose;
fprintf('Calculating IK for pick poses...\n');
approachConfig = ik(endEffector, approachPose, weights, startConfig);
graspConfig    = ik(endEffector, graspPose,    weights, approachConfig);
% drawFrame(T_rebar_W, 'rebar', 0.15);
drawFrame(approachPose, 'approach', 0.12);
drawFrame(graspPose, 'pick', 0.12);
mustBeFree(mobileUR20, startConfig,    envCol, 'Start configuration is in collision.');
mustBeFree(mobileUR20, approachConfig, envCol, 'Approach pose results in a collision.');
%% ========================================================================
%  3. PLAN AND EXECUTE PICK SEQUENCE
%  ========================================================================
fprintf('Plan 1: Planning Home -> Approach...\n');
planner = makePlanner(mobileUR20, envCol); % Using standard RRT here
path1 = planOrRetry(planner, startConfig, approachConfig, 5);
traj1 = hs_interpPath(path1, 0.02);
assertPathCollisionFree(mobileUR20, traj1, envCol);
% for i = 1:size(traj1, 1)
%     show(mobileUR20, traj1(i,:), "PreservePlot", false, "Visuals","off","Collisions","on");
%     title("Plan 1: Moving to Approach Position"); drawnow;
% end
allStates = [allStates; traj1];
fprintf('Plan 2: Moving straight down to grasp...\n');
traj2 = hs_interpPath([approachConfig; graspConfig], 0.01);
% for i = 1:size(traj2, 1)
%     show(mobileUR20, traj2(i,:), "PreservePlot", false, "Visuals","off","Collisions","on");
%     title("Plan 2: Moving straight down to grasp"); drawnow;
% end
allStates = [allStates; traj2];
fprintf('Closing gripper & attaching rebar...\n');
grippedConfig = graspConfig;
% show(mobileUR20, grippedConfig, "PreservePlot", false, "Visuals","off","Collisions","on");
% title("Gripping the Rebar"); drawnow;
allStates = [allStates; grippedConfig];
rebarBody = rigidBody("rebar");
eeT = getTransform(mobileUR20, grippedConfig, endEffector);
setFixedTransform(rebarBody.Joint, eeT \ rebarObj.Pose);
rbCopy = rebarObj.copy; rbCopy.Pose = eye(4);
addCollision(rebarBody, rbCopy);
addBody(mobileUR20, rebarBody, endEffector); % mobileUR20 now has rebar attached
envS(rebarIdx) = [];
envCol = cellfun(@(s) s.CollisionObj, envS, 'uni', false);
fprintf('--- Predicted deflection ---\n');
diameter_mm = rebarMeta.DiameterM * 1000; length_m = rebarMeta.LengthM; gripPoint = rebarMeta.GripPoint;
fprintf('Rebar Ø = %.1f mm | Length = %.2f m | Grip = %s\n', diameter_mm, length_m, gripPoint);
deflection  = computeRebarDeflection(diameter_mm, length_m, gripPoint);
fprintf('Deflection = %.3f m (%.1f cm)\n\n', deflection, deflection*100);
rebarEnvelope = struct('displayAlways', true, ...
    'consideredInPlanning', useRebarEnvelope, ...
    'lengthFactor', 1.10, 'lengthM', 1.10*length_m, ...
    'heightM', max(deflection,0), 'widthM', rebarMeta.DiameterM);
fprintf('Rebar envelope checking: %s\n', onOff(useRebarEnvelope));
fprintf('Plan 3: Straight-up retreat...\n');
lift = deflection;
nSteps = max(10, ceil(lift / 0.01));
traj3 = zeros(nSteps+1, numel(grippedConfig));
T_start = getTransform(mobileUR20, grippedConfig, endEffector);
q_current = grippedConfig;
traj3(1,:) = q_current;
for k = 1:nSteps
    T_next = T_start; T_next(3,4) = T_next(3,4) + (lift * k / nSteps);
    q_next = ik(endEffector, T_next, weights, q_current);
    if any(isnan(q_next)), q_next = q_current; end
    traj3(k+1,:) = q_next; q_current = q_next;
end

retreatConfig = q_current;
% for i = 1:size(traj3, 1)
%     show(mobileUR20, traj3(i,:), "PreservePlot", false, "Visuals","off","Collisions","on");
%     title("Plan 3: Straight-up retreat"); drawnow;
% end
allStates = [allStates; traj3];
%% ========================================================================
%  4. DEFINE PLACE POSES
%  ========================================================================
% Define the final target pose first
targetPose = UR20Meta.T_tcp_place_touch_world;
% --- Calculate Offset Pose based on Target Pose ---
% The offset will be 500mm (0.5m) "in front" of the target, meaning
% 0.5m back along the target's Z-axis (which is the approach vector).
T_target = targetPose;
R_target = T_target(1:3, 1:3);   % Get the orientation matrix
P_target = T_target(1:3, 4);     % Get the position vector
% Get the Z-axis vector from the orientation matrix (it's the 3rd column)
Z_vec = R_target(:, 3);
% Calculate the new offset position by moving 0.5m along the *negative* Z-axis
P_offset = P_target - 0.2 * Z_vec;
% Create the new offsetPose: same orientation, new position
offsetPose = [R_target, P_offset;   
              0, 0, 0, 1];
% --- Visualization ---
drawFrame(offsetPose, 'offset', 0.12);
drawFrame(targetPose, 'target', 0.12);
%% ========================================================================
%  5. PLAN AND EXECUTE PLACE SEQUENCE (HS-Bi-RRT with PAPER COLLISION)
%  ========================================================================
fprintf('Plan 4: Planning Retreat -> Offset with HS-Bi-RRT (paper collision)...\n');
t_plan4p = tic;
t_plan4t = tic;

% --- 1) Resolve base joint indices ---
Sx  = resolveBaseIndices(mobileUR20);
ix  = Sx.baseIdx(1);
iy  = Sx.baseIdx(2);
iyaw = Sx.baseIdx(3);

% =========================================================================
% === SMART BASE SEEDING (XY + YAW) ======================================
% =========================================================================
baseIncircle = 0.25;  % physical base radius (same as used in A* / disks)

smartSeed = double(retreatConfig);

% 1. Teleport Seed Base to Target XY
smartSeed(ix) = offsetPose(1,4);  % Base X
smartSeed(iy) = offsetPose(2,4);  % Base Y

% 2. Calculate Path Direction for Base Yaw
dx = offsetPose(1,4) - retreatConfig(ix);
dy = offsetPose(2,4) - retreatConfig(iy);

if hypot(dx, dy) > 0.5
    targetYaw = atan2(dy, dx);
    smartSeed(iyaw) = targetYaw;
    fprintf('       [SmartSeed] Aligning Base Yaw to %.2f rad (Path Direction)\n', targetYaw);
else
    fprintf('       [SmartSeed] Short move; keeping original Yaw.\n');
end

% 3. Solve IK using the Smart Seed
fprintf('[Setup] Calculating qGoalPlan using Smart Base Seed...\n');
qGoalPlan    = double(ik(endEffector, offsetPose, weights, smartSeed));
offsetConfig = qGoalPlan;

mustBeFree(mobileUR20, offsetConfig, envCol, 'Goal (offsetPose) is in collision.');

% =========================================================================
% === 2) Focus regions generation (A* + Maximal Disks + Maximal Spheres) ==
% =========================================================================
fprintf('[HS-Bi-RRT] Building focus regions (A* path + Max Regions + Bubble Out)...\n');

start_xy   = retreatConfig([ix iy]);
tcpGoal_xy = offsetPose(1:2,4).';

% Filter 'floor' out before generating focus regions
names    = cellfun(@(s) s.Name, envS, 'uni', false);
floorIdx = find(strcmpi(names,'floor'));
if ~isempty(floorIdx)
    fprintf("[HS-Bi-RRT] Found and removed 'floor' from focus region generation.\n");
    envObsForFocus = envS;
    envObsForFocus(floorIdx) = [];
else
    fprintf("[HS-Bi-RRT] WARNING: No 'floor' object found; using all env objects for focus generation.\n");
    envObsForFocus = envS;
end

% --- Generate 2D A* guide path (with clearance) ---
fprintf('[HS-Bi-RRT] Generating 2D A* guide path with clearance=%.3f...\n', baseIncircle);
guidePathXY = generateGuidePathAStar(start_xy, tcpGoal_xy, envObsForFocus, baseIncircle);
if isempty(guidePathXY)
    error('Failed to generate A* guide path with required clearance.');
end
fprintf('[HS-Bi-RRT] Generated A* guide path with %d points.\n', size(guidePathXY, 1));

% --- Generate Maximal Base Disks along the path ---
fprintf('[HS-Bi-RRT] Generating maximal Base Disks along path with center refinement...\n');
t_bd_tic = tic;
baseDisks = generateMaximalBaseDisksAlongPath(start_xy, tcpGoal_xy, envObsForFocus, guidePathXY, baseIncircle);
t_baseDisks_s = toc(t_bd_tic);
fprintf('[HS-Bi-RRT] Generated %d Base Disks in %.3f s.\n', size(baseDisks, 1), t_baseDisks_s);

% --- Generate Maximal EE spheres along the path ---
fprintf('[HS-Bi-RRT] Generating maximal EE Spheres along path with center refinement...\n');
eeInflate = 0.1;  % 10 cm buffer for EE spheres
envObsEE  = inflateEnv(envObsForFocus, eeInflate, "floor");

pEE0 = getTransform(mobileUR20, retreatConfig, endEffector); pEE0 = pEE0(1:3,4).';
pEEg = getTransform(mobileUR20, offsetConfig,   endEffector); pEEg = pEEg(1:3,4).';

t_ee_tic = tic;
[eeCenters, eeRadiusVec] = generateMaximalEESpheresAlongPath( ...
    pEE0, pEEg, envObsEE, guidePathXY);
t_eePucks_s = toc(t_ee_tic);
fprintf('[HS-Bi-RRT] Generated %d EE pucks in %.3f s.\n', size(eeCenters,1), t_eePucks_s);

eeSpheres = [eeCenters, eeRadiusVec(:)];

% Pack focus structure for plotting / saving
focus = struct();
focus.baseCenters   = baseDisks(:,1:2);
focus.baseRadiusVec = baseDisks(:,3);
focus.eeCenters     = eeCenters;
focus.eeRadiusVec   = eeRadiusVec;
focus.ix            = ix;
focus.iy            = iy;
focus.iyaw          = iyaw;

fprintf('[HS-Bi-RRT] Focus regions built. Proceeding to planner.\n');

% --- Visualization of focus regions ---
% figure('Name','Focus Regions (Plan 4)','Units','normalized','OuterPosition',[0 0 1 1]);
% hold on; grid on; view(45,30);
% title('Plan 4: HS-Bi-RRT Focus Regions (Maximal Regions along A* Path + Bubble Out)');
% for k = 1:numel(envS)
%     show(envS{k}.CollisionObj);
% end
% if ~isempty(guidePathXY)
%     plot3(guidePathXY(:,1), guidePathXY(:,2), zeros(size(guidePathXY,1),1), ...
%           'g-', 'LineWidth', 1.5);
% end
% drawFocusRegions(focus, gca);
% drawFrame(getTransform(mobileUR20, retreatConfig, endEffector),'start_tcp',0.12);
% drawFrame(offsetPose,'offset_tcp',0.12);
% axis equal; xlabel('X'); ylabel('Y'); zlabel('Z');
% camlight headlight; lighting gouraud;
% drawnow;

% =========================================================================
% === 3) HS-Bi-RRT parameters (with paper-style collision geometry) =======
% =========================================================================
P = struct( ...
    'R_s', 0.5, ...
    'lambda', 0.2, ...
    'xi', 0.05, ...
    'xi_prime', 0.75, ...
    'xi_double', 3.0, ...
    'alpha', pi/8, ...
    'beta', pi/8, ...
    'stepSize', 0.4, ...
    'valStep', 0.02, ...
    'nearRadius', 0.4, ...
    'maxTime', 300, ...
    'verboseEvery', 20 );

diameter_mm = rebarMeta.DiameterM * 1000;
length_m    = rebarMeta.LengthM;
safetyMargin = 0.02;
rebarRadius = (diameter_mm/1000)/2;  % [m]
rebarLen    = length_m;              % [m]

P.baseIncircle = baseIncircle;
P.rebarRadius  = rebarRadius + safetyMargin;
P.rebarLen     = chooseValue(useRebarEnvelope, rebarEnvelope.lengthM, rebarLen + safetyMargin*2);
P.rebarPhysicalLen = rebarLen;
P.rebarEnvelopeLength = rebarEnvelope.lengthM;
P.rebarEnvelopeLengthFactor = rebarEnvelope.lengthFactor;
P.rebarEnvelopeHeight = rebarEnvelope.heightM;
P.rebarEnvelopeWidth = rebarEnvelope.widthM;
P.useRebarSagEnvelope = useRebarEnvelope;
P.zFloor       = 0; 

P.armKeyNames = [ ...
    "C-2003903",...
    "C-2003904", ...
    "C-2003905", ...
    "C-2003906", ...
    "C-2003907", ...
    "C-2007588",...
    "C-2003908" ...
    
];
P.armRadius   = 0.1;       % ~8 cm "thickness" of arm keypoints
% =========================================================================
% === 4) Run the HS-Bi-RRT planner (paper-style collision) ================
% =========================================================================
envCol_paper = inflateEnv(envCol, 0.08, "floor"); 

%% === DIAGNOSTIC: WHY IS THE PLANNER STUCK? ===
fprintf('\n=== DIAGNOSTIC START ===\n');

% 1. Check Standard Collision (Physical)
[isColStart, colBodyIdx] = checkCollision(mobileUR20, retreatConfig, envCol, ...
    "Exhaustive","on", "SkippedSelfCollisions","parent");
[isColGoal, ~] = checkCollision(mobileUR20, offsetConfig, envCol, ...
    "Exhaustive","on", "SkippedSelfCollisions","parent");

if any(isColStart)
    fprintf(2, '[Physical] Start Config is in collision with Environment!\n');
else
    fprintf('[Physical] Start Config is physically safe.\n');
end

% 2. Check Self Collision
isSelfStart = checkCollision(mobileUR20, retreatConfig, {}, ...
    "Exhaustive","on", "SkippedSelfCollisions","parent");
if any(isSelfStart)
    fprintf(2, '[Self] Start Config has SELF-COLLISION (Arm hits Body)!\n');
else
    fprintf('[Self] Start Config is free of self-collision.\n');
end

% 3. Check Paper Model Constraints (The likely culprit)
% Recalculate clearance for the base
S_diag = resolveBaseIndices(mobileUR20);
baseXY_start = retreatConfig(S_diag.baseIdx(1:2));
dBase_start  = minClearance2D(baseXY_start, envCol);

fprintf('[Paper] Base Clearance at Start: %.3f m\n', dBase_start);
fprintf('[Paper] Required Incircle:       %.3f m\n', P.baseIncircle);

if dBase_start < P.baseIncircle
    fprintf(2, 'FAIL: The robot is too close to a wall for the "Base Incircle" setting.\n');
    fprintf(2, '      Solution: Reduce P.baseIncircle (e.g., to %.3f)\n', dBase_start - 0.01);
end

% Check Rebar Height
T_start = getTransform(mobileUR20, retreatConfig, endEffector);
% ... (Simplified height check) ...
if T_start(3,4) < P.zFloor
   fprintf(2, 'FAIL: Rebar is lower than zFloor.\n');
end

fprintf('=== DIAGNOSTIC END ===\n\n');
[path4a, ok, info] = hsBiRRT_paper_v2( ...
    mobileUR20, envCol_paper, retreatConfig, offsetConfig, ...
    endEffector, P, baseDisks, eeSpheres);

if ~ok
    error('HS-Bi-RRT failed to find a solution within the time limit.');
else
    t_plan4p = toc(t_plan4p);
    fprintf('[HS-Bi-RRT] Planning success — %.1fs | Nodes A = %d | Nodes B = %d | Gap = %.3f\n', ...
        t_plan4p, info.nodesA, info.nodesB, info.bestGap);

    % === COMMON POST-PROCESSING (same as MA-HG-Bi-RRT) ==================
    % 1) Densify with angle-aware interpolation
    traj4a = hs_interpPath_segmentFreeStyle(mobileUR20, path4a, P.valStep);

    % 2) Optionally project to the RPZ manifold at transportZ.
    % traj4a = projectTrajectoryToPlaneRP( ...
    %     mobileUR20, traj4a, endEffector, transportZ, [0 0]);

    % 3) Adaptive full-body collision check + local repair
    fprintf('[Check] Adaptive precheck (coarse=0.06, fine=0.02)...\n');
    okCheck = assertPaperPathCollisionFree(mobileUR20, traj4a, envCol, endEffector, P);
    
    if ~okCheck
        fprintf('[Repair] Subdividing only colliding segments...\n');
        [traj4a, okRepair] = repairCollidingSegments_localRRT( ...
            mobileUR20, envCol, traj4a, 0.02, 3, endEffector, P);
    
        if ~okRepair
            error('HS-Bi-RRT local repair failed for run %d.', runID);
        end
    
        % Final strict pass – but now use return flag, don't throw:
        okStrict = assertPaperPathCollisionFree(mobileUR20, traj4a, envCol, endEffector, P);
        if ~okStrict
            error('HS-Bi-RRT strict paper-model check failed after repair for run %d.', runID);
        end
    end

    t_plan4t = toc(t_plan4t);
    fprintf('Plan 4 TOTAL time: %.3f seconds.\n', t_plan4t);

    allStates = [allStates; traj4a];
end

%% ========================================================================
% --- Plan 6: Final Approach to Place (Guarded Move) ---
% (This section is now valid, as 'offsetConfig' is at 'offsetPose')
%% ========================================================================
fprintf('Plan 5: Straight-in to Target...\n');
[traj5, qAtPlace] = guardedApproachToolAxis( ...
    mobileUR20, offsetConfig, targetPose, ...
    envCol, ik, endEffector, weights, +0.01);
% for i = 1:size(traj5, 1)
%     show(mobileUR20, traj5(i,:), "PreservePlot", false, "Visuals","off","Collisions","on");
%     title("Plan 5: Final placement move"); drawnow;
% end
allStates = [allStates; traj5];
%% ========================================================================
%  6. plan 4 replay
%  ========================================================================
% Optional detailed trajectory replay.
% fprintf('Detailed replay...\n');
% figure("Name","Detailed Trajectory Replay","Units","normalized","OuterPosition",[0,0,1,1]);
% ax = gca; hold(ax,'on'); grid(ax,'on');
% axis(ax, [-8 8 -8 6 -0.1 2.5]); view(ax, 60, 25);
% % 1) draw environment
% for i = 1:numel(envS), show(envS{i}.CollisionObj, 'Parent', ax); end
% % 2) trails: base XY and EE
% S = resolveBaseIndices(mobileUR20);
% baseXY = traj4a(:, S.baseIdx(1:2));
% plot3(ax, baseXY(:,1), baseXY(:,2), zeros(size(baseXY,1),1), 'k-', 'LineWidth', 1.5);
% eeTrail = zeros(size(traj4a,1),3);
% for i = 1:size(traj4a,1)
%     T = getTransform(mobileUR20, traj4a(i,:), endEffector);
%     eeTrail(i,:) = T(1:3,4)';
% end
% plot3(ax, eeTrail(:,1), eeTrail(:,2), eeTrail(:,3), 'b-', 'LineWidth', 1.5);
% % 3) breadcrumbs: sample points along EACH path edge (exact validator spacing)
% plotEdgeSamples(mobileUR20, path4a, endEffector, ax, P.valStep);
% camlight(ax,'headlight'); lighting(ax,'gouraud');
% % 4) animate slowly so the avoidance is visible
% show(mobileUR20, traj4a(1,:), "Visuals","off","Collisions","on", 'Parent', ax);
% for k = 2:size(traj4a,1)
%     show(mobileUR20, traj4a(k,:), "PreservePlot", false, "Visuals","off","Collisions","on", 'Parent', ax);
%     title(ax, sprintf("Replay %d/%d", k, size(traj4a,1)));
%     drawnow limitrate; pause(0.01);   % <-- slow down so details are visible
% end
%% ========================================================================
%  6. SAVE TO SEED (ENHANCED METRICS FOR COMPARISON)
%  ========================================================================
outDir  = "seeds";
dateDir = fullfile(outDir, string(datetime('now','Format','yyyyMMdd')));
if ~exist(dateDir,'dir'), mkdir(dateDir); end
stamp   = string(datetime('now','Format','yyyyMMdd_HHmmss'));
% -- Re-add rebar into env copy (for complete replay context)
envForSave = envS;
if exist('rebarObj','var')
    envForSave = [envForSave, {struct('Name','rebar','CollisionObj',rebarObj)}];
end
% -- Compute comprehensive metrics for comparison --
fprintf('Computing comprehensive metrics for comparison...\n');
metrics = computeComprehensiveMetrics(mobileUR20, allStates, traj1, traj2, traj3, traj4a, traj5, endEffector);
% -- Motion/rehydration settings
rehydration = struct( ...
    'mode',         "RMR", ... % This script uses Rotate-Move-Rotate interpolation
    'val_step',     P.valStep, ...
    'check_coarse', 0.06, ...
    'check_fine',   0.02 ...
);
% -- Guidance artefacts & corridor params
if exist('guidePathXY','var'),   guideXY_save = guidePathXY;       else, guideXY_save = []; end
if exist('res','var'),           gridRes_save = res;               else, gridRes_save = NaN; end
corridor = struct( ...
    'guideXY',      guideXY_save, ...
    'bestHeightXY', [], ...           % This script does not use bestHeightXY
    'gridRes',      gridRes_save, ...
    'tubeRadius',   NaN, ...
    'penaltyW',     NaN, ...
    'rSafe',        NaN ...
);
% -- Focus regions (spheres or circles)
if exist('baseDisks','var'), baseDisks_save = baseDisks; else, baseDisks_save = []; end
if exist('eeSpheres','var'), eeSpheres_save = eeSpheres; else, eeSpheres_save = []; end
focusSave = struct( ...
    'baseDisks',  baseDisks_save, ...
    'eeSpheres',  eeSpheres_save, ...
    'eeCircles',  [], ...
    'eeType',     "sphere" ...
);
% -- Poses used in 4a/5 (safe defaults if missing)
if exist('retreatConfig','var')
    T_retreat_save = getTransform(mobileUR20, retreatConfig, endEffector); 
else
    T_retreat_save = eye(4); 
end
if exist('offsetPose','var'),  offsetPose_save  = offsetPose;  else, offsetPose_save  = eye(4); end
if exist('targetPose','var'),  targetPose_save  = targetPose;  else, targetPose_save  = eye(4); end
if exist('graspPose','var'),   graspPose_save   = graspPose;   else, graspPose_save   = eye(4); end
poses = struct( ...
    'pickPose',      pickPose, ...
    'graspPose',     graspPose_save, ...
    'retreatPoseEE', T_retreat_save, ...
    'offset1Pose',   [], ... % This script does not use offset1Pose
    'offsetPose',    offsetPose_save, ...
    'targetPose',    targetPose_save ...
);
% -- Planner info
if exist('info','var'), info4a_save = info; else, info4a_save = struct(); end
rng_state = rng;
if isfield(P,'maxTime'), maxTimeSave = P.maxTime; else, maxTimeSave = NaN; end
% -- Timing structure
timingStruct = struct();
timingStruct.bestZ_s = NaN; % No bestZ search
timingStruct.gen_baseDisks_s = t_baseDisks_s; % Base-disk generation
timingStruct.gen_eePucks_s = t_eePucks_s;
timingStruct.plan4_planning_s = t_plan4p;  % Planning time for HS-Bi-RRT
timingStruct.plan4_total_s = t_plan4t;    % Total time for Plan 4 (planning + interpolation + validation)
timingStruct.total_s = t_plan4t; % Total for entire operation (no bestZ)
% -- Build comprehensive bundle
bundle = struct( ...
  'meta', struct( ...
      'approach', 'A* Guide + HS-Bi-RRT (9-DoF) + (BaseDisk + EE Sphere Focus)', ...
      'created',  datetime('now'), ...
      'runID',    runID, ...  
      'desc',     'Plan 4 (HS-Bi-RRT 9-DoF) + Plan 5 (guarded approach)', ...
      'matlab',   version, ...
      'script',   mfilename, ...
      'rng',      rng_state ), ...
  'robot',            mobileUR20, ...
  'robot5dof',        [], ... % This script uses full 9-DoF
  'freezeList',       [], ...
  'endEffector',      endEffector, ...
  'env',              envForSave, ...
  'rebarMeta',        rebarMeta, ...
  'rebarEnvelope',    rebarEnvelope, ...
  'bestZ_circle_chain', NaN, ...
  'deflectionM',      deflection, ...
  'focus',            focusSave, ...
  'corridor',         corridor, ...
  'poses',            poses, ...
  'planner', struct( ...
      'P',            P, ...
      'info4a',       info4a_save, ...
      'maxTime_s',    maxTimeSave ...
  ), ...
  'validation',       rehydration, ...
  'timing',           timingStruct, ... 
  'paths', struct( ...
      'path4a_nodes_5dof', [], ...
      'traj4a_exec_5dof',  [], ...
      'traj4a_full_9dof',  traj4a, ... % Save the 9-DoF trajectory
      'traj5_full_9dof',   traj5, ...
      'full_allStates',    allStates, ...
      'traj1', traj1, ...
      'traj2', traj2, ...
      'traj3', traj3 ...
  ), ...
  'metrics',          metrics, ...
  'success',          true ...
);
saveFile = fullfile(dateDir, "hsBiRRT_CS1_2m" + stamp + ...
                    sprintf("_run%02d.mat", runID));
save(saveFile, 'bundle', '-v7.3');
fprintf('[SAVE] HS-Bi-RRT Guided Plan -> %s\n', saveFile);
fprintf('[SAVE] Comprehensive metrics saved for comparison:\n');
fprintf('       - TCP path length: %.3f m\n', metrics.tcp.total_length);
fprintf('       - Base path length: %.3f m\n', metrics.base.total_length_xy);
fprintf('       - Plan 4 Planning time: %.3f s\n', timingStruct.plan4_planning_s);
fprintf('       - Plan 4 Total time: %.3f s\n', timingStruct.plan4_total_s);
fprintf('       - Total planning time: %.3f s\n', timingStruct.total_s);
fprintf('       - Total waypoints: %d\n', metrics.total_waypoints);
%% ========================================================================
%  7. REPLAY FULL TRAJECTORY
%  ========================================================================
fprintf('Replaying full path...\n');
figure("Name","Full Trajectory Replay","Units","normalized","OuterPosition",[0,0,1,1]);
axReplay = gca; % Get handle to axes for drawing focus regions
hold(axReplay, 'on'); grid(axReplay, 'on');
axis(axReplay, [-8 8 -8 6 -0.1 2.5]); view(axReplay, 60,25);
% --- Draw static elements FIRST ---
% Reload and show original environment
[~, ~, envS_replay] = loadRebarTransportScenario;
for i = 1:numel(envS_replay), show(envS_replay{i}.CollisionObj, 'Parent', axReplay); end
% Draw focus regions (both disks and spheres)
if exist('focus','var') && isstruct(focus) && ...
   ((isfield(focus, 'eeCenters') && ~isempty(focus.eeCenters)) || ...
    (isfield(focus, 'baseCenters') && ~isempty(focus.baseCenters)))
     fprintf('Drawing focus regions during replay...\n');
     drawFocusRegions(focus, axReplay); 
else
     fprintf('Focus regions not available for replay visualization.\n');
end
% Add lighting
camlight(axReplay, 'headlight'); lighting(axReplay, 'gouraud');
% --- Animate Robot Trajectory ---
if ~isempty(allStates)
    % Show initial pose
    show(mobileUR20, allStates(1,:), "Visuals","off","Collisions","on", 'Parent', axReplay);
    envHandle = drawRebarSagEnvelope(mobileUR20, allStates(1,:), endEffector, P, axReplay);
    title(axReplay, sprintf("Full Replay (State %d/%d)", 1, size(allStates,1)));
    drawnow; % Draw static scene + initial pose
    % Loop through remaining poses
    for k = 2:size(allStates, 1) % Start loop from 2
        show(mobileUR20, allStates(k,:), "PreservePlot", false, "Visuals","off","Collisions","on", 'Parent', axReplay);
        if all(isgraphics(envHandle)), delete(envHandle); end
        envHandle = drawRebarSagEnvelope(mobileUR20, allStates(k,:), endEffector, P, axReplay);
        title(axReplay, sprintf("Full Replay (State %d/%d)", k, size(allStates,1)));
        drawnow; % Update animation in loop
    end
else
    title(axReplay, "Replay Failed: No valid trajectory generated.");
    show(mobileUR20, startConfig, "Visuals","off","Collisions","on", 'Parent', axReplay); % Show start config if planning failed
    drawnow;
end
hold(axReplay, 'off');
%% ========================================================================
%  7. CORE HELPER FUNCTIONS (FOCUS REGION GENERATION - A* + Max Regions + Bubble Out)
%  ========================================================================
% --- Generate 2D A* Guide Path (with clearance) ---
function guidePathXY = generateGuidePathAStar(start_xy, goal_xy, envObs, baseIncircle)
% Generates a 2D collision-free path using A* on an occupancy grid.
% Tries a list of obstacle-inflation radii, starting large (more
% centered path) and decreasing until a feasible path is found.

    % ---------------------------------------------------------------------
    % Inflation radii to try (meters).
    % Larger -> path farther from walls, closer to corridor center.
    % If A* fails for a radius, try the next smaller radius.
    % ---------------------------------------------------------------------
    inflationList = [0.5 0.4 0.3 0.20];

    % Grid resolution (unchanged)
    res = 0.05; 

    guidePathXY = [];   % default

    % Loop over candidate inflation radii
    for k = 1:numel(inflationList)
        requiredClearance = inflationList(k);

        extra = max(0, requiredClearance - baseIncircle);
        fprintf('A* Path: Try #%d with inflation radius %.3f (Base %.3f + Extra %.3f)\n', ...
                k, requiredClearance, baseIncircle, extra);

        % --- Build Occupancy Grid with this inflation ---
        [xlimW, ylimW] = envBoundsXY(envObs, start_xy, goal_xy, 1.0); % Pad bounds generously
        fprintf('  Building occupancy grid (res=%.3f)...\n', res);
        occ = buildOccGridFromEnvS(envObs, xlimW, ylimW, res, requiredClearance);

        % --- Convert start/goal to grid coords ---
        [sIJ, gIJ] = world2grid(start_xy, goal_xy, occ);

        % Try to find nearest free cell if exact start/goal is covered
        sIJ = ensureFreeIJ(sIJ, occ);
        gIJ = ensureFreeIJ(gIJ, occ);

        if any(isnan(sIJ)) || any(isnan(gIJ))
            fprintf('  -> Start/Goal buried in obstacles for infl=%.3f, trying smaller...\n', ...
                    requiredClearance);
            continue;  % try next (smaller) inflation
        end

        % --- Run A* Search ---
        [pathIJ, okA] = aStar8(occ, sIJ, gIJ);

        if ~okA || isempty(pathIJ)
            fprintf('  -> A* failed for infl=%.3f, trying smaller...\n', requiredClearance);
            continue;  % try next
        end

        % --- Convert grid path to world coords ---
        pathXY = grid2world(pathIJ, occ);

        % Smooth the path 
        pathXY = shortcutSmoothing(pathXY, occ);

        % Resample finely for region generation
        guidePathXY = resamplePolylineByStep(pathXY, 0.10);

        fprintf('A* Path: SUCCESS with infl=%.3f | %d points.\n', ...
                requiredClearance, size(guidePathXY, 1));
        return;  % done – we keep the first successful inflation
    end

    % ---------------------------------------------------------------------
    % Fall back to a straight line if all inflation values fail.
    % ---------------------------------------------------------------------
    warning('generateGuidePathAStar: All inflation radii failed. Using straight-line fallback.');
    guidePathXY = [start_xy; goal_xy];
    guidePathXY = resamplePolylineByStep(guidePathXY, 0.10);
end
% --- Generate Maximal Base Disks Along Path with Center Refinement ---
function baseDisks = generateMaximalBaseDisksAlongPath(start_xy, goal_xy, envObs, guidePathXY, baseIncircle)
% Generates maximal free-space Base Disks along guidePathXY. 
    % Parameters - Made more conservative
    rMin = 0.20;    % Minimum disk radius to accept
    rCap = 1.2;     % Maximum disk radius cap
    margin = 0.05;  % Margin from obstacles
    maxDisks = 100;
    advanceFactor = 0.6;
    % Gradient Ascent Parameters
    gaSteps = 5;
    gaStepSize = 0.02;
    baseDisksList = {};
    % Add start disk
    center_refined = refineCenter2D(start_xy, envObs, gaSteps, gaStepSize);
    clearance = minClearance2D(center_refined, envObs);
    radius = max(0, min(rCap, clearance - baseIncircle - margin));
    
    if radius >= rMin
        baseDisksList{1} = [center_refined, radius];
        fprintf('Start disk: center [%.3f, %.3f], radius %.3f (clearance %.3f)\n', ...
            center_refined(1), center_refined(2), radius, clearance);
    else
        % Even if radius < rMin, still add it but warn
        warning('Start disk radius %.3f < rMin %.3f. Adding anyway.', radius, rMin);
        baseDisksList{1} = [center_refined, max(0.05, radius)]; % Ensure minimum 5cm
    end
    % If goal is reachable from start, return early
    if norm(goal_xy - baseDisksList{1}(1:2)) <= baseDisksList{1}(3)
        baseDisks = cell2mat(baseDisksList');
        return;
    end
    
    % If no guide path, create a simple straight-line path
    if isempty(guidePathXY)
        fprintf('No guide path available, using straight-line interpolation.\n');
        nPoints = 10;
        t = linspace(0, 1, nPoints)';
        guidePathXY = start_xy + t .* (goal_xy - start_xy);
    end
    
    % Iteratively generate maximal disks along the guide path
    currentPathIdx = 1;
    
    while numel(baseDisksList) < maxDisks
        lastCenter = baseDisksList{end}(1:2);
        lastRadius = baseDisksList{end}(3);
        
        % Check if goal is covered by the last disk
        if norm(goal_xy - lastCenter) <= lastRadius
            break; 
        end
        
        % Find next point on path
        advanceDist = advanceFactor * lastRadius;
        nextIdxOnPath = currentPathIdx;
        foundNext = false;
        distTravelled = 0;
        
        searchStartIdx = currentPathIdx + 1;
        if searchStartIdx > size(guidePathXY,1), break; end
        for checkIdx = searchStartIdx : size(guidePathXY, 1)
            distSegment = norm(guidePathXY(checkIdx,:) - guidePathXY(checkIdx-1,:));
            distFromLastCenter = norm(guidePathXY(checkIdx,:) - lastCenter);
            
            if distTravelled + distSegment >= advanceDist || distFromLastCenter >= advanceDist * 0.9
                currentPathIdx = checkIdx;
                foundNext = true;
                break;
            end
            distTravelled = distTravelled + distSegment;
        end
        if ~foundNext, break; end
        
        candidateCenterOnPath = guidePathXY(currentPathIdx, :);
        center_refined = refineCenter2D(candidateCenterOnPath, envObs, gaSteps, gaStepSize);
        clearance = minClearance2D(center_refined, envObs);
        radius = max(0, min(rCap, clearance - baseIncircle - margin));
        
        % Add disk even if radius is small, but track quality
        if radius >= rMin
            if isempty(baseDisksList) || norm(center_refined - baseDisksList{end}(1:2)) > rMin * 0.3
                baseDisksList{end+1} = [center_refined, radius];
                fprintf('Disk %d: center [%.3f, %.3f], radius %.3f (clearance %.3f)\n', ...
                    numel(baseDisksList), center_refined(1), center_refined(2), radius, clearance);
            end
        else
            % Add small radius disk with warning
            warning('Disk near path index %d: radius %.3f < rMin %.3f. Adding with radius %.3f', ...
                currentPathIdx, radius, rMin, max(0.05, radius));
            if isempty(baseDisksList) || norm(center_refined - baseDisksList{end}(1:2)) > 0.1
                baseDisksList{end+1} = [center_refined, max(0.05, radius)];
            end
        end
    end
    
    % Final goal disk
    if isempty(baseDisksList) || norm(goal_xy - baseDisksList{end}(1:2)) > baseDisksList{end}(3)
        center_refined_goal = refineCenter2D(goal_xy, envObs, gaSteps, gaStepSize);
        clearance = minClearance2D(center_refined_goal, envObs);
        radius = max(0, min(rCap, clearance - baseIncircle - margin));
        
        if radius >= rMin
            baseDisksList{end+1} = [center_refined_goal, radius];
            fprintf('Goal disk: center [%.3f, %.3f], radius %.3f (clearance %.3f)\n', ...
                center_refined_goal(1), center_refined_goal(2), radius, clearance);
        else
            warning('Final goal disk: radius %.3f < rMin %.3f. Adding anyway.', radius, rMin);
            baseDisksList{end+1} = [center_refined_goal, max(0.05, radius)];
        end
    end
    if isempty(baseDisksList)
        warning('Failed to generate any base disks. Creating minimal start and goal disks.');
        % Create minimal disks at start and goal as fallback
        baseDisksList{1} = [start_xy, 0.1];
        baseDisksList{2} = [goal_xy, 0.1];
    end
    
    baseDisks = cell2mat(baseDisksList');
    fprintf('Generated %d base disks along path.\n', size(baseDisks, 1));
end

% --- Generate Maximal EE Spheres Along Path with Center Refinement ---
function [C, R] = generateMaximalEESpheresAlongPath(pStart, pGoal, envObs, guidePathXY)
% Generates maximal free-space EE Spheres along guidePathXY.
% Centers are refined. Spheres only added if radius >= rMin.
    % Parameters
    rMin = 0.3;    % Minimum sphere radius to accept
    rMax = 1.2;     % Maximum sphere radius cap
    margin = 0.01;  % Small margin from obstacles
    maxSpheres = 150; % Safety limit
    numCoordPoints = max(50, size(guidePathXY,1)); % Interpolate guide path 
    advanceFactor = 0.6; % Advance along path by fraction of current radius
    % Gradient Ascent Parameters for Center Refinement
    gaSteps = 5;    % Max iterations for hill climbing
    gaStepSize = 0.02; % Step size for moving center
    C_list = {}; % Use cell array for dynamic sizing
    R_list = {};
    % Add start sphere
    center_refined_start = refineCenter3D(pStart(:).', envObs, gaSteps, gaStepSize);
    clearance = minClearance3D(center_refined_start, envObs);
    radius = max(0, min(rMax, clearance - margin)); % Use max(0,...)
    if radius >= rMin % Check against minimum required radius
        C_list{1} = center_refined_start;
        R_list{1} = radius;
    else
        warning('Could not generate valid starting sphere (radius %.3f < rMin %.3f).', radius, rMin);
        C = []; R = []; return; % Cannot proceed
    end
    % If goal is reachable from start, return early
    if norm(pGoal(:).' - C_list{1}) <= R_list{1}
        C = cell2mat(C_list'); R = cell2mat(R_list'); return;
    end
    
    % --- Create an interpolated 3D path for guiding sphere centers ---
    coordinatedPath3D = generateCoordinatedPath_FromGuide(pStart, pGoal, guidePathXY, numCoordPoints);
   
    % --- Iteratively generate maximal spheres along the 3D path ---
    currentPathIdx = 1; % Index on coordinatedPath3D
    
    while numel(C_list) < maxSpheres
        lastCenter = C_list{end};
        lastRadius = R_list{end};
        
        % Check if goal is covered by the last sphere
        if norm(pGoal(:).' - lastCenter) <= lastRadius
            break; 
        end
        
        % Find the next point on coordinatedPath3D sufficiently far away
        advanceDist = advanceFactor * lastRadius;
        nextIdxOnPath = currentPathIdx;
        foundNext = false;
        distTravelled = 0; 
        
        searchStartIdx = currentPathIdx + 1;
        if searchStartIdx > size(coordinatedPath3D,1), break; end
        for checkIdx = searchStartIdx : size(coordinatedPath3D, 1)
            distSegment = norm(coordinatedPath3D(checkIdx,:) - coordinatedPath3D(checkIdx-1,:));
            distFromLastCenter = norm(coordinatedPath3D(checkIdx,:) - lastCenter);
            
            if distTravelled + distSegment >= advanceDist || distFromLastCenter >= advanceDist * 0.9
                currentPathIdx = checkIdx; 
                foundNext = true;
                break;
            end
            distTravelled = distTravelled + distSegment;
        end
        if ~foundNext, break; end
        
        candidateCenterOnPath = coordinatedPath3D(currentPathIdx, :);
        center_refined = refineCenter3D(candidateCenterOnPath, envObs, gaSteps, gaStepSize);
        clearance = minClearance3D(center_refined, envObs);
        radius = max(0, min(rMax, clearance - margin)); % Use max(0,...)
        
        % Check minimum radius requirement
        if radius >= rMin && ~any(isnan(center_refined))
            if isempty(C_list) || norm(center_refined - C_list{end}) > rMin * 0.3
                C_list{end+1} = center_refined; %#ok<AGROW>
                R_list{end+1} = radius; %#ok<AGROW>
            end
        else
            fprintf('Skipping sphere near path index %d (radius %.3f < rMin %.3f or NaN).\n', currentPathIdx, radius, rMin);
             % Don't add sphere, just continue
        end
    end % End while loop
    
    % Final check: If goal wasn't covered, add a sphere centered at goal (refined)
    if isempty(R_list) || norm(pGoal(:).' - C_list{end}) > R_list{end}
         center_refined_goal = refineCenter3D(pGoal(:).', envObs, gaSteps, gaStepSize);
         clearance = minClearance3D(center_refined_goal, envObs);
         radius = max(0, min(rMax, clearance - margin)); % Use max(0,...)
         if radius >= rMin && ~any(isnan(center_refined_goal)) % Check min radius
             C_list{end+1} = center_refined_goal;
             R_list{end+1} = radius;
             fprintf('Goal not covered, added final refined sphere (r=%.3f) at goal.\n', radius);
         else
              warning('Could not add final sphere at goal (max radius %.3f < rMin %.3f).', radius, rMin);
         end
    end
    if numel(C_list) >= maxSpheres
       warning('Reached maximum sphere limit (%d) before covering goal.', maxSpheres); 
    end
    
    if isempty(C_list)
        warning('Failed to generate any valid EE spheres.');
        C = []; R = []; % Return empty matrices
    else
        C = cell2mat(C_list');
        R = cell2mat(R_list');
    end
end
% --- Center Refinement Helper Functions ---
function center_refined = refineCenter2D(center_start, envObs, maxSteps, stepSize)
% Performs gradient ascent on minClearance2D to find a local maximum.
    center_refined = center_start;
    eps = 1e-4; % Small step for numerical gradient calculation
    for i = 1:maxSteps
        d0 = minClearance2D(center_refined, envObs);
        if d0 <= 0, break; end % Stop if already in collision or too close
        % Numerical gradient
        dx = (minClearance2D(center_refined + [eps, 0], envObs) - d0) / eps;
        dy = (minClearance2D(center_refined + [0, eps], envObs) - d0) / eps;
        grad = [dx, dy];
        gradNorm = norm(grad);
        if gradNorm < 1e-5 % Close to maximum or flat region
            break;
        end
        % Move center along gradient
        center_next = center_refined + stepSize * (grad / gradNorm);
        % Optional: Line search or check if next step improves clearance
        % d_next = minClearance2D(center_next, envObs);
        % if d_next <= d0, break; end % Stop if step doesn't improve
        center_refined = center_next;
    end
end
function center_refined = refineCenter3D(center_start, envObs, maxSteps, stepSize)
% Performs gradient ascent on minClearance3D to find a local maximum.
    center_refined = center_start;
    eps = 1e-4; % Small step for numerical gradient calculation
    for i = 1:maxSteps
        d0 = minClearance3D(center_refined, envObs);
         if d0 <= 0, break; end % Stop if already in collision or too close
        % Numerical gradient
        dx = (minClearance3D(center_refined + [eps, 0, 0], envObs) - d0) / eps;
        dy = (minClearance3D(center_refined + [0, eps, 0], envObs) - d0) / eps;
        dz = (minClearance3D(center_refined + [0, 0, eps], envObs) - d0) / eps;
        grad = [dx, dy, dz];
        gradNorm = norm(grad);
        if gradNorm < 1e-5 % Close to maximum or flat region
            break;
        end
        % Move center along gradient
        center_next = center_refined + stepSize * (grad / gradNorm);
        % d_next = minClearance3D(center_next, envObs);
        % if d_next <= d0, break; end 
        center_refined = center_next;
    end
end
% --- Helper function to generate coordinated 3D path (Simplified) ---
function coordinatedPath = generateCoordinatedPath_FromGuide(pStart, pGoal, guidePathXY, numPoints)
% Interpolates guidePathXY and Z coordinate to create a 3D path.
    coordinatedPath = zeros(numPoints, 3);
    if ~isempty(guidePathXY) && size(guidePathXY, 1) > 1
         % Interpolate XY guide path based on cumulative length
        pathLength = [0; cumsum(vecnorm(diff(guidePathXY), 2, 2))];
        totalLength = pathLength(end);
        if totalLength < 1e-6 % Avoid division by zero if path is point
             t_old_xy = linspace(0,1,size(guidePathXY, 1)); % Fallback to index-based interp
        else
            t_old_xy = pathLength' / totalLength; % Normalized path length
        end
        t_new_xy = linspace(0, 1, numPoints);
        % Use try-catch for interpolation as duplicate points in path can cause issues
        try
            coordX = interp1(t_old_xy, guidePathXY(:,1), t_new_xy, 'pchip'); 
            coordY = interp1(t_old_xy, guidePathXY(:,2), t_new_xy, 'pchip');
        catch ME
             warning('Interpolation failed for coordinated path XY: %s. Using linear fallback.', ME.message);
             coordX = interp1(t_old_xy, guidePathXY(:,1), t_new_xy, 'linear'); 
             coordY = interp1(t_old_xy, guidePathXY(:,2), t_new_xy, 'linear');
        end
        % Interpolate Z linearly between start and goal Z
        coordZ = linspace(pStart(3), pGoal(3), numPoints);
        coordinatedPath = [coordX(:), coordY(:), coordZ(:)];
    else 
        % Fallback: If guidePath is empty or single point, interpolate linearly in 3D
        coordinatedPath = [linspace(pStart(1),pGoal(1),numPoints)', ...
                             linspace(pStart(2),pGoal(2),numPoints)', ...
                             linspace(pStart(3),pGoal(3),numPoints)'];
    end
end
% --- Core/Utility Helper Functions ---
function planner = configurePlanner(planner)
    if isprop(planner,'SkippedSelfCollisions'),   planner.SkippedSelfCollisions = "parent"; end
    if isprop(planner,'ValidationDistance'),      planner.ValidationDistance = 0.02; end
    if isprop(planner,'MaxConnectionDistance'),   planner.MaxConnectionDistance = 0.4; end
    if isprop(planner,'MaxIterations'),           planner.MaxIterations = 8000; end
    if isprop(planner,'EnableConnectHeuristic'),  planner.EnableConnectHeuristic = true; end
    if isprop(planner,'GoalBias'),                planner.GoalBias = 0.3; end
end
function path = planOrRetry(planner, qStart, qGoal, tries)
    for t = 1:max(1,tries)
        try
            path = plan(planner, qStart, qGoal);
            if ~isempty(path), return; end
        catch ME
            fprintf('Planning attempt %d failed: %s\n', t, ME.message);
        end
    end
    error("RRT failed after %d attempts", tries);
end

function assertPathCollisionFree(robot, states, envCol)
    if isempty(envCol) || isempty(states), return; end 
    for k = 1:size(states, 1), isColliding = any(checkCollision(robot, states(k,:), envCol, "Exhaustive","on","SkippedSelfCollisions","parent"), 'all'); assert(~isColliding, "Collision in planned path at step %d", k); end
end
function mustBeFree(robot, q, envCol, msg)
    if isempty(q) || any(isnan(q)), error('Invalid config provided to mustBeFree: %s', msg); end
    isColliding = any(checkCollision(robot, q, envCol, "Exhaustive","off","SkippedSelfCollisions","parent"), 'all'); assert(~isColliding, msg);
end
function drawFrame(T, name, scale)
    if nargin < 3, scale = 0.1; end; o = T(1:3,4); R = T(1:3,1:3);
    quiver3(o(1),o(2),o(3), scale*R(1,1),scale*R(2,1),scale*R(3,1), 'r','LineWidth',2,'MaxHeadSize',0.5); hold on;
    quiver3(o(1),o(2),o(3), scale*R(1,2),scale*R(2,2),scale*R(3,2), 'g','LineWidth',2,'MaxHeadSize',0.5);
    quiver3(o(1),o(2),o(3), scale*R(1,3),scale*R(2,3),scale*R(3,3), 'b','LineWidth',2,'MaxHeadSize',0.5);
    if nargin >= 2 && ~isempty(name), text(o(1),o(2),o(3), ['  ' char(name)], 'FontSize', 12, 'Color','k','FontWeight','bold'); end
end
function S = resolveBaseIndices(robot)
    % Find the indices of the mobile base joints in the configuration vector.
    bxBody = getBody(robot, "base_x");   jx = string(bxBody.Joint.Name);
    byBody = getBody(robot, "base_y");   jy = string(byBody.Joint.Name);
    chBody = getBody(robot, "chassis");  jz = string(chBody.Joint.Name);
    
    fmt0 = robot.DataFormat; robot.DataFormat = 'struct';
    cfg = homeConfiguration(robot);
    robot.DataFormat = fmt0;
    
    jointNamesInOrder = string({cfg.JointName});
    
    ix   = find(jointNamesInOrder == jx, 1);
    iy   = find(jointNamesInOrder == jy, 1);
    iyaw = find(jointNamesInOrder == jz, 1);
    assert(~isempty(ix) && ~isempty(iy) && ~isempty(iyaw), ...
        'resolveBaseIndices: base joints not found in configuration order.');
        
    S.baseIdx    = [ix iy iyaw];
    S.baseNames  = [jx jy jz];
    S.jointNames = jointNamesInOrder; % <-- This is the corrected line
end


%% ========================================================================
%  8. HS-BI-RRT PLANNER (WITH PAPER-STYLE COLLISION)
%  ========================================================================
function [path, ok, info] = hsBiRRT_paper_v2(robot, envCol, qStart, qGoal, ee, P, baseDisks, eeSpheres)
%HSBIRRT_PAPER_V2  Focused HS-Bi-RRT using both Base Disks and EE Spheres.
% Uses a paper-style collision model inside the planner:
%   - Base = 2D disc (radius = P.baseIncircle)
%   - Rebar = capsule along TCP x-axis (radius = P.rebarRadius, length = P.rebarLen)
% Final path is still checked with full rigidBodyTree checkCollision.

    % Resolve base indices and wrap mask
    Sx = resolveBaseIndices(robot);
    ix   = Sx.baseIdx(1);
    iy   = Sx.baseIdx(2);
    iyaw = Sx.baseIdx(3);

    wrapIdx = true(1, numel(qStart));
    wrapIdx([ix iy]) = false;

    % Fill defaults (and keep geometry fields)
    P = fillDefaults(P);

    % Precompute sizes
    numBaseDisks  = size(baseDisks, 1);
    numEEspheres  = size(eeSpheres, 1);

    % Initialize Tree A (start) and Tree B (goal)
    TA.q       = qStart;
    TA.parent  = 0;
    TA.eePos   = tcpPos(robot, qStart, ee);
    TA.sigma_d = P.xi;
    TA.sigma_s = P.xi;
    TA.dIdx    = 1;
    TA.sIdx    = 1;

    TB.q       = qGoal;
    TB.parent  = 0;
    TB.eePos   = tcpPos(robot, qGoal, ee);
    TB.sigma_d = P.xi;
    TB.sigma_s = P.xi;
    TB.dIdx    = max(1, numBaseDisks);
    TB.sIdx    = max(1, numEEspheres);

    TAin = TA;
    TBin = TB;

    ok      = false;
    path    = [];
    t0      = tic;
    it      = 0;
    bestGap = inf;
    bestPair = [1 1];

    fprintf('[HS-Bi-RRT] Starting loop (maxTime = %.1f s)...\n', P.maxTime);

    while toc(t0) < P.maxTime
        it = it + 1;

        % Select source and target trees (smaller first)
        if size(TAin.q,1) <= size(TBin.q,1)
            treeSrc = 'A';
            Tin  = TAin;
            Tout = TBin;
        else
            treeSrc = 'B';
            Tin  = TBin;
            Tout = TAin;
        end

        grew      = false;
        hit       = false;
        spaceUsed = 'C';  % 'C' for C-space, 'W' for workspace

        sampleTic = tic;

        % ---------------------- Sampling --------------------------
        if rand < P.R_s
            % --- C-space sampling with Base disks ---
            dIdx = max(1, min(Tin.dIdx, numBaseDisks));
            if isempty(baseDisks)
                ctr = [Tin.q(end,ix) Tin.q(end,iy)];
                Rb  = 0.5;
            else
                ctr = baseDisks(dIdx,1:2);
                Rb  = baseDisks(dIdx,3);
            end

            % Aim one disk ahead (or behind) along disk chain
            if treeSrc == 'A'
                dAimIdx = min(dIdx+1, numBaseDisks);
            else
                dAimIdx = max(dIdx-1, 1);
            end
            aimCtr = ctr;
            if ~isempty(baseDisks) && dAimIdx > 0
                aimCtr = baseDisks(dAimIdx,1:2);
            end

            useNext = (rand < 0.7);
            aimXY   = useNext*aimCtr + (~useNext)*ctr;

            % Gaussian sample inside disk then clamp to radius
            base_xy = aimXY + (Tin.sigma_d * Rb)*randn(1,2);
            v       = base_xy - ctr;
            r       = norm(v);
            if r > max(Rb,1e-9)
                base_xy = ctr + (Rb/r)*v;
            end

            % Small random yaw relative to nearest node
            base_yaw = wrapToPi(P.alpha*(rand-0.5));

            qNearIdx = nearNonholonomic( ...
                Tin.q, [base_xy base_yaw], ix, iy, iyaw, P.alpha, P.beta);
            qNear = Tin.q(qNearIdx,:);

            qRand = qNear;
            qRand([ix iy]) = base_xy;
            qRand(iyaw)    = wrapToPi(qNear(iyaw) + base_yaw);

            armIdx = setdiff(1:numel(qNear), [ix iy iyaw]);
            if ~isempty(armIdx)
                qRand(armIdx) = qNear(armIdx) + 0.15*randn(1,numel(armIdx));
            end

            qNew      = steer(qNear, qRand, P.stepSize, wrapIdx, ix, iy, iyaw);
            spaceUsed = 'C';

        else
            % --- W-space sampling with EE spheres ---
            sIdx = max(1, min(Tin.sIdx, numEEspheres));
            if isempty(eeSpheres)
                if treeSrc == 'A'
                    TgoalEE = getTransform(robot, qGoal, ee);
                else
                    TgoalEE = getTransform(robot, qStart, ee);
                end
                ee_tgt = TgoalEE(1:3,4).';
            else
                cE = eeSpheres(sIdx,1:3);
                Re = eeSpheres(sIdx,4);
                ee_tgt = cE + (Tin.sigma_s*Re)*randn(1,3);
                dE     = norm(ee_tgt-cE);
                if dE > Re
                    ee_tgt = cE + (Re/dE)*(ee_tgt-cE);
                end
            end

            eeDists = vecnorm(Tin.eePos - ee_tgt, 2, 2);
            [~, qNearIdx] = min(eeDists);
            qNear = Tin.q(qNearIdx,:);

            Tnear = getTransform(robot, qNear, ee);
            Ttgt  = [Tnear(1:3,1:3), ee_tgt(:); 0 0 0 1];

            qNew = jacobianStep(robot, qNear, Ttgt, ee, P.stepSize, wrapIdx);
            spaceUsed = 'W';
        end

        sampleTime = toc(sampleTic);
        extendTic  = tic;

        % ----------------- Extension + collision check -------------------
        if segmentFree(robot, envCol, qNear, qNew, P.valStep, wrapIdx, ...
                       ix, iy, iyaw, ee, P)

            % Add new node
            Tin.q(end+1,:)     = qNew;
            Tin.parent(end+1)  = qNearIdx;
            Tin.eePos(end+1,:) = tcpPos(robot, qNew, ee);
            grew               = true;
            idxNew             = size(Tin.q,1);

            % --- Disk index update ---
            if ~isempty(baseDisks)
                curD_idx = Tin.dIdx;
                if treeSrc == 'A'
                    nxtD_idx = min(curD_idx+1, numBaseDisks);
                else
                    nxtD_idx = max(curD_idx-1, 1);
                end

                if nxtD_idx ~= curD_idx && numBaseDisks > 1
                    qBaseXY      = qNew([ix iy]);
                    centerCurXY  = baseDisks(curD_idx,1:2);
                    centerNxtXY  = baseDisks(nxtD_idx,1:2);
                    dCur         = norm(qBaseXY-centerCurXY);
                    dNxt         = norm(qBaseXY-centerNxtXY);
                    inNext       = dNxt <= 0.95*baseDisks(nxtD_idx,3);
                    if inNext || (dNxt+0.05 < dCur)
                        Tin.dIdx    = nxtD_idx;
                        Tin.sigma_d = P.xi_prime;
                    end
                end
            end

            % --- EE sphere index update ---
            if ~isempty(eeSpheres)
                curS = Tin.sIdx;
                if treeSrc == 'A'
                    nxtS = min(curS+1, numEEspheres);
                else
                    nxtS = max(curS-1, 1);
                end
                if nxtS ~= curS && numEEspheres > 1
                    eeNow  = Tin.eePos(end,:);
                    dCurE  = norm(eeNow - eeSpheres(curS,1:3));
                    dNxtE  = norm(eeNow - eeSpheres(nxtS,1:3));
                    inNextE = dNxtE <= 0.95*eeSpheres(nxtS,4);
                    if inNextE || (dNxtE + 0.05 < dCurE)
                        Tin.sIdx    = nxtS;
                        Tin.sigma_s = P.xi_prime;
                    end
                end
            end

            % --- Try to connect the other tree toward qNew ---
            [Tout, hit, idxOther, qOtherNew] = connectToward( ...
                robot, envCol, ee, Tout, qNew, P, wrapIdx, ix, iy, iyaw);

            % Track best near pair
            if isempty(qOtherNew) || size(qOtherNew,2) ~= size(qNew,2)
                gapNow = inf;
            else
                gapNow = distJoint_safe(qOtherNew, qNew, wrapIdx);
            end
            if gapNow < bestGap
                bestGap  = gapNow;
                bestPair = [idxNew, idxOther];
            end

            % Write back the modified trees
            if treeSrc == 'A'
                TAin = Tin; TBin = Tout;
            else
                TBin = Tin; TAin = Tout;
            end

            if hit
                if treeSrc == 'A'
                    path = stitchPath(TAin, TBin, idxNew, idxOther);
                else
                    path = stitchPath(TAin, TBin, idxOther, idxNew);
                end
                ok = true;
                break;
            end
        end

        extendTime = toc(extendTic);

        % --- Sigma adaptation ---
        if spaceUsed == 'C'
            sigmaField = 'sigma_d';
        else
            sigmaField = 'sigma_s';
        end
        if grew
            Tin.(sigmaField) = max(P.xi, Tin.(sigmaField)*(1-P.lambda));
        else
            Tin.(sigmaField) = min(P.xi_double, Tin.(sigmaField)*(1+P.lambda));
            if Tin.(sigmaField) >= P.xi_double
                Tin.(sigmaField) = P.xi_prime;
            end
        end

        if treeSrc == 'A'
            TAin = Tin;
        else
            TBin = Tin;
        end

        if P.verboseEvery > 0 && mod(it, P.verboseEvery)==0
            ngap = nearestGap(TAin.q, TBin.q, wrapIdx);
            fprintf(['[HS-Bi-RRT] it=%d (%.1fs)| |A|=%d |B|=%d | Gap=%.3f ', ...
                     '| dA=%d sA=%d | dB=%d sB=%d | SigD=%.2f SigS=%.2f ', ...
                     '| SampT=%.3fs ExtConT=%.3fs\n'], ...
                    it, toc(t0), ...
                    size(TAin.q,1), size(TBin.q,1), ...
                    ngap, TAin.dIdx, TAin.sIdx, ...
                    TBin.dIdx, TBin.sIdx, ...
                    Tin.sigma_d, Tin.sigma_s, ...
                    sampleTime, extendTime);
        end
    end

    % --- Post-loop fallback: connect best near pair if within nearRadius ---
    if ~ok && bestGap < P.nearRadius && all(bestPair>0) && ...
       size(TAin.q,1) >= bestPair(1) && size(TBin.q,1) >= bestPair(2)

        qA = TAin.q(bestPair(1),:);
        qB = TBin.q(bestPair(2),:);

        if segmentFree(robot, envCol, qA, qB, P.valStep, wrapIdx, ...
                       ix, iy, iyaw, ee, P)
            path = stitchPath(TAin, TBin, bestPair(1), bestPair(2));
            ok   = true;
            fprintf('[HS-Bi-RRT] Connected best near pair post-loop (euc-gap=%.3f).\n', bestGap);
        else
            fprintf('[HS-Bi-RRT] Fallback failed collision check (euc-gap=%.3f).\n', bestGap);
        end
    elseif ~ok
        fprintf('[HS-Bi-RRT] Failed to find solution & fallback gap (%.3f) > nearRadius (%.3f).\n', ...
            bestGap, P.nearRadius);
    end

    info = struct( ...
        'iterations', it, ...
        'bestGap',    bestGap, ...
        'nodesA',     size(TAin.q,1), ...
        'nodesB',     size(TBin.q,1));

    % --------------------------------------------------------------------
    % Nested helpers
    % --------------------------------------------------------------------
    function S = resolveBaseIndices(rb)
        bxBody = getBody(rb,"base_x");   jx = string(bxBody.Joint.Name);
        byBody = getBody(rb,"base_y");   jy = string(byBody.Joint.Name);
        chBody = getBody(rb,"chassis");  jz = string(chBody.Joint.Name);

        fmt0 = rb.DataFormat;
        rb.DataFormat = 'struct';
        cfg = homeConfiguration(rb);
        rb.DataFormat = fmt0;

        jnames = string({cfg.JointName});
        ix_ = find(jnames==jx,1);
        iy_ = find(jnames==jy,1);
        iz_ = find(jnames==jz,1);
        assert(~isempty(ix_) && ~isempty(iy_) && ~isempty(iz_), ...
            'Base joints not found.');
        S.baseIdx = [ix_ iy_ iz_];
    end

    function Pth = fillDefaults(Pin)
        D = struct( ...
            'R_s',         0.5, ...
            'lambda',      0.2, ...
            'xi',          0.05, ...
            'xi_prime',    0.75, ...
            'xi_double',   3.0, ...
            'alpha',       pi/8, ...
            'beta',        pi/8, ...
            'stepSize',    0.5, ...
            'valStep',     0.06, ...
            'nearRadius',  1.2, ...
            'maxTime',     600, ...
            'verboseEvery',20, ...
            'baseIncircle',0.35, ...
            'rebarRadius', 0.02, ...
            'rebarLen',    1.0, ...
            'baseWidth',   0.4, ...  % Replace with your col.Y
            'baseLength',  0.7, ...  % Replace with your col.X
            'baseHeight',  0.5 ...   % Replace with your col.Z
            );
        Pth = D;
        fn = fieldnames(D);
        if nargin>0 && ~isempty(Pin)
            for k_ = 1:numel(fn)
                f = fn{k_};
                if isfield(Pin,f) && ~isempty(Pin.(f))
                    Pth.(f) = Pin.(f);
                end
            end
        end
    end

    function p = tcpPos(rb, q, ee_)
        T = getTransform(rb, q, ee_);
        p = T(1:3,4).';
    end

    function a = wrapToPi(a)
        a = mod(a+pi, 2*pi) - pi;
    end

    function dq = angleAwareDiff(a, b, wrapMask)
        dq = a - b;
        dq(wrapMask) = atan2(sin(dq(wrapMask)), cos(dq(wrapMask)));
    end

    % --- joint distance helpers -----------------------------------------
    function d = distJoint(q1, q2, wrapMask)
        sz1 = size(q1); sz2 = size(q2);
        isVec1 = sz1(1)==1; isVec2 = sz2(1)==1;

        if isVec1 && ~isVec2
            D = bsxfun(@minus, q2, q1);
        elseif ~isVec1 && isVec2
            D = bsxfun(@minus, q1, q2);
        elseif isVec1 && isVec2 && all(sz1==sz2)
            D = q2 - q1;
        elseif ~isVec1 && ~isVec2 && all(sz1==sz2)
            D = q2 - q1;
        else
            error('distJoint: Incompatible input sizes.');
        end

        maskCols = find(wrapMask);
        if ~isempty(maskCols)
            for c_idx = maskCols
                D(:,c_idx) = atan2(sin(D(:,c_idx)), cos(D(:,c_idx)));
            end
        end
        d = sqrt(sum(D.^2,2));
    end

    function d = distJoint_safe(q1, q2, wrapMask)
        if isempty(q1) || isempty(q2)
            d = inf;
            return;
        end
        if isvector(q1), q1 = reshape(q1,1,[]); end
        if isvector(q2), q2 = reshape(q2,1,[]); end
        n1 = size(q1,2);
        n2 = size(q2,2);
        if n1 ~= n2
            error('distJoint:DoFMismatch', 'q1 has %d DoF, q2 has %d DoF.', n1, n2);
        end
        D = bsxfun(@minus, q1, q2);
        mask = false(1,n1);
        if ~isempty(wrapMask)
            mask(1:min(numel(wrapMask),n1)) = logical(wrapMask(1:min(numel(wrapMask),n1)));
        end
        if any(mask)
            D(:,mask) = atan2(sin(D(:,mask)), cos(D(:,mask)));
        end
        d = sqrt(sum(D.^2,2));
    end

    % --- nonholonomic nearest -------------------------------------------
    function idx = nearNonholonomic(Q, tgt, ix_, iy_, iyaw_, alpha_, beta_)
        dXY = Q(:,[ix_ iy_]) - tgt(1:2);
        L   = sqrt(sum(dXY.^2,2));

        angTo = atan2(tgt(2) - Q(:,iy_), tgt(1) - Q(:,ix_));
        dYaw = abs(wrapToPi(tgt(3) - Q(:,iyaw_)));
        dArc = abs(wrapToPi(angTo - Q(:,iyaw_)));

        penalty = zeros(size(L));
        badArc  = dArc > beta_;
        penalty(badArc) = penalty(badArc) + 0.8*L(badArc);
        badYaw  = dYaw > alpha_;
        penalty(badYaw) = penalty(badYaw) + 0.3*L(badYaw);

        d = L + penalty;

        closeIdx = L < 1e-5;
        if any(closeIdx)
            d(closeIdx) = dYaw(closeIdx);
        end

        [~, idx] = min(d);
    end

    % --- steer with bounded yaw -----------------------------------------
    function qB = steer(qA, qT, step, wrapMask, ix_, iy_, iyaw_)
        if ~islogical(wrapMask)
            m = false(1, numel(qA));
            m(wrapMask) = true;
            wrapMask = m;
        end

        yawPerMeter = 2.0;
        yawMinStep  = 5*pi/180;

        pA   = qA([ix_ iy_]);
        pT   = qT([ix_ iy_]);
        vXY  = pT - pA;
        Lxy  = norm(vXY);

        dYaw = atan2(sin(qT(iyaw_) - qA(iyaw_)), cos(qT(iyaw_) - qA(iyaw_)));

        armIdx = setdiff(1:numel(qA), [ix_ iy_ iyaw_]);
        qa_arm = qA(armIdx);
        qt_arm = qT(armIdx);
        dq_arm = qt_arm - qa_arm;

        if ~isempty(armIdx)
            angCols = armIdx(wrapMask(armIdx));
            if ~isempty(angCols)
                loc = ismember(armIdx, angCols);
                dq_arm(loc) = atan2( ...
                    sin(qT(armIdx(loc)) - qA(armIdx(loc))), ...
                    cos(qT(armIdx(loc)) - qA(armIdx(loc))));
            end
        end

        moveDist = min(step, Lxy);
        if Lxy > 1e-9
            sMove = moveDist / Lxy;
            pB    = pA + sMove*vXY;
        else
            sMove = 0;
            pB    = pA;
        end

        if moveDist > 1e-9
            yawCap = yawPerMeter * moveDist;
        else
            yawCap = yawMinStep;
        end
        dYawStep = sign(dYaw) * min(abs(dYaw), yawCap);

        sYaw = min(1.0, abs(dYawStep) / max(abs(dYaw),1e-9));
        s    = max(sMove, sYaw);

        qB          = qA;
        qB([ix_ iy_]) = pB;
        qB(iyaw_)   = qA(iyaw_) + dYawStep;

        if ~isempty(armIdx)
            qB(armIdx) = qa_arm + s*dq_arm;
        end

        wrapCols = find(wrapMask);
        if ~isempty(wrapCols)
            qB(wrapCols) = atan2(sin(qB(wrapCols)), cos(qB(wrapCols)));
        end
    end

    % --- paper-style collision model ------------------------------------
    function coll = paperCollision(q, rb, ee_, env, baseIncircle, rebarRadius, rebarLen)
        % This nested function has access to 'P' from the parent hsBiRRT scope.
        % We pass 'P' down to the external helper to access base dimensions and zFloor.
        coll = paperCollisionSimple(q, rb, ee_, env, ...
                                    baseIncircle, rebarRadius, rebarLen, P);
    end
    % --- segmentFree using paperCollision -------------------------------
    function okSeg = segmentFree(rb, env, qa, qb, step, wrapMask, ix_, iy_, iyaw_, ee_, P_)
        okSeg = true;
    
        % --- setup indices / interpolation ---
        armIdx_ = setdiff(1:numel(qa), [ix_ iy_ iyaw_]);
        angCols = find(wrapMask);
    
        pA   = qa([ix_ iy_]);
        pB   = qb([ix_ iy_]);
        vXY  = pB - pA;
        Lxy  = norm(vXY);
    
        dYaw = atan2(sin(qb(iyaw_) - qa(iyaw_)), cos(qb(iyaw_)));
    
        qa_arm = []; qb_arm = []; dq_arm = [];
        if ~isempty(armIdx_)
            qa_arm = qa(armIdx_);
            qb_arm = qb(armIdx_);
            dq_arm = qb_arm - qa_arm;
    
            if ~isempty(angCols)
                [~, loc] = intersect(armIdx_, angCols);
                aCols    = armIdx_(loc);
                dq_arm(loc) = atan2( ...
                    sin(qb(aCols) - qa(aCols)), ...
                    cos(qb(aCols) - qa(aCols)));
            end
        end
    
        yawStep = pi/30;
        armStep = 0.08;
    
        n_xy  = max(2, ceil(Lxy / max(step,1e-6)));
        n_yaw = max(2, ceil(abs(dYaw) / yawStep));
        n_arm = 2;
        if ~isempty(dq_arm)
            n_arm = max(2, ceil(norm(dq_arm,2) / armStep));
        end
    
        n = max([n_xy, n_yaw, n_arm]);
    
        q = qa;
        for k_ = 1:n
            s = k_ / n;
    
            % --- interpolate config along segment ---
            q([ix_ iy_]) = pA + s*vXY;
            q(iyaw_)     = qa(iyaw_) + s*dYaw;
            q(iyaw_)     = atan2(sin(q(iyaw_)), cos(q(iyaw_)));
    
            if ~isempty(armIdx_)
                q(armIdx_) = qa_arm + s*dq_arm;
            end
    
            if ~isempty(angCols)
                q(angCols) = atan2(sin(q(angCols)), cos(q(angCols)));
            end
    
            % --- ground-plane constraint (rebar must stay above zFloor) -----
            if isfield(P_, 'zFloor')
                Ttcp = getTransform(rb, q, ee_);
                o    = Ttcp(1:3,4);
                Rtcp = Ttcp(1:3,1:3);
            
                % Rebar along TCP Y-axis (consistent with paper/validator)
                dir  = Rtcp(:,2);
                dir  = dir / max(norm(dir), 1e-9);
            
                p0   = o - 0.5 * P_.rebarLen * dir;
                p1   = o + 0.5 * P_.rebarLen * dir;
                zMinBar = min(p0(3), p1(3)) - P_.rebarRadius - rebarEnvelopeSag(P_);
            
                if zMinBar < P_.zFloor
                    okSeg = false;
                    return;
                end
            end
    
            % --- Base disc + rebar capsule vs obstacles ---------------------
            if paperCollision(q, rb, ee_, env, ...
                              P_.baseIncircle, P_.rebarRadius, P_.rebarLen)
                okSeg = false;
                return;
            end
    
            % --- Arm key-links as spheres (cheap arm collision) -------------
            if isfield(P_,'armRadius') && P_.armRadius > 0 && ...
               isfield(P_,'armKeyNames') && ~isempty(P_.armKeyNames)
                if armKeyCollision(q, rb, env, P_.armKeyNames, P_.armRadius,0.003)
                    okSeg = false;
                    return;
                end
            end
        end
    
        % --- also verify the exact end state with same models ---------------
    
        % zFloor again at qb
        if isfield(P_, 'zFloor')
            Ttcp = getTransform(rb, qb, ee_);
            o    = Ttcp(1:3,4);
            Rtcp = Ttcp(1:3,1:3);
    
            dir  = Rtcp(:,2);
            dir  = dir / max(norm(dir), 1e-9);
    
            p0   = o - 0.5 * P_.rebarLen * dir;
            p1   = o + 0.5 * P_.rebarLen * dir;
            zMinBar = min(p0(3), p1(3)) - P_.rebarRadius - rebarEnvelopeSag(P_);
    
            if zMinBar < P_.zFloor
                okSeg = false;
                return;
            end
        end
    
        % Base + rebar vs obstacles at qb
        if paperCollision(qb, rb, ee_, env, ...
                          P_.baseIncircle, P_.rebarRadius, P_.rebarLen)
            okSeg = false;
            return;
        end
    
        % Arm key-links vs obstacles at qb
        if isfield(P_,'armRadius') && P_.armRadius > 0 && ...
           isfield(P_,'armKeyNames') && ~isempty(P_.armKeyNames)
            if armKeyCollision(qb, rb, env, P_.armKeyNames, P_.armRadius)
                okSeg = false;
                return;
            end
        end
    end


    function okCfg = checkConfig(rb, env, q)
        if any(isnan(q))
            okCfg = false;
            return;
        end
        okCfg = ~paperCollision(q, rb, ee, env, ...
                                P.baseIncircle, P.rebarRadius, P.rebarLen);
    end

    % --- small Jacobian step toward workspace target --------------------
    function qN = jacobianStep(rb, q0, Ttgt, ee_, step, wrapMask)
        qN = q0;
        for it_ = 1:2
            T = getTransform(rb, qN, ee_);
            e = Ttgt(1:3,4) - T(1:3,4);
            if norm(e) < 1e-3, break; end
            try
                J = geometricJacobian(rb, qN, ee_);
            catch
                break;
            end
            Jp = J(1:3,:);
            dq = (Jp.'/(Jp*Jp.' + 1e-4*eye(3))) * e;
            s  = norm(dq);
            if s > step
                dq = dq*(step/s);
            end
            qN = qN + dq.';
            cols = find(wrapMask);
            if ~isempty(cols)
                qN(cols) = atan2(sin(qN(cols)), cos(qN(cols)));
            end
        end
    end

    % --- connectToward with step-by-step extension ----------------------
    function [Tout, hit, idxOther, qOtherNew] = connectToward( ...
        rb, env, ee_, Tin, qGoal, P_, wrapMask, ix_, iy_, iyaw_)

        hit       = false;
        qOtherNew = [];
        idxOther  = [];
        Tout      = Tin;

        idxNear = nearNonholonomic(Tout.q, [qGoal(ix_) qGoal(iy_) qGoal(iyaw_)], ...
                                   ix_, iy_, iyaw_, P_.alpha, P_.beta);
        qSteer  = Tout.q(idxNear,:);
        idxPrev = idxNear;

        maxSteps  = 10;
        stepCount = 0;

        while stepCount < maxSteps
            stepCount = stepCount + 1;

            qNext = steer(qSteer, qGoal, P_.stepSize, wrapMask, ix_, iy_, iyaw_);

            if distJoint(qNext, qSteer, wrapMask) < 1e-9
                break;
            end

            if ~segmentFree(rb, env, qSteer, qNext, P_.valStep, wrapMask, ...
                            ix_, iy_, iyaw_, ee_, P_)
                break;
            end

            Tout.q(end+1,:)     = qNext;
            Tout.parent(end+1)  = idxPrev;
            Tout.eePos(end+1,:) = tcpPos(rb, qNext, ee_);
            idxPrev             = size(Tout.q,1);
            qOtherNew           = qNext;
            idxOther            = idxPrev;

            if distJoint(qNext, qGoal, wrapMask) <= P_.stepSize
                if segmentFree(rb, env, qNext, qGoal, P_.valStep, wrapMask, ...
                               ix_, iy_, iyaw_, ee_, P_)
                    Tout.q(end+1,:)     = qGoal;
                    Tout.parent(end+1)  = idxPrev;
                    Tout.eePos(end+1,:) = tcpPos(rb, qGoal, ee_);
                    idxOther            = size(Tout.q,1);
                    qOtherNew           = qGoal;
                    hit = true;
                end
                break;
            end

            qSteer = qNext;
        end
    end

    % --- path stitching & utilities -------------------------------------
    function path_ = stitchPath(treeA_, treeB_, idxA_, idxB_)
        A = unwind(treeA_, idxA_);
        B = unwind(treeB_, idxB_);
        path_ = [A; flipud(B)];
        if size(path_,1) > 1 && ...
           distJoint(path_(size(A,1),:), path_(size(A,1)+1,:), wrapIdx) < 1e-6
            path_(size(A,1)+1,:) = [];
        end
    end

    function Pth = unwind(tree_, idx_)
        if idx_ > size(tree_.q,1) || idx_ < 1
            error('Invalid index in unwind: %d', idx_);
        end
        Pth = tree_.q(idx_,:);
        current_idx = idx_;
        while current_idx ~= 1 && tree_.parent(current_idx) ~= 0
            parent_idx = tree_.parent(current_idx);
            if parent_idx < 1 || parent_idx >= current_idx
                warning('Bad parent idx %d for node %d', parent_idx, current_idx);
                break;
            end
            Pth = [tree_.q(parent_idx,:); Pth];
            current_idx = parent_idx;
        end
    end

    function gap = nearestGap(QA, QB, wrapMask)
        if isempty(QA) || isempty(QB)
            gap = inf;
            return;
        end
        gap = inf;
        startIdx = max(1, size(QA,1)-20);
        for ii = startIdx:size(QA,1)
            d = distJoint(QA(ii,:), QB, wrapMask);
            m = min(d);
            if m < gap
                gap = m;
            end
        end
    end
end

% --- Grid and Path Utilities ---
function occ = buildOccGridFromEnvS(envS_in, xlimW, ylimW, res, inflateR)
    nx = ceil(diff(xlimW)/res)+1; 
    ny = ceil(diff(ylimW)/res)+1; 
    occ.res = res; 
    occ.xv = linspace(xlimW(1),xlimW(2),nx); 
    occ.yv = linspace(ylimW(1),ylimW(2),ny);
    
    [XX,YY] = meshgrid(occ.xv, occ.yv); 
    BW = false(ny,nx);
    
    % Define the height of the mobile base (e.g., 0.5 meters)
    % Objects purely above this height will be IGNORED by the 2D map.
    baseHeightThreshold = 0.5; 
    for i_=1:numel(envS_in)
        co = envS_in{i_}.CollisionObj; 
        T = co.Pose;
        
        % --- Check Height Overlap First ---
        z_center = T(3,4);
        if isa(co,'collisionBox')
            z_half = co.Z/2;
            z_min = z_center - z_half;
            z_max = z_center + z_half;
        elseif isa(co,'collisionCylinder')
            z_min = z_center - co.Height/2; % Assuming centered origin
            z_max = z_center + co.Height/2;
            % Note: MATLAB collisionCylinder origin is usually the center
        elseif isa(co,'collisionSphere')
            z_min = z_center - co.Radius;
            z_max = z_center + co.Radius;
        else
            z_min = -inf; z_max = inf; % Unknown shape, assume blocking
        end
        
        % If the object is completely above the robot base, SKIP IT
        if z_min > baseHeightThreshold
            continue; 
        end
        % ----------------------------------
        if isa(co,'collisionBox')
            dims=[co.X,co.Y]; 
            c_=T(1:2,4).'; 
            R_=T(1:2,1:2); 
            p=[XX(:)-c_(1),YY(:)-c_(2)]/R_.'; 
            hx=dims(1)/2; hy=dims(2)/2; 
            BW = BW | reshape(abs(p(:,1))<=hx & abs(p(:,2))<=hy, size(XX));
            
        elseif isa(co,'collisionCylinder')||isa(co,'collisionSphere')
            R_=co.Radius; 
            c_=T(1:2,4).'; 
            BW = BW | (XX-c_(1)).^2+(YY-c_(2)).^2 <= R_.^2; 
        end
    end
    
    if inflateR>0
        occ.grid = imdilate(BW, strel('disk', max(1, round(inflateR/res)), 0)); 
    else
        occ.grid = BW; 
    end
end
function [xlimW, ylimW] = envBoundsXY(envS_in, sxy, gxy, pad) 
    xs = [sxy(1) gxy(1)]; ys = [sxy(2) gxy(2)]; for i_=1:numel(envS_in), co = envS_in{i_}.CollisionObj; T = co.Pose;
        if isa(co,'collisionBox'), c = T(1:2,4).'; h=[co.X co.Y]/2; xs=[xs, c(1)-h, c(1)+h]; ys=[ys, c(2)-h, c(2)+h];
        elseif isa(co,'collisionCylinder')||isa(co,'collisionSphere'), c = T(1:2,4).'; R_ = co.Radius; xs=[xs, c(1)-R_, c(1)+R_]; ys=[ys, c(2)-R_, c(2)+R_]; end; end
    xlimW = [min(xs)-pad, max(xs)+pad]; ylimW = [min(ys)-pad, max(ys)+pad];
end
function [sIJ,gIJ] = world2grid(sxy, gxy, occ)
    find_last = @(v, val) find(v <= val, 1, 'last'); s_row = find_last(occ.yv, sxy(2)); s_col = find_last(occ.xv, sxy(1)); g_row = find_last(occ.yv, gxy(2)); g_col = find_last(occ.xv, gxy(1));
    s_row=max(1,min(s_row,numel(occ.yv))); s_col=max(1,min(s_col,numel(occ.xv))); g_row=max(1,min(g_row,numel(occ.yv))); g_col=max(1,min(g_col,numel(occ.xv))); sIJ = [s_row, s_col]; gIJ = [g_row, g_col];
end
function ijFree = ensureFreeIJ(ij, occ)
    H=size(occ.grid,1); W=size(occ.grid,2); i0=min(max(round(ij(1)),1),H); j0=min(max(round(ij(2)),1),W); if ~occ.grid(i0,j0), ijFree=[i0 j0]; return; end
    q=java.util.LinkedList(); q.add([i0 j0]); visited=containers.Map('KeyType','char','ValueType','logical'); visited(mat2str([i0 j0]))=true; minDist=inf; ijFree=[NaN NaN];
    while ~q.isEmpty(), curr=q.remove(); i=curr(1); j=curr(2); if ~occ.grid(i,j), dist=hypot(i-ij(1),j-ij(2)); if dist<minDist,minDist=dist; ijFree=[i j];end; end
        if hypot(i-i0,j-j0)>minDist+2,continue;end; for di=-1:1,for dj=-1:1, if di==0&&dj==0,continue;end; ni=i+di; nj=j+dj; if ni>=1&&ni<=H&&nj>=1&&nj<=W, key=mat2str([ni nj]); if ~isKey(visited,key),visited(key)=true;q.add([ni nj]);end; end; end; end; end
    if isinf(minDist), warning('ensureFreeIJ: No free cell near [%d %d]', ij(1), ij(2)); end
end
function [path, ok] = aStar8(occ, sIJ, gIJ)
    G=~occ.grid; [H,W]=size(G); s=sub2ind([H,W],sIJ(1),sIJ(2)); g=sub2ind([H,W],gIJ(1),gIJ(2)); if ~G(s)||~G(g),path=[];ok=false;fprintf('A* Error: Start/Goal in collision.\n');return;end
    nb=[-1 -1;-1 0;-1 1;0 -1;0 1;1 -1;1 0;1 1]; cost=[sqrt(2) 1 sqrt(2) 1 1 sqrt(2) 1 sqrt(2)]; cameFrom=containers.Map('KeyType','int32','ValueType','int32');
    gScore=inf(H*W,1); gScore(s)=0; fScore=inf(H*W,1); fScore(s)=heuristic(s,g,H,W,occ); pq=PriorityQueue(); pq.insert(s,fScore(s)); openSet=containers.Map('KeyType','int32','ValueType','double'); openSet(s)=fScore(s); 
    while ~pq.isEmpty(), current=pq.extractMin(); remove(openSet,current); if current==g,path=reconstruct_path(cameFrom,current,[H W]);ok=true;return;end; [ci,cj]=ind2sub([H W],current);
        for n=1:8, ni=ci+nb(n,1); nj=cj+nb(n,2); if ni<1||ni>H||nj<1||nj>W,continue;end; neighbor=sub2ind([H W],ni,nj); if ~G(neighbor),continue;end; tentative_gScore=gScore(current)+cost(n);
            if tentative_gScore<gScore(neighbor), cameFrom(neighbor)=current; gScore(neighbor)=tentative_gScore; fNew=tentative_gScore+heuristic(neighbor,g,H,W,occ); fScore(neighbor)=fNew;
                 if isKey(openSet,neighbor),pq.decreaseKey(neighbor,fNew); else,pq.insert(neighbor,fNew); openSet(neighbor)=fNew; end; end; end; end
    path=[]; ok=false; fprintf('A* Error: Failed to find path.\n');
    function h=heuristic(a,b,H,W,occ_), [ia,ja]=ind2sub([H W],a); [ib,jb]=ind2sub([H W],b); h=hypot(occ_.xv(ja)-occ_.xv(jb), occ_.yv(ia)-occ_.yv(ib)); end
    function P=reconstruct_path(cameFrom_,current,sz), total_path=current; while isKey(cameFrom_,current),current=cameFrom_(current);total_path=[current total_path];end; [ii,jj]=ind2sub(sz,total_path.'); P=[ii(:) jj(:)]; end
end
function pathXY = grid2world(pathIJ, occ), if isempty(pathIJ), pathXY=[]; return; end; pathXY=[occ.xv(pathIJ(:,2)).', occ.yv(pathIJ(:,1)).']; end
function out = shortcutSmoothing(P, occ)
    if size(P,1)<=2, out=P; return; end; out = P(1,:); i = 1; while i < size(P,1), j = size(P,1); 
        while j > i+1, if collisionFreeSegment(P(i,:),P(j,:),occ), out = [out; P(j,:)]; i = j; break; end; j = j-1; end %#ok<AGROW>
        if j == i+1, out = [out; P(i+1,:)]; i = i+1; end; if i >= size(P,1), break; end; end %#ok<AGROW>
end
function ok = collisionFreeSegment(pA, pB, occ)
    L=hypot(pB(1)-pA(1),pB(2)-pA(2)); n=max(2,ceil(L/occ.res)); xs=linspace(pA(1),pB(1),n); ys=linspace(pA(2),pB(2),n); ok=true;
    for k_=1:n, [r,c]=world2grid_scalar([xs(k_) ys(k_)], occ); if r<1||r>size(occ.grid,1)||c<1||c>size(occ.grid,2)||occ.grid(r,c), ok=false; return; end; end
end
function [i,j] = world2grid_scalar(p, occ) 
    ix=find(occ.xv<=p(1),1,'last'); iy=find(occ.yv<=p(2),1,'last'); i=max(1,min(iy,size(occ.grid,1))); j=max(1,min(ix,size(occ.grid,2)));
end
function pts = resamplePolylineByStep(XY, step), if isempty(XY)||size(XY,1)<=1,pts=XY;return;end; seg=sqrt(sum(diff(XY).^2,2)); L=[0;cumsum(seg)]; s=0:step:L(end); if isempty(s)||s(end)<L(end),s=[s L(end)];end; pts=interp1(L,XY,unique(s),'linear'); end

function d = minClearance2D(p, env)
    d = inf;
    p = p(:).';
    
    % --- CONFIGURATION ---
    % Objects with TOP below this are floor (IGNORE)
    ignoreFloorThreshold = 0.2; 
    
    % Ignore obstacles whose lower surface is above the base.
    % The mobile base is roughly 0.4-0.5m tall. 
    % We set this to 0.6m to be safe.
    ignoreCeilingThreshold = 0.6; 
    % ---------------------

    for i = 1:numel(env)
        entry = env{i};
        if isstruct(entry) && isfield(entry,'CollisionObj')
            co = entry.CollisionObj;
        else
            co = entry;
        end
        T = co.Pose;
        z_center = T(3,4);
        
        % --- CALCULATE Z EXTENTS ---
        if isa(co,'collisionBox')
            z_half = co.Z/2;
        elseif isa(co,'collisionCylinder')
            z_half = co.Height/2;
        elseif isa(co,'collisionSphere')
            z_half = co.Radius;
        else
            z_half = inf; % Unknown shape, treat as infinite wall
        end
        
        z_min = z_center - z_half;
        z_max = z_center + z_half;
        
        % --- FILTERING STEP ---
        
        % 1. Floor Filter: If the object is fully below the floor threshold
        if z_max < ignoreFloorThreshold
            continue; 
        end

        % Ignore obstacles that start above the robot base.
        if z_min > ignoreCeilingThreshold
            continue;
        end
        
        % ----------------------
        
        % Standard 2D Distance Calculation
        if isa(co,'collisionBox')
            R = T(1:2,1:2);
            t = T(1:2,4).';
            pL = (p - t)/R.';
            h  = [co.X, co.Y]/2;
            q  = abs(pL) - h;
            di = norm(max(q,0));
            if all(q <= 0)
                di = -min(max(-q));
            end
        elseif isa(co,'collisionSphere') || isa(co,'collisionCylinder')
            c  = T(1:2,4).';
            di = norm(p - c) - co.Radius;
        else
            continue;
        end
        d = min(d, di);
    end
    if isinf(d)
        d = 100;
    end
end

function d = minClearance3D(p, env)
    d = inf;
    p = p(:).';
    for i = 1:numel(env)
        entry = env{i};
        if isstruct(entry) && isfield(entry,'CollisionObj')
            co = entry.CollisionObj;
        else
            co = entry;
        end
        T = co.Pose;

        if isa(co,'collisionBox')
            R = T(1:3,1:3);
            t = T(1:3,4).';
            pL = (p - t)/R.';
            h  = [co.X, co.Y, co.Z]/2;
            q  = abs(pL) - h;
            di = norm(max(q,0));
            if all(q <= 0)
                di = max(q);  % negative inside
            end

        elseif isa(co,'collisionSphere')
            c  = T(1:3,4).';
            di = norm(p - c) - co.Radius;

        elseif isa(co,'collisionCylinder')
            R  = T(1:3,1:3);
            t  = T(1:3,4).';
            pL = (p - t)/R.';
            r  = co.Radius;
            h2 = co.Height/2;
            vec_d = [hypot(pL(1),pL(2)) - r, abs(pL(3)) - h2];
            di    = norm(max(vec_d,0)) + min(max(vec_d(1),vec_d(2)),0);

        else
            continue;
        end
        d = min(d, di);
    end
    if isinf(d)
        d = 100;
    end
end

function drawFocusRegions(focus, ax) 
    if nargin<2||isempty(ax),ax=gca;end; hold(ax,'on'); grid(ax,'on');
    if isfield(focus,'baseCenters')&&~isempty(focus.baseCenters), baseR=focus.baseRadiusVec(:); th=linspace(0,2*pi,40); baseCol=[1 0 1]; baseAlpha=0.15;
        for k=1:size(focus.baseCenters,1), cx=focus.baseCenters(k,1); cy=focus.baseCenters(k,2); if k<=numel(baseR),Rk=baseR(k);else Rk=0.1;end; xv=cx+Rk*cos(th); yv=cy+Rk*sin(th); patch(ax,xv,yv,zeros(size(xv)),baseCol,'FaceAlpha',baseAlpha,'EdgeColor',baseCol*0.8,'LineWidth',1); end
        if size(focus.baseCenters,1)>=2, plot3(ax,focus.baseCenters(:,1),focus.baseCenters(:,2),zeros(size(focus.baseCenters,1),1),'m--','LineWidth',1); end; end
    if isfield(focus,'eeCenters')&&~isempty(focus.eeCenters), eeR=focus.eeRadiusVec(:); eeCol=[0 0.6 1]; eeAlpha=0.12; [XS,YS,ZS]=sphere(20); 
        for k=1:size(focus.eeCenters,1), c=focus.eeCenters(k,:); if k<=numel(eeR),Rk=eeR(k);else Rk=0.1;end; surf(ax,c(1)+Rk*XS,c(2)+Rk*YS,c(3)+Rk*ZS,'FaceAlpha',eeAlpha,'EdgeColor','none','FaceColor',eeCol); plot3(ax,c(1),c(2),c(3),'k.','MarkerSize',10); end
         if size(focus.eeCenters,1)>=2, plot3(ax,focus.eeCenters(:,1),focus.eeCenters(:,2),focus.eeCenters(:,3),'b--','LineWidth',1); end; end
    axis(ax,'equal');
end




function S = sampleAlongPolyline(XY, K), if isempty(XY),S=[];return;end; if size(XY,1)<=1,S=repmat(XY,K,1);return;end; seg=sqrt(sum(diff(XY).^2,2)); L=[0;cumsum(seg)]; s=linspace(0,L(end),K).'; S=interp1(L,XY,s,'linear'); end
function ok = losFreePointSegment(pA, pB, envObs, rSafe, step)
    if nargin<5,step=0.05;end; L=norm(pB-pA); if L<1e-9,ok=(minClearance3D(pA,envObs)>=rSafe);return;end; n=max(2,ceil(L/step)); ok=true; for k=0:n, pk=pA+(k/n)*(pB-pA); if minClearance3D(pk,envObs)<rSafe,ok=false;return;end; end
end

function [trace, qLastFree] = guardedApproachToolAxis(robot, qStart, goalPose, envCol, ik, ee, w, step)
    qStart = double(qStart);
    trace = qStart;
    q = qStart;
    Rgoal = goalPose(1:3,1:3);
    tGoal = goalPose(1:3,4);
    zToolWorld = Rgoal(:,3); 
    Tcur = getTransform(robot, q, ee);
    t = Tcur(1:3,4);
    dirVec = sign(step)*zToolWorld;  mag = abs(step);
    while dot(tGoal - t, dirVec) > 1e-4
        distToGoal = norm(tGoal - t);
        ds = min(mag, distToGoal);
        tNew = t + ds*dirVec;
        Tnew = [Rgoal, tNew; 0 0 0 1];
        qNew = double(ik(ee, Tnew, w, q));
        isColliding = any(checkCollision(robot, qNew, envCol, ...
            "Exhaustive","off","SkippedSelfCollisions","parent"), 'all');
        if isColliding
            qLastFree = q; return;
        else
            q = qNew;
            t = getTransform(robot, q, ee); t = t(1:3,4);
            trace = [trace; q]; %#ok<AGROW>
        end
    end
    qLastFree = q;
end

function metrics = computeComprehensiveMetrics(robot, allStates, traj1, traj2, traj3, traj4, traj5, endEffector)
% Compute comprehensive metrics for path comparison
    
    % Resolve base indices
    Sx = resolveBaseIndices(robot);
    ix = Sx.baseIdx(1); iy = Sx.baseIdx(2); iyaw = Sx.baseIdx(3);
    
    metrics = struct();
    
    % === Basic counts ===
    metrics.total_waypoints = size(allStates, 1);
    metrics.waypoints_per_plan = struct(...
        'plan1', size(traj1,1), ...
        'plan2', size(traj2,1), ...
        'plan3', size(traj3,1), ...
        'plan4', size(traj4,1), ...
        'plan5', size(traj5,1) ...
    );
    
    % === Base metrics ===
    base_xy = allStates(:, [ix iy]);
    base_yaw = allStates(:, iyaw);
    
    % Base path length (XY)
    dxy = sqrt(sum(diff(base_xy, 1, 1).^2, 2));
    metrics.base.total_length_xy = sum(dxy);
    
    % Base yaw changes
    dyaw = atan2(sin(diff(base_yaw)), cos(diff(base_yaw)));
    metrics.base.total_yaw_change = sum(abs(dyaw));
    metrics.base.max_yaw_rate = max(abs(dyaw ./ [diff(dxy); 1e-6]));
    
    % Base bounding box
    metrics.base.bbox_x = [min(base_xy(:,1)), max(base_xy(:,1))];
    metrics.base.bbox_y = [min(base_xy(:,2)), max(base_xy(:,2))];
    metrics.base.bbox_area = diff(metrics.base.bbox_x) * diff(metrics.base.bbox_y);
    
    % === TCP metrics ===
    tcp_positions = zeros(size(allStates,1), 3);
    tcp_rpy = zeros(size(allStates,1), 3); % [roll, pitch, yaw]
    
    for i = 1:size(allStates,1)
        T = getTransform(robot, allStates(i,:), endEffector);
        tcp_positions(i,:) = T(1:3,4)';
        
        % Extract RPY from rotation matrix
        R = T(1:3,1:3);
        tcp_rpy(i,1) = atan2(R(3,2), R(3,3)); % roll
        tcp_rpy(i,2) = asin(-R(3,1));         % pitch  
        tcp_rpy(i,3) = atan2(R(2,1), R(1,1)); % yaw
    end
    
    % TCP path length
    dtcp = sqrt(sum(diff(tcp_positions, 1, 1).^2, 2));
    metrics.tcp.total_length = sum(dtcp);
    
    % TCP height statistics
    metrics.tcp.height_min = min(tcp_positions(:,3));
    metrics.tcp.height_max = max(tcp_positions(:,3));
    metrics.tcp.height_range = metrics.tcp.height_max - metrics.tcp.height_min;
    metrics.tcp.height_std = std(tcp_positions(:,3));
    
    % TCP orientation changes
    metrics.tcp.roll_range = range(tcp_rpy(:,1));
    metrics.tcp.pitch_range = range(tcp_rpy(:,2)); 
    metrics.tcp.yaw_range = range(tcp_rpy(:,3));
    metrics.tcp.roll_std = std(tcp_rpy(:,1));
    metrics.tcp.pitch_std = std(tcp_rpy(:,2));
    metrics.tcp.yaw_std = std(tcp_rpy(:,3));
    
    % TCP plane changes (XY movement vs Z movement)
    dtcp_xy = sqrt(sum(diff(tcp_positions(:,1:2), 1, 1).^2, 2));
    dtcp_z = abs(diff(tcp_positions(:,3)));
    metrics.tcp.total_xy_movement = sum(dtcp_xy);
    metrics.tcp.total_z_movement = sum(dtcp_z);
    metrics.tcp.xy_to_z_ratio = metrics.tcp.total_xy_movement / max(metrics.tcp.total_z_movement, 1e-6);
    
    % === Path smoothness metrics ===
    % Jerk-like metrics (second derivative of position)
    if size(tcp_positions,1) > 2
        accel = diff(tcp_positions, 2, 1);
        metrics.smoothness.max_acceleration = max(sqrt(sum(accel.^2, 2)));
        metrics.smoothness.mean_acceleration = mean(sqrt(sum(accel.^2, 2)));
    else
        metrics.smoothness.max_acceleration = 0;
        metrics.smoothness.mean_acceleration = 0;
    end
    
    % === Plan-specific metrics ===
    if ~isempty(traj4)
        % Plan 4 specific metrics (HS-Bi-RRT)
        base_xy_4 = traj4(:, [ix iy]);
        dxy_4 = sqrt(sum(diff(base_xy_4, 1, 1).^2, 2));
        metrics.plan4.base_length = sum(dxy_4);
        metrics.plan4.waypoints = size(traj4,1);
        
        % TCP length for plan 4
        tcp_len_4 = 0;
        T_prev = getTransform(robot, traj4(1,:), endEffector);
        for k = 2:size(traj4,1)
            T_cur = getTransform(robot, traj4(k,:), endEffector);
            tcp_len_4 = tcp_len_4 + norm(T_cur(1:3,4) - T_prev(1:3,4));
            T_prev = T_cur;
        end
        metrics.plan4.tcp_length = tcp_len_4;
    end
    
    % === Efficiency metrics ===
    metrics.efficiency.tcp_to_base_ratio = metrics.tcp.total_length / max(metrics.base.total_length_xy, 1e-6);
    metrics.efficiency.waypoints_per_meter = metrics.total_waypoints / max(metrics.tcp.total_length, 1e-6);
    
    % === Store raw data for plotting ===
    metrics.raw.base_xy = base_xy;
    metrics.raw.base_yaw = base_yaw;
    metrics.raw.tcp_positions = tcp_positions;
    metrics.raw.tcp_rpy = tcp_rpy;
    metrics.raw.timestamps = (0:size(allStates,1)-1)'; % Relative time indices
    
    fprintf('Comprehensive metrics computed:\n');
    fprintf('  TCP: %.3f m total, Z-range: %.3f m\n', metrics.tcp.total_length, metrics.tcp.height_range);
    fprintf('  Base: %.3f m total, Yaw: %.3f rad\n', metrics.base.total_length_xy, metrics.base.total_yaw_change);
    fprintf('  Waypoints: %d total\n', metrics.total_waypoints);
end

function envOut = inflateEnv(envIn, m, excludeNames)
% Inflate boxes/cylinders/spheres by margin m (meters).
% excludeNames: string or string array of env.Name to skip (e.g., "floor").

    if nargin < 3, excludeNames = strings(0,1); end
    if ischar(excludeNames), excludeNames = string(excludeNames); end

    envOut = envIn;
    for i = 1:numel(envIn)
        % unwrap
        hadName = isstruct(envIn{i}) && isfield(envIn{i},'Name');
        if hadName, name = string(envIn{i}.Name); co = envIn{i}.CollisionObj;
        else,      name = "";                    co = envIn{i};
        end

        if any(strcmpi(name, excludeNames))
            coNew = co; % skip inflation
        else
            if isa(co, 'collisionBox')
                coNew = collisionBox(co.X + 2*m, co.Y + 2*m, co.Z + 2*m);
                coNew.Pose = co.Pose;
            elseif isa(co, 'collisionCylinder')
                coNew = collisionCylinder(co.Radius + m, co.Height + 2*m);
                coNew.Pose = co.Pose;
            elseif isa(co, 'collisionSphere')
                coNew = collisionSphere(co.Radius + m);
                coNew.Pose = co.Pose;
            else
                coNew = co; % unknown: leave untouched
            end
        end

        % re-wrap
        if hadName
            envOut{i} = struct('Name', char(name), 'CollisionObj', coNew);
        else
            envOut{i} = coNew;
        end
    end
end


function plotEdgeSamples(robot, path, ee, ax, step)
    S = resolveBaseIndices(robot);
    ix_ = S.baseIdx(1); iy_ = S.baseIdx(2); iyaw_ = S.baseIdx(3);
    wrapMask = true(1, size(path,2)); wrapMask([ix_ iy_]) = false;

    for e = 1:size(path,1)-1
        qa = path(e,:); qb = path(e+1,:);
        pts = reconstructSamples(qa, qb, step, wrapMask, ix_, iy_, iyaw_);
        eepts = zeros(size(pts,1),3);
        for i = 1:size(pts,1)
            T = getTransform(robot, pts(i,:), ee);
            eepts(i,:) = T(1:3,4)';
        end
        plot3(ax, eepts(:,1), eepts(:,2), eepts(:,3), 'r.', 'MarkerSize', 6); % breadcrumbs
    end
end

function Q = reconstructSamples(qa, qb, step, wrapMask, ix_, iy_, iyaw_)
    pA = qa([ix_ iy_]); pB = qb([ix_ iy_]); v = pB - pA;
    L  = norm(v); n = max(2, ceil(L/max(step,1e-6)));

    armIdx_ = setdiff(1:numel(qa), [ix_ iy_ iyaw_]);
    Q = qa;   % include start
    if ~isempty(armIdx_)
        qa_arm = qa(armIdx_); qb_arm = qb(armIdx_);
        dq_arm = qb_arm - qa_arm;
        maskCols = find(wrapMask);
        if ~isempty(maskCols)
            [~,loc] = intersect(armIdx_, maskCols);
            acols = armIdx_(loc);
            dq_arm(loc) = atan2(sin(qb(acols)-qa(acols)), cos(qb(acols)-qa(acols)));
        end
    end
    dYaw = atan2(sin(qb(iyaw_)-qa(iyaw_)), cos(qb(iyaw_)-qa(iyaw_))); % 0 if yaw is frozen

    q = qa;
    for k = 1:n
        s = k/n;
        q([ix_ iy_]) = pA + s*v;
        q(iyaw_)     = qa(iyaw_) + s*dYaw;
        if ~isempty(armIdx_), q(armIdx_) = qa_arm + s*dq_arm; end
        cols = find(wrapMask);
        if ~isempty(cols), q(cols) = atan2(sin(q(cols)), cos(q(cols))); end
        Q = [Q; q]; %#ok<AGROW>
    end

    q = qb; cols = find(wrapMask);
    if ~isempty(cols), q(cols) = atan2(sin(q(cols)), cos(q(cols))); end
    Q = [Q; q];
end

function traj = safeInterpolateWithCollisionCheck(robot, path, envCol, maxStep, validationStep)
% Safe interpolation with embedded collision checking
    if isempty(path) || size(path,1) <= 1
        traj = path;
        return;
    end
    
    % Resolve base indices for proper angle wrapping
    Sx = resolveBaseIndices(robot);
    ix = Sx.baseIdx(1); iy = Sx.baseIdx(2); iyaw = Sx.baseIdx(3);
    
    % Wrap mask: treat everything as angular EXCEPT base X,Y
    wrapMask = true(1, size(path,2));
    wrapMask([ix iy]) = false;
    
    traj = path(1,:);
    
    for i = 2:size(path,1)
        qa = traj(end,:);
        qb = path(i,:);
        
        % Calculate distance between configurations
        dq = qb - qa;
        % Apply angle wrapping to angular joints
        dq(wrapMask) = atan2(sin(dq(wrapMask)), cos(dq(wrapMask)));
        L = norm(dq);
        
        if L < 1e-6
            continue;
        end
        
        % Determine number of interpolation steps
        nSteps = max(2, ceil(L / maxStep));
        
        % Interpolate with collision checking
        for k = 1:nSteps
            s = k / nSteps;
            q_interp = qa + s * dq;
            % Apply angle wrapping
            q_interp(wrapMask) = atan2(sin(q_interp(wrapMask)), cos(q_interp(wrapMask)));
            
            % Check collision at this intermediate point
            coll = checkCollision(robot, q_interp, envCol, ...
                                 "Exhaustive", "on", ...
                                 "SkippedSelfCollisions", "parent");
            if any(coll, 'all')
                warning('Collision detected during interpolation at step %d-%d, fraction %.3f', ...
                        i-1, i, s);
                % Try to find a safe intermediate point using binary search
                q_safe = findSafeIntermediate(robot, qa, q_interp, envCol, wrapMask, validationStep);
                if ~isempty(q_safe)
                    traj = [traj; q_safe];
                else
                    error('Cannot find safe interpolation between steps %d and %d', i-1, i);
                end
                break;
            else
                if k == nSteps || ~isequal(round(q_interp, 4), round(traj(end,:), 4))
                    traj = [traj; q_interp];
                end
            end
        end
    end
    
    % Remove duplicates
    if size(traj,1) > 1
        dupMask = all(abs(diff(traj, 1, 1)) < 1e-8, 2);
        traj(dupMask, :) = [];
    end
end

function ok = assertPaperPathCollisionFree(robot, states, envCol, eeName, P)
%ASSERTPAPERPATHCOLLISIONFREE
%   Uses the same paper-style collision model & constraints as the planner:
%   - Base: 2D disc (radius = P.baseIncircle)
%   - Rebar: capsule along TCP Y-axis (P.rebarRadius, P.rebarLen)
%   - Ground-plane: rebar must stay above P.zFloor (if provided)
%   - Arm: key links as spheres (P.armKeyNames, P.armRadius)
%
%   If called with an output, returns:
%       ok = true  if all states are collision-free
%       ok = false if any collision / violation is found
%
%   If called with NO output, it throws an error on the first collision.

    ok = true;
    if isempty(states)
        return;
    end

    for k = 1:size(states, 1)
        q = states(k,:);
        % PASS P HERE vvv
        if paperCollisionSimple(q, robot, eeName, envCol, ...
                                P.baseIncircle, P.rebarRadius, P.rebarLen, P)
            if nargout == 0
                error('Paper collision model: collision in planned path at step %d (base/rebar).', k);
            else
                ok = false;
                return;
            end
        end

        % --- 2) Ground plane constraint (rebar must stay above zFloor) ---
        if isfield(P, 'zFloor')
            Ttcp = getTransform(robot, q, eeName);
            o    = Ttcp(1:3,4);
            Rtcp = Ttcp(1:3,1:3);

            % Rebar along TCP Y-axis (consistent with planner & paperCollisionSimple)
            dir  = Rtcp(:,2);
            dir  = dir / max(norm(dir), 1e-9);

            p0   = o - 0.5 * P.rebarLen * dir;
            p1   = o + 0.5 * P.rebarLen * dir;
            zMinBar = min(p0(3), p1(3)) - P.rebarRadius - rebarEnvelopeSag(P);

            % numeric tolerance (e.g. 1 cm)
            epsZ = 0.1;

            if zMinBar < (P.zFloor - epsZ)
                if nargout == 0
                    error('Paper model: rebar dips below zFloor at step %d (zMin=%.3f < %.3f).', ...
                          k, zMinBar,  (P.zFloor - epsZ));
                else
                    ok = false;
                    return;
                end
            end
        end


        % --- 3) Arm key-link spheres (cheap arm collision) --------------
        if isfield(P,'armRadius') && P.armRadius > 0 && ...
           isfield(P,'armKeyNames') && ~isempty(P.armKeyNames)
            if armKeyCollision(q, robot, envCol, P.armKeyNames, P.armRadius,0.003)
                if nargout == 0
                    error('Paper model: arm key-link collision at step %d.', k);
                else
                    ok = false;
                    return;
                end
            end
        end
    end
end


% -------------------------------------------------------------------------
function coll = paperCollisionSimple(q, rb, eeName, envCol, ...
                                     baseIncircle, rebarRadius, rebarLen, P)
% Added 'P' to input arguments to access base dimensions

    % --- 1) Base-disc check vs Environment (Standard) ---
    S = resolveBaseIndices_global(rb);
    baseXY = q(S.baseIdx(1:2));
    dBase  = minClearance2D(baseXY, envCol); 
    if dBase < baseIncircle
        coll = true; return;
    end

    % --- Calculate Rebar Geometry in World Frame ---
    Ttcp = getTransform(rb, q, eeName);
    o    = Ttcp(1:3,4);           
    Rtcp = Ttcp(1:3,1:3);
    
    dir  = Rtcp(:,2); % Rebar along TCP Y-axis
    dir  = dir / max(norm(dir), 1e-9);
    
    p0_world = o - 0.5*rebarLen*dir;
    p1_world = o + 0.5*rebarLen*dir;

    % --- 2) Rebar vs Environment (Standard) ---
    sagDepth = rebarEnvelopeSag(P);
    N = max(ceil(rebarLen / 0.05), 2);
    Nz = max(1, ceil(sagDepth / 0.05));
    for i = 0:N
        s = i / N;
        pCenter = p0_world + s*(p1_world - p0_world);
        for j = 0:Nz
            p = pCenter - [0; 0; sagDepth*(j/Nz)];
            if minClearance3D(p, envCol) < rebarRadius
                coll = true; return;
            end
        end
    end

    % =====================================================================
    % --- 3) Rebar-to-base self-collision check --------------------------
    % =====================================================================
    if nargin >= 8 && isfield(P, 'baseHeight')
        % Get Base Configuration
        bx = q(S.baseIdx(1));
        by = q(S.baseIdx(2));
        bz = q(S.baseIdx(3)); % Yaw

        % Create Transform from World to Base (Inverse of Base to World)
        % Rotation matrix for Base Yaw
        c = cos(bz); s = sin(bz);
        R_wb = [c -s 0; s c 0; 0 0 1];
        p_wb = [bx; by; 0];
        
        % Transform Rebar Points to Base Frame: P_local = R' * (P_world - P_base)
        p0_local = R_wb' * (p0_world - p_wb);
        p1_local = R_wb' * (p1_world - p_wb);

        % Define Base Box Limits (Local Frame centered at 0,0)
        % Assuming base is centered. Adjust offsets if chassis is offset.
        halfL = P.baseLength / 2;
        halfW = P.baseWidth / 2;
        H     = P.baseHeight;

        % Simple Segment vs AABB (Axis Aligned Bounding Box) Check
        % We check discrete points along the rebar for simplicity
        
        % Is any part of the rebar INSIDE the base box?
        % We check endpoints + midpoints
        pointsToCheck = [p0_local, p1_local, (p0_local+p1_local)/2];
        
        % Padding to prevent "skin" contact
        pad = rebarRadius + 0.02; 

        for k = 1:3
            pt = pointsToCheck(:,k);
            if (pt(3) < H + pad) && (pt(3) > -0.1) && ... % Z check
               (abs(pt(1)) < halfL + pad) && ...          % X check
               (abs(pt(2)) < halfW + pad)                 % Y check
               
               % COLLISION DETECTED: Rebar is hitting the mobile base
               coll = true; 
               return;
            end
        end
    end
    
    coll = false;
end

% -------------------------------------------------------------------------
function S = resolveBaseIndices_global(rb)
%RESOLVEBASEINDICES_GLOBAL  Find indices of base_x, base_y, chassis joints.
%   Separate from any nested version inside hsBiRRT_paper_v2, so there is
%   no name conflict.

    bxBody = getBody(rb, "base_x");   jx = string(bxBody.Joint.Name);
    byBody = getBody(rb, "base_y");   jy = string(byBody.Joint.Name);
    chBody = getBody(rb, "chassis");  jz = string(chBody.Joint.Name);

    fmt0 = rb.DataFormat;
    rb.DataFormat = 'struct';
    cfg = homeConfiguration(rb);
    rb.DataFormat = fmt0;

    jnames = string({cfg.JointName});
    ix = find(jnames == jx, 1);
    iy = find(jnames == jy, 1);
    iz = find(jnames == jz, 1);

    assert(~isempty(ix) && ~isempty(iy) && ~isempty(iz), ...
           'resolveBaseIndices_global: failed to find base joints.');

    S.baseIdx = [ix iy iz];
end


function traj = hs_interpPath(path, maxStep)
% Joint-space interpolation.
    if isempty(path), traj = []; return; end; if size(path,1)<=1, traj=path; return; end
    traj = path(1,:); 
    % --- Must define S_interp or find robot in base workspace ---
    try
        % Cerca il robot COMPLETO (mobileUR20) per la maschera angolare
        S_interp = resolveBaseIndices(evalin('base', 'mobileUR20')); 
        % Trova quali giunti del robot completo sono nel percorso 'path'
        
        % TENTATIVO 1: Il percorso è già 9-DoF?
        if size(path,2) == numel(S_interp.jointNames)
            isAngle = true(1, size(path,2)); 
            isAngle(S_interp.baseIdx(1:2)) = false; % Base X, Y are not angles
        else
            % TENTATIVO 2: Il percorso è 5-DoF? Trova i giunti del robot 5dof
            robot_5dof = evalin('base', 'robot5dof');
            S_interp_5dof = resolveBaseIndices(robot_5dof);
            if size(path,2) == numel(S_interp_5dof.jointNames)
                 isAngle = true(1, size(path,2));
                 isAngle(S_interp_5dof.baseIdx(1:2)) = false;
            else
                error('hs_interpPath: Mismatch DoF. Path has %d, Full has %d, 5dof has %d', ...
                    size(path,2), numel(S_interp.jointNames), numel(S_interp_5dof.jointNames));
            end
        end
        
    catch ME
        warning('hs_interpPath: could not find robot in base. Assuming default base indices 1,2,3. Error: %s', ME.message);
        S_interp.baseIdx = [1 2 3]; % Fallback
        isAngle = true(1, size(path,2)); 
        isAngle(S_interp.baseIdx(1:2)) = false; % Base X, Y are not angles
    end
    % ---------------------------------------------------------------
    for i = 2:size(path,1)
        qa = traj(end,:); qb = path(i,:); 
        dq = qb - qa; 
        dq(isAngle) = atan2(sin(dq(isAngle)), cos(dq(isAngle))); 
        L = norm(dq);
        if L < 1e-6, continue; end
        n = max(2, ceil(L / maxStep));
        for k = 1:n
            s = k/n; 
            q = qa + s*dq; 
            q(isAngle) = atan2(sin(q(isAngle)), cos(q(isAngle))); 
            traj = [traj; q]; %#ok<AGROW>
        end 
    end
end
function okAll = assertPathCollisionFreeAdaptive(robot, env, Q, coarseStep, fineStep, doStrict)
% Fast: sweep each segment with coarse step and "Exhaustive","off".
% Only if something looks suspicious, recheck that segment with fine step
% and "Exhaustive","on". Optional final strict mode when doStrict=true.

if nargin<6, doStrict=false; end
Sx = resolveBaseIndices(robot); ix=Sx.baseIdx(1); iy=Sx.baseIdx(2);
wrap = true(1, size(Q,2)); wrap([ix iy]) = false;

N = size(Q,1);
okAll = true;
lastPct = -1;

for i = 1:N-1
    qa = Q(i,:); qb = Q(i+1,:);

    % --- coarse pass (fast broad-phase)
    if ~segmentFreeWrapEx(robot, env, qa, qb, coarseStep, wrap, "off")
        % --- fine pass (strict)
        if ~segmentFreeWrapEx(robot, env, qa, qb, fineStep, wrap, "on")
            okAll = false;
            if ~doStrict
                % Early exit during precheck
                return;
            else
                error('Collision detected between segment %d-%d (fine pass).', i, i+1);
            end
        end
    end

    % Lightweight progress to avoid "hang" feeling
    pct = floor(100*i/(N-1));
    if pct >= lastPct + 10
        fprintf('  [Check] %3d%% done (%d/%d)\n', pct, i, N-1);
        lastPct = pct;
    end
end
end

function okSeg = segmentFreeWrapEx(rb, env, qa, qb, step, wrap, exhaustiveFlag)
% Linear joint-space sweep with angle wrapping; choose exhaustive on/off.
dq = qb - qa;
for c = find(wrap), dq(c) = atan2(sin(dq(c)), cos(dq(c))); end
L = norm(dq);
n = max(2, ceil(L / max(step,1e-6)));
okSeg = true;
for k = 1:n
    qk = qa + (k/n)*dq;
    for c = find(wrap), qk(c) = atan2(sin(qk(c)), cos(qk(c))); end
    coll = checkCollision(rb, qk, env, "Exhaustive", exhaustiveFlag, "SkippedSelfCollisions","parent");
    if any(coll,'all'), okSeg = false; return; end
end
end

function qm = midWrap(qa, qb, wrap)
dq = qb - qa;
for c = find(wrap), dq(c) = atan2(sin(dq(c)), cos(dq(c))); end
qm = qa + 0.5*dq;
for c = find(wrap), qm(c) = atan2(sin(qm(c)), cos(qm(c))); end
end

function Qrepaired = repairCollidingSegments(robot, env, Q, fineStep, maxDepth)
% Repairs only failing segments via bounded bisection with wrap-aware mids.
Sx = resolveBaseIndices(robot); ix=Sx.baseIdx(1); iy=Sx.baseIdx(2);
wrap = true(1, size(Q,2)); wrap([ix iy]) = false;

i = 1; Qrepaired = Q;
while i < size(Qrepaired,1)
    qa = Qrepaired(i,:); qb = Qrepaired(i+1,:);
    if ~segmentFreeWrapEx(robot, env, qa, qb, fineStep, wrap, "on")
        % Bounded recursive bisect
        stack = struct('a',qa,'b',qb,'depth',0);
        newSeg = [];
        okSeg  = true;
        S = stack; clear stack;
        while ~isempty(S)
            cur = S(end); S(end) = [];
            a=cur.a; b=cur.b; d=cur.depth;
            if segmentFreeWrapEx(robot, env, a, b, fineStep, wrap, "on")
                % keep endpoint b
                newSeg = [newSeg; b]; %#ok<AGROW>
            else
                if d >= maxDepth
                    okSeg = false; break; % give up; let strict assert handle it
                end
                m = midWrap(a, b, wrap);
                S(end+1) = struct('a',m,'b',b,'depth',d+1); %#ok<AGROW>
                S(end+1) = struct('a',a,'b',m,'depth',d+1); %#ok<AGROW>
            end
        end
        if okSeg
            % Replace (i,i+1) with subdivided chain [i, newSeg...]
            Qrepaired = [Qrepaired(1:i,:); newSeg; Qrepaired(i+2:end,:)];
        else
            % Couldn't fix within depth; just move on (final strict assert will stop if needed)
            i = i + 1;
        end
    else
        i = i + 1;
    end

    if mod(i, max(10,round(0.05*size(Qrepaired,1))))==0
        fprintf('  [Repair] processed %d/%d waypoints\n', i, size(Qrepaired,1));
    end
end
end

function traj = hs_interpPath_segmentFreeStyle(robot, path, maxStep)
%HS_INTERPPATH_SEGMENTFREESTYLE
%  Densify a path using the same "style" as the non-holonomic segment
%  extension:
%    - Interpolate base X,Y linearly
%    - Interpolate base yaw with angle wrapping
%    - Interpolate arm joints linearly with angle wrapping
%
%  No collision checks are done here – this purely reconstructs a smooth
%  trajectory between planner nodes, consistent with segmentFree's logic.

    % Trivial cases
    if isempty(path)
        traj = [];
        return;
    end
    if size(path,1) <= 1
        traj = path;
        return;
    end

    % --- Resolve base joint indices (X, Y, yaw) ---
    S   = resolveBaseIndices(robot);
    ix_   = S.baseIdx(1);
    iy_   = S.baseIdx(2);
    iyaw_ = S.baseIdx(3);

    % Everything except base X,Y is treated as angular
    angMask        = true(1, size(path,2));
    angMask([ix_ iy_]) = false;
    angCols        = find(angMask);

    % Arm joints = all joints except base X,Y,yaw
    armIdx_ = setdiff(1:size(path,2), [ix_ iy_ iyaw_]);

    % Start trajectory at the first node
    traj = path(1,:);

    % ---------------------------------------------------------------------
    % Interpolate segment by segment
    % ---------------------------------------------------------------------
    for i = 2:size(path,1)
        % Start from the last emitted config (qa) and move to qb
        qa = traj(end,:);  % important: start from last point in traj
        qb = path(i,:);

        % Base XY interpolation
        pA = qa([ix_ iy_]);
        pB = qb([ix_ iy_]);
        v  = pB - pA;
        L  = norm(v);
        n  = max(2, ceil(L / maxStep));   % at least 2 samples per segment

        % Arm interpolation (with angle wrapping)
        qa_arm = []; qb_arm = []; dq_arm = [];
        if ~isempty(armIdx_)
            qa_arm = qa(armIdx_);
            qb_arm = qb(armIdx_);
            dq_arm = qb_arm - qa_arm;

            if ~isempty(angCols)
                % Find angular joints that belong to the arm subset
                [~,loc] = intersect(armIdx_, angCols);
                aCols = armIdx_(loc);
                % Wrap angular differences properly
                dq_arm(loc) = atan2( ...
                    sin(qb(aCols) - qa(aCols)), ...
                    cos(qb(aCols) - qa(aCols)));
            end
        end

        % Base yaw interpolation (with angle wrapping)
        dYaw = atan2( ...
            sin(qb(iyaw_) - qa(iyaw_)), ...
            cos(qb(iyaw_) - qa(iyaw_)));  % =0 if yaw is frozen

        q = qa;
        for k = 1:n
            s = k / n;

            % Base XY
            q([ix_ iy_]) = pA + s * v;

            % Base yaw
            q(iyaw_) = qa(iyaw_) + s * dYaw;

            % Arm joints
            if ~isempty(armIdx_)
                q(armIdx_) = qa_arm + s * dq_arm;
            end

            % Wrap all angular joints to [-pi, pi]
            if ~isempty(angCols)
                q(angCols) = atan2(sin(q(angCols)), cos(q(angCols)));
            end

            traj = [traj; q]; %#ok<AGROW>
        end

        % Append the exact end configuration qb (matches the planner node)
        q = qb;
        if ~isempty(angCols)
            q(angCols) = atan2(sin(q(angCols)), cos(q(angCols)));
        end
        traj = [traj; q]; %#ok<AGROW>
    end

    % Remove exact duplicates caused by segment joins
    if size(traj,1) > 1
        dup = all(abs(diff(traj,1,1)) < 1e-12, 2);
        traj(dup,:) = [];
    end
end

function [trajFixed, ok] = repairCollidingSegments_localRRT(robot, envCol, traj, maxStep, maxDepth, eeName, P)
% Added eeName and P to inputs to enforce constraints during repair
    trajFixed = traj;
    ok = true;
    i = 1;
    while i < size(trajFixed,1)
        qA = trajFixed(i,:);
        qB = trajFixed(i+1,:);
        
        % Check collision using BOTH standard check AND Paper/P constraints
        if segmentCollides(robot, envCol, qA, qB, maxStep, eeName, P)
            
            % Try local RRT first
            [subtraj, okLocal] = localJointSpaceRRT(robot, envCol, qA, qB, maxStep, eeName, P);
            
            if ~okLocal
                fprintf('    [Repair Failed] Segment %d-%d could not be repaired.\n', i, i+1);
                ok = false;
                return;   % this edge cannot be locally fixed
            end
            
            % Replace [qA; qB] by [qA; subtraj(2:end-1,:); qB]
            trajFixed = [trajFixed(1:i,:); ...
                         subtraj(2:end-1,:); ...
                         trajFixed(i+1:end,:)];
            % Do not increment i, re-check the new segments
        else
            i = i + 1;
        end
    end
end



function coll = segmentCollides(robot, envCol, qA, qB, maxStep, eeName, P)
% Checks Standard Collision + Paper Constraints (zFloor, rebar capsule)
    coll = false;
    if isempty(qA) || isempty(qB), return; end

    dq = qA - qB; % Direction doesn't matter for norm
    % Note: A rigorous implementation would wrap angles here, skipping for brevity 
    % as this is a local check usually on small segments.
    L  = norm(dq);
    
    % Determine samples
    if L < 1e-9
        nSamples = 1;
    else
        nSamples = max(2, ceil(L / max(maxStep,1e-6)));
    end

    for k = 0:nSamples
        s = k / nSamples;
        q = qA + s * (qB - qA);
        
        % 1. Standard Robot Collision
        C = checkCollision(robot, q, envCol, "Exhaustive","off","SkippedSelfCollisions","parent");
        if any(C,'all')
            coll = true; return;
        end

        % 2. Paper Model Constraints (if P provided)
        if nargin >= 7 && ~isempty(P)
            % PASS P HERE vvv
            if paperCollisionSimple(q, robot, eeName, envCol, ...
                                    P.baseIncircle, P.rebarRadius, P.rebarLen, P)
                 coll = true; return;
            end

            % Check the rebar floor-clearance constraint.
            if isfield(P, 'zFloor')
                Ttcp = getTransform(robot, q, eeName);
                o    = Ttcp(1:3,4);
                Rtcp = Ttcp(1:3,1:3);
                dir  = Rtcp(:,2); dir = dir / max(norm(dir), 1e-9); % Rebar along Y
                p0   = o - 0.5 * P.rebarLen * dir;
                p1   = o + 0.5 * P.rebarLen * dir;
                zMinBar = min(p0(3), p1(3)) - P.rebarRadius - rebarEnvelopeSag(P);
                
                if zMinBar < (P.zFloor - 0.005) % Small tolerance
                    coll = true; return;
                end
            end
        end
    end
end

function [subtraj, okLocal] = localJointSpaceRRT(robot, envCol, qStart, qGoal, maxStep, eeName, P)
    okLocal = false;
    subtraj = [];
    
    % Try simple straight-line repair first (cheapest)
    [subtraj, okLocal] = localStraightLineRepair(robot, envCol, qStart, qGoal, maxStep, eeName, P);
    if okLocal, return; end

    % If straight line fails, try RRT
    try
        planner = manipulatorRRT(robot, envCol);
        planner.ValidationDistance = maxStep;
        planner.MaxConnectionDistance = 2*maxStep;
        planner.MaxIterations = 1000; % Reduced for speed
        planner.SkippedSelfCollisions = "parent";
        
        % Plan
        pathLocal = plan(planner, qStart, qGoal);
        
        if ~isempty(pathLocal)
            % Densify
            subtraj = hs_interpPath_segmentFreeStyle(robot, pathLocal, maxStep);
            
            % CRITICAL: Validate the RRT result against P constraints
            % (manipulatorRRT doesn't know about zFloor)
            for k=1:size(subtraj,1)-1
                if segmentCollides(robot, envCol, subtraj(k,:), subtraj(k+1,:), maxStep, eeName, P)
                    okLocal = false; subtraj = []; return;
                end
            end
            okLocal = true;
        end
    catch
        okLocal = false;
    end
end

function [trajOut, ok] = localStraightLineRepair(robot, envCol, qStart, qGoal, maxStep, eeName, P)
    % Same as before but passes P to the collision check loop
    qStart = double(qStart(:)).';
    qGoal  = double(qGoal(:)).';
    ok = false; trajOut = [];

    % Check endpoints using strict constraints
    if segmentCollides(robot, envCol, qStart, qStart, maxStep, eeName, P) || ...
       segmentCollides(robot, envCol, qGoal, qGoal, maxStep, eeName, P)
       return;
    end

    dq = qGoal - qStart; 
    % (Add angle wrapping logic here if desired, simplified for brevity)
    L = norm(dq);
    nSteps = max(2, ceil(L / maxStep));
    
    traj = qStart;
    for k = 1:nSteps
        s = k / nSteps;
        q = qStart + s*dq;
        % (Add angle wrapping re-wrap here)
        
        % Use the updated segmentCollides which checks P.zFloor
        if segmentCollides(robot, envCol, q, q, maxStep, eeName, P)
            ok = false; return;
        end
        traj = [traj; q]; %#ok<AGROW>
    end
    trajOut = traj;
    ok = true;
end

function value = chooseValue(condition, trueValue, falseValue)
    if condition, value = trueValue; else, value = falseValue; end
end

function label = onOff(value)
    if value, label = 'ON'; else, label = 'OFF'; end
end

function sagDepth = rebarEnvelopeSag(P)
    sagDepth = 0;
    if isfield(P,'useRebarSagEnvelope') && P.useRebarSagEnvelope && ...
            isfield(P,'rebarEnvelopeHeight')
        sagDepth = max(0, P.rebarEnvelopeHeight);
    end
end

function h = drawRebarSagEnvelope(robot, q, eeName, P, ax)
% Always draw the configured envelope; useRebarSagEnvelope affects checks only.
    h = gobjects(0);
    if isempty(ax) || ~isvalid(ax), return; end
    L = P.rebarEnvelopeLength;
    W = max(2*P.rebarRadius, P.rebarEnvelopeWidth);
    H = max(P.rebarEnvelopeHeight, 0.02);
    Ttcp = getTransform(robot, q, eeName);
    centerTop = Ttcp(1:3,4);
    xAxis = Ttcp(1:3,2); xAxis = xAxis/max(norm(xAxis),1e-9);
    zWorld = [0;0;1];
    yAxis = cross(zWorld,xAxis);
    if norm(yAxis)<1e-9, yAxis=cross([1;0;0],xAxis); end
    yAxis = yAxis/max(norm(yAxis),1e-9);
    center = centerTop - 0.5*H*zWorld;
    signs = [-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1; ...
             -1 -1 1;1 -1 1;1 1 1;-1 1 1];
    V = zeros(8,3);
    for i=1:8
        V(i,:)=(center + signs(i,1)*0.5*L*xAxis + ...
            signs(i,2)*0.5*W*yAxis + signs(i,3)*0.5*H*zWorld).';
    end
    F=[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
    h=patch('Parent',ax,'Vertices',V,'Faces',F, ...
        'FaceColor',[1 .45 .05],'FaceAlpha',.18, ...
        'EdgeColor',[.85 .25 0],'EdgeAlpha',.65,'LineWidth',1, ...
        'DisplayName','Sag-aware rebar envelope');
end

function coll = armKeyCollision(q, rb, env, keyNames, rArm, tol)
%ARMKEYCOLLISION  Cheap approximate collision for the arm.
%   Treat each key link as a sphere of radius rArm and use minClearance3D.
%   tol: small numerical tolerance (meters), default ~2mm.

    if nargin < 6
        tol = 0.002;   % 2 mm tolerance
    end

    coll = false;
    if isempty(keyNames) || rArm <= 0
        return;
    end

    for i = 1:numel(keyNames)
        try
            T = getTransform(rb, q, keyNames(i));
        catch
            continue;
        end
        p = T(1:3,4).';
        d = minClearance3D(p, env);

        % allow small "numerical" violations
        if d < (rArm - tol)
            % fprintf('[ArmKeyCollision] %s: d=%.3f < rArm-tol=%.3f\n', ...
            %         keyNames(i), d, rArm - tol);
            coll = true;
            return;
        end
    end
end
