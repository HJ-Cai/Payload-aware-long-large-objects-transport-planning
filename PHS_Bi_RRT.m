%% UR20 Mobile Pick & Place with PHS-Bi-RRT
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
useRebarEnvelope = true;
% =========================================================================
%% ========================================================================
%  1. INITIALIZATION AND SETUP
%  ========================================================================
% --- Load robot and environment ---
[mobileUR20, startConfig, envS, pickPose, placePose, placeRebarCenterPosition, rebarMeta] = ...
    loadRebarTransportScenario;
fprintf('Using default start configuration from the helper function.\n');
fprintf('\n==========================================\n');
fprintf('   PHS-Bi-RRT: single simulation run');
fprintf('\n==========================================\n');
fprintf('Using default start configuration from the helper function.\n');
floorObj = collisionBox(20, 20, 0.1); 
% Position it so the top surface is slightly BELOW Z=0 (e.g., at -0.02m)
% Center Z = Top - HalfHeight = -0.02 - 0.05 = -0.07
floorObj.Pose = trvec2tform([0, 0, -0.07]); 
% Add to envS (for visualization) and envCol (for collision checking)
envS = [envS, {struct('Name','Floor','CollisionObj',floorObj)}];
fprintf('[Setup] Added physical floor to envCol. Top surface at Z=-0.02m.\n');
% =========================================================================
startConfig = double(startConfig); % Ensure double precision from the start
% Keep both formats: structs (envS) and plain collision objects (envCol) 
envCol = cellfun(@(s) s.CollisionObj, envS, 'uni', false);
% --- Configure Mobile Base ---
bx = getBody(mobileUR20, "base_x");  bx.Joint.PositionLimits = [-6 6];
by = getBody(mobileUR20, "base_y");  by.Joint.PositionLimits = [-6 6];
bz = getBody(mobileUR20, "chassis"); bz.Joint.PositionLimits = [-pi  pi];
% --- Setup Visualization ---
figure("Name","Mobile UR20 Rebar Pick and Place Using RRT", ...
       "Units","normalized","OuterPosition",[0,0,1,1]);
ax3D = gca; % Store handle for 3D axes
show(mobileUR20, startConfig, "Visuals","off","Collisions","on");
title("Initial Mobile UR20 and Environment Setup");
hold on; grid on;
axis([-5 5 -5 5 -0.1 2.5]); view(120, 25); % Set initial view
%% Show environment collision objects
for i = 1:numel(envS), show(envS{i}.CollisionObj); end
% --- Initialize Solvers and Planners ---
ik = inverseKinematics('RigidBodyTree', mobileUR20);
weights = [0.1 0.1 0.1 1 1 1]; % prioritize orientation
endEffector = 'tcp';
makePlanner = @(robot, env) configurePlanner(manipulatorRRT(robot, env));
allStates = [];
%% ========================================================================
%  2. DEFINE PICK POSES (use helper-provided poses)
%  ========================================================================
% Find rebar object
rebarIdx = find(cellfun(@(s) isa(s.CollisionObj,'collisionCylinder') && strcmpi(s.Name,'rebar'), envS), 1);
if isempty(rebarIdx), rebarIdx = find(cellfun(@(s) isa(s.CollisionObj,'collisionCylinder'), envS), 1); end
assert(~isempty(rebarIdx), 'Could not find a cylinder "rebar" in envS.');
rebarObj = envS{rebarIdx}.CollisionObj;
UR20Meta = evalin('base','UR20Meta');  % helper assigned in base
approachPose = pickPose;
graspPose    = UR20Meta.T_tcp_touch_world;
retreatPose  = approachPose;
% --- Solve IK for Key Pick Configurations ---
fprintf('Calculating IK for pick poses...\n');
approachConfig = double(ik(endEffector, approachPose, weights, startConfig));
graspConfig    = double(ik(endEffector, graspPose,    weights, approachConfig));
% retreatConfig  = double(ik(endEffector, retreatPose,  weights, graspConfig)); % Note: This is pre-retreat
assert(all(isfinite(approachConfig)), 'Approach IK failed');
assert(all(isfinite(graspConfig)), 'Grasp IK failed');
% --- Visualize Pick Poses for Verification ---
drawFrame(approachPose, 'approach', 0.12);
drawFrame(graspPose, 'pick', 0.12);
% --- Pre-flight Collision Checks (use envCol) ---
mustBeFree(mobileUR20, startConfig,    envCol, 'Start configuration is in collision.');
mustBeFree(mobileUR20, approachConfig, envCol, 'Approach pose results in a collision.');
%% ========================================================================
%  3. PLAN AND EXECUTE PICK SEQUENCE
%  ========================================================================
% --- Plan 1: Home -> Approach (RRT) ---
fprintf('Plan 1: Planning Home -> Approach...\n');
planner = makePlanner(mobileUR20, envCol);
path1 = planOrRetry(planner, startConfig, approachConfig, 5, runID);
% Use a dense, collision-checked interpolation.
traj1 = interpolate(planner, path1, 10);
assertPathCollisionFree(mobileUR20, traj1, envCol);
% for i = 1:size(traj1, 1)
%     show(mobileUR20, traj1(i,:), "PreservePlot", false, "Visuals","off","Collisions","on");
%     title("Plan 1: Moving to Approach Position"); drawnow;
% end
allStates = [allStates; traj1];
% --- Plan 2: Approach -> Grasp (Linear Interpolation) ---
fprintf('Plan 2: Moving straight down to grasp...\n');
traj2 = lerpJoints(approachConfig, graspConfig, 20); % Use lerpJoints
allStates = [allStates; traj2];
% --- Action: Close Gripper and Attach Rebar ---
fprintf('Closing gripper & attaching rebar...\n');
grippedConfig = graspConfig;
grippedConfig(end-1:end) = [0 0]; % close fingers (adjust if your gripper differs)
% show(mobileUR20, grippedConfig, "PreservePlot", false, "Visuals","off","Collisions","on");
% title("Gripping the Rebar"); drawnow;
allStates = [allStates; grippedConfig];
% Attach rebar to robot; remove from env
rebarBody = rigidBody("rebar");
eeT = getTransform(mobileUR20, grippedConfig, endEffector);
setFixedTransform(rebarBody.Joint, eeT \ rebarObj.Pose);
rbCopy = rebarObj.copy; rbCopy.Pose = eye(4);
addCollision(rebarBody, rbCopy);
addBody(mobileUR20, rebarBody, endEffector);
%% Remove the attached rebar from the static environment
envS(rebarIdx) = [];
envCol = cellfun(@(s) s.CollisionObj, envS, 'uni', false);
% --- Plan 3: Retreat from Pick + Best Height Search ---
% ---- Deflection prediction ----
diameter_mm = rebarMeta.DiameterM * 1000;
length_m    = rebarMeta.LengthM;
gripPoint   = rebarMeta.GripPoint;
deflection  = computeRebarDeflection(diameter_mm, length_m, gripPoint);
fprintf('--- Predicted deflection (NN) ---\n');
fprintf('Rebar Ø = %.1f mm | Length = %.2f m | Grip = %s\n', diameter_mm, length_m, gripPoint);
fprintf('Deflection = %.3f m (%.1f cm)\n\n', deflection, deflection*100);

% Rebar swept envelope for PHS planning.
% Length uses a 10% safety margin; height follows the estimated sag.
rebarEnvelope = struct( ...
    'displayAlways', true, ...
    'consideredInPlanning', useRebarEnvelope, ...
    'lengthFactor', 1.10, ...
    'lengthM',      1.10 * length_m, ...
    'heightM',      max(deflection, 0.0), ...
    'widthM',       rebarMeta.DiameterM, ...
    'sagM',         max(deflection, 0.0));
fprintf('Rebar envelope checking: %s\n', onOff(useRebarEnvelope));
if useRebarEnvelope
    fprintf('Envelope dimensions: length = %.3f m (1.10 x %.3f), height/sag = %.3f m, width = %.3f m\n\n', ...
        rebarEnvelope.lengthM, length_m, rebarEnvelope.heightM, rebarEnvelope.widthM);
end
% ---- Compute best transport height (circle-chain) ----
t_bestZ_tic = tic;
% Goal XY for the height planner (already as before)
goalXY_for_height = placeRebarCenterPosition(1:2).';
[bestZ, bestOut] = findTransportHeightCircleChain( ...
    envS, rebarMeta, ...
    'computeOnly', true, ...
    'bounds', [-4.5 4.5; -4.5 4.5], ...
    'res', 0.075, ...
    'goalXY', goalXY_for_height);
t_bestZ_s = toc(t_bestZ_tic);
if isnan(bestZ)
    warning('FindOptimalTransportHeight failed. Using fallback Z=1.5m');
    bestZ = 1.5;
end
% Height-invariant transport rule.
marginZ   = 0.10;                               % vertical safety margin
pickZ     = pickPose(3,4);
targetZ   = rebarMeta.placeCenter_W(3);         % from helper metadata
z_pickDM  = pickZ + deflection + marginZ;
useFlatRule = isfield(bestOut,'flags') && isfield(bestOut.flags,'heightInvariant') ...
              && logical(bestOut.flags.heightInvariant);
if useFlatRule
    if targetZ <= z_pickDM + 1e-6
        transportZ = z_pickDM;
        fprintf(['[Height-invariant] Using transportZ = pickZ + deflection + margin = %.3f ', ...
                 '(pickZ=%.3f, defl=%.3f, margin=%.3f)\n'], transportZ, pickZ, deflection, marginZ);
    else
        transportZ = targetZ;   % no deflection/margin added
        fprintf(['[Height-invariant] Target above pick+defl+margin. ', ...
                 'Using transportZ = targetZ = %.3f (no extra margin)\n'], transportZ);
    end
else
    % normal behavior: use bestZ from circle-chain + deflection/margin
    transportZ = bestZ + deflection + marginZ;
    fprintf('Best transport height from circle-chain = %.3f (defl %.3f + margin %.3f) -> transportZ=%.3f (%.3f s)\n', ...
        bestZ, deflection, marginZ, transportZ, t_bestZ_s);
end
% Keep this so other code can still draw the 2D reference if available
bestHeightXY = [];
if isfield(bestOut,'best') && isfield(bestOut.best,'chainXY') && ~isempty(bestOut.best.chainXY)
    bestHeightXY = bestOut.best.chainXY(:,1:2);
else
    warning('No circle-chain produced; reference will fallback later.');
end
% ---- Perform Straight-up retreat to transportZ (unchanged) ----
fprintf('Plan 3: Straight-up retreat to transport height Z=%.3f...\n', transportZ);
T_rebar_pick = getTransform(mobileUR20, grippedConfig, 'rebar');
currentZ = T_rebar_pick(3,4);
lift = max(0, transportZ - currentZ);
traj3 = cartesianLift(mobileUR20, grippedConfig, endEffector, ik, weights, lift, 0.05);
retreatConfig = double(traj3(end,:));
T_retreat = getTransform(mobileUR20, retreatConfig, endEffector);
drawFrame(T_retreat, 'retreat', 0.12);
% for i = 1:size(traj3, 1)
%     show(mobileUR20, traj3(i,:), "PreservePlot", false, "Visuals","off","Collisions","on");
%     title("Plan 3: Straight-up retreat"); drawnow;
% end
allStates = [allStates; traj3];
%% ========================================================================
%  4. DEFINE PLACE POSES (use helper-provided placePose)
%  ========================================================================
R_place = rebarMeta.R_place;              % 3x3 from helper (matches old convention now)
p_target = rebarMeta.placeCenter_W(:);      % 3x1 from helper
targetPose = [R_place, p_target; 0 0 0 1];

% Offset 2: translate BACK along tool Z by +offsetClearance
offsetClearance = 0.2;
offsetPose = targetPose;
Ry90 = axang2tform([0 1 0 pi/2]);
offsetPose = targetPose * Ry90;
offsetPose(1:3,4) = targetPose(1:3,4) - offsetClearance * targetPose(1:3,3);


% --- Offset 1 definition ---
offsetClearance1 = 0.3;
        % +90 deg about TCP-Y
offset1Pose = targetPose * Ry90;
offset1Pose(1:3,4) = targetPose(1:3,4) - (offsetClearance + offsetClearance1)*targetPose(1:3,3);
% ** CRITICAL: Update Offset 1 Z to match the new retreat height **
offset1Pose(3,4) = T_retreat(3,4);        % keep the Z from the retreat
% drawFrame(offset1Pose, 'offset 1', 0.10);
% drawFrame(offsetPose, 'offset 2', 0.12 );
drawFrame(targetPose, 'target',   0.12 );
%% ========================================================================
%  5. PLAN AND EXECUTE PLACE SEQUENCE (PLAN 4 — 9-DoF, NO ARM FROZEN)
%  ========================================================================
fprintf('Plan 4: Planning Retreat -> Offset with HS-Bi-RRT (9-DoF, RP lock)...\n');
t_plan4p = tic;  % pure planning time (planner loop)
t_plan4t = tic;  % total plan 4 block time (setup + plan + densify + checks)

% --- Plan 4-Setup: Generate Focus Regions (Base Disks + EE Pucks) ---
fprintf('[HS-Bi-RRT] Setup: Generating Focus Regions...\n');

% 2D env (drop 'floor')
env2D = envS; names = cellfun(@(s) s.Name, env2D, 'uni', false);
floorIdx = find(strcmpi(names,'floor')); if ~isempty(floorIdx), env2D(floorIdx) = []; end

% Corridor / grid params
baseIncircle = 0.35;    % base radius used for clearance
rMin         = 0.15;    % minimum accepted base-disk radius
margin       = 0.05;    % extra safety margin
gridRes      = 0.075;   % grid resolution (m)
tubeRadius   = 2.0;     % bias width around best-height polyline
penaltyW     = 6.0;     % distance-from-polyline weight
smoothStep   = 0.10;    % resample spacing for the final guide
maxDisks     = 120;     % cap on number of disks

% Start and goal XY coordinates; Z is constrained later.
Sx = resolveBaseIndices(mobileUR20);
ix = Sx.baseIdx(1); iy = Sx.baseIdx(2); iyaw = Sx.baseIdx(3);
start_xy = retreatConfig([ix iy]).';
goal_xy_desired = offset1Pose(1:2,4).';

% Fallback if bestHeightXY is empty
if ~exist('bestHeightXY','var') || isempty(bestHeightXY)
    % Ensure both are row vectors of length 2
    start_xy_row = retreatConfig([ix iy]);               % should be 1x2
    goal_xy_row  = offset1Pose(1:2,4).';                 % should be 1x2
    
    % Validate dimensions
    assert(isvector(start_xy_row) && numel(start_xy_row) == 2, ...
        'start_xy must be a 2-element vector.');
    assert(isvector(goal_xy_row) && numel(goal_xy_row) == 2, ...
        'goal_xy_desired must be a 2-element vector.');
    
    start_xy_row = start_xy_row(:)';  % force 1x2
    goal_xy_row  = goal_xy_row(:)';   % force 1x2
    
    bestHeightXY = [start_xy_row; goal_xy_row];  % 2x2 matrix: [start; goal]
end

% Occupancy + weighted A*  (biased to stay near bestHeightXY)
rSafe = baseIncircle + rMin + margin;
[xlimW, ylimW] = envBoundsXY(env2D, start_xy, goal_xy_desired, 0.6 + rSafe + tubeRadius);
O = buildOccGridFromEnvS(env2D, xlimW, ylimW, gridRes, rSafe);
D = polylineDistField(bestHeightXY, O.xv, O.yv);                  % [ny x nx]
band    = tubeRadius;
distNor = min(D / max(band,1e-6), 1.0);
Cw      = 1 + penaltyW*(distNor.^2);
Cw(O.grid) = inf;
[sIJ, gIJ] = world2grid(start_xy.', goal_xy_desired.', O);
sIJ = ensureFreeIJ(sIJ, O); gIJ = ensureFreeIJ(gIJ, O);
goal_xy = grid2world_scalar(gIJ, O);  % snapped goal on free cell

[pathIJ, okA] = aStar8_weighted(O, Cw, sIJ, gIJ);
assert(okA, 'Corridor-biased A*: no path. Increase tubeRadius or reduce penaltyW.');
pathXY  = grid2world(pathIJ, O);
pathXY  = shortcutSmoothing(pathXY, O);
guideXY = resamplePolylineByStep(pathXY, smoothStep);

% --- Base disks along the corridor ---
fprintf('[HS-Bi-RRT] Generating maximal Base Disks...\n');
t_bd_tic = tic;
bd = generateMaximalBaseDisksAlongPath(start_xy.', goal_xy, env2D, guideXY, ...
                                       baseIncircle, rMin, margin, maxDisks);
t_baseDisk_s = toc(t_bd_tic);
fprintf('[HS-Bi-RRT] Generated %d Base Disks in %.3f s.\n', size(bd, 1), t_baseDisk_s);

% --- EE pucks (2D circles at fixed Z = transportZ) ---
fprintf('[HS-Bi-RRT] Generating EE pucks (2D circles at Z=%.3f)...\n', transportZ);
t_ee_tic = tic;
T_start_ee = getTransform(mobileUR20, retreatConfig, endEffector);
pEE0 = T_start_ee(1:3,4).';  pEE0(3) = transportZ;
pEEg = offset1Pose(1:3,4).';  pEEg(3) = transportZ;

[eeCircXY, eeCircR, eeZ] = generateMaximalEECircles_Puck( ...
    pEE0, pEEg, env2D, guideXY, transportZ, ...
    'eeIncircle', 0.2, ...   % <-- NEW
    'margin', 0.03, ...          % visual/safety gap like base disks' margin
    'rMin', 0.20, 'rMax', 1.20);
t_eePuck_s = toc(t_ee_tic);
fprintf('[HS-Bi-RRT] Generated %d EE pucks.\n', size(eeCircXY,1));

% Focus struct for drawing/saving
focus = struct();
focus.eeCirclesXY = eeCircXY;
focus.eeCirclesR  = eeCircR(:);
focus.eeZ         = eeZ;

if exist('bd','var') && ~isempty(bd)
    focus.baseCenters   = bd(:,1:2);
    focus.baseRadiusVec = bd(:,3);
end

% % Optional focus-region visualization
% try
%     figure('Name','Focus Regions (Plan 4)','Units','normalized','OuterPosition',[0 0 1 1]);
%     axF = gca; hold(axF,'on'); grid(axF,'on'); view(axF,45,30);
%     title('Plan 4: HS-Bi-RRT Focus Regions (Base-Disks + EE-Pucks)');
%     for k = 1:numel(envS), show(envS{k}.CollisionObj, 'Parent', axF); end
%     if ~isempty(guideXY)
%         plot3(guideXY(:,1), guideXY(:,2), zeros(size(guideXY,1),1), 'g-', 'LineWidth', 1.2, 'Parent', axF);
%     end
%     drawFocusRegions(focus, axF);
%     axis(axF,'equal'); xlabel('X'); ylabel('Y'); zlabel('Z');
%     camlight(axF,'headlight'); lighting(axF,'gouraud'); drawnow;
% catch, end

% --- Planning robot & goals (NO freezing) ---
robotPlan  = mobileUR20;               % full 9-DoF
qStartPlan = double(retreatConfig);

% Plan 4 terminates at offset1Pose.
% The EE Pucks guide to offset1Pose, so the qGoal must also match.
% We just need a valid 9-DoF IK solution for this pose, using the
% retreatConfig as a seed.
qGoalPlan = double(ik(endEffector, offsetPose, weights, retreatConfig));

% Check this new, correct goal
mustBeFree(mobileUR20, qGoalPlan, envCol, 'Goal (offset1Pose) is in collision.');

% --- Planner parameters ---
P = struct( ...
    'R_s', 0.5, ...        % C-space sample ratio
    'lambda', 0.2, ...
    'xi', 0.05, ...
    'xi_prime', 0.75, ...
    'xi_double', 3.0, ...
    'alpha', pi/8, ...
    'beta',  pi/8, ...
    'stepSize', 0.4, ...
    'valStep', 0.02, ...
    'nearRadius', 0.4, ...
    'maxTime', 300, ...
    'verboseEvery', 20, ...
    ... % === geometric model parameters ===
    'baseIncircle',  baseIncircle, ...            % base footprint ~ incircle
    'rebarLen',    chooseValue(useRebarEnvelope, rebarEnvelope.lengthM, rebarMeta.LengthM), ...
    'rebarPhysicalLen', rebarMeta.LengthM, ...    % physical rebar length [m]
    'rebarRadius', rebarMeta.DiameterM/2, ...     % physical radius [m]
    'rebarEnvelopeLength', rebarEnvelope.lengthM, ...
    'rebarEnvelopeLengthFactor', rebarEnvelope.lengthFactor, ...
    'rebarEnvelopeHeight', rebarEnvelope.heightM, ...
    'rebarEnvelopeWidth', rebarEnvelope.widthM, ...
    'rebarSag', rebarEnvelope.sagM, ...
    'useRebarSagEnvelope', useRebarEnvelope, ...
    'armRadius',   0.1, ...
    'baseWidth',   0.4, ...   % Width of chassis (Y)
    'baseLength',  0.7, ...   % Length of chassis (X)
    'baseHeight',  0.5);

% Pack EE pucks for the planner: [cx cy z R]
eePucks = [eeCircXY, eeZ*ones(size(eeCircXY,1),1), eeCircR(:)];

% --- Run HS-Bi-RRT (9-DoF with RPZ lock internally) ---
fprintf('[HS-Bi-RRT] Running planner (no freeze, RPZ lock in W-space)...\n');
envColPlan = inflateEnvForPlanning(envCol, 0.05);
[path4_raw, ok, info] = hsBiRRT_paper_v2( ...
    robotPlan, envColPlan, qStartPlan, qGoalPlan, endEffector, P, bd, eePucks);

if ~ok
    error('HS-Bi-RRT (9-DoF) failed to find a solution within the time limit.');
end

t_plan4p = toc(t_plan4p);
fprintf('[HS-Bi-RRT] Planning success — %.1fs | Nodes A = %d | Nodes B = %d | Gap = %.3f\n', ...
        t_plan4p, info.nodesA, info.nodesB, info.bestGap);

% --- 1) Densify with angle-aware interpolation (no collisions here) ---
traj4_exec = hs_interpPath(path4_raw, 0.02);   % same for all planners


% --- 3) Adaptive full-body collision check + local repair ---
% [Check] Adaptive precheck + local repair
fprintf('[Check] Adaptive precheck + local repair (full mobileUR20 + rebar)...\n');
okCheck = assertPaperPathCollisionFree(mobileUR20, traj4_exec, envCol, endEffector, P);

if ~okCheck
    fprintf('[Repair] Subdividing only colliding segments...\n');
    
    % Repair using the same end-effector and planner parameters.
    traj4_exec = repairCollidingSegments(mobileUR20, envCol, ...
                                         traj4_exec, 0.02, 3, endEffector, P);
                                         
    % Final strict pass (throws if still bad)
    assertPaperPathCollisionFree(mobileUR20, traj4_exec, envCol, endEffector, P);
end

%% old check
% traj4_exec = safeInterpolateWithCollisionCheck(mobileUR20, path4_raw, envCol, 0.04, 0.02);
% 
% % Project the trajectory to transportZ with zero roll and pitch.
% % traj4_exec = projectTrajectoryToPlaneRP( ...
% %     mobileUR20, traj4_exec, endEffector, transportZ, [0 0]);
% 
% % --- Re-check collisions on the projected trajectory (adaptive) ---
% fprintf('[Check] Adaptive precheck after RPZ projection...\n');
% okCheck = assertPathCollisionFreeAdaptive(mobileUR20, envCol, traj4_exec, 0.06, 0.02, true);
% if ~okCheck
%     error('Plan 4: RPZ-projected trajectory still in collision.');
% end

t_plan4t = toc(t_plan4t);
fprintf('[HS-Bi-RRT] Total time — %.1fs\n', t_plan4t);

% === Output for downstream (Plan 5) ===
allStates    = [allStates; traj4_exec];
offsetConfig = traj4_exec(end,:);

% t_plan4t = toc(t_plan4t);
% fprintf('Plan 4 (HS-Bi-RRT 9-DoF) TOTAL time: %.3f seconds.\n', t_plan4t);


%% ========================================================================
%  5. PLAN AND EXECUTE PLACE SEQUENCE (PLAN 5 — NEW 2-PART APPROACH)
%  ========================================================================
fprintf('Plan 5: Planning final placement sequence...\n');

% --- Plan 5a: Rotate -90 deg about TCP-Y at Offset 1 position ---
fprintf('Plan 5a: Rotating -90 deg about TCP-Y...\n');

% Define the -90 degree rotation about the TCP's Y-axis
R_tcp_y_neg90 = axang2tform([0 1 0 -pi/2]);

% Calculate the new pose: same position as offset1Pose, but rotated
preTargetPose = offsetPose * R_tcp_y_neg90;
% The orientation of preTargetPose should now match targetPose
% (because offset1Pose was targetPose * R_tcp_y_POS90)

% Solve IK for this new "pre-target" pose
preTargetConfig = double(ik(endEffector, preTargetPose, weights, offsetConfig));
assert(all(isfinite(preTargetConfig)), 'preTargetConfig IK failed');
mustBeFree(mobileUR20, preTargetConfig, envCol, 'preTargetConfig is in collision.');

% Create the rotation trajectory (linear joint interpolation)
traj5_rot = lerpJoints(offsetConfig, preTargetConfig, 20); % 20 steps

% --- Plan 5b: Final Approach to Place (Guarded Move) ---
fprintf('Plan 5b: Straight-in to Target...\n');

% This move now starts from the newly rotated configuration
[traj5_app, qAtPlace] = guardedApproachToolAxis( ...
    mobileUR20, preTargetConfig, targetPose, ...  % <-- START from preTargetConfig
    envCol, ik, endEffector, weights, +0.01);

% --- Combine and store trajectories ---
traj5 = [traj5_rot; traj5_app]; % Combine for metrics and saving

% (Optional visualization loops)
% for i = 1:size(traj5_rot, 1)
%     show(mobileUR20, traj5_rot(i,:), "PreservePlot", false, "Visuals","off","Collisions","on");
%     title("Plan 5a: Rotating"); drawnow;
% end
% for i = 1:size(traj5_app, 1)
%     show(mobileUR20, traj5_app(i,:), "PreservePlot", false, "Visuals","off","Collisions","on");
%     title("Plan 5b: Final placement move"); drawnow;
% end

allStates = [allStates; traj5]; % Add the complete Plan 5 to allStates
%% ========================================================================
%  6. SAVE TO SEED (ENHANCED METRICS FOR COMPARISON)
%  ========================================================================
outDir  = "seeds";
dateDir = fullfile(outDir, string(datetime('now','Format','yyyyMMdd')));
if ~exist(dateDir,'dir'), mkdir(dateDir); end

% Include RunID in filename
stamp = string(datetime('now','Format','yyyyMMdd_HHmmss')) + "_Run" + runID;

% -- Re-add rebar into env copy (for complete replay context)
envForSave = envS;
if exist('rebarObj','var')
    envForSave = [envForSave, {struct('Name','rebar','CollisionObj',rebarObj)}];
end

% -- Compute comprehensive metrics for comparison --
fprintf('Computing comprehensive metrics for comparison...\n');
metrics = computeComprehensiveMetrics(mobileUR20, allStates, traj1, traj2, traj3, traj4_exec, traj5, endEffector);

% -- Motion/rehydration settings
if isfield(P,'motion')
    if strcmpi(P.motion,'BLEND'), modeStr = "BLEND";
    elseif strcmpi(P.motion,'RTR'), modeStr = "RTR";
    else, modeStr = string(P.motion); end
else
    modeStr = "RTR";
end
if isfield(P,'valStep'), valStepSave = P.valStep; else, valStepSave = 0.02; end

rehydration = struct( ...
    'mode',         modeStr, ...
    'val_step',     valStepSave, ...
    'check_coarse', 0.06, ...
    'check_fine',   0.02 ...
);

% -- Guidance artefacts & corridor params
if exist('guideXY','var'),       guideXY_save = guideXY;           else, guideXY_save = []; end
if exist('bestHeightXY','var'),  bestHeightXY_save = bestHeightXY; else, bestHeightXY_save = []; end
if exist('gridRes','var'),       gridRes_save = gridRes;           else, gridRes_save = NaN; end
if exist('tubeRadius','var'),    tubeRadius_save = tubeRadius;     else, tubeRadius_save = NaN; end
if exist('penaltyW','var'),      penaltyW_save = penaltyW;         else, penaltyW_save = NaN; end
if exist('rSafe','var'),         rSafe_save = rSafe;               else, rSafe_save = NaN; end

corridor = struct( ...
    'guideXY',      guideXY_save, ...
    'bestHeightXY', bestHeightXY_save, ...
    'gridRes',      gridRes_save, ...
    'tubeRadius',   tubeRadius_save, ...
    'penaltyW',     penaltyW_save, ...
    'rSafe',        rSafe_save ...
);

% -- Focus regions (spheres or circles)
if exist('eeSpheres','var'), eeSpheres_save = eeSpheres; else, eeSpheres_save = []; end
if exist('eeCircles','var'), eeCircles_save = eeCircles; else, eeCircles_save = []; end
if ~isempty(eeCircles_save), eeTypeStr = "circle"; else, eeTypeStr = "sphere"; end

% [NOTE] Using 'baseDisks' (plural) to match Analysis script
focusSave = struct( ...
    'baseDisks',  bd, ... 
    'eeSpheres',  eeSpheres_save, ...
    'eeCircles',  eeCircles_save, ...
    'eeType',     eeTypeStr ...
);

% -- Poses used in 4a/5 (safe defaults if missing)
if exist('T_retreat','var'), T_retreat_save = T_retreat; else, T_retreat_save = eye(4); end
if exist('offset1Pose','var'), offset1Pose_save = offset1Pose; else, offset1Pose_save = eye(4); end
if exist('offsetPose','var'),  offsetPose_save  = offsetPose;  else, offsetPose_save  = eye(4); end
if exist('targetPose','var'),  targetPose_save  = targetPose;  else, targetPose_save  = eye(4); end
if exist('graspPose','var'),   graspPose_save   = graspPose;   else, graspPose_save   = eye(4); end

poses = struct( ...
    'pickPose',      pickPose, ...
    'graspPose',     graspPose_save, ...
    'retreatPoseEE', T_retreat_save, ...
    'offset1Pose',   offset1Pose_save, ...
    'offsetPose',    offsetPose_save, ...
    'targetPose',    targetPose_save ...
);

% -- Optional paths from planning stage
if exist('path4_5dof','var'),       path4_5dof_save = path4_5dof;         else, path4_5dof_save = []; end
if exist('traj4_exec_5dof','var'),  traj4_exec_5dof_save = traj4_exec_5dof; else, traj4_exec_5dof_save = []; end

% -- Planner info
if exist('info','var'), info4a_save = info; else, info4a_save = struct(); end
rng_state = rng;
if isfield(P,'maxTime'), maxTimeSave = P.maxTime; else, maxTimeSave = NaN; end

% -- Optional saves
if exist('robot5dof','var'), robot5dof_save = robot5dof; else, robot5dof_save = []; end
if exist('freezeList','var'), freezeList_save = freezeList; else, freezeList_save = []; end

% Timing fields used by the analysis scripts.
timingStruct = struct();
timingStruct.bestZ_s = t_bestZ_s;
timingStruct.gen_baseDisks_s = t_baseDisk_s;
timingStruct.gen_eePucks_s = t_eePuck_s;
timingStruct.plan4_planning_s = t_plan4p;     % Planning time for HS-Bi-RRT
timingStruct.plan4_total_s = t_plan4t;        % Total time for Plan 4 
timingStruct.total_s = t_bestZ_s + t_baseDisk_s + t_eePuck_s + t_plan4t; 

% -- Build comprehensive bundle
bundle = struct( ...
  'meta', struct( ...
      'approach', 'bestHeight + HS-Bi-RRT  + (BaseDisk + EE Focus)', ...
      'created',  datetime('now'), ...
      'desc',     'Plan 4 (HS-Bi-RRT) + Plan 5 (guarded approach)', ...
      'matlab',   version, ...
      'script',   mfilename, ...
      'rng',      rng_state ), ...
  'robot',            mobileUR20, ...
  'robot5dof',        robot5dof_save, ...
  'freezeList',       freezeList_save, ...
  'endEffector',      endEffector, ...
  'env',              envForSave, ...
  'rebarMeta',        rebarMeta, ...
  'rebarEnvelope',    rebarEnvelope, ...
  'bestZ_circle_chain', bestZ, ...
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
      'path4a_nodes_5dof', path4_5dof_save, ...
      'traj4a_exec_5dof',  traj4_exec_5dof_save, ...
      'traj4a_full_9dof',  traj4_exec, ...
      'traj5_full_9dof',   traj5, ...
      'full_allStates',    allStates, ...
      'traj1', traj1, ...
      'traj2', traj2, ...
      'traj3', traj3 ...
  ), ...
  'metrics',          metrics, ...
  'success',          true ...
);

saveFile = fullfile(dateDir, "MA_hsBiRRT_CS1_2m_" + stamp + ".mat");
save(saveFile, 'bundle', '-v7.3');
fprintf('[SAVE] MA_HS-Bi-RRT Guided Plan -> %s\n', saveFile);

%% ========================================================================
%  7. REPLAY FULL TRAJECTORY
%  ========================================================================
fprintf('Replaying full path...\n');
figure("Name","Full Trajectory Replay","Units","normalized","OuterPosition",[0,0,1,1]);
axReplay = gca; % Use a new axes for replay
hold(axReplay, 'on'); grid(axReplay, 'on');
axis(axReplay, [-6 6 -6 6 -0.1 2.5]);
view(axReplay, 120, 25); % Match the initial view
% Reload the environment for replay.
[~, ~, envS_replay] = loadRebarTransportScenario;
for i = 1:numel(envS_replay), show(envS_replay{i}.CollisionObj, 'Parent', axReplay); end
% Draw focus regions during replay.
if exist('focus','var') && isstruct(focus)
     fprintf('Drawing focus regions during replay...\n');
     drawFocusRegions(focus, axReplay); 
else
     fprintf('Focus regions not available for replay visualization.\n');
end
camlight(axReplay, 'headlight'); lighting(axReplay, 'gouraud');
% --- Animate Robot Trajectory ---
show(mobileUR20, allStates(1,:), "Visuals","off","Collisions","on", "Parent", axReplay);
envHandle = drawRebarSagEnvelope(mobileUR20, allStates(1,:), endEffector, P, axReplay);
title(axReplay, sprintf("Full Replay (State %d/%d)", 1, size(allStates,1)));
drawnow;
for k = 2:size(allStates, 1) % Start from 2
     show(mobileUR20, allStates(k,:), "Parent", axReplay, "PreservePlot", false, "Visuals","off","Collisions","on", "FastUpdate", true);
     if exist('envHandle','var') && all(isgraphics(envHandle))
         delete(envHandle);
     end
     envHandle = drawRebarSagEnvelope(mobileUR20, allStates(k,:), endEffector, P, axReplay);
     title(axReplay, sprintf("Full Replay (State %d/%d)", k, size(allStates,1)));
     drawnow;
end
hold(axReplay, 'off');
%%=========================================================================
%  ========================================================================
%                  >>> ALL HELPER FUNCTIONS START HERE <<<
%  ========================================================================
%  ========================================================================
%% ========================================================================
%  A) CORE HELPERS

function value = chooseValue(condition, trueValue, falseValue)
    if condition, value = trueValue; else, value = falseValue; end
end

function label = onOff(value)
    if value, label = 'ON'; else, label = 'OFF'; end
end
%  ========================================================================
function planner = configurePlanner(planner)
    if isprop(planner,'SkippedSelfCollisions'),   planner.SkippedSelfCollisions = "parent"; end
    if isprop(planner,'ValidationDistance'),      planner.ValidationDistance = 0.02; end
    if isprop(planner,'MaxConnectionDistance'),   planner.MaxConnectionDistance = 0.3; end
    if isprop(planner,'MaxIterations'),           planner.MaxIterations = 8000; end
    if isprop(planner,'EnableConnectHeuristic'),  planner.EnableConnectHeuristic = true; end
    if isprop(planner,'GoalBias'),                planner.GoalBias = 0.3; end
end
function path = planOrRetry(planner, qStart, qGoal, tries, seedOffset)
    if nargin < 5, seedOffset = 0; end % Default if not provided
    
    qStart = double(qStart); qGoal = double(qGoal);
    path = [];
    for t = 1:max(1,tries)
        % Combine the run index with the attempt number.
        % This ensures Run 1 Attempt 1 != Run 2 Attempt 1
        current_seed = (seedOffset * 1000) + t; 
        rng(current_seed); 
        
        try
            p = plan(planner, qStart, qGoal); 
        catch
            p = []; 
        end
        
        if ~isempty(p)
            fprintf('  [RRT] Success on attempt %d (Seed %d)\n', t, current_seed);
            path = p; 
            return; 
        end
    end
    error("RRT failed to find a solution after %d attempts", tries);
end
function assertPathCollisionFree(robot, states, envCol)
% ASSERTPATHCOLLISIONFREE
%   Waypoint-level collision check.
%   - robot  : rigidBodyTree (e.g., mobileUR20)
%   - states : [N x DoF] joint configurations
%   - envCol : cell array of collision objects

    if isempty(envCol) || isempty(states)
        return;
    end 

    for k = 1:size(states, 1)
        coll = checkCollision(robot, states(k,:), envCol, ...
                              "Exhaustive","on", ...
                              "SkippedSelfCollisions","parent");
        if any(coll, 'all')
            error("Collision in planned path at step %d", k);
        end
    end
end

function mustBeFree(robot, q, envCol, msg)
    if isempty(q) || any(isnan(q)), error('Invalid config provided to mustBeFree: %s', msg); end
    isColliding = any(checkCollision(robot, double(q), envCol, ...
        "Exhaustive","on","SkippedSelfCollisions","parent"), 'all');
    assert(~isColliding, msg);
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
            "Exhaustive","on","SkippedSelfCollisions","parent"), 'all');
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
function drawFrame(T, name, s)
    if nargin < 3, s = 0.1; end
    o = T(1:3,4); R = T(1:3,1:3);
    plot3(o(1),o(2),o(3),'k.','MarkerSize',14); hold on;
    quiver3(o(1),o(2),o(3), s*R(1,1),s*R(2,1),s*R(3,1), 'r','LineWidth',2,'MaxHeadSize',0.5);
    quiver3(o(1),o(2),o(3), s*R(1,2),s*R(2,2),s*R(3,2), 'g','LineWidth',2,'MaxHeadSize',0.5);
    quiver3(o(1),o(2),o(3), s*R(1,3),s*R(2,3),s*R(3,3), 'b','LineWidth',2,'MaxHeadSize',0.5);
    if nargin >= 2 && ~isempty(name)
        text(o(1),o(2),o(3), ['  ' char(name)], 'FontSize', 12, 'Color','k','FontWeight','bold');
    end
end
function S = resolveBaseIndices(robot)
    bxBody = getBody(robot, "base_x");   jx = string(bxBody.Joint.Name);
    byBody = getBody(robot, "base_y");   jy = string(byBody.Joint.Name);
    chBody = getBody(robot, "chassis");  jz = string(chBody.Joint.Name);
    fmt0 = robot.DataFormat; robot.DataFormat = 'struct';
    cfg = homeConfiguration(robot);
    robot.DataFormat = fmt0;
    if isstruct(cfg) && isfield(cfg,'JointName')
        jointNamesInOrder = string({cfg.JointName});
    else
        jointNamesInOrder = strings(1,0);
        for i = 1:numel(robot.Bodies)
            j = robot.Bodies{i}.Joint;
            if ~strcmpi(j.Type,'fixed')
                jointNamesInOrder(end+1) = string(j.Name); %#ok<AGROW>
            end
        end
    end
    ix   = find(jointNamesInOrder == jx, 1);
    iy   = find(jointNamesInOrder == jy, 1);
    iyaw = find(jointNamesInOrder == jz, 1);
    assert(~isempty(ix) && ~isempty(iy) && ~isempty(iyaw), ...
        'resolveBaseIndices: base joints not found in configuration order.');
    S.baseIdx    = [ix iy iyaw];
    S.baseNames  = [jx jy jz];
    S.jointNames = jointNamesInOrder;
end
function traj = lerpJoints(qA, qB, N)
qA=double(qA); qB=double(qB);
traj = zeros(N, numel(qA));
for k=1:N, s=(k-1)/(N-1); traj(k,:)=(1-s)*qA+s*qB; end
end
function traj = cartesianLift(robot, q0, ee, ik, w, dz, step)
q0 = double(q0);
n = max(1,ceil(dz/max(step,1e-6))); traj = zeros(n+1, numel(q0)); traj(1,:)=q0; T0 = getTransform(robot, q0, ee);
q = q0;
for k=1:n
    T = T0; T(3,4) = T0(3,4) + (dz*k/n);
    q = double(ik(ee, T, w, q));
    if any(isnan(q)), q = traj(k,:); end
    traj(k+1,:)=q;
end
end
function a = wrapToPi(a), a = mod(a+pi,2*pi)-pi; end
%% ========================================================================
%  B) BASE-DISK GENERATION HELPERS
%  ========================================================================
function baseDisks = generateMaximalBaseDisksAlongPath(start_xy, goal_xy, envS, guidePathXY, baseIncircle, rMin, margin, maxDisks)
    % Maximal disk chain along a corridor, with a goal-depth guarantee.
    % Ensures the goal lies *well inside* the last disk, not just touching.
    if isempty(guidePathXY), guidePathXY = [start_xy; goal_xy]; end
    % ---- tunables ----
    rCap            = 0.8;      % hard cap on disk radius
    gaSteps         = 3;        % small refine steps (keeps center near guide/goal)
    gaStepSize      = 0.02;
    advanceFactor   = 1.0;      % step distance multiplier (times lastR)
    minStepAbs      = 0.3;     % minimum absolute move along the path
    goalCoverFrac   = 0.80;     % goal must be within 80% of last disk radius
    goalShiftFrac   = 0.25;     % per-iteration pull of last center toward goal (× lastR)
    goalShiftIters  = 10;       % max iterations for shifting the last center
    L = {};
    % ---- seed at start ----
    c0  = refineCenter2D(start_xy, envS, gaSteps, gaStepSize);
    clr = minClearance2D(c0, envS);
    R0  = max(rMin, min(rCap, clr - baseIncircle - margin));
    L{end+1} = [c0, R0];
    idxOnPath = 1;
    safety_counter = 0;
    max_iterations = maxDisks * 2;
    while numel(L) < maxDisks && safety_counter < max_iterations
        safety_counter = safety_counter + 1;
        lastC = L{end}(1:2);
        lastR = L{end}(3);
        % stop if the goal is already nicely inside the last disk
        if norm(goal_xy - lastC) <= goalCoverFrac * lastR
            break;
        end
        % otherwise advance along the guide
        advanceDist = max(minStepAbs, advanceFactor * max(lastR, rMin));
        [found, idxOnPath] = advanceAlongPolyline(guidePathXY, idxOnPath, lastC, advanceDist);
        if ~found
            break;
        end
        cCand = guidePathXY(idxOnPath,:);
        cRef  = refineCenter2D(cCand, envS, gaSteps, gaStepSize);
        clr   = minClearance2D(cRef, envS);
        R     = max(rMin, min(rCap, clr - baseIncircle - margin));
        % accept if it actually progresses
        if norm(cRef - lastC) > 0.5 * rMin
            L{end+1} = [cRef, R];
        else
            % no meaningful progress: stop trying to add more
            break;
        end
    end
    % ---- post-pass: pull the last disk center toward goal for *depth* ----
    if ~isempty(L)
        % try to shift the *last* disk center toward the goal while keeping it valid
        for it = 1:goalShiftIters
            lastC = L{end}(1:2);
            lastR = L{end}(3);
            if norm(goal_xy - lastC) <= goalCoverFrac * lastR
                break; % already deep enough
            end
            dir = goal_xy - lastC; d = norm(dir);
            if d < 1e-9, break; end
            stepToward = goalShiftFrac * lastR;
            cTry0 = lastC + (stepToward/d) * dir;
            % Apply a small refinement to remain near the goal.
            cTry  = refineCenter2D(cTry0, envS, 2, 0.01);
            clr   = minClearance2D(cTry, envS);
            RTry  = max(rMin, min(rCap, clr - baseIncircle - margin));
            % accept the shift if the new disk is valid (not too small)
            if RTry >= 0.75 * rMin
                L{end} = [cTry, RTry];
            else
                break; % too tight to move further
            end
        end
    end
    % ---- if still shallow, append a compact goal disk closer to goal ----
    if ~isempty(L)
        lastC = L{end}(1:2); lastR = L{end}(3);
        if norm(goal_xy - lastC) > goalCoverFrac * lastR
            % place a new candidate center between lastC and goal (closer to goal)
            u = goal_xy - lastC; un = u / max(norm(u), 1e-9);
            c0 = goal_xy - un * min(0.5 * lastR, 0.6 * norm(u));
            cRef = refineCenter2D(c0, envS, 2, 0.01);
            clr  = minClearance2D(cRef, envS);
            Rg   = max(rMin, min(rCap, clr - baseIncircle - margin));
            L{end+1} = [cRef, Rg];
        end
    end
    baseDisks = cell2mat(L.');
end
function [ok, idx] = advanceAlongPolyline(P, idx0, c, distNeed)
ok=false; idx = idx0; if size(P,1)<2, return; end
acc = 0; for k=max(2,idx0):size(P,1)
    seg = norm(P(k,:)-P(k-1,:)); acc = acc + seg;
    if acc >= distNeed || norm(P(k,:)-c) >= 0.9*distNeed
        ok=true; idx=k; return;
    end
end
end
function center_refined = refineCenter2D(c, envS, maxSteps, step)
center_refined = c; eps=1e-4;
for i=1:maxSteps
    d0 = minClearance2D(center_refined, envS); if d0<=0, break; end
    dx = (minClearance2D(center_refined+[eps 0], envS)-d0)/eps;
    dy = (minClearance2D(center_refined+[0 eps], envS)-d0)/eps;
    g = [dx dy]; ng = norm(g);
    if ng<1e-6, break; end
    center_refined = center_refined + step*(g/ng);
end
end
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
xs=[sxy(1) gxy(1)]; ys=[sxy(2) gxy(2)];
for i_=1:numel(envS_in)
    co=getObj(envS_in{i_}); % Use getObj
    if isempty(co), continue; end
    T=co.Pose;
    if isa(co,'collisionBox'), c=T(1:2,4).'; h=[co.X co.Y]/2; xs=[xs,c(1)-h(1),c(1)+h(1)]; ys=[ys,c(2)-h(2),c(2)+h(2)];
    elseif isa(co,'collisionCylinder')||isa(co,'collisionSphere'), c=T(1:2,4).'; R=co.Radius; xs=[xs,c(1)-R,c(1)+R]; ys=[ys,c(2)-R,c(2)+R];
    end
end
xlimW=[min(xs)-pad, max(xs)+pad]; ylimW=[min(ys)-pad, max(ys)+pad];
end
function [sIJ,gIJ] = world2grid(sxy, gxy, occ)
find_last = @(v,val) find(v <= val, 1, 'last');
if isempty(find_last(occ.yv, sxy(2))), s_row=1; else, s_row=find_last(occ.yv, sxy(2)); end
if isempty(find_last(occ.xv, sxy(1))), s_col=1; else, s_col=find_last(occ.xv, sxy(1)); end
if isempty(find_last(occ.yv, gxy(2))), g_row=1; else, g_row=find_last(occ.yv, gxy(2)); end
if isempty(find_last(occ.xv, gxy(1))), g_col=1; else, g_col=find_last(occ.xv, gxy(1)); end
s_row=max(1,min(s_row,numel(occ.yv))); s_col=max(1,min(s_col,numel(occ.xv)));
g_row=max(1,min(g_row,numel(occ.yv))); g_col=max(1,min(g_col,numel(occ.xv)));
sIJ=[s_row s_col]; gIJ=[g_row g_col];
end
function ijFree = ensureFreeIJ(ij, occ)
H=size(occ.grid,1); W=size(occ.grid,2);
i0=min(max(round(ij(1)),1),H); j0=min(max(round(ij(2)),1),W);
if ~occ.grid(i0,j0), ijFree=[i0 j0]; return; end
% small BFS ring
best=[NaN NaN]; bestD=Inf;
for r=1:15
    for di=-r:r
        for dj=-r:r
            if di==0 && dj==0, continue; end
            i=i0+di; j=j0+dj;
            if i>=1 && i<=H && j>=1 && j<=W && ~occ.grid(i,j)
                d=hypot(i-ij(1),j-ij(2));
                if d<bestD, bestD=d; best=[i j]; end
            end
        end
    end
    if ~isnan(best(1)), ijFree=best; return; end
end
ijFree=[i0 j0]; % fallback
end
function [pathIJ, ok] = aStar8_weighted(occ, C, sIJ, gIJ)
% Weighted 8-connected A* on a grid with per-cell traversal costs C.
    Gfree = isfinite(C);            % free if not Inf
    [H,W] = size(C);
    s = sub2ind([H W], sIJ(1), sIJ(2));
    g = sub2ind([H W], gIJ(1), gIJ(2));
    if ~Gfree(s) || ~Gfree(g), pathIJ = []; ok = false; return; end
    nb  = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];
    nbc = [sqrt(2) 1 sqrt(2) 1 1 sqrt(2) 1 sqrt(2)];  % step lengths
    cameFrom = containers.Map('KeyType','int32','ValueType','int32');
    gScore   = inf(H*W,1); gScore(s) = 0;
    fScore   = inf(H*W,1); fScore(s) = heuristic(s,g,occ);
    openIds  = s; 
    openKeys = fScore(s);
    ok = false;
    while ~isempty(openIds)
        [~,iMin] = min(openKeys);
        current  = openIds(iMin);
        openIds(iMin)  = [];
        openKeys(iMin) = [];
        if current == g
            ok = true; break;
        end
        [ci,cj] = ind2sub([H W], current);
        for n = 1:8
            ni = ci + nb(n,1); nj = cj + nb(n,2);
            if ni<1 || ni>H || nj<1 || nj>W, continue; end
            nbidx = sub2ind([H W], ni, nj);
            if ~Gfree(nbidx), continue; end
            stepCost = (C(current) + C(nbidx))*0.5 * nbc(n);
            tentative = gScore(current) + stepCost;
            if tentative < gScore(nbidx)
                cameFrom(nbidx) = current;
                gScore(nbidx)   = tentative;
                fNew = tentative + heuristic(nbidx, g, occ);
                fScore(nbidx) = fNew;
                k = find(openIds==nbidx, 1);
                if isempty(k)
                    openIds(end+1)  = nbidx; %#ok<AGROW>
                    openKeys(end+1) = fNew;  %#ok<AGROW>
                else
                    openKeys(k) = min(openKeys(k), fNew);
                end
            end
        end
    end
    if ~ok, pathIJ = []; return; end
    seq = g;
    while isKey(cameFrom, seq(1))
        seq = [cameFrom(seq(1)) seq]; %#ok<AGROW>
    end
    [ii,jj] = ind2sub([H W], seq(:)');
    pathIJ = [ii(:) jj(:)];
    function h = heuristic(a, b, occ_)
        [ia,ja] = ind2sub([H W], a);
        [ib,jb] = ind2sub([H W], b);
        h = hypot(occ_.xv(ja) - occ_.xv(jb), occ_.yv(ia) - occ_.yv(ib));
    end
end
function pathXY = grid2world(pathIJ, occ)
if isempty(pathIJ), pathXY=[]; return; end
pathXY=[occ.xv(pathIJ(:,2)).', occ.yv(pathIJ(:,1)).'];
end
function out = shortcutSmoothing(P, occ)
if size(P,1)<=2, out=P; return; end
out = P(1,:); i = 1;
while i < size(P,1)
    j = size(P,1);
    while j > i+1
        if collisionFreeSegment(P(i,:),P(j,:),occ)
            out = [out; P(j,:)]; i=j; break; %#ok<AGROW>
        end
        j=j-1;
    end
    if j==i+1, out=[out; P(i+1,:)]; i=i+1; end %#ok<AGROW>
    if i >= size(P,1), break; end
end
end
function ok = collisionFreeSegment(pA, pB, occ)
L=hypot(pB(1)-pA(1), pB(2)-pA(2));
n=max(2,ceil(L/occ.res));
xs=linspace(pA(1),pB(1),n); ys=linspace(pA(2),pB(2),n);
ok=true;
for k=1:n
    [r,c]=world2grid_scalar([xs(k) ys(k)], occ);
    if r<1||r>size(occ.grid,1)||c<1||c>size(occ.grid,2)||occ.grid(r,c)
        ok=false; return;
    end
end
end
function [i,j]=world2grid_scalar(p,occ)
ix=find(occ.xv<=p(1),1,'last'); iy=find(occ.yv<=p(2),1,'last');
if isempty(ix), ix=1; end; if isempty(iy), iy=1; end
i=max(1,min(iy,size(occ.grid,1))); j=max(1,min(ix,size(occ.grid,2)));
end
function p = grid2world_scalar(ij, occ)
    p = [occ.xv(ij(2)), occ.yv(ij(1))];
end
function pts = resamplePolylineByStep(XY, step)
if isempty(XY) || size(XY,1)<=1, pts=XY; return; end
seg=sqrt(sum(diff(XY).^2,2)); L=[0;cumsum(seg)];
s=0:step:L(end); if isempty(s) || s(end)<L(end), s=[s L(end)]; end
pts=interp1(L,XY,unique(s),'linear');
end
function D = polylineDistField(polyXY, xv, yv)
% returns [ny x nx] distances (meters) from each grid cell center to the polyline
[XX,YY] = meshgrid(xv, yv);  % [ny x nx]
P = [XX(:) YY(:)];
D = inf(size(P,1),1);
if isempty(polyXY) || size(polyXY,1)<2
    D(:) = 1e3; % No path, return large distance
else
    for i = 1:size(polyXY,1)-1
        A = polyXY(i,:); B = polyXY(i+1,:);
        v = B - A; vv = sum(v.*v);
        t = max(0, min(1, ((P - A) * v.') / max(vv,1e-12)));
        proj = A + t.*v;
        dseg = sqrt(sum((P - proj).^2, 2));
        D = min(D, dseg);
    end
end
D = reshape(D, numel(yv), numel(xv));
end
function drawBaseDisks3D(ax, baseDisks, zConst, varargin)
% Draw base-disk chain as flat translucent disks at z = zConst
p = inputParser;
addParameter(p,'FaceColor',[0.20 0.60 1.00]);  % blue-ish
addParameter(p,'FaceAlpha',0.22);
addParameter(p,'EdgeColor',[0.10 0.30 0.50]);
addParameter(p,'EdgeAlpha',0.7);
addParameter(p,'EdgeWidth',0.8);
addParameter(p,'ShowCenters',true);
parse(p,varargin{:});
if isempty(baseDisks), return; end
% Handle both struct and matrix input
if isstruct(baseDisks)
    centers = vertcat(baseDisks.center);
    radii = vertcat(baseDisks.radius);
else
    centers = baseDisks(:,1:2);
    radii = baseDisks(:,3);
end
th = linspace(0,2*pi,80);
hold(ax,'on');
for k=1:size(centers,1)
    cx=centers(k,1); cy=centers(k,2); R=radii(k);
    xv = cx + R*cos(th); yv = cy + R*sin(th);
    zv = zConst*ones(size(xv));
    patch('Parent',ax,'XData',xv,'YData',yv,'ZData',zv, ...
        'FaceColor',p.Results.FaceColor,'FaceAlpha',p.Results.FaceAlpha, ...
        'EdgeColor',p.Results.EdgeColor,'EdgeAlpha',p.Results.EdgeAlpha, ...
        'LineWidth',p.Results.EdgeWidth);
end
if p.Results.ShowCenters
    plot3(ax, centers(:,1), centers(:,2), ...
        zConst*ones(size(centers,1),1), '--', ...
        'Color', p.Results.EdgeColor, 'LineWidth', 1.3);
end
end
function obj = getObj(entry)
if isstruct(entry) && isfield(entry,'CollisionObj')
    obj = entry.CollisionObj;
else
    obj = entry;
end
end
%% ========================================================================
%  D) HS-Bi-RRT AND EE-SPHERE HELPERS
%  ========================================================================
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
function d = minClearance3D(p, env)
% Three-dimensional clearance calculation.
    d = inf; p = p(:).'; 
    for i = 1:numel(env)
        % --- Handle struct {Name, CollisionObj} ---
        if isstruct(env{i}) && isfield(env{i},'CollisionObj')
             co = env{i}.CollisionObj;
        else
             co = env{i}; % Assume it's a raw collision object
        end
        % -------------------------------------------
        T = co.Pose; 
        if isa(co,'collisionBox')
            R=T(1:3,1:3); t=T(1:3,4).'; pL=(p-t)/R.'; h=[co.X,co.Y,co.Z]/2; 
            q=abs(pL)-h; di=norm(max(q,0)); 
            if all(q<=0), di=max(q); end 
        elseif isa(co,'collisionSphere')
            c=T(1:3,4).'; di=norm(p-c)-co.Radius;
        elseif isa(co,'collisionCylinder')
            R=T(1:3,1:3); t=T(1:3,4).'; pL=(p-t)/R.'; r=co.Radius; h2=co.Height/2; 
            vec_d=[hypot(pL(1),pL(2))-r, abs(pL(3))-h2]; 
            di=norm(max(vec_d,0))+min(max(vec_d(1),vec_d(2)),0);
        else
            continue; 
        end
        d = min(d, di); 
    end
    if isinf(d), d = 100; end
end
function [C, R] = generateMaximalEESpheresAlongPath(pStart, pGoal, envObs, guidePathXY)
% Generates maximal free-space end-effector spheres along guidePathXY.
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
function drawFocusRegions(focus, ax) 
% Focus-region visualization.
    if nargin<2||isempty(ax),ax=gca;end; hold(ax,'on'); grid(ax,'on');
    if isfield(focus,'baseCenters')&&~isempty(focus.baseCenters)
        baseR=focus.baseRadiusVec(:); th=linspace(0,2*pi,40); baseCol=[1 0 1]; baseAlpha=0.15;
        for k=1:size(focus.baseCenters,1)
            cx=focus.baseCenters(k,1); cy=focus.baseCenters(k,2); 
            if k<=numel(baseR),Rk=baseR(k);else Rk=0.1;end
            xv=cx+Rk*cos(th); yv=cy+Rk*sin(th); 
            patch(ax,xv,yv,zeros(size(xv)),baseCol,'FaceAlpha',baseAlpha,'EdgeColor',baseCol*0.8,'LineWidth',1); 
        end
        if size(focus.baseCenters,1)>=2
            plot3(ax,focus.baseCenters(:,1),focus.baseCenters(:,2),zeros(size(focus.baseCenters,1),1),'m--','LineWidth',1); 
        end
    end
    if isfield(focus,'eeCirclesXY') && ~isempty(focus.eeCirclesXY)
        th = linspace(0,2*pi,80);
        z  = focus.eeZ;
        for k=1:size(focus.eeCirclesXY,1)
            cx = focus.eeCirclesXY(k,1);
            cy = focus.eeCirclesXY(k,2);
            R  = focus.eeCirclesR(k);
            xv = cx + R*cos(th); yv = cy + R*sin(th);
            patch('Parent',ax,'XData',xv,'YData',yv,'ZData',z*ones(size(xv)), ...
                  'FaceColor',[0 0.6 1],'FaceAlpha',0.14, ...
                  'EdgeColor',[0 0.3 0.6],'LineWidth',0.8);
        end
        plot3(ax, focus.eeCirclesXY(:,1), focus.eeCirclesXY(:,2), ...
                  z*ones(size(focus.eeCirclesXY,1),1), 'b--', 'LineWidth',1.0);
    end
    axis(ax,'equal');
end
%% ========================================================================
%  E) HS-BI-RRT PLANNER
%  ========================================================================
function [path, ok, info] = hsBiRRT_paper_v2(robot, envCol, qStart, qGoal, ee, P, baseDisks, eeGuidance)
%HSBIRRT_PAPER_V2  HS-Bi-RRT with Base-Disks (C-space) + EE-Pucks (W-space).
% (RPZ-locked variant: locks TCP Z & roll/pitch during W-space steps; yaw free.)
%
% INPUTS:
%   robot      : rigidBodyTree (full 9-DoF)
%   envCol     : {collisionObjects...} or {struct('CollisionObj',co)...}
%   qStart     : 1×D start configuration (row)
%   qGoal      : 1×D goal configuration  (row)
%   ee         : end-effector body name
%   P          : planner params (R_s, stepSize, valStep, etc.)
%   baseDisks  : N×3 [cx cy R] (can be empty)
%   eeGuidance : N×4 [cx cy z R] EE pucks (fixed Z)
%
% OUTPUTS:
%   path : M×D path (row states), possibly empty if ok==false
%   ok   : true if a connection was found
%   info : struct(iterations, bestGap, nodesA, nodesB)

    % ------------------------ Setup / Defaults ------------------------
    Sx = resolveBaseIndices(robot);
    ix   = Sx.baseIdx(1);
    iy   = Sx.baseIdx(2);
    iyaw = Sx.baseIdx(3);

    % Wrap mask: treat everything as angular EXCEPT base X,Y
    wrapIdx = true(1, numel(qStart));
    wrapIdx([ix iy]) = false;

    
    P = fillDefaults(P);
    armBodyNames = findArmBodyNames(robot, ee);

    % Parse EE pucks
    eeCirclesXY = [];
    eeCirclesR  = [];
    eeZ         = [];
    if ~isempty(eeGuidance) && isnumeric(eeGuidance) && size(eeGuidance,2)==4
        eeCirclesXY = eeGuidance(:,1:2);
        eeZ         = eeGuidance(1,3);
        eeCirclesR  = eeGuidance(:,4);
    end
    numEEpucks = size(eeCirclesXY,1);

    % Trees
    TA.q = qStart; TA.parent = 0; TA.eePos = tcpPos(robot, qStart, ee);
    TA.sigma_d = P.xi; TA.sigma_s = P.xi; TA.dIdx=1; TA.sIdx=1;
    TB.q = qGoal;  TB.parent = 0; TB.eePos = tcpPos(robot, qGoal,  ee);
    TB.sigma_d = P.xi; TB.sigma_s = P.xi; TB.dIdx=max(1,size(baseDisks,1)); TB.sIdx=max(1,numEEpucks);

    TAin = TA; TBin = TB;
    ok = false; path = []; t0=tic; it=0; bestGap=inf; bestPair=[1 1];

    fprintf('[HS-Bi-RRT] Starting loop (maxTime = %.1f s)...\n', P.maxTime);

    % ------------------------- Main Loop ------------------------------
    while toc(t0) < P.maxTime
        it = it + 1;

        % Choose source/target tree
        if size(TAin.q,1) <= size(TBin.q,1)
            treeSrc = 'A'; Tin = TAin; Tout = TBin;
        else
            treeSrc = 'B'; Tin = TBin; Tout = TAin;
        end

        grew = false; hit = false; spaceUsed = 'C';
        sampleTic = tic;

        if rand < P.R_s
            % ---------- C-space sampling: move base only; keep arm fixed ----------
            dIdx = max(1, min(Tin.dIdx, size(baseDisks,1)));
            if isempty(baseDisks)
                ctr = [Tin.q(end,ix) Tin.q(end,iy)]; Rb = 0.5;
            else
                ctr = baseDisks(dIdx,1:2); Rb = baseDisks(dIdx,3);
            end
            if treeSrc=='A', dAimIdx = min(dIdx+1, size(baseDisks,1));
            else,            dAimIdx = max(dIdx-1, 1);
            end
            aimCtr = ctr; if ~isempty(baseDisks) && dAimIdx>0, aimCtr = baseDisks(dAimIdx,1:2); end
            useNext = (rand < 0.7);
            aimXY   = useNext*aimCtr + (~useNext)*ctr;

            base_xy  = aimXY + (Tin.sigma_d*Rb) * randn(1,2);
            v        = base_xy - ctr; r = norm(v);
            if r > max(Rb,eps), base_xy = ctr + (Rb/r)*v; end

            base_yaw = wrapToPi(P.alpha*(rand-0.5));

            qNearIdx = nearNonholonomic(Tin.q, [base_xy base_yaw], ix, iy, iyaw, P.alpha, P.beta);
            qNear    = Tin.q(qNearIdx,:);

            qRand = qNear;
            qRand([ix iy]) = base_xy;
            qRand(iyaw)    = wrapToPi(qNear(iyaw) + base_yaw);

            % *** keep arm still ***
            armIdx = setdiff(1:numel(qNear), [ix iy iyaw]);
            if ~isempty(armIdx), qRand(armIdx) = qNear(armIdx); end

            qNew = steer(qNear, qRand, P.stepSize, wrapIdx, ix, iy, iyaw);
            spaceUsed = 'C';

        else
            % ---------- W-space sampling: aim TCP into EE puck (Z & RP locked) ----------
            if numEEpucks == 0
                % fallback: aim to other tree's EE XY on current Z
                if treeSrc=='A', TgoalEE = getTransform(robot, qGoal, ee);
                else,            TgoalEE = getTransform(robot, qStart, ee);
                end
                ee_tgt = TgoalEE(1:3,4).';
                ee_tgt(3) = Tin.eePos(end,3);
            else
                sIdx = max(1, min(Tin.sIdx, numEEpucks));
                cE   = eeCirclesXY(sIdx,:); Re = eeCirclesR(sIdx);
                samp = cE + (Tin.sigma_s * Re) * randn(1,2);
                v    = samp - cE; rr = norm(v);
                if rr > max(Re,eps), samp = cE + (Re/rr) * v; end
                if isempty(eeZ), zHere = Tin.eePos(end,3); else, zHere = eeZ; end
                ee_tgt = [samp, zHere];
            end

            % choose nearest node by XY distance of TCP
            difXY = Tin.eePos(:,1:2) - ee_tgt(1:2);
            eeDistsXY = hypot(difXY(:,1), difXY(:,2));
            [~, qNearIdx] = min(eeDistsXY);
            qNear = Tin.q(qNearIdx,:);

            % Build a pose target at (ee_tgt) but lock Z and RP (yaw from current)
            Tnear = getTransform(robot, qNear, ee);
            Ttgt  = Tnear; Ttgt(1:3,4) = ee_tgt(:);

            qNew  = jacobianStep(robot, qNear, Ttgt, ee, P.stepSize, wrapIdx); % RPZ-locked IK
            spaceUsed = 'W';
        end

        sampleTime = toc(sampleTic);
        extendTic  = tic;

        if segmentFree(robot, envCol, qNear, qNew, P.valStep, wrapIdx, ix, iy, iyaw)
            % Add node
            Tin.q(end+1,:)      = qNew;
            Tin.parent(end+1)   = qNearIdx;
            Tin.eePos(end+1,:)  = tcpPos(robot, qNew, ee);
            grew = true;
            idxNew = size(Tin.q,1);

            % Advance base disks
            if ~isempty(baseDisks)
                curD = Tin.dIdx;
                if treeSrc=='A', nxtD = min(curD+1, size(baseDisks,1));
                else,            nxtD = max(curD-1, 1);
                end
                if nxtD ~= curD && size(baseDisks,1) > 1
                    qBaseXY = qNew([ix iy]);
                    cCur    = baseDisks(curD,1:2);
                    cNxt    = baseDisks(nxtD,1:2);
                    dCur    = norm(qBaseXY - cCur);
                    dNxt    = norm(qBaseXY - cNxt);
                    inNext  = dNxt <= 0.95 * baseDisks(nxtD,3);
                    if inNext || (dNxt + 0.05 < dCur)
                        Tin.dIdx   = nxtD;
                        Tin.sigma_d = P.xi_prime;
                    end
                end
            end

            % Advance EE pucks
            if numEEpucks > 0
                curS = Tin.sIdx;
                if treeSrc=='A', nxtS = min(curS+1, numEEpucks);
                else,            nxtS = max(curS-1, 1);
                end
                if nxtS ~= curS && numEEpucks > 1
                    eeNowXY = Tin.eePos(end,1:2);
                    dCurE   = hypot(eeNowXY(1)-eeCirclesXY(curS,1), eeNowXY(2)-eeCirclesXY(curS,2));
                    dNxtE   = hypot(eeNowXY(1)-eeCirclesXY(nxtS,1), eeNowXY(2)-eeCirclesXY(nxtS,2));
                    inNextE = dNxtE <= 0.95 * eeCirclesR(nxtS);
                    if inNextE || (dNxtE + 0.05 < dCurE)
                        Tin.sIdx   = nxtS;
                        Tin.sigma_s = P.xi_prime;
                    end
                end
            end

            % Try to connect the other tree toward the new node
            [Tout, hit, idxOther, qOtherNew] = connectToward(robot, envCol, ee, Tout, qNew, P, wrapIdx, ix, iy, iyaw);

            % Track best Euclidean gap
            gapNow = distJoint(qOtherNew, qNew, wrapIdx);
            if gapNow < bestGap, bestGap = gapNow; bestPair = [idxNew, idxOther]; end

            % Writeback
            if treeSrc=='A', TAin = Tin; TBin = Tout; else, TBin = Tin; TAin = Tout; end

            if hit
                if treeSrc=='A', path = stitchPath(TAin, TBin, idxNew, idxOther);
                else,             path = stitchPath(TAin, TBin, idxOther, idxNew);
                end
                ok = true; break;
            end
        end

        extendTime = toc(extendTic);

        % Focus radii cooling/heating
        if spaceUsed=='C', sigmaField = 'sigma_d'; else, sigmaField = 'sigma_s'; end
        if grew
            Tin.(sigmaField) = max(P.xi, Tin.(sigmaField) * (1 - P.lambda));
        else
            Tin.(sigmaField) = min(P.xi_double, Tin.(sigmaField) * (1 + P.lambda));
            if Tin.(sigmaField) >= P.xi_double, Tin.(sigmaField) = P.xi_prime; end
        end
        if treeSrc=='A', TAin = Tin; else, TBin = Tin; end

        if P.verboseEvery > 0 && mod(it, P.verboseEvery) == 0
            ngap = nearestGap(TAin.q, TBin.q, wrapIdx);
            fprintf('[HS-Bi-RRT] it=%d (%.1fs)| |A|=%d |B|=%d | Gap=%.3f | dA=%d sA=%d | dB=%d sB=%d | SigD=%.2f SigS=%.2f | SampT=%.3fs ExtConT=%.3fs\n', ...
                it, toc(t0), size(TAin.q,1), size(TBin.q,1), ngap, ...
                TAin.dIdx, TAin.sIdx, TBin.dIdx, TBin.sIdx, ...
                Tin.sigma_d, Tin.sigma_s, sampleTime, extendTime);
        end
    end

    % Fallback: stitch best near pair if segmentFree
    if ~ok && all(bestPair > 0) && size(TAin.q,1) >= bestPair(1) && size(TBin.q,1) >= bestPair(2)
        qA = TAin.q(bestPair(1),:); qB = TBin.q(bestPair(2),:);
        if segmentFree(robot, envCol, qA, qB, P.valStep, wrapIdx, ix, iy, iyaw)
            path = stitchPath(TAin, TBin, bestPair(1), bestPair(2)); ok = true;
            fprintf('[HS-Bi-RRT] Connected best near pair post-loop (euc-gap=%.3f).\n', bestGap);
        else
            fprintf('[HS-Bi-RRT] Fallback failed collision check (euc-gap=%.3f).\n', bestGap);
        end
    elseif ~ok
        fprintf('[HS-Bi-RRT] Failed to find solution & fallback invalid.\n');
    end

    info = struct('iterations', it, 'bestGap', bestGap, ...
                  'nodesA', size(TAin.q,1), 'nodesB', size(TBin.q,1));

% =========================== Helpers (nested) ===========================
    function S = resolveBaseIndices(rb)
        bxBody = getBody(rb, "base_x"); jx = string(bxBody.Joint.Name);
        byBody = getBody(rb, "base_y"); jy = string(byBody.Joint.Name);
        chBody = getBody(rb, "chassis"); jz = string(chBody.Joint.Name);
        fmt0 = rb.DataFormat; rb.DataFormat = 'struct';
        cfg = homeConfiguration(rb); rb.DataFormat = fmt0;
        jnames = string({cfg.JointName});
        ix_ = find(jnames==jx,1); iy_ = find(jnames==jy,1); iz_ = find(jnames==jz,1);
        assert(~isempty(ix_)&&~isempty(iy_)&&~isempty(iz_), 'Base joints not found.');
        S.baseIdx = [ix_ iy_ iz_];
    end
    function armBodies = findArmBodyNames(rb, eeName)
        % Walk up from ee to "chassis", collecting manipulator bodies.
        armBodies = strings(0,1);
        b = getBody(rb, eeName);
        % Stop at "chassis" or base
        while ~strcmpi(b.Name,"chassis") && ~strcmpi(b.Name, rb.BaseName)
            % Skip the attached rebar body if present
            if ~strcmpi(b.Name,"rebar")
                armBodies(end+1,1) = string(b.Name); %#ok<AGROW>
            end
            if isempty(b.Parent)
                break;
            end
            b = b.Parent;
        end
        armBodies = flipud(armBodies); % from base side to ee side

        % Optional: subsample to reduce cost (e.g. keep at most 6 bodies)
        if numel(armBodies) > 6
            idx = round(linspace(1, numel(armBodies), 6));
            armBodies = armBodies(idx);
        end
    end

    function Pth = fillDefaults(Pin)
        D = struct( ...
            'R_s',0.5, 'lambda',0.2, 'xi',0.05, 'xi_prime',0.75, ...
            'xi_double',3.0, 'alpha',pi/12, 'beta',pi/12, ...
            'stepSize',0.5, 'valStep',0.02, 'nearRadius',1.0, ...
            'maxTime',600, 'verboseEvery',20, ...
            ...
            'baseIncircle', 0.25, ...        % Reduced to match reality
            'rebarLen',   1.0, ...
            'rebarPhysicalLen', 1.0, ...
            'rebarRadius',0.01, ...
            'rebarEnvelopeLength', 1.0, ...
            'rebarEnvelopeLengthFactor', 1.0, ...
            'rebarEnvelopeHeight', 0.0, ...
            'rebarEnvelopeWidth', 0.02, ...
            'rebarSag', 0.0, ...
            'useRebarSagEnvelope', false, ...
            'armRadius',  0.08, ...
            'zFloor',     0.005, ...       % Allow touching Z=0.005
            ...
            ... % Mobile-base box dimensions
            'baseWidth',   0.40, ...  % Width (Y)
            'baseLength',  0.7, ...  % Length (X)
            'baseHeight',  0.5 ...   % Height (Z)
        );
        Pth = D;
        if nargin>0 && ~isempty(Pin)
            fn = fieldnames(D);
            for k = 1:numel(fn)
                f = fn{k};
                if isfield(Pin,f) && ~isempty(Pin.(f))
                    Pth.(f) = Pin.(f);
                end
            end
        end
    end

    function p = tcpPos(rb, q, ee_), T = getTransform(rb, q, ee_); p = T(1:3,4).'; end
    function a = wrapToPi(a), a = mod(a+pi, 2*pi) - pi; end

    function dq = angleAwareDiff(a, b, wrapMask)
        dq = a - b;
        idx = find(wrapMask);
        for c = idx, dq(c) = atan2(sin(dq(c)), cos(dq(c))); end
    end

    function d = distJoint(q1, q2, wrapMask)
        D = q2 - q1;
        idx = find(wrapMask); for c = idx, D(:,c)=atan2(sin(D(:,c)),cos(D(:,c))); end
        d = sqrt(sum(D.^2, 2));
    end

    function idx = nearNonholonomic(Q, tgtXYw, ix_, iy_, iyaw_, alpha_, beta_)
        dMin = inf; idx = 1;
        for ii = 1:size(Q,1)
            x = Q(ii,ix_); y = Q(ii,iy_); phi = Q(ii,iyaw_);
            v = [tgtXYw(1)-x, tgtXYw(2)-y]; L=norm(v);
            if L < 1e-9
                d = abs(wrapToPi(tgtXYw(3) - phi));
                if d < dMin, dMin = d; idx = ii; end
                continue;
            end
            angTo = atan2(v(2), v(1));
            arcOK = abs(wrapToPi(angTo - phi)) <= beta_;
            yawOK = abs(wrapToPi(tgtXYw(3) - phi)) <= alpha_;
            hdgPenalty = (~arcOK)*0.8*L + (~yawOK)*0.3*L;
            d = L + hdgPenalty;
            if d < dMin, dMin = d; idx = ii; end
        end
    end

    function qB = steer(qA, qT, step, wrapMask, ix_, iy_, iyaw_)
        % base move only; arm locked to qA
        armIdx_ = setdiff(1:numel(qA), [ix_ iy_ iyaw_]);
        qB = qA;
        pA = qA([ix_ iy_]); pT = qT([ix_ iy_]);
        v = pT - pA; dist = norm(v);
        if dist >= 1e-6
            moveDist = min(step, dist);
            pB   = pA + (moveDist/max(dist,1e-6))*v;
            yawB = qA(iyaw_);
            qB([ix_ iy_]) = pB;
            qB(iyaw_)     = yawB;
        end
        if ~isempty(armIdx_), qB(armIdx_) = qA(armIdx_); end
        idxW = find(wrapMask); for c = idxW, qB(c)=atan2(sin(qB(c)),cos(qB(c))); end
    end

    function okSeg = segmentFree(rb, env, qa, qb, step, wrapMask, ix_, iy_, iyaw_) %#ok<INUSD>
        % Geometric segment check:
        %  - base as sphere of radius P.baseRadius
        %  - rebar as capsule along TCP Y-axis (P.rebarLen, P.rebarRadius)
        %  - arm as a few spherical pads of radius P.armRadius
        
        dq = qb - qa;
        angIdx = find(wrapMask);
        for c = angIdx
            dq(c) = atan2(sin(dq(c)), cos(dq(c)));
        end

        L = norm(dq);
        n = max(2, ceil(L / max(step, 1e-6)));
        okSeg = true;

        for k = 1:n
            s = k / n;
            qk = qa + s * dq;
            % wrap again
            for c = angIdx
                qk(c) = atan2(sin(qk(c)), cos(qk(c)));
            end

            if configInCollision_geom(rb, env, qk, ee, armBodyNames, P, ix_, iy_)
                okSeg = false;
                return;
            end
        end
    end

    function d = minClearance3D_withSkip(p, env, skipFloor)
        % Like global minClearance3D, but can ignore 'floor' by name.
        d = inf; 
        p = p(:).';
        for i = 1:numel(env)
            entry = env{i};
            if isstruct(entry) && isfield(entry,'CollisionObj')
                if skipFloor && isfield(entry,'Name') && strcmpi(entry.Name,'floor')
                    continue; % ignore floor
                end
                co = entry.CollisionObj;
            else
                co = entry;
            end

            T = co.Pose;
            if isa(co,'collisionBox')
                R = T(1:3,1:3); 
                t = T(1:3,4).';
                pL = (p - t)/R.';
                h = [co.X, co.Y, co.Z]/2;
                q = abs(pL) - h;
                di = norm(max(q,0));
                if all(q <= 0)
                    di = max(q); % negative inside
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
                di = norm(max(vec_d,0)) + min(max(vec_d(1),vec_d(2)), 0);
            else
                continue;
            end
            d = min(d, di);
        end
        if isinf(d), d = 100; end
    end

    function ok = checkConfig(rb, env, q)
        try
            if any(isnan(q)), ok=false; return; end
            % hybrid: quick test first, then strict if anything looks suspicious
            collFast = checkCollision(rb, q, env, "Exhaustive","off", "SkippedSelfCollisions","parent");
            if ~any(collFast,'all')
                ok = true; return;
            end
            collStrict = checkCollision(rb, q, env, "Exhaustive","on", "SkippedSelfCollisions","parent");
            ok = ~any(collStrict,'all');
        catch
            ok = false;
        end
    end

    function qN = jacobianStep(rb, q0, Ttgt, ee_, step, wrapMask)
        % IK step that locks Z and roll/pitch; yaw free (kept from q0).
        zConst = Ttgt(3,4);

        % Lock roll and pitch to zero for a level tool.
        T0 = getTransform(rb, q0, ee_);
        eulZYX0 = rotm2eul(T0(1:3,1:3),'ZYX');
        yawDes  = eulZYX0(1);
        rpLock  = [eulZYX0(3), eulZYX0(2)];    % [roll0, pitch0] instead of [0,0]
        Rdes    = eul2rotm([yawDes, rpLock(2), rpLock(1)], 'ZYX');
        % T0 = getTransform(rb, q0, ee_); eulZYX0 = rotm2eul(T0(1:3,1:3),'ZYX');
        % rpLock = [eulZYX0(3) eulZYX0(2)];  % [roll pitch] from q0

        T0 = getTransform(rb, q0, ee_);
        eulZYX0 = rotm2eul(T0(1:3,1:3),'ZYX');
        yawDes  = eulZYX0(1);
        Rdes    = eul2rotm([yawDes, rpLock(2), rpLock(1)], 'ZYX');

        posDes  = Ttgt(1:3,4).';
        posDes(3) = zConst;
        Tdes = [Rdes, posDes(:); 0 0 0 1];

        persistent ikLocal
        if isempty(ikLocal), ikLocal = inverseKinematics('RigidBodyTree', rb); end
        w = [1 1 200 200 200 1];  % strong Z + orientation

        qCand = double(ikLocal(ee_, Tdes, w, q0));

        dq = qCand - q0; L = norm(dq);
        if L > max(step,1e-6), qN = q0 + (step/L)*dq; else, qN = qCand; end
        for c = find(wrapMask), qN(c) = atan2(sin(qN(c)), cos(qN(c))); end
    end

    function coll = configInCollision_geom(rb, env, q, eeName, armBodies, Pgeom, ix_, iy_)
        % Apply the external base-box and floor checker.
        % Pgeom contains all the dimensions (baseWidth, zFloor, etc.)
        
        coll = paperCollisionSimple(q, rb, eeName, env, ...
                                    Pgeom.baseIncircle, ...
                                    Pgeom.rebarRadius, ...
                                    Pgeom.rebarLen, ...
                                    Pgeom);
    end

    function [Tout_, hit_, idxOther_, qOtherNew_] = connectToward(rb, env, ee_, Tin_, qGoal_, P_, wrapMask_, ix_, iy_, iyaw_)
        hit_ = false;
        qGoal_xyw = [qGoal_(ix_) qGoal_(iy_) qGoal_(iyaw_)];
        idxNear_  = nearNonholonomic(Tin_.q, qGoal_xyw, ix_, iy_, iyaw_, P_.alpha, P_.beta);
        qNear_    = Tin_.q(idxNear_,:);
        idxOther_ = idxNear_; qOtherNew_= qNear_; qSteer_ = qNear_;

        while true
            qNext_ = steer(qSteer_, qGoal_, P_.stepSize, wrapMask_, ix_, iy_, iyaw_);
            if distJoint(qNext_, qSteer_, wrapMask_) < 1e-6, break; end
            if ~segmentFree(rb, env, qSteer_, qNext_, P_.valStep, wrapMask_, ix_, iy_, iyaw_), break; end

            Tin_.q(end+1,:)     = qNext_;
            Tin_.parent(end+1)  = idxOther_;
            Tin_.eePos(end+1,:) = tcpPos(rb, qNext_, ee_);
            idxOther_  = size(Tin_.q,1);
            qOtherNew_ = qNext_;

            if segmentFree(rb, env, qNext_, qGoal_, P_.valStep, wrapMask_, ix_, iy_, iyaw_)
                hit_ = true; break;
            end

            if distJoint(qNext_, qSteer_, wrapMask_) < P_.stepSize*0.1, break; end
            qSteer_ = qNext_;
        end
        Tout_ = Tin_;
    end

    function path_ = stitchPath(treeA_, treeB_, idxA_, idxB_)
        A = unwind(treeA_, idxA_); B = unwind(treeB_, idxB_);
        path_ = [A; flipud(B)];
        if size(path_,1)>1 && distJoint(path_(size(A,1),:), path_(size(A,1)+1,:), wrapIdx) < 1e-6
            path_(size(A,1)+1,:) = [];
        end
    end

    function Pth = unwind(tree_, idx_)
        Pth = tree_.q(idx_,:); current = idx_;
        while current ~= 1 && tree_.parent(current) ~= 0
            parent = tree_.parent(current);
            Pth = [tree_.q(parent,:); Pth]; %#ok<AGROW>
            current = parent;
        end
    end

    function gap = nearestGap(QA, QB, wrapMask)
        if isempty(QA) || isempty(QB), gap = inf; return; end
        gap = inf; from = max(1, size(QA,1)-20);
        for ii = from:size(QA,1)
            d = distJoint(QA(ii,:), QB, wrapMask);
            m = min(d);
            if m < gap, gap = m; end
        end
    end
end


%% ========================================================================
%  F) NUOVI HELPER (per Blocco Giunti)
%  ========================================================================
function robotR = freezeJoints(robotIn, jointNamesToFreeze, qrefRow)
%FREEZEJOINTS  Turn listed joints into FIXED at reference config qrefRow.
    qrefRow = double(qrefRow); % Ensure double
    robotR      = copy(robotIn);
    fmtIn0      = robotIn.DataFormat;
    fmtR0       = robotR.DataFormat;
    robotIn.DataFormat = "row";
    robotR.DataFormat  = "row";
    dofIn = numel(homeConfiguration(robotIn));
    if size(qrefRow,2) ~= dofIn
        error('freezeJoints:qrefSize', ...
            'qrefRow has %d cols, but robotIn has %d DoF.', size(qrefRow,2), dofIn);
    end
    bodiesIn        = robotIn.Bodies;
    jointNamesAllIn = string(cellfun(@(b) b.Joint.Name,  bodiesIn, 'uni', false));
    bodyNamesAllIn  = string(cellfun(@(b) b.Name,        bodiesIn, 'uni', false));
    parentNamesIn   = string(cellfun(@(b) b.Parent.Name, bodiesIn, 'uni', false));
    function bIdxR = idxR_byBodyName(name)
        bIdxR = find(strcmp(name, string(cellfun(@(b) b.Name, robotR.Bodies, 'uni', false))), 1);
    end
    for jn = string(jointNamesToFreeze(:).')
        bIdx = find(jointNamesAllIn == jn, 1);
        if isempty(bIdx)
            warning('freezeJoints:notFound','Joint "%s" not found. Skipping.', jn);
            continue;
        end
        bodyName   = bodyNamesAllIn(bIdx);
        parentName = parentNamesIn(bIdx);
        T_parent = getTransform(robotIn, qrefRow, parentName);
        T_body   = getTransform(robotIn, qrefRow, bodyName);
        Tpb      = T_parent \ T_body;
        newJ = rigidBodyJoint("fix_"+jn, "fixed");
        setFixedTransform(newJ, Tpb);
        try
            replaceJoint(robotR, bodyName, newJ);
        catch
            bIdxR = idxR_byBodyName(bodyName);
            if isempty(bIdxR)
                warning('freezeJoints:notInCopy','Body "%s" not found in copy. Skipping.', bodyName);
                continue;
            end
            oldB = robotR.Bodies{bIdxR};
            newB = rigidBody(oldB.Name);
            newB.Joint = newJ;
            replaceBody(robotR, oldB.Name, newB);
        end
    end
    robotIn.DataFormat = fmtIn0;
    robotR.DataFormat  = fmtR0;
end
function names = configJointNames(robot)
    fmt0 = robot.DataFormat; robot.DataFormat = "struct";
    cfg  = homeConfiguration(robot);
    names = string({cfg.JointName});
    robot.DataFormat = fmt0;
end
function [maskFull, maskFrozen, namesFull] = makeMasks(fullRobot, smallRobot, freezeList)
    namesFull   = configJointNames(fullRobot);
    namesSmall  = configJointNames(smallRobot);
    maskFull    = ismember(namesFull, namesSmall);
    maskFrozen  = ismember(namesFull, string(freezeList));
end

function [centers2D, radii2D, zConst] = generateMaximalEECircles_Puck( ...
        pStartXYZ, pGoalXYZ, envObs, guideXY, zConst, varargin)

p = inputParser;
addParameter(p,'rMin',0.30);
addParameter(p,'rMax',1.20);
addParameter(p,'margin',0.01);       % extra safety gap (visual gap)
addParameter(p,'eeIncircle',0.00);   % <-- NEW: EE footprint radius (rebar+gripper)
addParameter(p,'maxPucks',150);
addParameter(p,'advanceFrac',0.6);
addParameter(p,'gaSteps',5);
addParameter(p,'gaStep',0.02);
parse(p,varargin{:});

RMIN = p.Results.rMin;  RMAX = p.Results.rMax;
MARG = p.Results.margin; EEF  = p.Results.eeIncircle;   % <-- use both
ADV  = p.Results.advanceFrac; NMAX = p.Results.maxPucks;
GA_N = p.Results.gaSteps;     GA_H = p.Results.gaStep;

% --- gradient-ascent on minClearance3D, but move only in XY at fixed zConst
refineXY = @(xy0) local_refine_xy(xy0, zConst, envObs, GA_N, GA_H);

C = {}; R = {};
% seed at (pStart on Z plane)
c0 = refineXY(pStartXYZ(1:2));
r0 = max(0, min(RMAX, minClearance3D([c0 zConst], envObs) - (EEF + MARG)));  % <-- CHANGED
if r0 < RMIN, warning('EE puck start too small (%.3f).', r0); centers2D=[]; radii2D=[]; return; end
C{1}=c0; R{1}=r0;

% Build a blended 3-D path and sample its XY coordinates.
numPts = max(50, size(guideXY,1));
P3 = generateCoordinatedPath_FromGuide(pStartXYZ, pGoalXYZ, guideXY, numPts);

kPath = 1;
while numel(C) < NMAX
    lastC = C{end}; lastR = R{end};
    if norm(pGoalXYZ(1:2) - lastC) <= lastR, break; end

    % advance along the 3D coordinated path by ~ADV*lastR (XY distance)
    need = max(RMIN*0.4, ADV*lastR);
    [found, kPath] = advance_along_path_xy(P3, kPath, lastC, need);
    if ~found, break; end
    cCand = P3(kPath,1:2);
    cRef  = refineXY(cCand);
    r     = max(0, min(RMAX, minClearance3D([cRef zConst], envObs) - (EEF + MARG)));  % <-- CHANGED
    if r >= RMIN && norm(cRef - lastC) > 0.25*RMIN
        C{end+1} = cRef; R{end+1} = r;
    end
end

% ensure goal is covered (final puck near goal)
if isempty(C) || norm(pGoalXYZ(1:2) - C{end}) > R{end}
    cG  = refineXY(pGoalXYZ(1:2));
    rG  = max(0, min(RMAX, minClearance3D([cG zConst], envObs) - (EEF + MARG)));      % <-- CHANGED
    if rG >= max(0.75*RMIN, 0.05)
        C{end+1} = cG; R{end+1} = rG;
    end
end

centers2D = cell2mat(C');
radii2D   = cell2mat(R');

    function cR = local_refine_xy(xy, z, env, n, h)
        cR = xy; eps = 1e-4;
        for ii=1:n
            d0 = minClearance3D([cR z], env); if d0<=0, break; end
            dx = (minClearance3D([cR(1)+eps cR(2) z], env) - d0)/eps;
            dy = (minClearance3D([cR(1) cR(2)+eps z], env) - d0)/eps;
            g  = [dx dy]; ng = norm(g);
            if ng < 1e-6, break; end
            cR = cR + h*(g/ng);
        end
    end
    function [ok, idx] = advance_along_path_xy(P, idx0, c, distNeed)
        ok=false; idx=idx0; if size(P,1)<2, return; end
        acc=0;
        for kk=max(2,idx0):size(P,1)
            seg = norm(P(kk,1:2)-P(kk-1,1:2));
            acc = acc + seg;
            if acc >= distNeed || norm(P(kk,1:2)-c) >= 0.9*distNeed
                ok=true; idx=kk; return;
            end
        end
    end
end

function jnames = cfgJointNames(rb)
    fmt = rb.DataFormat; rb.DataFormat = "struct";
    c = homeConfiguration(rb);
    rb.DataFormat = fmt;
    jnames = string({c.JointName});
end

function [idxSmallInFull, idxFrozenInFull, namesFull, namesSmall] = jointIndexMap(fullRobot, smallRobot, freezeList)
    namesFull  = cfgJointNames(fullRobot);
    namesSmall = cfgJointNames(smallRobot);           % order of columns in 5-DoF path
    [tf, idxSmallInFull] = ismember(namesSmall, namesFull);
    assert(all(tf), 'Some free joints in 5-DoF not found in full robot.');
    [tf2, idxFrozenInFull] = ismember(string(freezeList(:).'), namesFull);
    assert(all(tf2), 'Some frozen joints not found in full robot.');
end

function Qfixed = repairCollisionsOnExpandedPath(robot, env, Q, valStep)
    % Uses robot's base indices for proper angle wrapping.
    Sx = resolveBaseIndices(robot); ix = Sx.baseIdx(1); iy = Sx.baseIdx(2);

    wrap = true(1, size(Q,2));
    wrap([ix iy]) = false;  % base x,y are linear, not angles

    Qfixed = Q;
    changed = true; it = 0; maxIt = 5;
    while changed && it < maxIt
        changed = false; it = it + 1;
        i = 1;
        while i < size(Qfixed,1)
            qa = Qfixed(i,:); qb = Qfixed(i+1,:);
            if ~segmentFreeWrap(robot, env, qa, qb, valStep, wrap)
                qm = midWrap(qa, qb, wrap);
                Qfixed = [Qfixed(1:i,:); qm; Qfixed(i+1:end,:)];
                changed = true;
            else
                i = i + 1;
            end
        end
    end
end

function okSeg = segmentFreeWrap(rb, env, qa, qb, step, wrap)
    dq = qb - qa;
    for c = find(wrap), dq(c) = atan2(sin(dq(c)), cos(dq(c))); end
    L = norm(dq);
    n = max(2, ceil(L/max(step,1e-6)));
    for k = 1:n
        qk = qa + (k/n)*dq;
        for c = find(wrap), qk(c) = atan2(sin(qk(c)), cos(qk(c))); end
        if any(checkCollision(rb, qk, env, "Exhaustive","on","SkippedSelfCollisions","parent"), 'all')
            okSeg = false; return;
        end
    end
    okSeg = true;
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

%% ========================================================================
%  REPAIR & COLLISION HELPERS (Robust Version)
%  ========================================================================

function Qrepaired = repairCollidingSegments(robot, env, Q, fineStep, maxDepth, eeName, P)
%REPAIRCOLLIDINGSEGMENTS
%   Recursively bisects segments that violate either standard collision OR
%   paper constraints (Floor/BaseBox).
    
    % Setup Wrap Mask (Base X/Y linear, others angular)
    Sx = resolveBaseIndices_global(robot); 
    ix = Sx.baseIdx(1); iy = Sx.baseIdx(2);
    wrap = true(1,size(Q,2)); 
    wrap([ix iy]) = false;
    
    i = 1; 
    Qrepaired = Q;
    
    while i < size(Qrepaired,1)
        qa = Qrepaired(i,:); 
        qb = Qrepaired(i+1,:);
        
        % Check if segment is valid (Standard + Paper Constraints)
        if ~segmentFreeRepair(robot, env, qa, qb, fineStep, wrap, eeName, P)
            
            % It failed -> Attempt Bisection
            stack = struct('a',qa, 'b',qb, 'd',0); 
            newSeg = []; 
            okSeg = true; 
            S = stack; clear stack;
            
            while ~isempty(S)
                cur = S(end); S(end) = [];
                
                % Check the sub-segment
                if segmentFreeRepair(robot, env, cur.a, cur.b, fineStep, wrap, eeName, P)
                    newSeg = [newSeg; cur.b]; %#ok<AGROW>
                else
                    if cur.d >= maxDepth
                        % Max depth reached, cannot repair this sub-segment
                        okSeg = false; 
                        break; 
                    end
                    % Bisect
                    m = midWrap(cur.a, cur.b, wrap);
                    S(end+1) = struct('a',m, 'b',cur.b, 'd',cur.d+1); %#ok<AGROW>
                    S(end+1) = struct('a',cur.a, 'b',m, 'd',cur.d+1); %#ok<AGROW>
                end
            end
            
            if okSeg
                % Repair successful: Insert new segments
                fprintf('    [Repair] Fixed segment %d-%d via subdivision.\n', i, i+1);
                Qrepaired = [Qrepaired(1:i,:); newSeg; Qrepaired(i+2:end,:)];
                % Don't increment i, check the newly inserted connection next
            else
                % Repair failed: Leave it and warn (validator will catch it)
                fprintf('    [Repair] WARNING: Could not repair segment %d-%d (max depth).\n', i, i+1);
                i = i + 1; 
            end
        else
            % Segment was fine
            i = i + 1;
        end
    end
end

function ok = segmentFreeRepair(rb, env, qa, qb, step, wrap, eeName, P)
% Checks a segment against BOTH standard collisions AND Paper Constraints
    dq = qb - qa;
    for c = find(wrap), dq(c) = atan2(sin(dq(c)), cos(dq(c))); end
    
    L = norm(dq); 
    n = max(2, ceil(L/max(step,1e-6)));
    ok = true;
    
    for k = 1:n
        q = qa + (k/n)*dq;
        for c = find(wrap), q(c) = atan2(sin(q(c)), cos(q(c))); end
        
        % 1. Standard Robot Collision (Exhaustive)
        if any(checkCollision(rb, q, env, "Exhaustive","on","SkippedSelfCollisions","parent"),'all')
            ok = false; return; 
        end
        
        % 2. PAPER MODEL CHECK (Floor, Base Box, Rebar)
        if paperCollisionSimple(q, rb, eeName, env, ...
                                P.baseIncircle, P.rebarRadius, P.rebarLen, P)
             ok = false; return;
        end
    end
end

function qm = midWrap(qa, qb, wrap)
    dq = qb - qa;
    for c = find(wrap), dq(c) = atan2(sin(dq(c)), cos(dq(c))); end
    qm = qa + 0.5*dq;
    for c = find(wrap), qm(c) = atan2(sin(qm(c)), cos(qm(c))); end
end

function coll = paperCollisionSimple(q, rb, eeName, envCol, ...
                                     baseIncircle, rebarRadius, rebarLen, P)
%CHECKS: 1. Base(2D) 2. Rebar vs Env 3. Rebar vs Floor 4. Rebar vs Base Box

    coll = false;
    
    % --- 1. BASE vs ENVIRONMENT (2D) ---
    S = resolveBaseIndices_global(rb);
    baseXY = q(S.baseIdx(1:2));
    % Assuming minClearance2D correctly ignores the floor via threshold
    if minClearance2D(baseXY, envCol) < baseIncircle
        coll = true; return;
    end

    % --- PREP REBAR GEOMETRY ---
    Ttcp = getTransform(rb, q, eeName);
    o = Ttcp(1:3,4); Rtcp = Ttcp(1:3,1:3);
    dir = Rtcp(:,2); dir = dir/max(norm(dir),1e-9); % Y-axis
    p0_world = o - 0.5*rebarLen*dir;
    p1_world = o + 0.5*rebarLen*dir;
    sagDepth = 0;
    if nargin >= 8 && isfield(P,'useRebarSagEnvelope') && P.useRebarSagEnvelope
        if isfield(P,'rebarEnvelopeHeight')
            sagDepth = max(0, P.rebarEnvelopeHeight);
        elseif isfield(P,'rebarSag')
            sagDepth = max(0, P.rebarSag);
        end
    end

    % --- 2. REBAR vs FLOOR (Z-Limit) ---
    if nargin>=8 && isfield(P,'zFloor')
        lowest = min(p0_world(3), p1_world(3)) - rebarRadius - sagDepth;
        % Tiny tolerance for repair stability
        if lowest < (P.zFloor - 0.002) 
            coll = true; return;
        end
    end

    % --- 3. REBAR vs ENVIRONMENT (3D) ---
    % Sample the rebar envelope. The length is already expanded in P.rebarLen;
    % sagDepth sweeps the physical radius downward in world Z.
    N = max(ceil(rebarLen/0.1), 3);
    Nz = max(1, ceil(sagDepth/0.05));
    for k=0:N
        ptCenter = p0_world + (k/N)*(p1_world - p0_world);
        for j=0:Nz
            pt = ptCenter - [0; 0; sagDepth * (j/Nz)];
            if minClearance3D(pt, envCol) < rebarRadius
                coll = true; return;
            end
        end
    end

    % --- 4. REBAR vs MOBILE BASE BOX (Self-Collision) ---
    if nargin>=8 && isfield(P,'baseWidth')
        bx = q(S.baseIdx(1)); by = q(S.baseIdx(2)); bz = q(S.baseIdx(3));
        c=cos(bz); s=sin(bz); R_bw=[c s 0; -s c 0; 0 0 1]; p_base=[bx; by; 0];
        
        mid_world = (p0_world+p1_world)/2;
        sagVec = [0; 0; sagDepth];
        pts = [p0_world, p1_world, mid_world, ...
               p0_world-sagVec, p1_world-sagVec, mid_world-sagVec];
        pad = rebarRadius + 0.03; % Safety padding
        limX = P.baseLength/2 + pad;
        limY = P.baseWidth/2 + pad;
        limZ = P.baseHeight + pad;
        
        for k=1:size(pts,2)
            pt_local = R_bw * (pts(:,k) - p_base);
            if (pt_local(3)<limZ && pt_local(3)>-0.1) && ...
               (abs(pt_local(1))<limX) && (abs(pt_local(2))<limY)
                coll = true; return;
            end
        end
    end
end

function S = resolveBaseIndices_global(rb)
    bxBody = getBody(rb, "base_x");   jx = string(bxBody.Joint.Name);
    byBody = getBody(rb, "base_y");   jy = string(byBody.Joint.Name);
    chBody = getBody(rb, "chassis");  jz = string(chBody.Joint.Name);
    fmt0 = rb.DataFormat; rb.DataFormat = 'struct';
    cfg = homeConfiguration(rb); rb.DataFormat = fmt0;
    jnames = string({cfg.JointName});
    ix = find(jnames == jx, 1);
    iy = find(jnames == jy, 1);
    iz = find(jnames == jz, 1);
    S.baseIdx = [ix iy iz];
end

%% ========================================================================
%  G) COMPREHENSIVE METRICS COMPUTATION
%  ========================================================================
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

function Qp = projectTrajectoryToPlaneRP(robot, Q, ee, zConst, rpLock)
% Projects each waypoint to the manifold: Z=zConst and [roll pitch]=rpLock (yaw free).
    ikp = inverseKinematics('RigidBodyTree', robot);
    w   = [1 1 100 10 10 10];  % strong Z + orientation weights
    Qp  = Q;
    for i=1:size(Q,1)
        q0 = Qp(i,:);
        T0 = getTransform(robot, q0, ee);
        yaw = atan2(T0(2,1), T0(1,1));                        % keep yaw
        R   = eul2rotm([yaw, rpLock(2), rpLock(1)], 'ZYX');   % [yaw pitch roll]
        T   = T0; T(1:3,1:3)=R; T(3,4)=zConst;
        q1  = double(ikp(ee, T, w, q0));
        Qp(i,:) = q1;
    end
end


function envOut = inflateEnvForPlanning(envIn, m, varargin)
% Inflate obstacles by margin m, but skip any entries whose Name matches
% excludeNames (e.g., 'floor'). Works with envS ({struct}) or plain {collisionObj}.
%
% Usage:
%   envPlanS   = inflateEnvForPlanning(envS, 0.02, 'excludeNames',"floor");
%   envColPlan = cellfun(@(s) s.CollisionObj, envPlanS, 'uni', false);

    p = inputParser;
    addParameter(p,'excludeNames',"floor");  % string/char/cellstr/string array ok
    parse(p,varargin{:});
    excl = lower(string(p.Results.excludeNames(:)));

    if m <= 0
        envOut = envIn; 
        return;
    end

    envOut = cell(size(envIn));
    for i = 1:numel(envIn)
        % Unwrap (supports either struct with Name/CollisionObj or raw object)
        entry   = envIn{i};
        hadName = isstruct(entry) && isfield(entry,'Name') && isfield(entry,'CollisionObj');
        if hadName
            name = string(entry.Name);
            co   = entry.CollisionObj;
        else
            name = "";
            co   = entry;
        end

        % If this object is excluded by name (e.g., 'floor'), keep as-is
        if hadName && any(lower(name) == excl)
            coNew = co;

        else
            % Inflate supported primitives
            if isa(co,'collisionBox')
                coNew = collisionBox(co.X + 2*m, co.Y + 2*m, co.Z + 2*m);
                coNew.Pose = co.Pose;
            elseif isa(co,'collisionCylinder')
                coNew = collisionCylinder(co.Radius + m, co.Height + 2*m);
                coNew.Pose = co.Pose;
            elseif isa(co,'collisionSphere')
                coNew = collisionSphere(co.Radius + m);
                coNew.Pose = co.Pose;
            else
                % Unknown type: leave unchanged
                coNew = co;
            end
        end

        % Rewrap to preserve original structure
        if hadName
            envOut{i} = struct('Name', char(name), 'CollisionObj', coNew);
        else
            envOut{i} = coNew;
        end
    end
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

function q_safe = findSafeIntermediate(robot, qa, qb, envCol, wrapMask, stepSize)
% Find a safe intermediate configuration using binary search
    maxDepth = 8; % Maximum recursion depth
    q_safe = [];
    
    function result = binarySearchSafe(safe, risky, depth)
        if depth > maxDepth
            result = safe;
            return;
        end
        
        q_mid = 0.5 * (safe + risky);
        % Apply angle wrapping
        q_mid(wrapMask) = atan2(sin(q_mid(wrapMask)), cos(q_mid(wrapMask)));
        
        coll = checkCollision(robot, q_mid, envCol, ...
                             "Exhaustive", "on", ...
                             "SkippedSelfCollisions", "parent");
        
        if any(coll, 'all')
            % Midpoint is also in collision, search left half
            result = binarySearchSafe(safe, q_mid, depth + 1);
        else
            % Midpoint is safe, search right half for better point
            result = binarySearchSafe(q_mid, risky, depth + 1);
        end
    end
    
    % Check if endpoints are safe
    coll_a = checkCollision(robot, qa, envCol, "Exhaustive", "on", "SkippedSelfCollisions", "parent");
    if any(coll_a, 'all')
        error('Start configuration for interpolation is in collision');
    end
    
    q_safe = binarySearchSafe(qa, qb, 1);
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
        % Pass the planner geometry parameters.
        if paperCollisionSimple(q, robot, eeName, envCol, ...
                                P.baseIncircle, P.rebarRadius, P.rebarLen, P)
             if nargout == 0
                error('Paper collision model: collision at step %d.', k);
             else
                ok = false; return;
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
            sagDepth = 0;
            if isfield(P,'useRebarSagEnvelope') && P.useRebarSagEnvelope
                if isfield(P,'rebarEnvelopeHeight')
                    sagDepth = max(0, P.rebarEnvelopeHeight);
                elseif isfield(P,'rebarSag')
                    sagDepth = max(0, P.rebarSag);
                end
            end
            zMinBar = min(p0(3), p1(3)) - P.rebarRadius - sagDepth;

            if zMinBar < P.zFloor
                if nargout == 0
                    error('Paper model: rebar dips below zFloor at step %d (zMin=%.3f < %.3f).', ...
                          k, zMinBar, P.zFloor);
                else
                    ok = false;
                    return;
                end
            end
        end

        % --- 3) Arm key-link spheres (cheap arm collision) --------------
        if isfield(P,'armRadius') && P.armRadius > 0 && ...
           isfield(P,'armKeyNames') && ~isempty(P.armKeyNames)
            if armKeyCollision(q, robot, envCol, P.armKeyNames, P.armRadius)
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

function h = drawRebarSagEnvelope(robot, q, eeName, P, ax)
%DRAWREBARSAGENVELOPE Draw the PHS sag-aware rebar envelope used for checking.
%   The long axis follows the TCP Y-axis, length uses P.rebarLen, and the
%   envelope extends downward in world Z by the predicted sag.

    h = gobjects(0);
    if nargin < 5 || isempty(ax) || ~isvalid(ax)
        ax = gca;
    end
    if nargin < 4 || isempty(P) || ~isfield(P,'rebarLen') || ~isfield(P,'rebarRadius')
        return;
    end

    L = P.rebarLen;
    if isfield(P,'rebarEnvelopeLength') && P.rebarEnvelopeLength > 0
        L = P.rebarEnvelopeLength;
    end
    W = 2 * P.rebarRadius;
    if isfield(P,'rebarEnvelopeWidth') && P.rebarEnvelopeWidth > 0
        W = max(W, P.rebarEnvelopeWidth);
    end

    sagDepth = 0;
    if isfield(P,'rebarEnvelopeHeight')
        sagDepth = max(0, P.rebarEnvelopeHeight);
    elseif isfield(P,'rebarSag')
        sagDepth = max(0, P.rebarSag);
    end
    H = max(sagDepth, 0.02); % keep the envelope visible for very small sag.

    Ttcp = getTransform(robot, q, eeName);
    centerTop = Ttcp(1:3,4);
    xAxis = Ttcp(1:3,2);
    xAxis = xAxis / max(norm(xAxis), 1e-9);
    zWorld = [0; 0; 1];
    yAxis = cross(zWorld, xAxis);
    if norm(yAxis) < 1e-9
        yAxis = cross([1; 0; 0], xAxis);
    end
    yAxis = yAxis / max(norm(yAxis), 1e-9);
    zAxis = cross(xAxis, yAxis);
    if dot(zAxis, zWorld) < 0
        zAxis = -zAxis;
    end

    center = centerTop - 0.5 * H * zWorld;
    signs = [-1 -1 -1;
              1 -1 -1;
              1  1 -1;
             -1  1 -1;
             -1 -1  1;
              1 -1  1;
              1  1  1;
             -1  1  1];
    V = zeros(8,3);
    for i = 1:8
        V(i,:) = (center ...
            + signs(i,1) * 0.5 * L * xAxis ...
            + signs(i,2) * 0.5 * W * yAxis ...
            + signs(i,3) * 0.5 * H * zWorld).';
    end
    F = [1 2 3 4;
         5 6 7 8;
         1 2 6 5;
         2 3 7 6;
         3 4 8 7;
         4 1 5 8];

    h = patch('Parent', ax, ...
        'Vertices', V, ...
        'Faces', F, ...
        'FaceColor', [1.0 0.45 0.05], ...
        'FaceAlpha', 0.18, ...
        'EdgeColor', [0.85 0.25 0.00], ...
        'EdgeAlpha', 0.65, ...
        'LineWidth', 1.0, ...
        'DisplayName', 'Sag-aware rebar envelope');
end


function coll = armKeyCollision(q, rb, env, keyNames, rArm)
%ARMKEYCOLLISION  Cheap approximate collision for the arm.
%   Treat each key link as a sphere of radius rArm and use minClearance3D.

    coll = false;
    if isempty(keyNames) || rArm <= 0
        return;
    end

    for i = 1:numel(keyNames)
        try
            T = getTransform(rb, q, keyNames(i));   % frame of the link
        catch
            % If the body name doesn't exist, just skip it.
            continue;
        end
        p = T(1:3,4).';                 % link origin position
        d = minClearance3D(p, env);

        if d < rArm
            coll = true;
            return;
        end
    end
end
