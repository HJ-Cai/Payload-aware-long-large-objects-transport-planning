%% UR20 Mobile Pick & Place with Standard Bi-RRT (Robust Saving)
%% ========================================================================
%  SINGLE-RUN CONFIGURATION
%  ========================================================================
clear;
clc;
close all;
rng(1, 'twister');                 % fixed seed for a reproducible single run
runIdx = 1;
% The envelope is always displayed. This switch controls only whether
% Plan 4 collision validation considers it. Keep false for published runs.
useRebarEnvelope = true;
    
    % Initialize run variables for both successful and failed runs.
    runSuccess = false;
    failReason = "Unknown";
    allStates  = [];
    traj1=[]; traj2=[]; traj3=[]; traj4a=[]; traj5=[];
    t_plan4p = 0; t_plan4t = 0;
    info = struct();
    rebarEnvelope = struct('displayAlways',true, ...
        'consideredInPlanning',useRebarEnvelope,'lengthFactor',1.10, ...
        'lengthM',NaN,'heightM',NaN,'widthM',NaN);
    
    % Prepare Save Directory Early
    outDir  = "seeds";
    dateDir = fullfile(outDir, string(datetime('now','Format','yyyyMMdd')));
    if ~exist(dateDir,'dir'), mkdir(dateDir); end
    
    fprintf('\n==================================================\n');
    fprintf('   Bi-RRT: single simulation run\n');
    fprintf('==================================================\n');

    try
        % Single-pass loop permits early exit to the save block.
        while true 
            
            % 1. Setup Environment
            clearvars -except runIdx runSuccess failReason allStates ...
                              traj1 traj2 traj3 traj4a traj5 t_plan4p t_plan4t info dateDir ...
                              useRebarEnvelope rebarEnvelope;
            close all; 
            
            [mobileUR20, startConfig, envS, pickPose, placePose, ~, rebarMeta] = ...
                loadRebarTransportScenario;
                
            % Add Floor (Fairness)
            floorObj = collisionBox(20, 20, 0.1); 
            floorObj.Pose = trvec2tform([0, 0, -0.07]); 
            envS = [envS, {struct('Name','Floor','CollisionObj',floorObj)}];
            envCol = cellfun(@(s) s.CollisionObj, envS, 'uni', false);
            
            % Fairness Rules (P)
            P = struct('baseIncircle', 0.35, 'rebarLen', rebarMeta.LengthM, ...
                       'rebarRadius', (rebarMeta.DiameterM*1000/1000)/2, 'zFloor', -0.02, ...
                       'baseWidth', 0.40, 'baseLength', 0.7, 'baseHeight', 0.5, ...
                       'armRadius', 0.12, 'armKeyNames', ["C-2003903","C-2003904","C-2003905","C-2003906","C-2003907","C-2007588","C-2003908"]);

            % Configure Robot Limits
            bx = getBody(mobileUR20, "base_x");  bx.Joint.PositionLimits = [-6 6];
            by = getBody(mobileUR20, "base_y");  by.Joint.PositionLimits = [-6 6];
            bz = getBody(mobileUR20, "chassis"); bz.Joint.PositionLimits = [-pi pi];
            
            ik = inverseKinematics('RigidBodyTree', mobileUR20);
            weights = [0.1 0.1 0.1 1 1 1]; 
            endEffector = 'tcp';
            makePlanner = @(robot, env) configurePlanner(manipulatorRRT(robot, env), 600); % 300s Timeout

            % 2. Check Pick Poses
            UR20Meta = evalin('base','UR20Meta');
            approachPose = pickPose;
            graspPose    = UR20Meta.T_tcp_touch_world;
            approachConfig = ik(endEffector, approachPose, weights, startConfig);
            graspConfig    = ik(endEffector, graspPose,    weights, approachConfig);
            
            if ~isConfigFree(mobileUR20, startConfig, envCol)
                 failReason = "StartConfigCollision"; fprintf(2, '   -> Failed: %s\n', failReason); break; 
            end
            if ~isConfigFree(mobileUR20, approachConfig, envCol)
                 failReason = "ApproachIKCollision"; fprintf(2, '   -> Failed: %s\n', failReason); break; 
            end

            % 3. Pick Sequence (Simplistic)
            planner1 = makePlanner(mobileUR20, envCol); 
            path1 = planOrRetry(planner1, startConfig, approachConfig, 5);
            if isempty(path1)
                failReason = "Plan1_ApproachFail"; fprintf(2, '   -> Failed: %s\n', failReason); break; 
            end
            traj1 = hs_interpPath(path1, 0.02);
            allStates = [allStates; traj1];
            
            traj2 = hs_interpPath([approachConfig; graspConfig], 0.01);
            allStates = [allStates; traj2];
            
            % Attach Rebar
            rebarIdx = find(cellfun(@(s) strcmpi(s.Name,'rebar'), envS), 1);
            rebarObj = envS{rebarIdx}.CollisionObj;
            grippedConfig = graspConfig;
            allStates = [allStates; grippedConfig];
            
            rebarBody = rigidBody("rebar");
            eeT = getTransform(mobileUR20, grippedConfig, endEffector);
            setFixedTransform(rebarBody.Joint, eeT \ rebarObj.Pose);
            rbCopy = rebarObj.copy; rbCopy.Pose = eye(4);
            addCollision(rebarBody, rbCopy);
            addBody(mobileUR20, rebarBody, endEffector); 
            envS(rebarIdx) = [];
            envCol = cellfun(@(s) s.CollisionObj, envS, 'uni', false);
            
            % Lift
            diameter_mm = rebarMeta.DiameterM * 1000; length_m = rebarMeta.LengthM; gripPoint = rebarMeta.GripPoint;
            deflection  = computeRebarDeflection(diameter_mm, length_m, gripPoint);
            rebarEnvelope = struct('displayAlways', true, ...
                'consideredInPlanning', useRebarEnvelope, ...
                'lengthFactor', 1.10, 'lengthM', 1.10*length_m, ...
                'heightM', max(deflection,0), 'widthM', rebarMeta.DiameterM);
            P.rebarPhysicalLen = length_m;
            P.rebarEnvelopeLength = rebarEnvelope.lengthM;
            P.rebarEnvelopeLengthFactor = rebarEnvelope.lengthFactor;
            P.rebarEnvelopeHeight = rebarEnvelope.heightM;
            P.rebarEnvelopeWidth = rebarEnvelope.widthM;
            P.useRebarSagEnvelope = useRebarEnvelope;
            if useRebarEnvelope, P.rebarLen = rebarEnvelope.lengthM; end
            fprintf('   Rebar envelope checking: %s\n', onOff(useRebarEnvelope));
            nSteps = max(10, ceil(deflection / 0.01));
            traj3 = zeros(nSteps+1, numel(grippedConfig));
            T_start = getTransform(mobileUR20, grippedConfig, endEffector);
            q_current = grippedConfig; traj3(1,:) = q_current;
            for k = 1:nSteps
                T_next = T_start; T_next(3,4) = T_next(3,4) + (deflection * k / nSteps);
                q_next = ik(endEffector, T_next, weights, q_current);
                if any(isnan(q_next)), q_next = q_current; end
                traj3(k+1,:) = q_next; q_current = q_next;
            end
            retreatConfig = q_current;
            allStates = [allStates; traj3];
            
            % 4. Define Place Poses
            targetPose = UR20Meta.T_tcp_place_touch_world;
            T_target = targetPose;
            R_target = T_target(1:3, 1:3); P_target = T_target(1:3, 4); Z_vec = R_target(:, 3);
            P_offset = P_target - 0.5 * Z_vec; 
            if P_offset(3) < 0.4, P_offset(3) = 0.4; end 
            offsetPose = [R_target, P_offset; 0, 0, 0, 1];
            
            offsetConfig = ik(endEffector, offsetPose, weights, retreatConfig);
            if ~isConfigFree(mobileUR20, offsetConfig, envCol)
                 failReason = "PlaceOffsetCollision"; fprintf(2, '   -> Failed: %s\n', failReason); break; 
            end
            
            % --- Create Inflated Environment (Plan 4 Only) ---
            % Create a deep copy of the environment and inflate its dimensions.
            envColInflated = cell(size(envS));
            inflateMargin  = 0.05; % 10cm inflation
            
            for iObj = 1:numel(envS)
                % 1. Copy the object
                origObj = envS{iObj}.CollisionObj;
                % Collision primitives support independent copies.
                newObj = copy(origObj); 
                
                % 2. Check exclusion (Do not inflate Floor)
                if ~strcmpi(envS{iObj}.Name, 'Floor')
                    % 3. Inflate based on geometry type
                    if isa(newObj, 'collisionBox')
                        newObj.X = newObj.X + 2*inflateMargin;
                        newObj.Y = newObj.Y + 2*inflateMargin;
                        newObj.Z = newObj.Z + 2*inflateMargin;
                    elseif isa(newObj, 'collisionCylinder')
                        newObj.Radius = newObj.Radius + inflateMargin;
                        newObj.Height = newObj.Height + 2*inflateMargin;
                    elseif isa(newObj, 'collisionSphere')
                        newObj.Radius = newObj.Radius + inflateMargin;
                    end
                end
                envColInflated{iObj} = newObj;
            end
            % 5. PLAN 4: STANDARD BI-RRT
            fprintf('   Planning Plan 4 (Standard RRT)...\n');
            planner4 = makePlanner(mobileUR20, envColInflated);
            
            t_plan4p = tic;
            [path4a, info] = planOrRetry(planner4, retreatConfig, offsetConfig, 1);
            t_plan4p = toc(t_plan4p);
            t_plan4t = tic;
            
            if isempty(path4a)
                failReason = "Plan4_Timeout"; 
                fprintf(2, '   -> Failed: %s (Standard RRT took too long)\n', failReason); 
                break; % Jump to Save
            end
            
            traj4a = hs_interpPath(path4a, 0.02);
            
            % --- VALIDATION ---
            okRules = assertPaperPathCollisionFree(mobileUR20, traj4a, envCol, endEffector, P);
            % --- CHECK 3: GROUND TRUTH (Meshes) ---
            isPhysicalCol = false;
            for k = 1:size(traj4a, 1)
                % Check one waypoint at a time
                % We use "Exhaustive","off" to stop immediately if a collision is found (faster)
                isCol = checkCollision(mobileUR20, traj4a(k,:), envCol, ...
                                       "Exhaustive","off","SkippedSelfCollisions","parent");
                if any(isCol, 'all')
                    isPhysicalCol = true;
                    break; % Stop checking, we found a crash
                end
            end
            
            if ~okRules
                failReason = "Validation_TaskRule"; 
                fprintf(2, '   -> Failed: %s (Base/Floor violation)\n', failReason); 
                break; 
            elseif isPhysicalCol
                failReason = "Validation_PhysicalCol"; 
                fprintf(2, '   -> Failed: %s (Mesh collision)\n', failReason); 
                break; 
            end
            
            fprintf('   -> Plan 4 Success!\n');
            t_plan4t = toc(t_plan4t) + t_plan4p;
            allStates = [allStates; traj4a];
            
            % 6. Plan 5 (Guarded)
            [traj5, ~] = guardedApproachToolAxis(mobileUR20, offsetConfig, targetPose, ...
                envCol, ik, endEffector, weights, +0.01);
            allStates = [allStates; traj5];
            
            runSuccess = true;
            failReason = "None";
            break; % Finished successfully, break the while true
        end % End While
        
        %% 7. SAVE RESULT (Run regardless of success/fail)
        stamp = string(datetime('now','Format','yyyyMMdd_HHmmss')) + sprintf("_run%02d", runIdx);
        
        if runSuccess
            statusStr = "PASS";
            fName = "stdBiRRT_CS2_4m_" + stamp + "_PASS.mat";
        else
            statusStr = "FAIL";
            fName = "stdBiRRT_" + stamp + "_FAIL_" + failReason + ".mat";
        end
        
        saveFile = fullfile(dateDir, fName);
        
        % Calculate Metrics (even partial)
        metrics = computeComprehensiveMetrics(mobileUR20, allStates, traj1, traj2, traj3, traj4a, traj5, endEffector);
        
        timingStruct = struct('plan4_planning_s', t_plan4p, 'plan4_total_s', t_plan4t, 'total_s', t_plan4t);
        
        % Dummy structs to match format
        focusSave = struct('baseDisks', [], 'eeSpheres', [], 'eeCircles', [], 'eeType', "none");
        corridor = struct('guideXY', [], 'bestHeightXY', [], 'gridRes', NaN);
        
        bundle = struct( ...
          'meta', struct('approach', 'Standard Bi-RRT', 'runID', runIdx, 'desc', 'Baseline'), ...
          'status', struct('success', runSuccess, 'reason', failReason), ...
          'robot', mobileUR20, 'endEffector', endEffector, 'env', envS, ...
          'rebarMeta', rebarMeta, 'rebarEnvelope', rebarEnvelope, ...
          'planner', struct('P', P, 'info', info), ...
          'timing', timingStruct, ...
          'paths', struct('traj4a_full_9dof', traj4a, 'full_allStates', allStates), ...
          'metrics', metrics, 'success', runSuccess ...
        );
        
        save(saveFile, 'bundle', '-v7.3');
        fprintf('[SAVE] %s -> %s\n', statusStr, fName);

        %% ========================================================================
        %  REPLAY FULL TRAJECTORY
        %  ========================================================================
        fprintf('Replaying full path...\n');
        figure("Name","Bi-RRT Full Trajectory Replay","Units","normalized","OuterPosition",[0,0,1,1]);
        axReplay = gca;
        hold(axReplay, 'on'); grid(axReplay, 'on');
        axis(axReplay, [-6 6 -6 6 -0.1 2.5]);
        view(axReplay, 120, 25);
        for i = 1:numel(envS), show(envS{i}.CollisionObj, 'Parent', axReplay); end
        camlight(axReplay, 'headlight'); lighting(axReplay, 'gouraud');

        if ~isempty(allStates)
            show(mobileUR20, allStates(1,:), "Visuals","off","Collisions","on", "Parent", axReplay);
            envHandle = drawRebarSagEnvelope(mobileUR20, allStates(1,:), endEffector, P, axReplay);
            title(axReplay, sprintf("Bi-RRT Replay (State %d/%d)", 1, size(allStates,1)));
            drawnow;
            for k = 2:size(allStates, 1)
                show(mobileUR20, allStates(k,:), "Parent", axReplay, "PreservePlot", false, ...
                     "Visuals","off","Collisions","on", "FastUpdate", true);
                if all(isgraphics(envHandle)), delete(envHandle); end
                envHandle = drawRebarSagEnvelope(mobileUR20, allStates(k,:), endEffector, P, axReplay);
                title(axReplay, sprintf("Bi-RRT Replay (State %d/%d)", k, size(allStates,1)));
                drawnow;
            end
        else
            title(axReplay, "Replay Failed: No valid trajectory generated.");
            if exist('startConfig','var')
                show(mobileUR20, startConfig, "Visuals","off","Collisions","on", "Parent", axReplay);
            end
            drawnow;
        end
        hold(axReplay, 'off');
        
    catch ME
        fprintf(2, '[CRITICAL ERROR] Run %d Exception: %s\n', runIdx, ME.message);
        for k = 1:length(ME.stack)
             fprintf(2, '   Line %d: %s\n', ME.stack(k).line, ME.stack(k).name);
        end
    end
fprintf('\nSINGLE RUN COMPLETE.\n');

%% ========================================================================
%% HELPER FUNCTIONS
%% ========================================================================

function planner = configurePlanner(planner, maxTime)
    if nargin < 2, maxTime = 600; end
    if isprop(planner,'SkippedSelfCollisions'),   planner.SkippedSelfCollisions = "parent"; end
    if isprop(planner,'ValidationDistance'),      planner.ValidationDistance = 0.04; end
    if isprop(planner,'MaxConnectionDistance'),   planner.MaxConnectionDistance = 0.8; end 
    if isprop(planner,'MaxIterations'),           planner.MaxIterations = 20000; end 
    if isprop(planner,'MaxTime'),                 planner.MaxTime = maxTime; end % SET TIMEOUT
    if isprop(planner,'EnableConnectHeuristic'),  planner.EnableConnectHeuristic = true; end
end

function [path, info] = planOrRetry(planner, qStart, qGoal, tries)
    qStart = double(qStart); qGoal = double(qGoal);
    path = []; info = struct();
    for t = 1:max(1,tries)
        rng('shuffle'); 
        try
            [path, info] = plan(planner, qStart, qGoal);
        catch
            path=[]; 
        end
        if ~isempty(path), return; end
    end
end

function free = isConfigFree(robot, q, env)
    free = ~any(checkCollision(robot, q, env, "Exhaustive","on","SkippedSelfCollisions","parent"),'all');
end

% --- Validation helpers ---

function ok = assertPaperPathCollisionFree(robot, states, envCol, eeName, P)
% Validates path against strict Task Rules (Base Safety, Z-Floor, Rebar Capsule)
    ok = true; 
    if isempty(states), return; end
    for k = 1:size(states, 1)
        if paperCollisionSimple(states(k,:), robot, eeName, envCol, ...
                                P.baseIncircle, P.rebarRadius, P.rebarLen, P)
            ok = false; return;
        end
    end
end

function coll = paperCollisionSimple(q, rb, eeName, envCol, baseIncircle, rebarRadius, rebarLen, P)
    coll = false;
    % 1. Base Incircle Check
    S = resolveBaseIndices(rb); baseXY = q(S.baseIdx(1:2));
    if minClearance2D(baseXY, envCol) < baseIncircle, coll=true; return; end
    
    % 2. Rebar Geometry
    Ttcp = getTransform(rb, q, eeName); o = Ttcp(1:3,4); Rtcp = Ttcp(1:3,1:3);
    dir = Rtcp(:,2)/max(norm(Rtcp(:,2)),1e-9);
    p0 = o - 0.5*rebarLen*dir; p1 = o + 0.5*rebarLen*dir;
    
    % 3. Z-Floor Constraint
    if nargin>=8 && isfield(P,'zFloor')
        if (min(p0(3),p1(3)) - rebarRadius - rebarEnvelopeSag(P)) < (P.zFloor - 0.001), coll=true; return; end
    end
    
    % 4. Rebar Capsule Check (Environment)
    sagDepth = rebarEnvelopeSag(P);
    N = max(ceil(rebarLen/0.1), 3);
    Nz = max(1, ceil(sagDepth/0.05));
    for k=0:N
        pCenter = p0 + (k/N)*(p1-p0);
        for j=0:Nz
            if minClearance3D(pCenter - [0;0;sagDepth*(j/Nz)], envCol) < rebarRadius
                coll=true; return;
            end
        end
    end
    
    % 5. Rebar vs Base Chassis (Self Collision)
    if nargin>=8 && isfield(P,'baseWidth')
        bx=q(S.baseIdx(1)); by=q(S.baseIdx(2)); bz=q(S.baseIdx(3));
        c=cos(bz); s=sin(bz); R_bw=[c s 0; -s c 0; 0 0 1]; p_base=[bx; by; 0];
        pts=[p0, p1, (p0+p1)/2];
        pad = rebarRadius + 0.03;
        limX = P.baseLength/2 + pad; limY = P.baseWidth/2 + pad; limZ = P.baseHeight + pad;
        for k=1:3
            pt_local = R_bw' * (pts(:,k) - p_base);
            if (pt_local(3)<limZ && pt_local(3)>-0.1) && (abs(pt_local(1))<limX) && (abs(pt_local(2))<limY)
                coll=true; return;
            end
        end
    end

    if isfield(P, 'armKeyNames') && isfield(P, 'armRadius')
        for iName = 1:numel(P.armKeyNames)
            bodyName = P.armKeyNames(iName);
            try
                Tlink = getTransform(rb, q, bodyName);
            catch
                continue;  % in case bodyName doesn't exist
            end
            center = Tlink(1:3,4).';
            if minClearance3D(center, envCol) < P.armRadius
                coll = true; 
                return;
            end
        end
    end
end

function d = minClearance2D(p, envIn)
    d = inf; p = p(:).'; ignoreHeightThreshold = 0.2; 
    for i = 1:numel(envIn)
        co = envIn{i}; if isempty(co)||~isprop(co,'Pose'), continue; end
        T = co.Pose; z_center = T(3,4);
        if isa(co,'collisionBox'), topZ=z_center+co.Z/2; elseif isa(co,'collisionCylinder'), topZ=z_center+co.Height/2; else, topZ=inf; end
        if topZ < ignoreHeightThreshold, continue; end
        if isa(co,'collisionBox')
            R=T(1:2,1:2); t=T(1:2,4).'; pL=(p-t)/R.'; h=[co.X,co.Y]/2; q=abs(pL)-h; 
            di=norm(max(q,0)); if all(q<=0), di=-min(max(-q)); end
        elseif isa(co,'collisionSphere')||isa(co,'collisionCylinder')
            c=T(1:2,4).'; di=norm(p-c)-co.Radius;
        else, continue; end
        d = min(d, di);
    end
    if isinf(d), d = 100; end
end

function d = minClearance3D(p, env)
    d = inf; p = p(:).'; 
    for i = 1:numel(env)
        co = env{i}; T = co.Pose; 
        if isa(co,'collisionBox')
            R=T(1:3,1:3); t=T(1:3,4).'; pL=(p-t)/R.'; h=[co.X,co.Y,co.Z]/2; 
            q=abs(pL)-h; di=norm(max(q,0)); if all(q<=0), di=max(q); end 
        elseif isa(co,'collisionSphere')
            c=T(1:3,4).'; di=norm(p-c)-co.Radius;
        elseif isa(co,'collisionCylinder')
            R=T(1:3,1:3); t=T(1:3,4).'; pL=(p-t)/R.'; r=co.Radius; h2=co.Height/2; 
            vec_d=[hypot(pL(1),pL(2))-r, abs(pL(3))-h2]; di=norm(max(vec_d,0))+min(max(vec_d(1),vec_d(2)),0);
        else, continue; end
        d = min(d, di); 
    end
    if isinf(d), d = 100; end
end

function S = resolveBaseIndices(robot)
    bxBody = getBody(robot, "base_x");   jx = string(bxBody.Joint.Name);
    byBody = getBody(robot, "base_y");   jy = string(byBody.Joint.Name);
    chBody = getBody(robot, "chassis");  jz = string(chBody.Joint.Name);
    fmt0 = robot.DataFormat; robot.DataFormat = 'struct';
    cfg = homeConfiguration(robot); robot.DataFormat = fmt0;
    jointNamesInOrder = string({cfg.JointName});
    ix = find(jointNamesInOrder == jx, 1);
    iy = find(jointNamesInOrder == jy, 1);
    iz = find(jointNamesInOrder == jz, 1);
    S.baseIdx = [ix iy iz];
end

function traj = hs_interpPath(path, maxStep)
    if isempty(path) || size(path,1)<=1, traj=path; return; end
    traj = path(1,:);
    for i = 2:size(path,1)
        qa = traj(end,:); qb = path(i,:); dq = qb - qa;
        isAngle = true(size(dq)); if numel(dq)>3, isAngle(1:2)=false; end
        dq(isAngle) = atan2(sin(dq(isAngle)), cos(dq(isAngle)));
        n = max(2, ceil(norm(dq)/maxStep));
        for k = 1:n
            s = k/n; q = qa + s*dq;
            q(isAngle) = atan2(sin(q(isAngle)), cos(q(isAngle)));
            traj = [traj; q]; %#ok<AGROW>
        end
    end
end

function [trace, qLastFree] = guardedApproachToolAxis(robot, qStart, goalPose, envCol, ik, ee, w, step)
    qStart = double(qStart); trace = qStart; q = qStart;
    Rgoal = goalPose(1:3,1:3); tGoal = goalPose(1:3,4); zToolWorld = Rgoal(:,3); 
    Tcur = getTransform(robot, q, ee); t = Tcur(1:3,4);
    dirVec = sign(step)*zToolWorld;  mag = abs(step);
    while dot(tGoal - t, dirVec) > 1e-4
        ds = min(mag, norm(tGoal - t));
        Tnew = [Rgoal, t + ds*dirVec; 0 0 0 1];
        qNew = double(ik(ee, Tnew, w, q));
        if any(checkCollision(robot, qNew, envCol, "Exhaustive","on","SkippedSelfCollisions","parent"), 'all')
            qLastFree = q; return;
        end
        q = qNew; t = getTransform(robot, q, ee); t = t(1:3,4);
        trace = [trace; q]; %#ok<AGROW>
    end
    qLastFree = q;
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
    required = {'rebarEnvelopeLength','rebarEnvelopeWidth','rebarEnvelopeHeight','rebarRadius'};
    if isempty(ax) || ~isvalid(ax) || ~all(isfield(P,required)), return; end
    dims = [P.rebarEnvelopeLength P.rebarEnvelopeWidth P.rebarEnvelopeHeight P.rebarRadius];
    if any(~isfinite(dims)), return; end
    L=P.rebarEnvelopeLength;
    W=max(2*P.rebarRadius,P.rebarEnvelopeWidth);
    H=max(P.rebarEnvelopeHeight,0.02);
    Ttcp=getTransform(robot,q,eeName); centerTop=Ttcp(1:3,4);
    xAxis=Ttcp(1:3,2); xAxis=xAxis/max(norm(xAxis),1e-9);
    zWorld=[0;0;1]; yAxis=cross(zWorld,xAxis);
    if norm(yAxis)<1e-9, yAxis=cross([1;0;0],xAxis); end
    yAxis=yAxis/max(norm(yAxis),1e-9);
    center=centerTop-0.5*H*zWorld;
    signs=[-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1; ...
           -1 -1 1;1 -1 1;1 1 1;-1 1 1];
    V=zeros(8,3);
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

function metrics = computeComprehensiveMetrics(robot, allStates, traj1, traj2, traj3, traj4, traj5, endEffector)
    Sx = resolveBaseIndices(robot);
    ix = Sx.baseIdx(1); iy = Sx.baseIdx(2); iyaw = Sx.baseIdx(3);
    metrics = struct();
    metrics.total_waypoints = size(allStates, 1);
    
    if isempty(allStates)
        metrics.base.total_length_xy = 0;
        metrics.base.total_yaw_change = 0;
        metrics.tcp.total_length = 0;
        metrics.tcp.height_range = 0;
        metrics.efficiency.tcp_to_base_ratio = 0;
        metrics.raw.base_xy = []; metrics.raw.base_yaw = []; metrics.raw.tcp_positions = []; metrics.raw.tcp_rpy = [];
        return;
    end
    
    base_xy = allStates(:, [ix iy]);
    base_yaw = allStates(:, iyaw);
    dxy = sqrt(sum(diff(base_xy, 1, 1).^2, 2));
    metrics.base.total_length_xy = sum(dxy);
    metrics.base.total_yaw_change = sum(abs(atan2(sin(diff(base_yaw)), cos(diff(base_yaw)))));
    
    tcp_positions = zeros(size(allStates,1), 3);
    tcp_rpy       = zeros(size(allStates,1), 3);
    for i = 1:size(allStates,1)
        T = getTransform(robot, allStates(i,:), endEffector);
        tcp_positions(i,:) = T(1:3,4)';
        R = T(1:3,1:3);
        tcp_rpy(i,1) = atan2(R(3,2), R(3,3)); 
        tcp_rpy(i,2) = asin(-R(3,1));         
        tcp_rpy(i,3) = atan2(R(2,1), R(1,1)); 
    end
    dtcp = sqrt(sum(diff(tcp_positions, 1, 1).^2, 2));
    metrics.tcp.total_length = sum(dtcp);
    metrics.tcp.height_range = range(tcp_positions(:,3));
    metrics.efficiency.tcp_to_base_ratio = metrics.tcp.total_length / max(metrics.base.total_length_xy, 1e-6);
    metrics.raw.base_xy = base_xy;
    metrics.raw.base_yaw = base_yaw;
    metrics.raw.tcp_positions = tcp_positions;
    metrics.raw.tcp_rpy = tcp_rpy;
end
