function [mobileUR20,config,env,pickPose,placePose,placeRebarCenterPosition,rebarMeta] = loadRebarTransportScenario
% Helper: builds mobile UR20 + gripper, defines a TCP, and returns pick/place poses.
% - TCP is defined here (not in the execution script).
% - Pick poses are cylinder-aware: radial approach, fingers parallel to the bar.

%% ===================== USER KNOBS (easy to tune) =======================
% TCP placement relative to parent link (default parent = 'tool0')
tcp_offset_along_tool0Z = 0.00;    % meters forward from parent to pinch point
tcp_flip_about_X_rad     = 0.00;    % rotate around parent X (e.g., pi to flip)
tcp_roll_about_Z_rad = pi/2;    % rotate around parent Z (yaw)

% Finger orientation w.r.t. the rebar for pick:
fingersAxis   = 'X';               % 'X' or 'Y': which tool axis runs along the bar
fingerRollDeg = 90;                % extra roll about TCP Z (deg), e.g. 90 or -90
grasp_clearance = 0.12;            % pre‑grasp stand‑off from cylinder surface (m)

% Set this to a gripper link name to attach the TCP to that link.
% Otherwise leave empty to attach TCP under 'tool0'.
tcp_parent_link_override = "";     % e.g., "left_finger_tip" or ""

%% ===================== 1) Mobile base ================================
mobileUR20 = rigidBodyTree('DataFormat','row','MaxNumBodies',40);
mobileUR20.BaseName = 'world';
chassisBody = rigidBody('chassis');
chassisHeight = 0.5;
chassisDims = [0.4, 0.7, chassisHeight];
chassisCollisionGeom = collisionBox(chassisDims(1), chassisDims(2), chassisDims(3));
% chassisCollisionGeom.Pose = trvec2tform([0 0 -0.05]);
addCollision(chassisBody, chassisCollisionGeom);

ground_clearance = 0.3;   % 8 cm lift; tune as needed
% ...
jointX = rigidBodyJoint('joint_x','prismatic'); jointX.JointAxis = [1 0 0];
jointY = rigidBodyJoint('joint_y','prismatic'); jointY.JointAxis = [0 1 0];
jointZ = rigidBodyJoint('joint_z','revolute');  jointZ.JointAxis  = [0 0 1];
% LIFT the whole mobile base by ground_clearance in Z:
setFixedTransform(jointX, trvec2tform([0 0 ground_clearance]));   % << add
setFixedTransform(jointY, trvec2tform([0 0 0]));
setFixedTransform(jointZ, trvec2tform([0 0 0]));

chassisBody.Joint = jointZ;
bodyX = rigidBody('base_x'); bodyX.Joint = jointX;
bodyY = rigidBody('base_y'); bodyY.Joint = jointY;

addBody(mobileUR20, bodyX, 'world');
addBody(mobileUR20, bodyY, 'base_x');
addBody(mobileUR20, chassisBody, 'base_y');

%% ===================== 2) UR20 + gripper + TCP =======================
ur20 = importrobot("ur20.urdf");  ur20.DataFormat = "row"; 
ur20 = makeCollisionLite(ur20, 0.01); % +1 cm inflation on each dimension (tune)
gripper = importrobot("gripper.urdf");

lastLinkName = 'C-2007588'; % your URDF's last arm link
toolBody = rigidBody('tool0');
toolJoint = rigidBodyJoint('wrist_to_tool0','fixed');
setFixedTransform(toolJoint, trvec2tform([0,0,0.13]));
toolBody.Joint = toolJoint;
addBody(ur20, toolBody, lastLinkName);
addSubtree(ur20, 'tool0', gripper);

% ---- TCP under chosen parent link ----
tcpParent = 'tool0';
if strlength(tcp_parent_link_override) > 0
    tcpParent = char(tcp_parent_link_override);
    % sanity check: ensure link exists; otherwise fall back to tool0
    try %#ok<TRYNC>
        getBody(ur20, tcpParent);
    catch
        warning("TCP parent link '%s' not found; using 'tool0' instead.", tcpParent);
        tcpParent = 'tool0';
    end
end
tcpBody  = rigidBody('tcp');
tcpJoint = rigidBodyJoint('to_tcp','fixed');
T_tool_tcp = trvec2tform([0,0,tcp_offset_along_tool0Z]) * ...
             eul2tform([tcp_flip_about_X_rad, 0, tcp_roll_about_Z_rad],'XYZ');
setFixedTransform(tcpJoint, T_tool_tcp);
tcpBody.Joint = tcpJoint;
addBody(ur20, tcpBody, tcpParent);

%% ===================== 3) Mount the arm on chassis ===================
mountTransform = trvec2tform([0, 0, chassisHeight/2+0.01]) * ...
                 eul2tform([0, pi/2, 0]) * eul2tform([0, 0, -pi/2]) * eul2tform([pi/2, 0, 0]);
armMountBody = rigidBody('arm_mount');
armMountJoint = rigidBodyJoint('arm_mount_joint','fixed');
setFixedTransform(armMountJoint, mountTransform);
armMountBody.Joint = armMountJoint;
addBody(mobileUR20, armMountBody, 'chassis');
addSubtree(mobileUR20, 'arm_mount', ur20);

%% ===================== 4) Start configuration ========================
config = [0.5 2 0 0 0 pi/2 0 pi/2 0 0 0];

% %% ===================== 5) Environment (rebar) ========================
env = {};
% % % Floor
% floor = collisionBox(10, 10, 0.01); 
% floor.Pose = trvec2tform([0 0 -0.01]); 
% env{end+1} = struct('Name', 'floor', 'CollisionObj', floor);
% 
% % Soil (This acts as the back wall for the 'place' room)
% % soil = collisionBox(6, 0.4, 2);
% % soil.Pose = trvec2tform([0, -3.4, 1]);
% % env{end+1} = struct('Name', 'soil', 'CollisionObj', soil);
% 
% Rebar (Cylinder)
rebarDiameter = 0.010;
rebarRadius = rebarDiameter/2;  
rebarLength = 3;
rebarGripPoint = "center";
rebar = collisionCylinder(rebarRadius, rebarLength);
epsilon = 1e-4;
rebarCenterZ = rebarRadius + epsilon + 0.5;
rebarCenterPosition = [1, 2, rebarCenterZ];
rebarOrientation   = eul2tform([0 0 pi/2]);  % bar roughly along world Y : pi/2 (3)
T_rebar_in_world   = trvec2tform(rebarCenterPosition) * rebarOrientation;
rebar.Pose         = T_rebar_in_world;
env{end+1}         = struct('Name', 'rebar', 'CollisionObj', rebar);
T_rebar_start_W = T_rebar_in_world; 

% %% ================= 5b) S-Corridor Environment ========================
% % This creates an S-shaped corridor with 1.5m width
% wallHeight = 2.5;
% wallThickness = 0.2;
% corridorWidth = 2;
% halfCorridor = corridorWidth / 2;
% halfWall = wallThickness / 2;
% 
% % --- Outer Boundary Walls (10x10 area) ---
% outerWall_Left = collisionBox(wallThickness, 10, wallHeight);
% outerWall_Left.Pose = trvec2tform([-5, 0, wallHeight/2]);
% env{end+1} = struct('Name', 'OuterWall_Left', 'CollisionObj', outerWall_Left);
% 
% outerWall_Right = collisionBox(wallThickness, 10, wallHeight);
% outerWall_Right.Pose = trvec2tform([5, 0, wallHeight/2]);
% env{end+1} = struct('Name', 'OuterWall_Right', 'CollisionObj', outerWall_Right);
% 
% outerWall_Top = collisionBox(10, wallThickness, wallHeight);
% outerWall_Top.Pose = trvec2tform([0, 5, wallHeight/2]);
% env{end+1} = struct('Name', 'OuterWall_Top', 'CollisionObj', outerWall_Top);
% 
% outerWall_Bottom = collisionBox(10, wallThickness, wallHeight);
% outerWall_Bottom.Pose = trvec2tform([0, -5, wallHeight/2]);
% env{end+1} = struct('Name', 'OuterWall_Bottom', 'CollisionObj', outerWall_Bottom);
% 
% % Wall B (East side of C1)
% wallB_len = 6; % from y=5 down to y=-1.5
% wallB = collisionBox(wallThickness, wallB_len, wallHeight);
% wallB.Pose = trvec2tform([-3 + halfCorridor + halfWall, 2 wallHeight/2]);
% env{end+1} = struct('Name', 'InnerWall_B', 'CollisionObj', wallB);
% 
% % Corridor 2 (Horizontal, Middle, centered at y=-1.5)
% % Wall C (North side of C2)
% wallC_len = 8; % from x=-3.85 to x=3.85
% wallC = collisionBox(wallC_len, wallThickness, wallHeight);
% wallC.Pose = trvec2tform([-1, -1.5 + halfCorridor + halfWall, wallHeight/2]);
% env{end+1} = struct('Name', 'InnerWall_C', 'CollisionObj', wallC);
% 
% % Wall D (South side of C2)
% wallD_len = 8;
% wallD = collisionBox(wallD_len, wallThickness, wallHeight);
% wallD.Pose = trvec2tform([1, -1.5 - halfCorridor - halfWall, wallHeight/2]);
% env{end+1} = struct('Name', 'InnerWall_D', 'CollisionObj', wallD);


% %% ===================== 5a) Walls and Door ============================
% % Wall at y=0 with a 1 m door gap (from x=-0.5 to x=0.5)
% % The floor is 10x10, centered at (0,0).
% wallHeight = 2.5;
% wallThickness = 0.2;
% wallY_pos = 0;
% doorWidth = 1.5;
% doorHalfWidth = doorWidth / 2;
% 
% % Door Wall Left Segment
% % Extends from x = -5 (floor edge) to x = -0.5 (door edge)
% doorWallLeft_Length = 2.5; % 5 - 0.5
% doorWallLeft_Center_X = - 3.75; 
% wallDoorLeft = collisionBox(doorWallLeft_Length, wallThickness, wallHeight);
% wallDoorLeft.Pose = trvec2tform([doorWallLeft_Center_X, wallY_pos, wallHeight/2]);
% env{end+1} = struct('Name', 'DoorWallLeft', 'CollisionObj', wallDoorLeft);
% 
% % Door Wall Right Segment
% % Extends from x = 0.5 (door edge) to x = 5 (floor edge)
% doorWallRight_Length = 6; % 5 - 0.5
% doorWallRight_Center_X = 2;
% wallDoorRight = collisionBox(doorWallRight_Length, wallThickness, wallHeight);
% wallDoorRight.Pose = trvec2tform([doorWallRight_Center_X, wallY_pos, wallHeight/2]);
% env{end+1} = struct('Name', 'DoorWallRight', 'CollisionObj', wallDoorRight);
% 
% beamHeight = 0.25;
% doorTop =  collisionBox(10, wallThickness, beamHeight);
% doorTop.Pose = trvec2tform([0, 0, 2.25 + beamHeight/2]);
% env{end+1} = struct('Name', 'wallBeam', 'CollisionObj', doorTop);
% 
% % Side Wall Left (at x = -5)
% wallSideLeft = collisionBox(wallThickness, 10, wallHeight);
% wallSideLeft.Pose = trvec2tform([-5, 0, wallHeight/2]);
% env{end+1} = struct('Name', 'WallSideLeft', 'CollisionObj', wallSideLeft);
% 
% % Side Wall Right (at x = 5)
% wallSideRight = collisionBox(wallThickness, 10, wallHeight);
% wallSideRight.Pose = trvec2tform([5, 0, wallHeight/2]);
% env{end+1} = struct('Name', 'WallSideRight', 'CollisionObj', wallSideRight);
% 
% % Back Wall (Start Room, at y = 5)
% wallBack = collisionBox(10, wallThickness, wallHeight);
% wallBack.Pose = trvec2tform([0, 5, wallHeight/2]);
% env{end+1} = struct('Name', 'WallBack', 'CollisionObj', wallBack);
% % % ===================== End of Walls and Door =======================



% Construction-site obstacle layouts
% ===================== Cluttered Site 1 ======================

%Column1
column1Height = 2.5;
column1 = collisionBox(0.25, 0.25, column1Height);
column1.Pose = trvec2tform([4.5, -0.5, column1Height/2]);
env{end+1} = struct('Name', 'column', 'CollisionObj', column1);

%Column2
column2Height = 2.5;
column2 = collisionBox(0.25, 0.25, column2Height);
column2.Pose = trvec2tform([0, -0.5, column2Height/2]);
env{end+1} = struct('Name', 'column', 'CollisionObj', column2);

%Column3
column3Height = 2.5;
column3 = collisionBox(0.25, 0.25, column3Height);
column3.Pose = trvec2tform([-4.5, -0.5, column3Height/2]);
env{end+1} = struct('Name', 'column', 'CollisionObj', column3);

% Material
material = collisionBox(1, 0.4, 1.2);
material.Pose = trvec2tform([-1, -0.5, 0.6]);
env{end+1} = struct('Name', 'material', 'CollisionObj', material);

% Material2
material2 = collisionBox(1, 0.4, 0.6);
material2.Pose = trvec2tform([-3.5, 0.5, 0.3]);
env{end+1} = struct('Name', 'material2', 'CollisionObj', material2);

% Material2
material3Height = 2.5;
material3 = collisionBox(1, 0.5, material3Height);
material3.Pose = trvec2tform([2, -1, material3Height/2]);
env{end+1} = struct('Name', 'material2', 'CollisionObj', material3);

material4Height = 1.5;
material4 = collisionBox(0.5, 0.5, material4Height);
material4.Pose = trvec2tform([2, 1, material4Height/2]);
env{end+1} = struct('Name', 'material2', 'CollisionObj', material4);

material5Height = 1.5;
material5 = collisionBox(0.5, 1, material5Height);
material5.Pose = trvec2tform([2, -2, material5Height/2]);
env{end+1} = struct('Name', 'material2', 'CollisionObj', material5);

material6Height = 1.5;
material6 = collisionBox(1, 0.5, material6Height);
material6.Pose = trvec2tform([-1, -2, material6Height/2]);
env{end+1} = struct('Name', 'material2', 'CollisionObj', material6);



% ===================== Cluttered Site 2 ======================

% % Material
% material = collisionBox(1, 0.4, 1.2);
% material.Pose = trvec2tform([-1, -0.5, 0.6]);
% env{end+1} = struct('Name', 'material', 'CollisionObj', material);
% 
% % Material2
% material2 = collisionBox(1, 0.4, 0.6);
% material2.Pose = trvec2tform([-3.5, 0.5, 0.3]);
% env{end+1} = struct('Name', 'material2', 'CollisionObj', material2);
% 
% % Material2
% material3Height = 1;
% material3 = collisionBox(1, 0.5, material3Height);
% material3.Pose = trvec2tform([2, -1, material3Height/2]);
% env{end+1} = struct('Name', 'material2', 'CollisionObj', material3);
% 
% material4Height = 1.2;
% material4 = collisionBox(0.5, 0.5, material4Height);
% material4.Pose = trvec2tform([2, 1, material4Height/2]);
% env{end+1} = struct('Name', 'material2', 'CollisionObj', material4);
% 
% material5Height = 1.5;
% material5 = collisionBox(0.5, 1, material5Height);
% material5.Pose = trvec2tform([2, -2, material5Height/2]);
% env{end+1} = struct('Name', 'material2', 'CollisionObj', material5);
% 
% material6Height = 1.5;
% material6 = collisionBox(1, 0.5, material6Height);
% material6.Pose = trvec2tform([0, -2, material6Height/2]);
% env{end+1} = struct('Name', 'material2', 'CollisionObj', material6);
% 
% material7Height = 1;
% material7 = collisionBox(1, 1, material7Height);
% material7.Pose = trvec2tform([4.5, -1, material7Height/2]);
% env{end+1} = struct('Name', 'material2', 'CollisionObj', material7);

%% ===================== 6) Pick (cylinder‑aware) ======================
% Build TCP poses so that:
%   - TCP −Z points toward the bar center (approach)
%   - Tool axis (X or Y) is parallel to the bar (fingers parallel)
%   - fingerRollDeg rotates around TCP Z to fine‑tune finger angle
fingerRoll = deg2rad(fingerRollDeg);
[T_tcp_above_world, T_tcp_touch_world] = cylTopApproach( ...
    T_rebar_in_world, rebarRadius, grasp_clearance, fingerRoll, fingersAxis);
pickPose = T_tcp_above_world;  % for compatibility with your execution code

%% ===================== 7) Place (same TCP, simple target) ============


% %below is code for S-coordor
% placeRebarCenterPosition = [4.5, -4, 1.25];
% T_placeObj_in_world      = trvec2tform(placeRebarCenterPosition);
% place_clearance = 0.10;
% x_tcp = [0; 0; -1];          % tool X down
% z_tcp = [1; 0; 0];          % tool Z toward -Y
% y_tcp = cross(z_tcp, x_tcp); % right-handed
% R_place = eye(4);
% R_place(1:3,1:3) = [x_tcp, y_tcp, z_tcp];   % 4x4, zero translation

% %below is code for door
% placeRebarCenterPosition = [4, -2, 1.25];
% T_placeObj_in_world      = trvec2tform(placeRebarCenterPosition);
% place_clearance = 0.10;
% x_tcp = [0; 0; -1];          % tool X down
% z_tcp = [1; 0; 0];         % tool Z toward -Y
% y_tcp = cross(z_tcp, x_tcp); % right-handed
% R_place = eye(4);
% R_place(1:3,1:3) = [x_tcp, y_tcp, z_tcp];   % 4x4, zero translation

%below is code for construction site
placeRebarCenterPosition = [0, -4, 1.25];
T_placeObj_in_world      = trvec2tform(placeRebarCenterPosition);
place_clearance = 0.10;
x_tcp = [0; 0; -1];          % tool X down
z_tcp = [0; -1; 0];          % tool Z toward -Y
y_tcp = cross(z_tcp, x_tcp); % right-handed
R_place = eye(4);
R_place(1:3,1:3) = [x_tcp, y_tcp, z_tcp];   % 4x4, zero translation

% Touch/above TCP poses (same multiplication order as before)
T_tcp_place_touch_world = T_placeObj_in_world * trvec2tform([0,0,0])              * R_place;
T_tcp_place_above_world = T_placeObj_in_world * trvec2tform([0,0,place_clearance]) * R_place;
placePose = T_tcp_place_above_world;

% --- also publish 3x3 into meta (so execution can build targetPose) ---
R_place_3x3 = R_place(1:3,1:3);

%% ===================== 8) Publish meta bundle ========================
UR20Meta = struct();
UR20Meta.tcpName                 = 'tcp';
UR20Meta.tcpParent               = tcpParent;
UR20Meta.T_tool_tcp              = T_tool_tcp;

% Start rebar pose (canonical + legacy)
UR20Meta.T_rebar_start_W         = T_rebar_start_W;
UR20Meta.T_rebar_in_world        = T_rebar_in_world;  % legacy name if used elsewhere

% TCP poses (pick & place)
UR20Meta.T_tcp_touch_world       = T_tcp_touch_world;
UR20Meta.T_tcp_above_world       = T_tcp_above_world;
UR20Meta.T_tcp_place_touch_world = T_tcp_place_touch_world;
UR20Meta.T_tcp_place_above_world = T_tcp_place_above_world;

% Place goal description
UR20Meta.R_place                  = R_place_3x3;             % 3x3 rotation
UR20Meta.placeCenter_W            = placeRebarCenterPosition(:); % 3x1 center

% ---- Rebar meta (add grip sign and start/goal info) ----
rebarMeta = struct();
rebarMeta.DiameterM   = rebarDiameter;
rebarMeta.LengthM     = rebarLength;
rebarMeta.GripPoint   = rebarGripPoint;   % "center" or "end"
rebarMeta.GripEndSign = +1;               % default (+1 or -1) if GripPoint=="end"

% Start pose & goal pose bits (so execution doesn't recompute)
rebarMeta.T_start_W     = T_rebar_start_W;       % 4x4
rebarMeta.R_place       = R_place_3x3;           % 3x3
rebarMeta.placeCenter_W = placeRebarCenterPosition(:);  % 3x1

% Optional: still publish UR20Meta to base for ad-hoc use
assignin('base','UR20Meta',UR20Meta);

end % =================== end main function ==============================

%% ==================== Local helpers ==================================
function [T_above, T_touch] = cylTopApproach(T_cyl_W, radius, clearance, fingerRoll, fingersAxis)
% Cylinder-aware TCP poses from the TOP (radial).
% -Z_tool points to cylinder center. 'fingersAxis' ('X' or 'Y') runs along the bar.
    if nargin < 5 || isempty(fingersAxis), fingersAxis = 'Y'; end
    Rw = T_cyl_W(1:3,1:3);
    cw = T_cyl_W(1:3,4);
    a  = Rw(:,3);                 % bar axis in world (cylinder local Z)
    
    zW = [0;0;1];
    % Radial direction toward the "top" side of the bar
    rhat = zW - a*dot(a,zW);
    if norm(rhat) < 1e-8, xW=[1;0;0]; rhat = xW - a*dot(a,xW); end
    rhat = rhat / norm(rhat);
    
    zTool = -rhat;  % approach into the bar
    switch upper(fingersAxis)
        case 'Y'
            yTool = a;                          % fingers along bar
            xTool = cross(yTool, zTool); xTool = xTool / norm(xTool);
            yTool = cross(zTool, xTool);
        case 'X'
            xTool = a;                          % fingers along bar
            yTool = cross(zTool, xTool); yTool = yTool / norm(yTool);
            xTool = cross(yTool, zTool);
        otherwise
            error("fingersAxis must be 'X' or 'Y'.");
    end
    
    Rtool = [xTool, yTool, zTool];
    if abs(fingerRoll) > 1e-12
        Rtool = Rtool * eul2rotm([0 0 fingerRoll], 'XYZ');
    end

    p_touch = cw + rhat * radius;
    p_above = cw + rhat * (radius + clearance);

    T_touch = [Rtool, p_touch; 0 0 0 1];
    T_above = [Rtool, p_above; 0 0 0 1];
end

function robotOut = makeCollisionLite(robotIn, margin)
% makeCollisionLite: Replace mesh collisions with single collisionBox per link.
% - margin: optional inflation (meters), e.g., 0.01 for +1 cm on each dimension.
    if nargin < 2, margin = 0.0; end
    robotOut = robotIn;
    assert(strcmpi(robotOut.DataFormat,'row') || strcmpi(robotOut.DataFormat,'column') ...
        || strcmpi(robotOut.DataFormat,'struct'), 'Set DataFormat first.');

    for i = 1:numel(robotOut.Bodies)
        b = robotOut.Bodies{i};
        colls = b.Collisions;
        if isempty(colls), continue; end
        
        % Accumulate all collision meshes on this link into a single AABB in the link frame
        mins = [ Inf;  Inf;  Inf];
        maxs = [-Inf; -Inf; -Inf];
        % Retain the first collision pose for box attachment.
        basePose = eye(4); hasBasePose = false;

        for k = 1:numel(colls)
            c = colls{k};
            if ~isa(c,'collisionMesh') && ~isa(c,'collisionBox') && ~isa(c,'collisionCylinder')
                continue; % skip unsupported types (rare)
            end
            
            % Get pose of this collision element relative to the link frame
            T_L_C = c.Pose;
            
            % Get a representative point cloud in the *collision element* frame
            % For mesh: use its vertices; for primitives, sample their corners
            P_C = [];
            if isa(c,'collisionMesh')
                V = c.Vertices; % Nx3
                P_C = [V, ones(size(V,1),1)]';  % 4xN
            elseif isa(c,'collisionBox')
                sz = [c.X c.Y c.Z];
                P_C = cornersOfBox(sz); % 4x8
            elseif isa(c,'collisionCylinder')
                % use an oriented bounding box around the cylinder (conservative)
                r = c.Radius; h = c.Length;
                P_C = cornersOfBox([2*r, 2*r, h]); % 4x8
            end
            
            % Transform points into the LINK frame
            P_L = T_L_C * P_C;       % 4xN
            
            mins = min(mins, min(P_L(1:3,:), [], 2));
            maxs = max(maxs, max(P_L(1:3,:), [], 2));
            
            if ~hasBasePose
                basePose = T_L_C;    % place the box at the first collision pose
                hasBasePose = true;
            end
        end
        
        if ~isfinite(mins(1)), continue; end  % nothing valid found
        
        % Size & center in LINK frame
        center_L = (mins + maxs)/2;
        size_L   = (maxs - mins);
        
        % Inflate
        size_L = size_L + 2*margin;
        
        % Build a box centered at center_L (in link frame)
        box = collisionBox(size_L(1), size_L(2), size_L(3));
        T_L_box = eye(4); T_L_box(1:3,4) = center_L;
        box.Pose = T_L_box;
        
        % Replace all collisions on this link with our single box
        b.Collisions = {box};
        robotOut.Bodies{i} = b;
    end
end

function P = cornersOfBox(sz)
% Returns 8 box corners (in its own frame) as homogeneous 4x8
    sx = sz(1)/2; sy = sz(2)/2; szz = sz(3)/2;
    U = [ -1 -1 -1; 1 -1 -1; -1 1 -1; 1 1 -1; -1 -1 1; 1 -1 1; -1 1 1; 1 1 1 ]';
    P = [diag([sx,sy,szz]) * U; ones(1,8)];
end
