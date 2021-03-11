kin = loadrobot('kinovaJacoJ2S7S300');

floor = collisionBox(1, 1, 0.01);
tabletop1 = collisionBox(0.4,1,0.02);
tabletop1.Pose = trvec2tform([0.3,0,0.6]);
tabletop2 = collisionBox(0.6,0.2,0.02);
tabletop2.Pose = trvec2tform([-0.2,0.4,0.5]);
can = collisionCylinder(0.03,0.16);
can.Pose = trvec2tform([0.3,0.0,0.7]);

% Create state space and set workspace goal regions (WGRs)
ss = ExampleHelperRigidBodyTreeStateSpace(kin);
ss.EndEffector = 'j2s7s300_end_effector';

% Define the workspace goal region (WGR)
% This WGR tells the planner that the can shall be grasped from
% the side and the actual grasp height may wiggle at most 1 cm.

% This is the orientation offset between the end-effector in grasping pose and the can frame
R = [0 0 1; 1 0 0; 0 1 0]; 

Tw_0 = can.Pose;
Te_w = rotm2tform(R);
bounds = [0 0;       % x
          0 0;       % y
          0 0.01;    % z
          0 0;       % R
          0 0;       % P
         -pi pi];    % Y
setWorkspaceGoalRegion(ss,Tw_0,Te_w,bounds);

sv = ExampleHelperValidatorRigidBodyTree(ss);

% Add obstacles in the environment
addFixedObstacle(sv,tabletop1, 'tabletop1', [71 161 214]/256);
addFixedObstacle(sv,tabletop2, 'tabletop2', [71 161 214]/256);
addFixedObstacle(sv,can, 'can', 'r');
addFixedObstacle(sv,floor, 'floor', [1,0.5,0]);

% Skip collision checking for certain bodies for performance
skipCollisionCheck(sv,'root'); % root will never touch any obstacles
skipCollisionCheck(sv,'j2s7s300_link_base'); % base will never touch any obstacles
skipCollisionCheck(sv,'j2s7s300_end_effector'); % this is a virtual frame

% Set the validation distance
sv.ValidationDistance = 0.01;

% Set random seeds for repeatable results
rng(0,'twister') % 0

% Compute the reference goal configuration. Note this is applicable only when goal bias is larger than 0. 
Te_0ref = Tw_0*Te_w; % Reference end-effector pose in world coordinates, derived from WGR
ik = inverseKinematics('RigidBodyTree',kin);
refGoalConfig = ik(ss.EndEffector,Te_0ref,ones(1,6),homeConfiguration(ss.RigidBodyTree));

% Compute the initial configuration (end-effector is initially under the table)
T = Te_0ref;
T(1,4) = 0.3;
T(2,4) = 0.0;
T(3,4) = 0.4;
initConfig = ik(ss.EndEffector,T,ones(1,6),homeConfiguration(ss.RigidBodyTree));


% Create the planner from previously created state space and state validator
planner = plannerRRT(ss,sv);

% If a node in the tree falls in the WGR, a path is considered found.
planner.GoalReachedFcn = @exampleHelperIsStateInWorkspaceGoalRegion;

% Set the max connection distance.
planner.MaxConnectionDistance = 0.5;

% With WGR, there is no need to specify a particular goal configuration (use
% initConfig to hold the place).
% As a result, one can set GoalBias to zero.
planner.GoalBias = 0;
[pthObj,solnInfo] = plan(planner,initConfig, initConfig);

% Smooth the path.
interpolate(pthObj,100);
newPathObj = exampleHelperPathSmoothing(pthObj,sv);
interpolate(newPathObj,200);

figure
states = newPathObj.States;

% Draw the robot.
ax = show(kin,states(1,:));
zlim(ax, [-0.03, 1.4])
xlim(ax, [-1, 1])
ylim(ax, [-1, 1])


% Render the environment.
hold on
showObstacles(sv, ax);
view(146, 33)
camzoom(1.5)

% Show the motion.
for i = 2:length(states)
    show(kin,states(i,:),'PreservePlot',false,'Frames','off','Parent',ax);
    drawnow
end


q = states(i,:);

% Grab the can.
q = exampleHelperEndEffectorGrab(sv,'can',q, ax);

targetPos = [-0.15,0.35,0.51];
exampleHelperDrawHorizontalCircle(targetPos,0.02,'y',ax);

Tw_0 = trvec2tform(targetPos+[0,0,0.08]); 
Te_w = rotm2tform(R);
bounds =  [0 0;       % x
           0 0;       % y
           0 0;       % z
           0 0;       % R
           0 0;       % P
          -pi pi];    % Y
setWorkspaceGoalRegion(ss,Tw_0,Te_w,bounds);
ss.UseConstrainedSampling = true;
planner.MaxConnectionDistance = 0.05;
[pthObj2,~] = plan(planner,q,q);

states = pthObj2.States;

view(ax, 152,45)
for i = 2:length(states)
    show(kin,states(i,:),'PreservePlot',false,'Frames','off','Parent',ax);
    drawnow
end
q = states(i,:);

% let go of the can
q = exampleHelperEndEffectorRelease(sv,q,ax);

