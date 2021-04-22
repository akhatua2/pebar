% TODO: import own model using importrobot(filename)
robot = loadrobot('kinovaJacoJ2S7S300');

% Not required
removeBody(robot,'j2s7s300_link_finger_tip_3');
removeBody(robot,'j2s7s300_link_finger_3');
removeBody(robot,'j2s7s300_link_finger_tip_2');
removeBody(robot,'j2s7s300_link_finger_2');
removeBody(robot,'j2s7s300_link_finger_tip_1');
removeBody(robot,'j2s7s300_link_finger_1');

% Create obstacles 
floor = collisionBox(1, 1, 0.01);
tabletop1 = collisionBox(0.4,1,0.02);
tabletop1.Pose = trvec2tform([0.3,0,0.6]);
tabletop2 = collisionBox(0.6,0.2,0.02);
tabletop2.Pose = trvec2tform([-0.2,0.4,0.5]);
can = collisionCylinder(0.0001,0.00001);
can.Pose = trvec2tform([0.3,0.0,0.7]);


% Builidng world
ss = HelperRigidBodyTreeStateSpace(robot);
ss.EndEffector = 'j2s7s300_end_effector';
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
sv = HelperValidatorRigidBodyTree(ss);

% Add obstacles in the world
addFixedObstacle(sv,tabletop1, 'tabletop1', [71 161 214]/256);
addFixedObstacle(sv,tabletop2, 'tabletop2', [71 161 214]/256);
addFixedObstacle(sv,can, 'can', 'r');
addFixedObstacle(sv,floor, 'floor', [1,0.5,0]);

% Path planner (Do not modify) - Arpandeep
skipCollisionCheck(sv,'root'); % root will never touch any obstacles
skipCollisionCheck(sv,'j2s7s300_link_base'); % base will never touch any obstacles
skipCollisionCheck(sv,'j2s7s300_end_effector'); % this is a virtual frame
sv.ValidationDistance = 0.01;
rng(0,'twister') % 0
Te_0ref = Tw_0*Te_w; % Reference end-effector pose in world coordinates, derived from WGR
ik = inverseKinematics('RigidBodyTree',robot);
refGoalConfig = ik(ss.EndEffector,Te_0ref,ones(1,6),homeConfiguration(ss.RigidBodyTree));
T = Te_0ref;
T(1,4) = 0.3;
T(2,4) = 0.0;
T(3,4) = 0.4;
initConfig = ik(ss.EndEffector,T,ones(1,6),homeConfiguration(ss.RigidBodyTree));
planner = plannerRRT(ss,sv);
planner.GoalReachedFcn = @HelperIsStateInWorkspaceGoalRegion;
planner.MaxConnectionDistance = 0.5;
planner.GoalBias = 0;
[pthObj,solnInfo] = plan(planner,initConfig, initConfig);
interpolate(pthObj,100);
newPathObj = HelperPathSmoothing(pthObj,sv);
interpolate(newPathObj,200);
figure
states = newPathObj.States;
ax = show(robot,states(1,:));
zlim(ax, [-0.03, 1.4])
xlim(ax, [-1, 1])
ylim(ax, [-1, 1])
hold on
showObstacles(sv, ax);
view(146, 33)
camzoom(1.5)
for i = 2:length(states)
    show(robot,states(i,:),'PreservePlot',false,'Frames','off','Parent',ax);
    drawnow
end
q = states(i,:);
targetPos = [-0.15,0.35,0.51];








% Add waypoints here

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
    show(robot,states(i,:),'PreservePlot',false,'Frames','off','Parent',ax);
    drawnow
end
q = states(i,:);





