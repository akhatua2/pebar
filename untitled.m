len = 6;
wid = 4;
room = ExampleHelperRoom(len, wid);
chair = ExampleHelperFurniture;
addFurniture(room,chair,trvec2tform([0 3.5 0]));
show(room,gca)
axis equal

bounds = [-0.8 0.8; [-1 5]; [-pi pi]];

ss = stateSpaceSE2(bounds);
ss.WeightTheta = 2;

% Set the initial pose of the furniture
initPose = trvec2tform([0 3.5 0]);

% Create a customized state validator
sv = ExampleHelperFurnitureInRoomValidator(ss, initPose);

% Reduce the validation distance
% Validation distance determines the granularity of interpolation when
% checking the motion that connects two states.
sv.ValidationDistance = 0.01;


% Create the planner
rrt = plannerRRTStar(ss, sv);

% Set ball radius for searching near neighbors
rrt.BallRadiusConstant = 1000;

% Exit as soon as a path is found
rrt.ContinueAfterGoalReached = false;

% The motion length between two furniture poses should be less than 0.4 m
rrt.MaxConnectionDistance = 0.4;

% Increase the max iterations
rrt.MaxIterations = 200000;

% Use a customized goal function 
rrt.GoalReachedFcn = @exampleHelperGoalFunc;


% Set the init and goal poses
start = [0 3.5 0];
goal = [0 -0.2 pi];

% Set random number seed for repeatability
rng(0, 'twister');
[path, solnInfo] = plan(rrt,start,goal);

hold on
% Search tree
plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-');
% Interpolate path and plot points
interpolate(path,300)
plot(path.States(:,1), path.States(:,2), 'r-', 'LineWidth', 2)

hold off

f = figure;

% Smooth the path, cut the corners wherever possible
pathSm = exampleHelperSmoothPath(path, sv);

interpolate(pathSm,100);
animateFurnitureMotion(sv.Room,1,pathSm.States, axes(f))

% show the trace of furniture
%skip = 6;
%states = pathSm.States([1:skip:end, pathSm.NumStates], :);
%exampleHelperShowFurnitureTrace(sv.Room.FurnituresInRoom{1}, states);




% [Yes] Create some hurdles in the environment 
% [Yes] Change the shape of the piano
% [IDK] Bending of the bar (wrt to speed)
%


