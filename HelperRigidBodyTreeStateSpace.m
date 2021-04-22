classdef HelperRigidBodyTreeStateSpace < nav.StateSpace & ...
                                   robotics.manip.internal.InternalAccess
    
    %   Copyright 2019 The MathWorks, Inc.
    
    properties
        %RigidBodyTree Robot model
        RigidBodyTree
        
        %EndEffector Name of the manipulator endeffector
        EndEffector
        
        %IKSolver
        IKSolver
        
        %PWGRGuidedSampling Probability of using workspace goal region
        %   guided sampling for random state
        PWGRGuidedSampling
        
        %UseConstrainedSampling
        UseConstrainedSampling
        
        %EndEffectorIsFree
        EndEffectorIsFree
        
        %NominalConfig
        NominalConfig
        
    end
    
    properties (SetAccess = protected)
        %WorkspaceGoalRegion
        WorkspaceGoalRegion
        
        %JointsNotOnKinPath
        JointsNotOnKinPath
        
                
        %DeltaQ
        DeltaQ
        
        %SkipCheck
        SkipCheck
    end
    
    methods
        function obj = HelperRigidBodyTreeStateSpace(robot)
            %EXAMPLEHELPERRIGIDBODYTREESTATESPACE Constructor
            robot.DataFormat = 'row';
            numDoF = length(robot.homeConfiguration);
            obj@nav.StateSpace('RigidBodyTreeStateSpace', numDoF, robot.TreeInternal.JointPositionLimits);
            obj.RigidBodyTree = robot;
            obj.IKSolver = inverseKinematics('RigidBodyTree', obj.RigidBodyTree);
            obj.IKSolver.SolverParameters.MaxIterations = 100;
            obj.IKSolver.SolverParameters.AllowRandomRestart = false; % disable random restart
            
            obj.PWGRGuidedSampling = 0.5;
            
            obj.UseConstrainedSampling = false;
            obj.EndEffectorIsFree = true;
            obj.DeltaQ = 0.9;
            obj.SkipCheck = false;
            obj.NominalConfig = obj.RigidBodyTree.homeConfiguration;
            
            warning('off', 'robotics:robotmanip:rigidbodytree:ConfigJointLimitsViolationAutoAdjusted');
        end

        function dist = distance(obj, state1, state2)
            %distance Distance in configuration space
            
            D = state2 - state1;
            
            dist = sqrt(D(:,1).^2 + D(:,2).^2 + D(:,3).^2);

        end

        function interpState = interpolate(obj, state1, state2, ratios)
            %interpolate
            stateDiff = state2 - state1;

            % Interpolation in the configuration space
            interpState = state1 + ratios(:) * stateDiff;
            
            obj.SkipCheck = true;
            if obj.UseConstrainedSampling
                
                interpState(end,:) = obj.constrainConfig_EE_YUp(interpState(end,:)); % only project the last one to save time.
            end
            obj.SkipCheck = false;
        end
        
    
        function rstate = sampleUniform(obj)
            %sampleUniform
            
            % there are two different modes to generate random sample
            % mode 1 - random C-space exploration
            % mode 2 - random sample in the WGR then convert to C-space configuration through IK 

            r = rand();
            if r < obj.PWGRGuidedSampling
                T = obj.sampleWorkspaceGoalRegion();
                weights = ones(1, 6);
                [rstate, solnInfo] = obj.IKSolver.step(obj.EndEffector, T, weights, obj.RigidBodyTree.homeConfiguration);
                %d = obj.distanceToWorkspaceGoalRegion(rstate)
            else
                rstate = obj.RigidBodyTree.randomConfiguration;
                                
                if obj.UseConstrainedSampling
                    % project rand config into constrained manifold
                    result = obj.constrainConfig_EE_YUp(rstate);
                    while any(isnan(result)) % this rand config doesn't work, try next rand config
                        rstate = obj.RigidBodyTree.randomConfiguration;
                        result = obj.constrainConfig_EE_YUp(rstate);
                    end
                    rstate = result;
                end

            end
            
            % exclude joints not on the immediate kinematic path
            rstate(obj.JointsNotOnKinPath) = obj.NominalConfig(obj.JointsNotOnKinPath);
            
        end
        
        function boundedState = enforceStateBounds(obj, state)
            %enforceStateBounds
            
            boundedState = state;
        end
        
                
        function setWorkspaceGoalRegion(obj, Twgr_0, Te_w, bounds)
            %setWorkspaceGoalRegion
            %   Twgr_0 - workspace goal region (WGR) pose in world coordinates
            %   Te_w   - end-effector pose in sampled workspace coordinates
            %   bounds - bounds on the WGR
            %   
            wgr.Twgr = Twgr_0;
            wgr.Te = Te_w;
            wgr.Bounds = bounds;
            obj.WorkspaceGoalRegion = wgr;
        end
        
        function dist = distanceToWorkspaceGoalRegion(obj, q)
            %distanceToWorkspaceGoalRegion
            Ts_0 = obj.RigidBodyTree.getTransform(q, obj.EndEffector);
            wgr = obj.WorkspaceGoalRegion;
            
            Tsp_0 = Ts_0*robotics.core.internal.SEHelpers.tforminvSE3(wgr.Te);
            Tsp_wgr = robotics.core.internal.SEHelpers.tforminvSE3(wgr.Twgr)*Tsp_0;
            
            % convert Tsp_wgr to displacement vector in WGR
            t = Tsp_wgr(1:3,4);
            R = Tsp_wgr(1:3,1:3);
            dw = [ t; atan2(R(3,2), R(3,3)); -asin(R(3,1)); atan2(R(2,1), R(1,1))];
            
            % taking into account of WGR bounds
            d = zeros(6,0);
            for i = 1:6
                if dw(i) < wgr.Bounds(i,1)
                    d(i) = dw(i) - wgr.Bounds(i,1);
                elseif dw(i) > wgr.Bounds(i,2)
                    d(i) = dw(i) - wgr.Bounds(i,2);
                else
                    d(i) = 0;
                end
            end
            
            dist = norm(d);
        end
                
        
    end
    
    methods
        
        function qNew = constrainConfig_EE_YUp(obj, q)
        %CONSTRAINCONFIG_EE_YUp

            % distance to constraint

            Ts_0 = obj.RigidBodyTree.getTransform(q, obj.EndEffector);
            R = [0 0 1; 1 0 0; 0 1 0]; 

            Tc_0 = rotm2tform(R); % pointing end-effector Y axis up
            bounds = [-inf, inf;
                      -inf, inf;
                      -inf, inf; 
                      -0.01, 0.01;
                      -pi, pi;
                      -0.01, 0.01];

            T = robotics.core.internal.SEHelpers.tforminvSE3(Tc_0)*Ts_0;

            % convert T to displacement vector in constraint
            t = T(1:3,4);
            R = T(1:3,1:3);
            dw = [ t; atan2(R(3,2), R(3,3)); -asin(R(3,1)); atan2(R(2,1), R(1,1))]; % extrinsic X, Y, Z is equal to intrisinc ZYX
                                                                                    % equivalent: ypr = tform2eul(T); rpy = [ ypr(3), ypr(2), ypr(1) ];

            d = zeros(1,6);
            for i = 1:6
                if dw(i) < bounds(i,1)
                    d(i) = dw(i) - bounds(i,1);
                elseif dw(i) > bounds(i,2)
                    d(i) = dw(i) - bounds(i,2);
                else
                    d(i) = 0;
                end
            end

            % convert displacement vector back to delta T
            dT = trvec2tform(d(1:3))*eul2tform([d(6), d(5), d(4)]);
            dT(1:3,4) = zeros(3,1);
            Ts_mod = Ts_0 * robotics.core.internal.SEHelpers.tforminvSE3(dT);


            
            % after fixing the orientation, try to limit the work space
            % otherwise the IK most likely won't find a good solution
            t = Ts_mod(1:3,4);
            boundsWorkspace = [-0.5, 0.5;
                      -0.5, 0.5;
                      0, 0.8];
            d2 = zeros(1,3);
            for i = 1:3
                if t(i) < boundsWorkspace(i,1)
                    d2(i) = t(i) - boundsWorkspace(i,1);
                elseif t(i) > boundsWorkspace(i,2)
                    d2(i) = t(i) - boundsWorkspace(i,2);
                else
                    d2(i) = 0;
                end
            end
            dT2 = trvec2tform(d2(1:3));
            Ts_mod = robotics.core.internal.SEHelpers.tforminvSE3(dT2 )  * Ts_mod ;
            
            [qNew, solInfo] = obj.IKSolver.step(obj.EndEffector, Ts_mod, ones(1, 6), q);
            
            % throw away the result if qNew if no good IK result is found
            if ~obj.SkipCheck && solInfo.ExitFlag > 1
                qNew = nan*qNew;
            end

        end
        
    end
    
    methods (Access = ?ExampleHelperRigidBodyTreeStateSpace)
        
        function T = sampleWorkspaceGoalRegion(obj)
            %sampleWorkspaceGoalRegion
            
            wgr = obj.WorkspaceGoalRegion;
            dif = wgr.Bounds(:,2) - wgr.Bounds(:,1);
            
            % get random sampled displacement vector in workspace
            dw = rand(6,1).*dif + wgr.Bounds(:,1);
            
            % convert to tranformation matrix in workspace coordinates
            Tw = trvec2tform(dw(1:3)') * eul2tform(dw(4:6)', 'XYZ');
            
            % convert into world coordinates
            T = wgr.Twgr * Tw * wgr.Te;
        end        
            

        function updateJointsNotOnKinPath(obj)
            %updateJointsNotOnKinPath
            indices = obj.RigidBodyTree.TreeInternal.kinematicPath(obj.RigidBodyTree.BaseName, obj.EndEffector);
            pDofMap = obj.RigidBodyTree.TreeInternal.PositionDoFMap;
            dofsAll = pDofMap(:,1);
            dofsAll = dofsAll(dofsAll > 0);
            dofsOnKinPath = pDofMap(indices(2:end),1);
            dofsOnKinPath = dofsOnKinPath(dofsOnKinPath>0);
            mask = ~(ismember(dofsAll, dofsOnKinPath));
            obj.JointsNotOnKinPath = dofsAll(mask)';
        end
        

        
    end
    
    
    
    methods
        function set.EndEffector(obj, ee)
            %set.EndEffector
            obj.EndEffector = ee;
            obj.updateJointsNotOnKinPath();
        end
        
        function copyObj = copy(obj)
            %copy
            
            % not used
        end
        
                
        function state = sampleGaussian(obj, meanState, stdDev)
            %sampleGaussian
            
            % not used
        end
        
    end
end
