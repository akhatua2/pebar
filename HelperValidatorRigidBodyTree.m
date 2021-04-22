classdef HelperValidatorRigidBodyTree < nav.StateValidator & ...
                                               robotics.manip.internal.InternalAccess
    
    properties
        %Parts Robot parts
        Parts
        
        %Obstacles Obstacles in environment
        Obstacles
        
        %PartMask
        PartMask
        
        %ValidationDistance
        ValidationDistance
    end

    properties (Access = protected)
        %ObstacleColors
        ObstacleColors
        
        %ObstaclePatchObjs
        ObstaclePatchObjs
        
        %ObstacleNameIDMap
        ObstacleNameIDMap
        
        %ObstacleMask If an obstacle is active
        ObstacleMask
        
        %CurrentObsID
        CurrentObsID
    end
    
    methods
        function obj = HelperValidatorRigidBodyTree(ssRBT)
            %EXAMPLEHELPERVALIDATORRIGIDBODYTREE
            obj@nav.StateValidator(ssRBT);
            robot = ssRBT.RigidBodyTree;
            
            obj.PartMask = ones(1, robot.NumBodies);
            
            simplificationThreshold = 50;
            for i = 1:robot.NumBodies
                V = robot.TreeInternal.Bodies{i}.VisualsInternal{1}.Vertices; % only extracting the first geometry in each body
                T = robot.TreeInternal.Bodies{i}.VisualsInternal{1}.Tform;
                V = double(V);
                
                originalNumV = size(V,1);
                VT = T*[V'; ones(1,originalNumV)];
                V = VT(1:3,:)';
                

                if originalNumV > simplificationThreshold
                    
                    %simplify the mesh
                    f = figure;
                    f.Visible = "off";
                    ax = axes(f);
                    F = convhull(V(:,1), V(:,2), V(:,3));
                    V1 = V(unique(F(:)),:);
                    F1 = convhull(V1(:,1), V1(:,2), V1(:,3));
                    if size(V1,1) <= simplificationThreshold
                        V2 = V1;
                    else
                        p = patch(ax, 'Faces',F1,'Vertices',V1);
                        [~, V2] = reducepatch(p, simplificationThreshold/size(V1,1));
                    end
                    close(f)
                else
                    V2 = V;
                end
                
                obj.Parts{i} = collisionMesh(V2);
                %obj.Parts{i}.show;
                %close(gcf)
            end
            
            obj.ValidationDistance = inf;
            
            obj.ObstacleNameIDMap = containers.Map();

        end
        
        function skipCollisionCheck(obj, bodyName)
            %skipCollisionCheck
            bid = obj.StateSpace.RigidBodyTree.TreeInternal.findBodyIndexByName(bodyName);
            obj.PartMask(bid) = 0;
        end
        
        function addFixedObstacle(obj, obsGeom, obsName, obsColor)
            %addFixedObstacle obsGeom is expected to be a collision geometry
            if ~obj.ObstacleNameIDMap.isKey(obsName)
                obj.Obstacles{end+1} = obsGeom;
                obj.ObstacleNameIDMap(obsName) = numel(obj.Obstacles);
                obj.ObstacleMask{end+1} = true; % is active;
                obj.ObstacleColors{end+1} = obsColor;
            end
        end
        
        function showObstacles(obj, ax)
            %showObstacles
            for i = 1:numel(obj.ObstaclePatchObjs)
                if ~isempty(obj.ObstaclePatchObjs{i}) && isvalid(obj.ObstaclePatchObjs{i})
                    delete(obj.ObstaclePatchObjs{i});
                end
            end
            
            N = numel(obj.Obstacles);
            obj.ObstaclePatchObjs = cell(1, N);
            
            for i = 1: N
                [~, po] = obj.Obstacles{i}.show('Parent', ax);
                po.LineStyle = 'none';
                if ~isempty(obj.ObstacleColors{i})
                    po.FaceColor = obj.ObstacleColors{i}; 
                end
                obj.ObstaclePatchObjs{i} = po;
            end

        end
        
        
        function attachTOEE(obj, obsName, q)
            %attachTOEE Currently only support cylinder workpiece
            obsID = obj.ObstacleNameIDMap(obsName);
            geom = copy(obj.Obstacles{obsID});
            if obj.StateSpace.EndEffectorIsFree
                body = rigidBody('workpiece');
                color = obj.ObstacleColors{obsID};
                if isnumeric(color)
                    c = [color, 1];
                else
                    c = [shortNameToRGBTriplet(color), 1];
                end
                colStrut.rgba = c;
                body.BodyInternal.addVisualInternal('Cylinder', [geom.Radius, geom.Length], eye(4), colStrut);
                T1 = obj.StateSpace.RigidBodyTree.getTransform(q, obj.StateSpace.EndEffector);
                T2 = geom.Pose;
                
                dT = robotics.core.internal.SEHelpers.tforminvSE3(T1) * T2;
                body.Joint.setFixedTransform(dT);
                obj.StateSpace.RigidBodyTree.addBody(body, obj.StateSpace.EndEffector);
                
                obj.Parts{end+1} = geom;
                obj.PartMask = [obj.PartMask, 1];
                
                obj.StateSpace.EndEffectorIsFree = false;
                
                obj.ObstacleMask{obsID} = false;
                obj.ObstaclePatchObjs{obsID}.Visible = 'off';
                
                obj.CurrentObsID = obsID;
            end
        end
        
        function removeFromEE(obj, q, ax)
            %removeFromEE
            T = obj.StateSpace.RigidBodyTree.getTransform(q, 'workpiece');
            obj.StateSpace.RigidBodyTree.removeBody('workpiece');
            obj.Parts(end) = [];
            obj.PartMask(end) = [];
            obj.ObstacleMask{obj.CurrentObsID} = true;
            obj.Obstacles{obj.CurrentObsID}.Pose = T;
            
            [~, p] = obj.Obstacles{obj.CurrentObsID}.show('Parent', ax);
            p.FaceColor = obj.ObstacleColors{obj.CurrentObsID};
            p.LineStyle = 'none';
            delete(obj.ObstaclePatchObjs{obj.CurrentObsID});
            obj.ObstaclePatchObjs{obj.CurrentObsID} = p;
            
            obj.StateSpace.EndEffectorIsFree = true;
        end
        
        
        function isValid = isStateValid(obj, state)
            %isStateValid
            
            Ttrees = obj.StateSpace.RigidBodyTree.TreeInternal.forwardKinematics(state);
            
            % first, self collision check
            
            % most well-designed serial robot arms do not self-collide in
            % normal working range, so here we can skip it.
            
            % then collision check with obstacles
            
            isValid = true;
            
            for j = 1:length(obj.Parts)
                for k = 1:length(obj.Obstacles)
                    if obj.PartMask(j) && obj.ObstacleMask{k}
                        %obj.Parts{j}.Pose = Ttrees{j};
                        %inCollision = checkCollision(obj.Parts{j}, obj.Obstacles{k});
                        
                        T = Ttrees{j};
                        %quat1 = tform2quat(T);
                        quat1 = rotmToQuaternion(T(1:3,1:3));
                        %pos1 = tform2trvec(T);
                        pos1 = T(1:3,4);
                        
                        %obj.Parts{j}.Pose = T;
                        inCollision = robotics.core.internal.intersect(obj.Parts{j}.GeometryInternal, ... %obj.Parts{j}.Position, obj.Parts{j}.Quaternion, ...
                                                                       pos1, quat1,...
                                                                       obj.Obstacles{k}.GeometryInternal, ...
                                                                       obj.Obstacles{k}.Position, obj.Obstacles{k}.Quaternion, false);
                        
                        if inCollision
                            isValid = false;
                            break;
                        end
                    end
                end
            end
        end
        
        function [isValid, lastValid] = isMotionValid(obj, state1, state2)
            %isMotionValid
            %isValid = obj.isStateValid(state1) && obj.isStateValid(state2);
            
            dist = obj.StateSpace.distance(state1, state2);
            interval = obj.ValidationDistance/dist;
            interpStates = obj.StateSpace.interpolate(state1, state2, [0:interval:1 1]);
            
            isValid = true;
            for i = 1: size(interpStates,1)
               
                interpSt = interpStates(i,:);
                
                if ~obj.isStateValid(interpSt)
                    isValid = false; 
                    break;
                end
            end
            
            lastValid = inf;
        end
        
        function newObj = copy(obj)
            %copy
        end
    end
end


function q = rotmToQuaternion(R)
    %rotmToQuaternion
    w = 0.5*sqrt(1 + trace(R));
    if w > 1e-10
        m = 1/(4*w);

        x = m * (R(3,2) - R(2,3));
        y = m * (R(1,3) - R(3,1));
        z = m * (R(2,1) - R(1,2));
    else
        x = 0.5*sqrt(1 + R(1,1) - R(2,2) - R(3,3));
        if x > 1e-10
            m = 1/(4*x);
            y = m * (R(1,2) + R(2,1));
            z = m * (R(1,3) + R(3,1));
            w = m * (R(3,2) - R(2,3));
        else
            y = 0.5*sqrt(1 - R(1,1) + R(2,2) - R(3,3));
            if y > 1e-10
                m = 1/(4*y);
                x = m * (R(2,1) + R(1,2));
                z = m * (R(3,2) + R(2,3));
                w = m * (R(1,3) - R(3,1));
            else
                z = 0.5*sqrt(1 - R(1,1) - R(2,2) + R(3,3));
                m = 1/(4*z);
                x = m * (R(1,3) + R(3,1));
                y = m * (R(3,2) + R(2,3));
                w = m * (R(2,1) - R(1,2));
            end
        end
    end
    
    q = [w x y z];
end

function c = shortNameToRGBTriplet(shortName)
    switch(shortName)
        case 'y'
            c = [1 1 0];
        case 'm'
            c = [1 0 1];
        case 'c'
            c = [0 1 1];
        case 'r'
            c = [1 0 0];
        case 'g'
            c = [0 1 0];
        case 'b'
            c = [0 0 1];
        case 'w'
            c = [1 1 1];
        case 'k'
            c = [0 0 0]; 
            
    end
end
