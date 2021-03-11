classdef ExampleHelperRoom < handle & ...
                             robotics.core.internal.InternalAccess
    %EXAMPLEHELPERROOM
    
    properties
        %Length Length of the room
        Length
        
        %Width Width of the room
        Width
        
        %RoomFixtures Collection of room fixtures (like poles, walls)
        RoomFixtures
        
        %FunituresInRoom Collection of furnitures in the room
        FurnituresInRoom
    end
    
    methods
        function obj = ExampleHelperRoom(length, width)
            %EXAMPLEHELPERROOM Constructor
            obj.Length = length;
            obj.Width = width;
            
            % Add poles
            obj.RoomFixtures{1} = collisionBox(0.2, 0.2, 0);
            obj.RoomFixtures{1}.Pose = trvec2tform([0, 3.5, 0]);

            obj.RoomFixtures{2} = collisionBox(0.2, 0.2, 0);
            obj.RoomFixtures{2}.Pose = trvec2tform([0, 0, 0]);
            
            obj.RoomFixtures{3} = collisionBox(1.0, 0.2, 0);
            obj.RoomFixtures{3}.Pose = trvec2tform([-0.5, 2.8, 0]);
            
            obj.RoomFixtures{7} = collisionBox(1.0, 0.2, 0);
            obj.RoomFixtures{7}.Pose = trvec2tform([0.5, 1.6, 0]);
            
            % Add walls
            obj.RoomFixtures{4} = collisionBox(obj.Width, 0.1, 0);
            obj.RoomFixtures{4}.Pose = trvec2tform([0, obj.Length - 1.3 , 0]);
            
            obj.RoomFixtures{5} = collisionBox(0.1, obj.Length, 0);
            obj.RoomFixtures{5}.Pose = trvec2tform([-obj.Width/2, obj.Length/3, 0]); % wall on the left
            
            obj.RoomFixtures{6} = collisionBox(0.1, obj.Length, 0);
            obj.RoomFixtures{6}.Pose = trvec2tform([obj.Width/2, obj.Length/3, 0]); % wall on the right
        end
        
        function fnID = addFurniture(obj, furniture, initialPose)
            %addFurniture
            fn = furniture;
            fn.moveTo(initialPose);
            obj.FurnituresInRoom{end+1} = fn;
            fnID = length(obj.FurnituresInRoom);
        end
        
        function show(obj, ax, poses)
            %show
            hold on;
            for k = 1 : length(obj.RoomFixtures)
                [~, p] = obj.RoomFixtures{k}.show('Parent', ax);
                p.FaceColor = [0.3 0.3 0.3];
            end
            for k = 1 : length(obj.FurnituresInRoom)
                if nargin > 2
                    obj.FurnituresInRoom{k}.moveTo(poses{k});
                end
                obj.FurnituresInRoom{k}.show(ax);
            end
            hold on;

        end
        
        function animateFurnitureMotion(obj, furnitureID, states, ax)
            %animateFurnitureMotion
            hold on;
            for k = 1 : length(obj.RoomFixtures)
                [~, p] = obj.RoomFixtures{k}.show('Parent', ax);
                p.FaceColor = [0.3 0.3 0.3];
            end
            
            set(gcf, 'color', [1 1 1]);
            box(ax, 'on');

            h = [];
            T0 = eye(4);
            axis equal
            for j = 1:size(states,1)
                s = states(j,:);
                T = trvec2tform([s(1) s(2) 0]) * eul2tform([s(3) 0 0]);
                obj.FurnituresInRoom{furnitureID}.moveTo(T);
                if isempty(h)
                    h = obj.FurnituresInRoom{furnitureID}.show(ax);
                    T0 = T;
                else
                    h.Matrix = T*robotics.core.internal.SEHelpers.tforminvSE3(T0);
                end
                 
                pause(0.1)
                 
            end
            
            hold on
            plot(ax, states(:,1), states(:,2), 'r.-', 'linewidth', 3)

        end
        
        function inCollision = checkCollision(obj, furnitureID)
            %checkCollision Check collision for the specified furniture
            inCollision = false;
            numParts = length(obj.FurnituresInRoom{furnitureID}.Parts);
            for i = 1:length(obj.RoomFixtures)
                for j = 1:numParts
                    %cc = checkCollision(obj.RoomFixtures{i}, obj.FurnituresInRoom{furnitureID}.Parts{j});
                    pos1 = obj.RoomFixtures{i}.Position;
                    quat1 = obj.RoomFixtures{i}.Quaternion;
                    pos2 = obj.FurnituresInRoom{furnitureID}.Parts{j}.Position;
                    quat2 = obj.FurnituresInRoom{furnitureID}.Parts{j}.Quaternion;
                    cc = robotics.core.internal.intersect(obj.RoomFixtures{i}.GeometryInternal, pos1, quat1,...
                                                     obj.FurnituresInRoom{furnitureID}.Parts{j}.GeometryInternal, pos2, quat2, 0);
                    if cc == 1
                        inCollision = true;
                        return
                    end
                end
            end
            
        end
        
    end
end