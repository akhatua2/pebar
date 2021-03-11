classdef ExampleHelperFurnitureInRoomValidator < nav.StateValidator
    %EXAMPLEHELPERFURNITUREINROOMVALIDATOR State validator for furnitures
    %   in the room
    
    properties
        %Room
        Room
        
        %FurnitureID
        FurnitureID
        
        %ValidationDistance
        ValidationDistance = inf
    end
    
    methods
        function obj = ExampleHelperFurnitureInRoomValidator(stateSpace, initialFurniturePose)
            %EXAMPLEHELPERFURNITUREINROOMVALIDATOR Constructor
            obj@nav.StateValidator(stateSpace);
            
            % Create a room
            obj.Room = ExampleHelperRoom(6, 2);
            
            % Add furniture to the room
            fn = ExampleHelperFurniture();
            obj.FurnitureID = obj.Room.addFurniture(fn, initialFurniturePose);
        end
        
        function isValid = isStateValid(obj, state)
            %isStateValid
            isValid = zeros(size(state,1), 1);
            for k = 1:size(state, 1)
                T1 = trvec2tform( [state(k,1:2), 0]);
                T2 = eul2tform( [state(k, 3), 0, 0]);
                obj.Room.FurnituresInRoom{obj.FurnitureID}.moveTo(T1*T2);
                inCollision = obj.Room.checkCollision(obj.FurnitureID);
                if inCollision
                    isValid(k) = false;
                else
                    isValid(k) = true;
                end
            end
        end
        
        function [isValid, lastValid] = isMotionValid(obj, state1, state2)
            %isMotionValid
            dist = obj.StateSpace.distance(state1, state2);
            interval = obj.ValidationDistance/dist;
            interpStates = obj.StateSpace.interpolate(state1, state2, 0:interval:1);
            
            interpValid = obj.isStateValid(interpStates);
            
            lastValidIdx = find(~interpValid, 1);
            if isempty(lastValidIdx)
                isValid = true;
                lastValid = state2;
            else
                isValid = false;
                lastValid = interpStates(lastValidIdx,:);
            end            
        end

        function newObj = copy(obj)
            %copy
        end
    end
end

