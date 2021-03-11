classdef ExampleHelperFurniture < handle & ...
                                  robotics.core.internal.InternalAccess
    %EXAMPLEHELPERFURNITURE
    
    properties
        %BoxLength
        BoxLength
        
        %BoxWidth
        BoxWidth
    end
    
    properties
        %Parts This furniture is assembled from three rectangle parts
        Parts
        
        %FixedTforms Fixed transforms for the rectangles relative to the
        %   furniture's coordinates
        FixedTforms
    end
    
    methods
        function obj = ExampleHelperFurniture()
            %EXAMPLEHELPERFURNITURE Constructor
            
            obj.BoxLength = 1.2;
            obj.BoxWidth = 0.2;
            obj.Parts{1} = collisionBox(obj.BoxLength, obj.BoxWidth, 0);
            obj.Parts{2} = collisionBox(obj.BoxLength, obj.BoxWidth, 0);
            obj.Parts{3} = collisionBox(obj.BoxLength, obj.BoxWidth, 0);
            
            obj.FixedTforms{1} = trvec2tform([0, 0.4, 0]);
            obj.FixedTforms{2} = trvec2tform([0, 0.4, 0]);
            obj.FixedTforms{3} = trvec2tform([0, 0.4, 0]); %* eul2tform([-pi/6 0 0]);
         
            %obj.FixedTforms{1} = trvec2tform([-0.2, 0.4, 0]) * eul2tform([pi/6 0 0]);
            
            obj.Parts{2}.Pose = obj.FixedTforms{2};
            obj.Parts{3}.Pose = obj.FixedTforms{3};
            obj.Parts{1}.Pose = obj.FixedTforms{1};
            
        end
        
        function moveTo(obj, tform)
            %moveTo
            for i = 1:length(obj.Parts)
                T = tform*obj.FixedTforms{i};
                obj.Parts{i}.PoseInternal = T;
                obj.Parts{i}.Position = T(1:3, 4)';
                obj.Parts{i}.Quaternion = rotmToQuaternion(T(1:3, 1:3));
            end
            
        end
        
        function h = show(obj, ax)
            %show
            h = hgtransform(ax);

            hold(ax, 'on');
            [~, p1] = obj.Parts{2}.show('Parent', ax);
            [~, p2] = obj.Parts{1}.show('Parent', ax);
            [~, p3] = obj.Parts{3}.show('Parent', ax);
            p1.Parent = h;
            p2.Parent = h;
            p3.Parent = h;
            hold(ax, 'off')

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