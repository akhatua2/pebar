function exampleHelperShowFurnitureTrace(furniture, states)
%EXAMPLEHELPERSHOWFURNITURETRACE

V = [   
   -0.5000    0.5000         0
    0.5000    0.5000         0
    0.5000   -0.5000         0
    0.3000   -0.5000         0
    0.3000    0.3000         0
   -0.3000    0.3000         0
   -0.3000   -0.5000         0
   -0.5000   -0.5000         0
   -0.5000    0.5000         0];


F = [1 2 3 4 5 6 7 8 9];

hold on
numStates = size(states,1);
numV = size(V,1);
for i = 1:numStates
    s = states(i,:);
    T = robotics.core.internal.SEHelpers.poseToTformSE2(s);
    if i < numStates
        tmp = T*[V(:,1:2), ones(numV,1)]';
        Vi = tmp(1:2,:)';
        p = patch('Faces', F, 'Vertices', Vi);
        p.LineStyle = '-';
        p.FaceColor = 0.38*ones(1,3) + 0.5*ones(1,3)*(numStates - i)/numStates;
    else
        T = [ [blkdiag(T(1:2,1:2),1), [T(1:2,3); 0] ]; [0 0 0 1] ];
        furniture.moveTo(T);
        furniture.show(gca);
    end
end

end

