function isReached = exampleHelperGoalFunc(obj, goalState, newState)
%EXAMPLEHELPERGOALFUNC

isReached = false;
if abs(newState(1) < goalState(1)) < 0.01 && ...
    abs(newState(2) < goalState(2)) < 0.01 && ...
     abs(newState(3) < goalState(3)) < 0.02 %#ok<*ELARLOG>
    isReached = true;
end

end

