function newPth = exampleHelperSmoothPath(pthObj, stateVal)
%EXAMPLEHELPERSMOOTHPATH Use "corner cutting" strategy to smooth the given path

    states = pthObj.States;
    
    numStates = pthObj.NumStates;
    prevNumStates = inf;
    originalValDist = stateVal.ValidationDistance;
    stateVal.ValidationDistance = originalValDist/4;
    
    while prevNumStates > numStates
        mask = true(numStates, 1);
        for i = 1:3:numStates
            if i+2 <= numStates
                s1 = states(i,:);
                s2 = states(i+2,:);
                if stateVal.isMotionValid(s1, s2)
                    mask(i+1) = false;
                end
            end
        end
        states = states(mask, :);
        prevNumStates = numStates;
        numStates = size(states,1);
    end
    
    newPth = navPath(stateVal.StateSpace, states);
         
end

