function isGoalReached = HelperIsStateInWorkspaceGoalRegion(planner, ~, newState)
dist = planner.StateSpace.distanceToWorkspaceGoalRegion(newState);
isGoalReached = dist < 0.02;
end

