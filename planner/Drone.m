classdef Drone
    properties
        StartPose
        GoalPoses
        StateSpace
        StateValidator
        Planner
        GoalCosts
        GoalSolns
        SolnInfo
        AssignedGoal
        InterpolatedPath
        FailedToFindGoals
    end

    methods
        function obj = Drone(startPose, goalPoses, ss, sv)
            obj.StartPose = startPose;
            obj.GoalPoses = goalPoses; % numGoals x numStates -> numGoals x 4
            obj.StateSpace = ss;
            obj.StateValidator = sv;
            obj.Planner = plannerMultiUAVMultiGoalRRT(ss, sv, goalPoses);
            obj.GoalCosts = zeros(size(goalPoses, 1), 1); %Column vector with numGoalPoses rows
            obj.GoalSolns = cell(size(goalPoses, 1), 1); %Column vector with numGoalPoses rows
            obj.InterpolatedPath = cell(1);
            
            %Initialize planner parameters
            obj.Planner.MaxConnectionDistance = 50;
            obj.Planner.GoalBias = 0.10;  
            obj.Planner.MaxIterations = 800;
            obj.Planner.GoalReachedFcn = @(~,x,y)(norm(x(1:3)-y(1:3)) < 30);
            obj.FailedToFindGoals = false;
        end

        function obj = planPaths(obj)
            [obj.GoalCosts, obj.GoalSolns, obj.SolnInfo] = plan(obj.Planner,obj.StartPose);
            % Check for zero costs and set them to infinity
            obj.GoalCosts(obj.GoalCosts == 0) = Inf;
        end

    end
end