classdef plannerMultiUAVMultiGoalRRT < nav.algs.internal.InternalAccess
    % Extends the navigation toolbox and references the plannerRRT class to create a multi-goal rapidly-exploring random tree (RRT) planner
    
    %#codegen

    %Status codes
    properties (Constant, Access=protected)
        NewGoalReached = 1
        AllGoalsReached = 2
        MaxIterationsReached = 3
        MaxNumTreeNodesReached = 4
        MotionInCollision = 5
        InProgress = 6
        ExistingState = 7
        Unassigned = 8
        PrevGoalReached = 9
    end
    
    %plannerRRT class and classes within nav.algs.etc, can modify
    properties (SetAccess = {?nav.algs.internal.InternalAccess})
        %StateSpace State space for the planner
        StateSpace

        %StateValidator State validator for the planner
        StateValidator
    end

    properties
     
        StateSampler

        %MaxNumTreeNodes Max number of nodes in the search tree
        %   This number does not count the root node.
        %
        %   Default: 1e4
        MaxNumTreeNodes

        %MaxIterations Max number of iterations
        %   This is also the max number of calls to the "extend" routine
        %
        %   Default: 1e4
        MaxIterations

        %MaxConnectionDistance Maximum length of a motion to be added to tree
        %
        %   Default: 0.1
        MaxConnectionDistance

        GoalReachedFcn

        %GoalBias Probability of choosing goal state during state sampling
        %    This property defines the probability of choosing the actual
        %    goal state during the process of randomly selecting states
        %    from the state space.
        %
        %    Default: 0.05
        GoalBias

        GoalPoses   % Matrix of goal poses [x y z heading; ...]
        ReachedGoals % Boolean array indicating which goals have been reached
        CurrentGoalIndex % Index of the goal currently being targeted
        GoalCosts
        GoalSolns
    end

    properties (Access = {?nav.algs.internal.InternalAccess})

        %KeepIntermediateStates
        KeepIntermediateStates

        %NearestNeighborMethod
        NearestNeighborMethod

        %MaxTime
        MaxTime

        %RandomSamplePool Cache for pre-generated random state samples
        RandomSamplePool

        %PregenerateSamples Whether to pre-generate samples
        PregenerateSamples
    end

    properties (Access = protected)

        CurrentGoalState
    end

    methods

        %varargin indicates optional args
        function obj = plannerMultiUAVMultiGoalRRT(stateSpace, stateValidator, goalPoses, varargin)
        %PLANNERRRT Constructor
            obj.validateConstructorInput(stateSpace, stateValidator);

            %Checks number of input arguments
            narginchk(3,15);

            nvPairs = obj.parseInputs(varargin{:});

            obj.StateSpace = stateSpace;
            obj.StateValidator = stateValidator;            

            obj.StateSampler = nvPairs.StateSampler; 
            obj.MaxNumTreeNodes = nvPairs.MaxNumTreeNodes;
            obj.MaxIterations = nvPairs.MaxIterations;
            obj.MaxConnectionDistance = nvPairs.MaxConnectionDistance;
            obj.GoalBias = nvPairs.GoalBias;

            % Initialize the multiple goals
            obj.GoalPoses = goalPoses;
            obj.ReachedGoals = false(size(goalPoses, 1), 1);
            obj.CurrentGoalIndex = 1; % Start with the first goal
            obj.GoalCosts = zeros(size(goalPoses, 1), 1); %Column vector with numGoalPoses rows
            obj.GoalSolns = cell(size(goalPoses, 1), 1); %Column vector with numGoalPoses rows

            if (isa(nvPairs.GoalReachedFcn,"function_handle") && strcmp(func2str(nvPairs.GoalReachedFcn),'undefined'))
                if coder.target('MATLAB')
                    obj.GoalReachedFcn = @nav.algs.checkIfGoalIsReached;
                end
            else
                obj.GoalReachedFcn = nvPairs.GoalReachedFcn;
            end
        end

        function [pathCosts, pathObjs, solnInfo] = plan(obj, startState)
        %plan Plan a path between two states
            if coder.target('MATLAB')
                cleaner = onCleanup(@()obj.cleanUp);
            end
            %validate the startState and goalState

            for i = 1:size(obj.GoalPoses, 1)
                [startState, ~] = validateStartGoal( ...
                    obj, startState, obj.GoalPoses(i,:));
            end

            if ~coder.internal.is_defined(obj.GoalReachedFcn)
                obj.GoalReachedFcn = @nav.algs.checkIfGoalIsReached;
            end

            tentativeGoalIds = [];

            treeInternal = initializeSearchTree(obj, startState);
            %populate the random sample pool and skip state validation for
            % manipulatorStateSpace/CollisionBodyValidator
            setupPlanLoop(obj);

            pathFound = false;
            
            statusCode = obj.Unassigned;
            numIterations = 0;
            for k = 1:obj.MaxIterations
                if obj.PregenerateSamples
                    randState = obj.RandomSamplePool(k,:); % dispense random sample from the pool
                else
                    randState = obj.sampleStates(1);       % <-- fallback-safe sampler
                end

                if rand() < obj.GoalBias
                    % Randomly bias towards one of the remaining goals
                    remainingGoals = obj.GoalPoses(~obj.ReachedGoals, :);
                    goalIdx = randi(size(remainingGoals, 1));
                    randState = remainingGoals(goalIdx, :);
                    
                end

                [newNodeId, statusCode, treeInternal] = extend(obj, randState, treeInternal);

                if statusCode == obj.AllGoalsReached
                    pathFound = true;
                    tentativeGoalIds = [tentativeGoalIds, newNodeId]; %#ok<AGROW>
                    numIterations = k;
                    break;
                end

                if statusCode == obj.NewGoalReached
                    pathFound = true;
                    tentativeGoalIds = [tentativeGoalIds, newNodeId]; %#ok<AGROW>
                end

                if statusCode == obj.MaxNumTreeNodesReached
                    numIterations = k;
                    break;
                end
            end

            [pathCosts, pathObjs, solnInfo]= postPlanningLoop( obj,...
                                                    treeInternal, tentativeGoalIds, numIterations, statusCode, pathFound);

            obj.cleanUp();

        end

        function objCopied = copy(obj)
        %COPY Create a deep copy of the plannerRRT object

            validator = obj.StateValidator.copy;
            sampler = obj.StateSampler.copy;
            sampler.StateSpace = validator.StateSpace;
            objCopied = plannerRRT(validator.StateSpace, validator, StateSampler=sampler);
            obj.copyProperties(objCopied);
        end
    end

    methods (Access = private)
        function X = sampleStates(obj, n)
            if nargin < 2, n = 1; end
    
            % 1) If user provided a sampler with sample(n), use it
            if ~isempty(obj.StateSampler) && ismethod(obj.StateSampler,'sample')
                X = obj.StateSampler.sample(n);
                return;
            end
    
            % 2) If state space has sampleUniform, prefer that
            if ismethod(obj.StateSpace, 'sampleUniform')
                X = obj.StateSpace.sampleUniform(n);
                return;
            end
    
            % 3) Otherwise, sample from bounds (robust lookup)
            B = [];
            candProps = {'Bounds','StateBounds','RandomStateBounds','DefaultRandomStateBounds'};
            for k = 1:numel(candProps)
                p = candProps{k};
                if isprop(obj.StateSpace, p), B = obj.StateSpace.(p); break; end
            end
            if isempty(B)
                candMeth = {'getBounds','bounds','stateBounds','randomStateBounds','defaultRandomStateBounds'};
                for k = 1:numel(candMeth)
                    m = candMeth{k};
                    if ismethod(obj.StateSpace, m), B = obj.StateSpace.(m); break; end
                end
            end
            assert(~isempty(B), 'Could not determine state-space bounds for sampling.');
    
            lo = B(:,1)'; hi = B(:,2)';
            X  = rand(n, numel(lo)).*(hi - lo) + lo;
    
            if ismethod(obj.StateSpace, 'enforceStateBounds')
                X = obj.StateSpace.enforceStateBounds(X);
            end
        end
    end

    methods (Access = {?nav.algs.internal.InternalAccess})


        function [newNodeId, statusCode, treeInternal] = extend(obj, randState, treeInternal)
        %extend The RRT "Extend" routine
            statusCode = obj.InProgress;
            newNodeId = nan;
            idx = treeInternal.nearestNeighbor(randState); %Get the nearest neighbor
            nnState = treeInternal.getNodeState(idx); %Get the state of the neighbor

            d = obj.StateSpace.distance(randState, nnState); %Calculate the distance
            newState = randState;

            % steer
            if d > obj.MaxConnectionDistance
                newState = obj.StateSpace.interpolate(nnState, randState, obj.MaxConnectionDistance/d);  % L/d*(randState - nnState) + nnState;
                % Making sure the interpolated output is a vector during run time.
                coder.internal.assert(isvector(newState), 'nav:navalgs:plannerrrt:InterpRowVector');
                % For codegen, making sure newState is a compile time row vector.
                newState = reshape(newState, 1, []);
            end

            % check motion validity
            if ~obj.StateValidator.isMotionValid(nnState, newState)
                statusCode = obj.MotionInCollision;
                return;
            end

            newNodeId = treeInternal.insertNode(newState, idx);

            % Check if the new node reaches any goals
            for i = 1:size(obj.GoalPoses, 1)
                if obj.GoalReachedFcn(obj, newState, obj.GoalPoses(i, :))

                    currGoalAlreadyReached = obj.ReachedGoals(i);
                    if ~currGoalAlreadyReached
                        obj.ReachedGoals(i) = true; % Mark this goal as reached
                        statusCode = obj.NewGoalReached; % Indicate a single goal is reached
                    else
                        statusCode = obj.PrevGoalReached; % Indicate we re-reached a previous goal
                    end

                    % Check if all goals are reached
                    if all(obj.ReachedGoals)
                        statusCode = obj.AllGoalsReached;          
                    end
                    return; % Exit immediately after setting the status
                end
            end

            if newNodeId == obj.MaxNumTreeNodes
                statusCode = obj.MaxNumTreeNodesReached;
                return
            end

        end

        function cleanUp(obj)
        %cleanUp To clean up after plan
            svInternal = obj.StateValidator;
            ssInternal = svInternal.StateSpace;
            switch class(ssInternal)
                case 'stateSpaceSE2'
                ssInternal.SkipStateValidation = false;
                case 'stateSpaceDubins'
                ssInternal.SkipStateValidation = false;
                case 'stateSpaceReedsShepp'
                ssInternal.SkipStateValidation = false;
                case 'manipulatorStateSpace'
                ssInternal.SkipStateValidation = false;
            end
            if isa(obj.StateValidator, 'validatorOccupancyMap') || ...
                    isa(obj.StateValidator, 'manipulatorCollisionBodyValidator')
                obj.StateValidator.SkipStateValidation = false;
                obj.StateSpace.SkipStateValidation = false;
            end
        end

        function treeInternal = initializeSearchTree(obj, startState)
    
            treeInternal = nav.algs.internal.SearchTree(startState, obj.MaxNumTreeNodes);
            extendsReversely = true;
            switch class(obj.StateSpace)
                case 'stateSpaceSE2'
                weights = [obj.StateSpace.WeightXY, obj.StateSpace.WeightXY, obj.StateSpace.WeightTheta];
                topologies = [0 0 1];
                treeInternal.configureCommonCSMetric(topologies, weights, ~extendsReversely);
                obj.StateSpace.SkipStateValidation = true;
                obj.PregenerateSamples = true;
                case 'stateSpaceDubins'
                treeInternal.configureDubinsMetric(obj.StateSpace.MinTurningRadius, ~extendsReversely);
                obj.StateSpace.SkipStateValidation = true;
                obj.PregenerateSamples = true;
                case 'stateSpaceReedsShepp'
                treeInternal.configureReedsSheppMetric(obj.StateSpace.MinTurningRadius, obj.StateSpace.ReverseCost, ~extendsReversely);
                obj.StateSpace.SkipStateValidation = true;
                obj.PregenerateSamples = true;
                case 'manipulatorStateSpace'
                obj.StateSpace.SkipStateValidation = true;
                obj.PregenerateSamples = true;
                otherwise %This case is always entered in the current implementation, the other options could be used for future work
                ss = obj.StateSpace;
                treeInternal.setCustomizedStateSpace(ss, ~extendsReversely); 
            end


        end

        function [pathCosts, pathObjs, solnInfo] = postPlanningLoop( obj,...
                                                            treeInternal, tentativeGoalIds, numIterations, statusCode, pathFound)
        

            if numIterations == 0
                numIterations = obj.MaxIterations;
            end

            treeData = treeInternal.inspect();
            numNodes = treeInternal.getNumNodes()-1;

            exitFlag = statusCode;
            if statusCode >= obj.MotionInCollision
                exitFlag = obj.MaxIterationsReached;
            end

            if pathFound
                %fprintf('Number of goals found: %d\n', length(tentativeGoalIds));
                for j = 1:length(tentativeGoalIds)
                    nid = tentativeGoalIds(j);
                    cost = treeInternal.getNodeCostFromRoot(nid);             
                    %fprintf('Cost: %d\n', cost);
                    pathStates = treeInternal.tracebackToRoot(nid);
                    goalState = treeInternal.getNodeState(nid);
                    %fprintf('GoalState: %d, %d, %d\n', goalState(1), goalState(2), goalState(3))
                    
                    % Find the corresponding index in GoalPoses
                    [~, goalIndex] = min(vecnorm(obj.GoalPoses(:,1:3) - goalState(1:3), 2, 2)); % second arg 2 = L2 Norm, third arg 2 = columns 
                    %fprintf('GoalIndex: %d\n', goalIndex);
                    
                    % Assign cost and solution to the correct index
                    obj.GoalCosts(goalIndex) = cost;
                    obj.GoalSolns{goalIndex} = navPath(obj.StateSpace, flip(pathStates'));
                end
            else
                % If no path is found, set all costs to Inf and solutions to empty
                obj.GoalCosts(:) = Inf;
                [obj.GoalSolns{:}] = deal(navPath(obj.StateSpace));
            end

            pathCosts = obj.GoalCosts;
            pathObjs = obj.GoalSolns;

            solnInfo = struct();
            solnInfo.IsPathFound = pathFound;
            solnInfo.ExitFlag = exitFlag;
            solnInfo.NumNodes = numNodes;
            solnInfo.NumIterations = numIterations;
            solnInfo.TreeData = treeData';
        end

        function [startState, goalState] = validateStartGoal( ...
            obj, startState, goalState)
        %validateStartGoal check whether the startState and goalState
        % are valid
            if ~all(obj.StateValidator.isStateValid(startState)) || ...
                    ~all(all(isfinite(startState)))
                coder.internal.error('nav:navalgs:plannerrrt:StartStateNotValid');
            end

            if ~all(obj.StateValidator.isStateValid(goalState)) || ...
                    ~all(all(isfinite(goalState)))
                coder.internal.error('nav:navalgs:plannerrrt:GoalStateNotValid');
            end

            startState = nav.internal.validation.validateStateVector(startState, ...
                                                                        obj.StateSpace.NumStateVariables, 'plan', 'startState');
            goalState = nav.internal.validation.validateStateVector(goalState, ...
                                                                    obj.StateSpace.NumStateVariables, 'plan', 'goalState');

        end

        function setupPlanLoop(obj)
        %setupPlanLoop populate the random sample pool and skip state
        % validation for validatorOccupancyMap and CollisionBodyValidator

            s = rng; %To improve performance, the samples can be cached in future work
            if obj.PregenerateSamples
                % populate random sample pool
                obj.RandomSamplePool = obj.sampleStates(obj.MaxIterations);  % <-- fallback-safe sampler
            else
                obj.RandomSamplePool = zeros(1,obj.StateSpace.NumStateVariables);
            end
            rng(s); % should not let the maxIterations affect the goalBias result

            if isa(obj.StateValidator, 'manipulatorCollisionBodyValidator')
                obj.StateValidator.SkipStateValidation = true;
            end

            if isa(obj.StateValidator, 'validatorOccupancyMap')  %This case is always entered in the current implementation
                obj.StateValidator.SkipStateValidation = true;
                obj.StateValidator.configureValidatorForFastOccupancyCheck();
            end
        end
    end


    methods (Access = protected)
        function validateConstructorInput(obj, ss, sv)
        %validateConstructorInput

            validateattributes(ss, {'nav.StateSpace'}, {'scalar', 'nonempty'}, 'plannerRRT', 'stateSpace');
            validateattributes(sv, {'nav.StateValidator'}, {'scalar', 'nonempty'}, 'plannerRRT', 'stateValidator');

            if coder.target('MATLAB')
                if ss == sv.StateSpace % reference to the same state space object
                    obj.StateSpace = ss;
                    obj.StateValidator = sv;
                else
                    coder.internal.error('nav:navalgs:plannerrrt:RequireSameStateSpace');
                end
            else
                % ignore the handle check during codegen
                obj.StateSpace = ss;
                obj.StateValidator = sv;
            end
        end

        function cname = getClassName(obj) %#ok<MANU>
        %getClassName
            cname = 'plannerRRT';
        end

        function copyProperties(obj, copyObj)
        %copyProperties Copy property data from this object to a new object
            copyObj.MaxIterations = obj.MaxIterations;
            copyObj.MaxNumTreeNodes = obj.MaxNumTreeNodes;
            copyObj.MaxConnectionDistance = obj.MaxConnectionDistance;
            if coder.internal.is_defined(obj.GoalReachedFcn)
                copyObj.GoalReachedFcn = obj.GoalReachedFcn;
            end
            copyObj.GoalBias = obj.GoalBias;
        end

    end

    % setters
    methods

        function set.StateSampler(obj, stateSampler)
            % Allow empty (use fallback) or any handle with a sample method
            if ~isempty(stateSampler)
                obj.validateStateSampler(stateSampler);
            end
            obj.StateSampler = stateSampler;
        end

        function set.GoalReachedFcn(obj, goalReachedFcn)
        %set.GoalReachedFcn Set the handle to the function that
        %   determines if goal is reached
            validateattributes(goalReachedFcn, {'function_handle'}, {'nonempty'}, getClassName(obj), 'GoalReachedFcn');
            obj.GoalReachedFcn = goalReachedFcn;
        end

        function set.MaxNumTreeNodes(obj, maxNumNodes)
        %set.MaxNumTreeNodes
            robotics.internal.validation.validatePositiveIntegerScalar(maxNumNodes, getClassName(obj), 'MaxNumTreeNodes');
            obj.MaxNumTreeNodes = maxNumNodes;
        end

        function set.MaxIterations(obj, maxIter)
        %set.MaxIterations
            robotics.internal.validation.validatePositiveIntegerScalar(maxIter, getClassName(obj), 'MaxIterations');
            obj.MaxIterations = maxIter;
        end

        function set.MaxConnectionDistance(obj, maxConnDist)
        %set.MaxConnectionDistance
            robotics.internal.validation.validatePositiveNumericScalar(maxConnDist, getClassName(obj), 'MaxConnectionDistance');
            obj.MaxConnectionDistance = maxConnDist;
        end

        function set.GoalBias(obj, goalBias)
        %set.GoalBias
            validateattributes(goalBias, {'double'}, ...
                                {'nonempty', 'scalar', 'real', 'nonnan', 'finite', '>=', 0.0, '<=', 1.0}, ...
                                getClassName(obj), 'GoalBias');
            obj.GoalBias = goalBias;
        end

    end

    methods(Access=private)
        function validateStateSampler(obj, stateSampler)
            % Accept any handle object that implements sample(n), or function handle
            if isa(stateSampler, 'function_handle')
                return;
            end
            if isempty(stateSampler)
                return;
            end
            if ~ishandle(stateSampler) && ~isobject(stateSampler)
                error('StateSampler must be an object or function handle.');
            end
            if ~ismethod(stateSampler, 'sample')
                error('StateSampler must implement a method sample(n).');
            end
        end

        function nvPairs = parseInputs(obj, varargin)
        % Retrieve default properties
            defaults = obj.propertyDefaults;

            % Parse inputs and return NV-pairs
            poptions = struct('PartialMatching','unique');
            pstruct = coder.internal.parseParameterInputs(defaults,poptions,varargin{:});
            nvPairs = coder.internal.vararginToStruct(pstruct,defaults,varargin{:});
        end

        function defaults = propertyDefaults(obj)
            defaults = struct(...
                'MaxNumTreeNodes',1e4,...
                'MaxIterations',1e4,...
                'MaxConnectionDistance',0.1,...
                'GoalBias', 0.05,...
                'GoalReachedFcn',@undefined,...
                'StateSampler', []);
        end      
    end

    methods(Static, Hidden)
        function obj = loadobj(s)
            %loadobj Load a previously saved plannerRRT object
            
            if isa(s, 'plannerRRT')
                obj = s;
                if isempty(s.StateSampler)
                    %StateSampler is introduced in R2023b
                    obj.StateSampler = SimpleUniformSampler(obj.StateSpace);
                end
            else
                % Older MAT files may be resolved as struct
                obj = plannerRRT(s.StateValidator.StateSpace, s.StateValidator);                
                props = setdiff(fieldnames(s), {'StateSpace', 'StateValidator'});
                for i = 1:length(props)
                    obj.(props{i}) = s.(props{i});
                end
            end
        end
    end
end
